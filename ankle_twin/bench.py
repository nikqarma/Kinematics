"""Bench-test rig for the ankle-foot model: hold, drive and load the foot like an examiner.

The foot is held with joint equality constraints rather than explicit PD
torques: the constraint solver handles them implicitly, so even tiny
inertias (the subtalar DOF) stay stable at the model's 2 ms time step.

    rig = Bench("model/ankle_foot_r.xml")
    rig.hold(ankle_deg=-10, subtalar_deg=0)        # examiner holds the foot
    rig.settle(1.0, heel_force=[150, 0, 0])        # anterior drawer, 150 N
    rig.q("talus_tx_r")                            # anterior talar translation (m)
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

HELD = ("ankle_angle_r", "subtalar_angle_r")
# Extra degrees of freedom of the 6-DOF joints (talocrural.py, subtalar.py), where the build has them.
TALAR_PLAY = ("talus_tx_r", "talus_ty_r", "talus_tz_r", "talus_tilt_r", "talus_rot_r")
SUBTALAR_PLAY = ("subtalar_tx_r", "subtalar_ty_r", "subtalar_tz_r", "subtalar_flex_r", "subtalar_twist_r")
LIGAMENTS = ("ATFL_r", "ATFLi_r", "CFL_r", "PTFL_r", "TNL_r", "TSL_r", "TCL_r", "dATTL_r", "dPTTL_r", "CL_r", "ITCL_r")

# Points on the calcaneus/foot body (calcn_r frame: x anterior, y superior, z lateral).
HEEL = np.array([0.0, 0.015, 0.0])        # posterior calcaneus, for drawer tests
SOLE_UNDER_ANKLE = np.array([0.049, -0.009, -0.008])  # plantar point under the talus


def ligament_tendon_ids(m: mujoco.MjModel, name: str) -> list[int]:
    """Tendon ids making up one ligament: the base spring plus any '<name>__sN' segments."""
    return [i for i in range(m.ntendon)
            if m.tendon(i).name == name or m.tendon(i).name.startswith(name + "__s")]


def _has_joint(spec: mujoco.MjSpec, name: str) -> bool:
    return any(j.name == name for j in spec.joints)


def ligament_force(m: mujoco.MjModel, d: mujoco.MjData, name: str) -> float:
    """Total elastic tension (N) in a ligament, summed over its spring segments."""
    ids = ligament_tendon_ids(m, name)
    return float(np.sum(m.tendon_stiffness[ids] * np.maximum(0.0, d.ten_length[ids] - m.tendon_lengthspring[ids, 1])))


class Bench:
    def __init__(self, xml: str | Path, gravity: bool = False):
        spec = mujoco.MjSpec.from_file(str(xml))
        self._subtalar_play = [j for j in SUBTALAR_PLAY if _has_joint(spec, j)]
        for j in list(HELD) + self._subtalar_play:
            eq = spec.add_equality()
            eq.name = f"hold_{j}"
            eq.type = mujoco.mjtEq.mjEQ_JOINT
            eq.objtype = mujoco.mjtObj.mjOBJ_JOINT
            eq.name1 = j
            eq.active = False
            eq.solref = [0.004, 1.0]
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        self.m.opt.gravity[2] = -9.81 if gravity else 0.0
        self._hold_ids = {j: self.m.equality(f"hold_{j}").id for j in list(HELD) + self._subtalar_play}
        self.calcn = self.m.body("calcn_r").id
        self.lig_ids = [self.m.tendon(n).id for n in LIGAMENTS if self._has_tendon(n)]
        self.reset()

    def _has_tendon(self, name: str) -> bool:
        try:
            self.m.tendon(name)
            return True
        except KeyError:
            return False

    # --- control ---------------------------------------------------------------
    def reset(self) -> None:
        mujoco.mj_resetData(self.m, self.d)
        self.hold(None, None)

    def hold(self, ankle_deg: float | None = None, subtalar_deg: float | None = None) -> None:
        """Hold the ankle and/or subtalar angle (None releases that joint).

        Holding the subtalar joint fixes the calcaneus to the talus, as a screw across the joint
        does: on a 6-DOF subtalar build its translations and other two rotations are held at zero too.
        """
        for j, val in zip(HELD, (ankle_deg, subtalar_deg)):
            i = self._hold_ids[j]
            self.d.eq_active[i] = val is not None
            if val is not None:
                self.m.eq_data[i, :5] = [math.radians(val), 0, 0, 0, 0]
        for j in self._subtalar_play:
            self.d.eq_active[self._hold_ids[j]] = subtalar_deg is not None

    def cut(self, *ligaments: str) -> None:
        """Section ligaments (zero their stiffness)."""
        for n in ligaments:
            ids = ligament_tendon_ids(self.m, n)
            self.m.tendon_stiffness[ids] = 0.0
            self.m.tendon_damping[ids] = 0.0

    def settle(self, seconds: float, heel_force=None, sole_force=None, foot_torque=None,
               ankle_ramp_deg: tuple[float, float] | None = None, ctrl: dict[str, float] | None = None) -> bool:
        """Simulate with constant external loads (world frame, N / N*m). Returns False if unstable."""
        t0 = self.d.time
        bad0 = self.d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number
        while self.d.time - t0 < seconds:
            # MuJoCo resets the state (and the clock) when it hits an instability; a clock check
            # alone misses that when t0 is 0, and the loop would rerun the same interval forever.
            if self.d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > bad0:
                return False
            if ankle_ramp_deg is not None:
                a0, a1 = ankle_ramp_deg
                s = min(1.0, (self.d.time - t0) / (0.6 * seconds))
                self.m.eq_data[self._hold_ids["ankle_angle_r"], 0] = math.radians(a0 + s * (a1 - a0))
            self.d.qfrc_applied[:] = 0
            for local, F in ((HEEL, heel_force), (SOLE_UNDER_ANKLE, sole_force)):
                if F is not None:
                    p = self.d.xpos[self.calcn] + self.d.xmat[self.calcn].reshape(3, 3) @ local
                    mujoco.mj_applyFT(self.m, self.d, np.asarray(F, float), np.zeros(3), p,
                                      self.calcn, self.d.qfrc_applied)
            if foot_torque is not None:
                mujoco.mj_applyFT(self.m, self.d, np.zeros(3), np.asarray(foot_torque, float),
                                  self.d.xpos[self.calcn], self.calcn, self.d.qfrc_applied)
            if ctrl:
                for k, v in ctrl.items():
                    self.d.ctrl[self.m.actuator(k).id] = v
            mujoco.mj_step(self.m, self.d)
            if not np.all(np.isfinite(self.d.qpos)) or self.d.time < t0:
                return False  # diverged (MuJoCo resets time on instability)
        return True

    # --- readouts --------------------------------------------------------------
    def q(self, joint: str) -> float:
        return float(self.d.qpos[self.m.joint(joint).qposadr[0]])

    def site_in_tibia(self, site: str) -> np.ndarray:
        """Current position (m) of a site expressed in the tibia frame."""
        tib = self.m.body("tibia_r").id
        R = self.d.xmat[tib].reshape(3, 3)
        return R.T @ (self.d.site_xpos[self.m.site(site).id] - self.d.xpos[tib])

    def has_joint(self, joint: str) -> bool:
        try:
            self.m.joint(joint)
            return True
        except KeyError:
            return False

    def strains(self) -> dict[str, float]:
        L0 = self.m.tendon_lengthspring[self.lig_ids, 1]
        return dict(zip(LIGAMENTS, self.d.ten_length[self.lig_ids] / L0 - 1))

    def ligament_forces(self) -> dict[str, float]:
        return {n: ligament_force(self.m, self.d, n) for n in LIGAMENTS if self._has_tendon(n)}

    def contact(self) -> tuple[float, float]:
        """(total normal contact force in the talocrural joint N, deepest penetration m)."""
        fn, pen = 0.0, 0.0
        f6 = np.zeros(6)
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            names = self.m.geom(c.geom1).name + self.m.geom(c.geom2).name
            if "trochlea" in names:
                mujoco.mj_contactForce(self.m, self.d, i, f6)
                fn += f6[0]
                pen = min(pen, c.dist)
        return fn, -pen


class HindfootRig:
    """Pellegrini 2016's cadaver rig: the heel in a cradle on a free translation platform.

    The tibia is fixed. The calcaneus is welded to a cradle that translates freely and can turn
    only about the foot's front-to-back axis (inversion-eversion) or its vertical axis
    (internal-external rotation), each of which can be locked. It cannot pitch, and the axis it
    turns about is the rig's, not the joint's: the subtalar joint's own oblique motion is not
    available, which is what makes the cadaver joint stiff in these tests. The ankle can be
    pinned (all six talocrural degrees of freedom held), as Pellegrini did to isolate the
    subtalar joint. Loads act on the cradle: an axial load pressing the heel towards the tibia
    (36 N in the paper), torques about the two axes and drawer forces on the platform.

        rig = HindfootRig(xml, ankle_deg=0)
        rig.pin_ankle()
        rig.settle(0.5)                     # seat under the axial load, both rotations locked
        rig.free("roll")
        rig.settle(1.5, roll=4.0)           # 4 N*m inversion
        rig.angle("roll")                   # calcaneal inversion, deg
    """

    AXES = {"roll": "cradle_roll", "yaw": "cradle_yaw"}

    def __init__(self, xml: str | Path, ankle_deg: float = 0.0):
        spec = mujoco.MjSpec.from_file(str(xml))
        # The cradle is set up on the foot as it sits at this ankle angle (the paper tilts the tibia
        # over a level foot; here the tibia is fixed and the rig's axes follow the foot).
        m0 = spec.compile()
        d0 = mujoco.MjData(m0)
        d0.qpos[m0.joint("ankle_angle_r").qposadr[0]] = math.radians(ankle_deg)
        mujoco.mj_forward(m0, d0)
        cid = m0.body("calcn_r").id
        R = d0.xmat[cid].reshape(3, 3)       # foot axes in the world: x anterior, y superior, z lateral
        # Rotation centre: a point on the subtalar axis under the posterior facet.
        local = R.T @ (d0.xanchor[m0.joint("subtalar_angle_r").id] - d0.xpos[cid])
        cradle = spec.worldbody.add_body(name="cradle", pos=d0.xpos[cid], quat=d0.xquat[cid])
        cradle.mass, cradle.inertia, cradle.explicitinertial = 0.2, [2e-4] * 3, True
        # Platform axes in the foot frame: anterior, medial, superior.
        for name, axis in (("cradle_x", [1, 0, 0]), ("cradle_y", [0, 0, -1]), ("cradle_z", [0, 1, 0])):
            cradle.add_joint(name=name, type=mujoco.mjtJoint.mjJNT_SLIDE, axis=axis, pos=local,
                             limited=False, damping=50.0)
        for name, axis in (("cradle_yaw", [0, 1, 0]), ("cradle_roll", [1, 0, 0])):
            cradle.add_joint(name=name, type=mujoco.mjtJoint.mjJNT_HINGE, axis=axis, pos=local,
                             limited=False, damping=0.5, armature=0.001)
        weld = spec.add_equality(name="cradle_weld", type=mujoco.mjtEq.mjEQ_WELD,
                                 objtype=mujoco.mjtObj.mjOBJ_BODY, name1="cradle", name2="calcn_r")
        weld.data[:11] = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1]     # same pose as the calcaneus
        weld.solref = [0.004, 1.0]
        self._pin = [j for j in ("ankle_angle_r",) + TALAR_PLAY if _has_joint(spec, j)]
        for j in self._pin + list(self.AXES.values()):
            eq = spec.add_equality(name=f"lock_{j}", type=mujoco.mjtEq.mjEQ_JOINT,
                                   objtype=mujoco.mjtObj.mjOBJ_JOINT, name1=j)
            eq.active = False
            eq.solref = [0.004, 1.0]
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        self.m.opt.gravity[:] = 0.0          # the paper counterbalances the cradle and foot
        self.ankle_deg = ankle_deg
        self.d.qpos[self.m.joint("ankle_angle_r").qposadr[0]] = math.radians(ankle_deg)
        for axis in self.AXES:
            self.lock(axis)

    def _set(self, joint: str, on: bool, value: float = 0.0) -> None:
        i = self.m.equality(f"lock_{joint}").id
        self.d.eq_active[i] = on
        self.m.eq_data[i, :5] = [value, 0, 0, 0, 0]

    def lock(self, axis: str) -> None:
        """Lock the cradle's 'roll' (inversion-eversion) or 'yaw' (rotation) at neutral."""
        self._set(self.AXES[axis], True)

    def free(self, axis: str) -> None:
        self._set(self.AXES[axis], False)

    def pin_ankle(self, on: bool = True) -> None:
        """Fix the talus to the tibia at the rig's ankle angle (pin and external fixator)."""
        for j in self._pin:
            self._set(j, on, math.radians(self.ankle_deg) if j == "ankle_angle_r" else 0.0)

    def cut(self, *ligaments: str) -> None:
        for n in ligaments:
            ids = ligament_tendon_ids(self.m, n)
            self.m.tendon_stiffness[ids] = 0.0
            self.m.tendon_damping[ids] = 0.0

    def settle(self, seconds: float, axial: float = 36.0, roll: float = 0.0, yaw: float = 0.0,
               drawer=(0.0, 0.0)) -> bool:
        """Simulate with constant loads on the cradle. roll: N*m, + = inversion; yaw: N*m, + = internal
        rotation of the heel; drawer: (anterior, medial) N on the platform. Returns False if unstable."""
        dof = {j: self.m.joint(j).dofadr[0] for j in ("cradle_x", "cradle_y", "cradle_z", "cradle_yaw", "cradle_roll")}
        t0 = self.d.time
        bad0 = self.d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number
        while self.d.time - t0 < seconds:
            if self.d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number > bad0:
                return False
            self.d.qfrc_applied[:] = 0
            for j, f in (("cradle_z", axial), ("cradle_x", drawer[0]), ("cradle_y", drawer[1]),
                         ("cradle_roll", roll), ("cradle_yaw", yaw)):
                self.d.qfrc_applied[dof[j]] = f
            mujoco.mj_step(self.m, self.d)
            if not np.all(np.isfinite(self.d.qpos)):
                return False
        return True

    def angle(self, axis: str) -> float:
        """Cradle rotation (deg): 'roll' + = inversion, 'yaw' + = internal rotation."""
        return math.degrees(float(self.d.qpos[self.m.joint(self.AXES[axis]).qposadr[0]]))

    def platform(self) -> np.ndarray:
        """Platform translation (m): (anterior, medial)."""
        return np.array([self.d.qpos[self.m.joint(j).qposadr[0]] for j in ("cradle_x", "cradle_y")])

    def ligament_forces(self) -> dict[str, float]:
        return {n: ligament_force(self.m, self.d, n) for n in LIGAMENTS if ligament_tendon_ids(self.m, n)}
