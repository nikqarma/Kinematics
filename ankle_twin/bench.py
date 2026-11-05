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
LIGAMENTS = ("ATFL_r", "CFL_r", "PTFL_r", "TNL_r", "TSL_r", "TCL_r", "dATTL_r", "dPTTL_r")

# Points on the calcaneus/foot body (calcn_r frame: x anterior, y superior, z lateral).
HEEL = np.array([0.0, 0.015, 0.0])        # posterior calcaneus, for drawer tests
SOLE_UNDER_ANKLE = np.array([0.049, -0.009, -0.008])  # plantar point under the talus


def ligament_tendon_ids(m: mujoco.MjModel, name: str) -> list[int]:
    """Tendon ids making up one ligament: the base spring plus any '<name>__sN' segments."""
    return [i for i in range(m.ntendon)
            if m.tendon(i).name == name or m.tendon(i).name.startswith(name + "__s")]


def ligament_force(m: mujoco.MjModel, d: mujoco.MjData, name: str) -> float:
    """Total elastic tension (N) in a ligament, summed over its spring segments."""
    ids = ligament_tendon_ids(m, name)
    return float(np.sum(m.tendon_stiffness[ids] * np.maximum(0.0, d.ten_length[ids] - m.tendon_lengthspring[ids, 1])))


class Bench:
    def __init__(self, xml: str | Path, gravity: bool = False):
        spec = mujoco.MjSpec.from_file(str(xml))
        for j in HELD:
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
        self._hold_ids = {j: self.m.equality(f"hold_{j}").id for j in HELD}
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
        """Hold the ankle and/or subtalar angle (None releases that joint)."""
        for j, val in zip(HELD, (ankle_deg, subtalar_deg)):
            i = self._hold_ids[j]
            self.d.eq_active[i] = val is not None
            if val is not None:
                self.m.eq_data[i, :5] = [math.radians(val), 0, 0, 0, 0]

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
        while self.d.time - t0 < seconds:
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
