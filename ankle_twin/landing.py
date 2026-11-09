"""Trapdoor inversion rig: the leg stands on a platform that is released and tips the lateral border down.

The bench rig (bench.py) loads a fixed shank with torques. This rig loads the foot the way a sudden-inversion
(trapdoor) test does: body weight through the leg, foot-platform contact, and a platform that drops when
released.

    rig = Landing("model_6dof/ankle_foot_r.xml", platform_deg=24)
    rig.stand()                 # body weight settles onto the latched platform
    log = rig.drop(0.3)         # release the trapdoor and record 300 ms
    log.summary()

Simplifications (see README, "Trapdoor rig"):
- The femur slides vertically only; the knee stays locked straight; the shank cannot lean.
- Ankle flexion is held at a set angle (muscle co-contraction stand-in); every other joint is free.
- Muscles are passive (no activation, no reflex) unless a controller is added.
- Foot-platform contact is three spheres (heel, 1st and 5th metatarsal heads) with a linear spring-damper that
  stands in for heel pad and shoe sole. Values are assumptions, not measured.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

import ligaments as LIG
from bench import LIGAMENTS, ligament_force, ligament_tendon_ids

# Plantar contact points in the calcn_r frame (x anterior, y up, z lateral), from the lowest r_foot.stl
# vertices: medial and lateral heel, 1st metatarsal head (medial), 5th metatarsal head (lateral). The coarse
# mesh puts the lowest lateral heel vertex 8 mm higher than the medial one; both heel points are set at the
# medial vertex's plantar level so the heel sits flat.
SOLE_POINTS = {"heel_med": (0.0148, -0.0098, -0.0170), "heel_lat": (0.0168, -0.0098, 0.0106),
               "mt1": (0.1725, -0.0050, -0.0267), "mt5": (0.162, -0.0055, 0.016)}
SPHERE_R = 0.010          # m
CONTACT_K = 2.0e5         # N/m per contact point (assumed: heel pad + shoe sole, order of 100-300 N/mm)
CONTACT_B = 1.0e3         # N*s/m per contact point (assumed)
FRICTION = 0.9            # shoe on platform (assumed)
FOOT_BIT = 4              # contact bit used only by the sole spheres and the platform
HIP_DAMPING = 5.0         # N*m*s/rad, hip ab/adduction (assumed)


@dataclass
class Log:
    t: list = field(default_factory=list)
    platform: list = field(default_factory=list)      # deg
    inversion: list = field(default_factory=list)     # hindfoot (calcaneus vs tibia) inversion, deg
    subtalar: list = field(default_factory=list)      # deg
    tilt: list = field(default_factory=list)          # talar tilt, deg
    rot: list = field(default_factory=list)           # talar internal rotation, deg
    grf: list = field(default_factory=list)           # total normal force on the sole, N
    force: dict = field(default_factory=dict)         # ligament elastic force, N
    damping: dict = field(default_factory=dict)       # ligament damping force (+ = resisting lengthening), N
    strain: dict = field(default_factory=dict)        # ligament strain
    strain_rate: dict = field(default_factory=dict)   # 1/s

    def arrays(self):
        return {k: np.asarray(v) for k, v in self.__dict__.items() if isinstance(v, list)}

    def summary(self) -> str:
        a = self.arrays()
        t, inv = a["t"], a["inversion"]
        vel = np.gradient(inv, t)
        i_pk = int(np.argmax(inv))
        lines = [f"peak hindfoot inversion {inv[i_pk]:.1f} deg at {t[i_pk] * 1e3:.0f} ms; "
                 f"peak inversion velocity {vel.max():.0f} deg/s at {t[int(np.argmax(vel))] * 1e3:.0f} ms",
                 f"platform {a['platform'][-1]:.1f} deg at end; peak subtalar {a['subtalar'].max():.1f}, "
                 f"talar tilt {a['tilt'].max():.1f}, talar rotation {a['rot'].max():.1f} deg; peak GRF {a['grf'].max():.0f} N",
                 f"{'ligament':<8}{'peak N':>8}{'t ms':>7}{'fail N':>8}{'peak strain %':>15}{'peak rate %/s':>15}"
                 f"{'damping N at peak':>19}"]
        fail = {l.name: l.failure_load for l in LIG.LIGAMENTS}
        for n, f in self.force.items():
            f = np.asarray(f)
            i = int(np.argmax(f))
            if f[i] < 1:
                continue
            lines.append(f"{n[:-2]:<8}{f[i]:8.0f}{t[i] * 1e3:7.0f}{(fail[n] or float('nan')):8.0f}"
                         f"{max(self.strain[n]) * 100:15.1f}{max(self.strain_rate[n]) * 100:15.0f}"
                         f"{self.damping[n][i]:19.1f}")
        return "\n".join(lines)


class Landing:
    def __init__(self, xml: str | Path, platform_deg: float = 24.0, supported_mass: float = 37.5,
                 ankle_deg: float = 0.0, contact_k: float = CONTACT_K, contact_b: float = CONTACT_B):
        """supported_mass: body mass carried by this leg (kg); the default is half of a 75 kg subject,
        as when standing on both feet before one trapdoor drops."""
        self.xml = Path(xml)
        self.ankle_deg = ankle_deg
        spec = self._spec(contact_k, contact_b, supported_mass, platform_deg, platform_top=None)
        m = spec.compile()
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        sole = [d.geom_xpos[m.geom(f"sole_{n}").id][2] - SPHERE_R for n in SOLE_POINTS]
        medial = max(d.geom_xpos[m.geom(f"sole_{n}").id][1] for n in SOLE_POINTS)
        # Second pass with the platform just under the sole, hinged on the foot's medial border.
        spec = self._spec(contact_k, contact_b, supported_mass, platform_deg,
                          platform_top=(min(sole) - 0.0005, medial + SPHERE_R + 0.005))
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        self.m.opt.gravity[:] = [0, 0, -9.81]
        self.lig = [n for n in LIGAMENTS if self._has(n)]
        self.base_ids = {n: self.m.tendon(n).id for n in self.lig}
        self.seg_ids = {n: ligament_tendon_ids(self.m, n) for n in self.lig}
        self.L0 = {n: self.m.tendon_lengthspring[self.base_ids[n], 1] for n in self.lig}
        self.sole_ids = [self.m.geom(f"sole_{n}").id for n in SOLE_POINTS]
        self._inv_sign = self._inversion_sign()
        mujoco.mj_resetData(self.m, self.d)

    # --- model construction -----------------------------------------------------------------
    def _spec(self, k, b, supported_mass, platform_deg, platform_top):
        spec = mujoco.MjSpec.from_file(str(self.xml))
        mount = spec.body("rig_mount")
        slide = mount.add_joint()
        slide.name, slide.type, slide.axis = "rig_slide", mujoco.mjtJoint.mjJNT_SLIDE, [0, 1, 0]  # local y = world z
        slide.damping = [0.0, 0.0, 0.0]
        slide.range = [-0.5, 0.5]  # the model's joint defaults are limited
        # Hip ab/adduction: the straight leg can swing in the frontal plane about the hip centre (femur
        # origin), so the foot can roll over its medial border. The supported mass sits at the pivot.
        hip = mount.add_joint()
        hip.name, hip.type, hip.axis = "rig_hip_adduction", mujoco.mjtJoint.mjJNT_HINGE, [1, 0, 0]
        hip.pos, hip.range = [0, 0, 0], [-0.35, 0.35]
        hip.damping = [HIP_DAMPING, 0.0, 0.0]
        # Supported body mass: added at the hip end of the femur.
        m0 = spec.compile()
        leg_mass = float(m0.body_subtreemass[m0.body("rig_mount").id])
        load = spec.body("femur_r").add_body()
        load.name = "body_load"
        load.mass = max(supported_mass - leg_mass, 0.0)
        load.inertia = [0.01, 0.01, 0.01]
        load.explicitinertial = True
        calcn = spec.body("calcn_r")
        for n, p in SOLE_POINTS.items():
            g = calcn.add_geom()
            g.name, g.type, g.size = f"sole_{n}", mujoco.mjtGeom.mjGEOM_SPHERE, [SPHERE_R, 0, 0]
            g.pos = [p[0], p[1] + SPHERE_R, p[2]]
            g.contype, g.conaffinity, g.condim = FOOT_BIT, 0, 3
            g.friction = [FRICTION, 0.005, 0.0001]
            g.solref = [-k, -b]
            g.mass = 0.0
            g.rgba = [0.9, 0.3, 0.3, 1]
        for eq_joint in ("ankle_angle_r",):
            eq = spec.add_equality()
            eq.name, eq.type, eq.objtype, eq.name1 = f"hold_{eq_joint}", mujoco.mjtEq.mjEQ_JOINT, \
                mujoco.mjtObj.mjOBJ_JOINT, eq_joint
            eq.data[0] = math.radians(self.ankle_deg)
            eq.solref = [0.004, 1.0]
        if platform_top is not None:
            z_top, y_hinge = platform_top
            plat = spec.worldbody.add_body()
            plat.name, plat.pos = "platform", [0.035, y_hinge, z_top]
            plat.mass, plat.inertia, plat.explicitinertial = 0.5, [0.003, 0.003, 0.003], True
            h = plat.add_joint()
            h.name, h.type, h.axis = "platform_hinge", mujoco.mjtJoint.mjJNT_HINGE, [1, 0, 0]
            h.limited, h.range = True, [0.0, math.radians(platform_deg)]
            h.damping = [0.0, 0.0, 0.0]
            g = plat.add_geom()
            g.name, g.type, g.size = "platform_top", mujoco.mjtGeom.mjGEOM_BOX, [0.13, 0.07, 0.01]
            g.pos = [0, -0.07, -0.01]  # extends laterally (-y) from the medial hinge; top surface at hinge height
            g.contype, g.conaffinity, g.condim = 0, FOOT_BIT, 3
            g.friction = [FRICTION, 0.005, 0.0001]
            g.rgba = [0.6, 0.6, 0.65, 1]
            latch = spec.add_equality()
            latch.name, latch.type, latch.objtype, latch.name1 = "latch", mujoco.mjtEq.mjEQ_JOINT, \
                mujoco.mjtObj.mjOBJ_JOINT, "platform_hinge"
            latch.solref = [0.004, 1.0]
        return spec

    def _has(self, name: str) -> bool:
        try:
            self.m.tendon(name)
            return True
        except KeyError:
            return False

    # --- measures -------------------------------------------------------------------------
    def _rel(self) -> np.ndarray:
        t, c = self.m.body("tibia_r").id, self.m.body("calcn_r").id
        return self.d.xmat[t].reshape(3, 3).T @ self.d.xmat[c].reshape(3, 3)

    @staticmethod
    def _rotvec(R: np.ndarray) -> np.ndarray:
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, R.flatten())
        ang = 2 * math.atan2(np.linalg.norm(q[1:]), q[0])
        n = np.linalg.norm(q[1:])
        return np.zeros(3) if n < 1e-12 else q[1:] / n * ang

    def _inversion_sign(self) -> float:
        """Sign that makes subtalar inversion read positive about the tibia's AP (x) axis."""
        mujoco.mj_resetData(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)
        R0 = self._rel()
        self.d.qpos[self.m.joint("subtalar_angle_r").qposadr[0]] = 0.1
        mujoco.mj_forward(self.m, self.d)
        return float(np.sign(self._rotvec(self._rel() @ R0.T)[0]))

    def q(self, joint: str) -> float:
        return float(self.d.qpos[self.m.joint(joint).qposadr[0]])

    def grf(self) -> float:
        f6, total = np.zeros(6), 0.0
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if c.geom1 in self.sole_ids or c.geom2 in self.sole_ids:
                mujoco.mj_contactForce(self.m, self.d, i, f6)
                total += f6[0]
        return total

    # --- protocol -------------------------------------------------------------------------
    def stand(self, seconds: float = 1.0) -> bool:
        """Latch the platform, hold ankle flexion, and let the supported mass settle onto the foot."""
        mujoco.mj_resetData(self.m, self.d)
        self.d.eq_active[self.m.equality("latch").id] = True
        self.d.eq_active[self.m.equality("hold_ankle_angle_r").id] = True
        return self._run(seconds)

    def _run(self, seconds: float, log: Log | None = None, R0=None) -> bool:
        t0 = self.d.time
        while self.d.time - t0 < seconds:
            mujoco.mj_step(self.m, self.d)
            if not np.all(np.isfinite(self.d.qpos)):
                return False
            if log is not None:
                self._record(log, self.d.time - t0, R0)
        return True

    def drop(self, seconds: float = 0.3, reflex_ms: float | None = None, reflex_level: float = 1.0,
             reflex_muscles: tuple[str, ...] = ("perlong_r", "perbrev_r")) -> Log:
        """Release the latch and record the trapdoor event.

        reflex_ms: if given, the peroneal muscles are excited (ctrl = reflex_level) from this many ms after
        release, standing in for the stretch reflex (latency 54-66 ms in healthy adults on trapdoors, as
        summarised in sudden-inversion reviews). MuJoCo's activation dynamics add the rise time."""
        R0 = self._rel()
        self.d.eq_active[self.m.equality("latch").id] = False
        log = Log(force={n: [] for n in self.lig}, damping={n: [] for n in self.lig},
                  strain={n: [] for n in self.lig}, strain_rate={n: [] for n in self.lig})
        t0 = self.d.time
        ids = [self.m.actuator(n).id for n in reflex_muscles]
        while self.d.time - t0 < seconds:
            on = reflex_ms is not None and (self.d.time - t0) * 1e3 >= reflex_ms
            self.d.ctrl[ids] = reflex_level if on else 0.0
            mujoco.mj_step(self.m, self.d)
            if not np.all(np.isfinite(self.d.qpos)):
                break
            self._record(log, self.d.time - t0, R0)
        return log

    def _record(self, log: Log, t: float, R0: np.ndarray) -> None:
        log.t.append(t)
        log.platform.append(math.degrees(self.q("platform_hinge")))
        log.inversion.append(math.degrees(self._inv_sign * self._rotvec(self._rel() @ R0.T)[0]))
        log.subtalar.append(math.degrees(self.q("subtalar_angle_r")))
        log.tilt.append(math.degrees(self.q("talus_tilt_r")) if self._joint("talus_tilt_r") else 0.0)
        log.rot.append(math.degrees(self.q("talus_rot_r")) if self._joint("talus_rot_r") else 0.0)
        log.grf.append(self.grf())
        for n in self.lig:
            i = self.base_ids[n]
            log.force[n].append(ligament_force(self.m, self.d, n))
            v = float(self.d.ten_velocity[i])
            log.damping[n].append(float(np.sum(self.m.tendon_damping[self.seg_ids[n]])) * v)
            log.strain[n].append(float(self.d.ten_length[i]) / self.L0[n] - 1)
            log.strain_rate[n].append(v / self.L0[n])

    def _joint(self, name: str) -> bool:
        try:
            self.m.joint(name)
            return True
        except KeyError:
            return False


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("xml", nargs="?", default="model_6dof/ankle_foot_r.xml")
    p.add_argument("--platform-deg", type=float, default=24.0)
    p.add_argument("--mass", type=float, default=37.5, help="body mass carried by this leg, kg")
    p.add_argument("--ankle-deg", type=float, default=0.0, help="held ankle flexion (+ = dorsiflexion)")
    p.add_argument("--seconds", type=float, default=0.3)
    a = p.parse_args()
    rig = Landing(a.xml, a.platform_deg, a.mass, a.ankle_deg)
    if not rig.stand():
        raise SystemExit("unstable while standing")
    print(f"standing: GRF {rig.grf():.0f} N, subtalar {math.degrees(rig.q('subtalar_angle_r')):.1f} deg")
    print(rig.drop(a.seconds).summary())


if __name__ == "__main__":
    main()
