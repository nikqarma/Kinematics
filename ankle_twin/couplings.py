"""Kinematic helpers for joints coupled by polynomial joint-equality constraints.

MuJoCo enforces <equality joint1=.. joint2=.. polycoef=..> only during
simulation. Code that poses the model kinematically (setting qpos and calling
mj_kinematics / mj_forward) must fill the coupled joints itself, otherwise the
knee's coupled translations and the moving ankle axis are left at zero.
"""

from __future__ import annotations

import mujoco
import numpy as np


def _joint_couplings(m: mujoco.MjModel):
    for i in range(m.neq):
        if m.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT and m.eq_obj2id[i] >= 0 and m.eq_active0[i]:
            yield i, int(m.eq_obj1id[i]), int(m.eq_obj2id[i]), m.eq_data[i, :5]


def apply_couplings(m: mujoco.MjModel, d: mujoco.MjData) -> None:
    """Set every coupled joint from its driving joint: y - y0 = sum_p c_p (x - x0)^p."""
    for _, j1, j2, c in _joint_couplings(m):
        a1, a2 = m.jnt_qposadr[j1], m.jnt_qposadr[j2]
        x = d.qpos[a2] - m.qpos0[a2]
        d.qpos[a1] = m.qpos0[a1] + sum(c[p] * x**p for p in range(5))


def pose(m: mujoco.MjModel, d: mujoco.MjData, joints: dict[str, float], forward: bool = True) -> None:
    """Reset to qpos0, set named joints, fill coupled joints, and run kinematics (or mj_forward)."""
    mujoco.mj_resetData(m, d)
    for name, val in joints.items():
        d.qpos[m.joint(name).qposadr[0]] = val
    apply_couplings(m, d)
    if forward:
        mujoco.mj_forward(m, d)
    else:
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        mujoco.mj_tendon(m, d)


def effective_generalized_force(m: mujoco.MjModel, d: mujoco.MjData, vec: np.ndarray, joint: str) -> float:
    """Generalized force conjugate to `joint` once its coupled joints follow it:
    vec[joint] + sum over coupled joints of vec[coupled] * d(coupled)/d(joint)."""
    j = m.joint(joint).id
    total = float(vec[m.jnt_dofadr[j]])
    for _, j1, j2, c in _joint_couplings(m):
        if j2 == j:
            x = d.qpos[m.jnt_qposadr[j2]] - m.qpos0[m.jnt_qposadr[j2]]
            slope = sum(p * c[p] * x ** (p - 1) for p in range(1, 5))
            total += float(vec[m.jnt_dofadr[j1]]) * slope
    return total
