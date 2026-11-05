"""Talocrural (ankle) axis options for the ankle-foot model.

``original``: the MS-Human-700 / OpenSim gait-model hinge axis, through the
talus origin with direction (-0.105, -0.174, 0.979) in the talus frame.

``inman``: re-registered to the bone meshes' own malleoli using Inman's
average offsets of the empirical axis from the malleolar tips (Inman 1976,
as cited in a functional-anatomy text): 5 mm distal to the tip of the
medial malleolus, and 3 mm distal and 8 mm anterior to the tip of the
lateral malleolus. The axis is the line through those two points.

``moving``: the Inman axis at neutral, with a coronal-plane inclination that
changes continuously with flexion, after Lundberg et al. (1989): the axis
"changes continuously throughout the range of movement" and in dorsiflexion
"tended to be oblique downward and laterally" (relative to plantarflexion,
7 of 8 subjects). Implemented as a second hinge, about the anterior axis
through the same centre (inferior direction), coupled to the flexion angle by a quadratic
polynomial equality (the mechanism MS-Human-700 already uses for its knee):
the instantaneous axis then tilts linearly with flexion, by SWING_DEG in
total across the range of motion. (The second hinge points inferiorly,
perpendicular to the axis: two hinges compose by adding angular velocities.) The swing magnitude is NOT from Lundberg
(only the abstract was available): see SWING_DEG.

At neutral (all joint angles zero) all options give exactly the same pose;
only the axis that the talus rotates about differs.

Frames follow OpenSim/MS-Human-700: x anterior, y superior, z lateral.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

import ligaments as LIG

ORIGINAL_AXIS = np.array([-0.105014, -0.174022, 0.979126])  # talus frame, through the talus origin

REFS = {
    "Inman1976": "Inman V.T. (1976) The Joints of the Ankle. Williams & Wilkins. Axis offsets from the "
                 "malleolar tips as cited in a functional-anatomy text (secondary)",
    "Lundberg1989": "Lundberg A. et al. (1989) The axis of rotation of the ankle joint. J Bone Joint Surg "
                    "Br 71(1):94-99: axes lay close to the midpoint of the line between the malleolar tips",
}

# (anterior, superior) offsets of the axis from each malleolar tip, metres.
INMAN_OFFSETS = {"medial": (0.0, -0.005), "lateral": (0.008, -0.003)}

# Total change in coronal-plane inclination of the instantaneous axis from
# full plantarflexion to full dorsiflexion (lateral end lower in dorsiflexion).
# ILLUSTRATIVE, not measured for this purpose: 14.4 deg is the difference
# between the morphology-predicted dorsiflexion and plantarflexion axes for
# one participant (+1.2 vs -13.2 deg) in Wolf et al. 2022 Sci Rep 12:13957,
# Fig. 5. Lundberg 1989 gives the direction but its magnitudes were not
# available. Note Wolf et al. found the OPPOSITE direction during walking in
# 4 of 5 participants; Lundberg's pattern is quasi-static.
SWING_DEG = 14.4
REFS["Wolf2022"] = ("Wolf P. et al. (2022) Human ankle joint movements during walking are probably not "
                    "determined by talar morphology. Sci Rep 12:13957 (PMC9381502)")

KINDS = ("moving", "inman", "original")


def moving_axis_coupling(u: np.ndarray, flex_range: tuple[float, float],
                         swing_deg: float = SWING_DEG) -> tuple[np.ndarray, float]:
    """Second-hinge axis (talus frame) and quadratic coupling coefficient for the moving axis.

    With the flexion hinge (angle q, axis u) followed by a hinge about m (angle
    beta = c * q**2), the angular velocity is q' * (u + 2*c*q*m): the
    instantaneous axis is the vector sum, tilted from u toward m by
    ~2*c*q rad. Taking m as the INFERIOR direction perpendicular to u lowers
    the axis's lateral end (coronal-plane inclination) as q grows. c is chosen
    so the total change over flex_range equals swing_deg.
    """
    y = np.array([0.0, 1.0, 0.0])
    m = -(y - (y @ u) * u)                  # inferior direction, perpendicular to the axis
    m /= np.linalg.norm(m)
    k = np.radians(swing_deg) / (flex_range[1] - flex_range[0])
    # u points laterally, so adding a positive multiple of m (inferior) lowers
    # the lateral end: the axis is more lateral-end-down in dorsiflexion (q > 0)
    # and less so in plantarflexion (q < 0), as Lundberg describes.
    return m, k / 2.0


def malleolar_tips(mesh_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    """(medial tip, lateral tip) in the tibia frame: the most distal vertex of the medial
    malleolus (medial to the talus's medial wall) and of the fibula."""
    tib = LIG.load_stl(mesh_dir / "r_tibia.stl")
    fib = LIG.load_stl(mesh_dir / "r_fibula.stl")
    tal = LIG.load_stl(mesh_dir / "r_talus.stl")
    wall = tal[:, 2].min() + LIG.BODY_ORIGIN_IN_TIBIA["talus_r"][2]
    med = tib[tib[:, 2] < wall]
    return med[med[:, 1].argmin()], fib[fib[:, 1].argmin()]


def axis(kind: str, mesh_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    """(point on the axis, unit direction), both in the talus frame at neutral."""
    if kind == "original":
        return np.zeros(3), ORIGINAL_AXIS / np.linalg.norm(ORIGINAL_AXIS)
    if kind not in ("inman", "moving"):   # 'moving' is the Inman axis at neutral
        raise ValueError(f"unknown axis {kind!r}")
    m_tip, l_tip = malleolar_tips(mesh_dir)
    (ma, ms), (la, ls) = INMAN_OFFSETS["medial"], INMAN_OFFSETS["lateral"]
    M = m_tip + np.array([ma, ms, 0.0])
    L = l_tip + np.array([la, ls, 0.0])
    u = L - M
    u /= np.linalg.norm(u)
    centre = 0.5 * (M + L) - LIG.BODY_ORIGIN_IN_TIBIA["talus_r"]
    return centre, u


def describe(point: np.ndarray, u: np.ndarray, mesh_dir: Path) -> dict[str, float]:
    """Where an axis passes relative to the malleolar tips (mm) and its orientation (deg)."""
    m_tip, l_tip = malleolar_tips(mesh_dir)
    p0 = point + LIG.BODY_ORIGIN_IN_TIBIA["talus_r"]
    out = {}
    for side, tip in (("medial", m_tip), ("lateral", l_tip)):
        q = p0 + (tip[2] - p0[2]) / u[2] * u
        out[f"{side}_anterior_mm"] = (q[0] - tip[0]) * 1000
        out[f"{side}_superior_mm"] = (q[1] - tip[1]) * 1000
    out["frontal_inclination_deg"] = float(np.degrees(np.arctan2(-u[1], u[2])))
    out["external_rotation_deg"] = float(np.degrees(np.arctan2(-u[0], u[2])))
    return out
