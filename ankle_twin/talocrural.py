"""6-DOF talocrural joint with idealised articular contact.

MS-Human-700 (like its OpenSim parent) models the talocrural joint as a
1-DOF hinge. Here the talus additionally gets three translations, a tilt
(inversion/eversion) and an axial rotation. None of these extra degrees of
freedom has a spring or a stop of its own. They are restrained only by
articular contact, the ankle ligaments and the muscles.

Why idealised contact
---------------------
The MS-Human-700 bone meshes are visualisation meshes, not articular
surfaces. The talus mesh sits 6-10 mm inside the tibial plafond and
overlaps the fibula, and MuJoCo collides meshes as convex hulls, which
would fill the concave mortise. So the joint surfaces are built from
primitives arranged around the model's own ankle axis:

* talar trochlea: a cylinder coaxial with the ankle axis;
* tibial plafond: flat plates tangent to a coaxial arc (radius from the
  tibia mesh);
* medial and lateral malleolar facets: plates perpendicular to the axis.

At zero translation/tilt/rotation the geometry is exactly congruent, so
the joint reproduces the original hinge; under load the talus can slide,
roll out of congruence, tilt and rotate.

Every parameter carries a ``source`` tag, as in ligaments.py.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# The joint is built around whatever axis the ankle hinge has (ankle_axis.py).

PARAMS = {
    # name: (value, unit, source)
    # plafond radius: measured from the tibia mesh about whichever ankle axis
    # the build uses (see plafond_radius()); ~20.7 mm about the original axis.
    "clearance": (0.0002, "m", "assumed: unloaded joint clearance"),
    "trochlea_half_width": (0.0140, "m", "assumed: 28 mm trochlear width (talus mesh too coarse to fit)"),
    "plafond_arc_deg": (45.0, "deg", "mesh: plafond vertices span about +-45 deg about the axis"),
    "plafond_plates": (10, "-", "discretisation of the plafond arc"),
    # Malleolar facet coverage of the trochlea's sides, as fractions of the
    # dome radius measured from the axis (+ = superior).
    # Medial: TUNED, not anatomical. A flat plate covering the anatomical
    # medial facet (upper third to half of the talar side) jams a tilting
    # cylinder, so inversion tilt is then set by clearance rather than by the
    # ligaments (~1 deg regardless of sectioning). A thin superior rim lets the
    # talus pivot about the medial plafond edge, as in a talar tilt test, and
    # leaves medial restraint to the deltoid.
    "medial_facet_span": ((0.85, 1.0), "R", "tuned: avoids geometric jamming in tilt (see note)"),
    "lateral_facet_span": ((-1.2, 1.0), "R", "assumed: lateral malleolus covers near full height"),
}

# Contact: frictionless (cartilage friction is ~0.01 or less), stiff and
# well damped. solref/solimp tuned so a body-weight axial load gives
# sub-millimetre penetration at the default 2 ms time step (see verify).
CONTACT = {
    "condim": "1",
    "solref": "0.004 1",
    "solimp": "0.99 0.999 0.0005 0.5 2",
    "margin": "0",
}

# Viscous damping on the extra talar DOFs, standing in for capsule/synovial
# damping. Not from the literature: with only the light ligament damping, a
# pure torque on the unloaded joint makes the talus overshoot out of the
# mortise and oscillate; these values remove that while leaving quasi-static
# behaviour unchanged.
TALAR_DAMPING = {
    "slide": (200.0, "N*s/m", "assumed: numerical/capsular damping"),
    "hinge": (1.0, "N*m*s/rad", "assumed: numerical/capsular damping"),
}

# Extra talar degrees of freedom: name, type, axis (talus frame), range, + meaning.
# Ranges are wide safety stops only (dislocation guards); a test that ends on
# one of them has dislocated the joint and should be reported as such.
EXTRA_DOFS_BEFORE = [  # translations act before the flexion hinge
    ("talus_tx_r", "slide", "1 0 0", "-0.015 0.015", "+ = anterior"),
    ("talus_ty_r", "slide", "0 1 0", "-0.015 0.015", "+ = superior"),
    ("talus_tz_r", "slide", "0 0 1", "-0.015 0.015", "+ = lateral"),
]
EXTRA_DOFS_AFTER = [  # rotations act after the flexion hinge
    ("talus_tilt_r", "hinge", "1 0 0", "-0.7 0.7", "+ = inversion (talar tilt)"),
    ("talus_rot_r", "hinge", "0 1 0", "-0.7 0.7", "+ = internal rotation"),
]
EXTRA_DOF_NAMES = [d[0] for d in EXTRA_DOFS_BEFORE + EXTRA_DOFS_AFTER]


def _p(name: str) -> float:
    return PARAMS[name][0]


def _fmt(v) -> str:
    return " ".join(f"{c:.6g}" for c in v)


def plafond_radius(mesh_dir: Path, centre: np.ndarray, a: np.ndarray) -> float:
    """Radius of the tibial plafond about an axis (centre in the tibia frame, unit direction a).

    Tibia vertices within +-45 deg of superior, within 15 mm of the centre along
    the axis and 12-35 mm from it are binned in 15 deg sectors; the radius is
    the median of the sector minima (the articular lower envelope).
    """
    import ligaments as LIG
    tib = LIG.load_stl(mesh_dir / "r_tibia.stl") - centre
    e2 = np.array([0.0, 1.0, 0.0]) - a[1] * a
    e2 /= np.linalg.norm(e2)
    e1 = np.cross(e2, a)
    s = tib @ a
    r = tib - np.outer(s, a)
    rad = np.linalg.norm(r, axis=1)
    th = np.degrees(np.arctan2(r @ e1, r @ e2))
    keep = (np.abs(th) < 45) & (np.abs(s) < 0.015) & (rad > 0.012) & (rad < 0.035)
    mins = [rad[keep & (th >= lo) & (th < lo + 15)].min()
            for lo in range(-45, 45, 15) if np.any(keep & (th >= lo) & (th < lo + 15))]
    return float(np.median(mins))


def add_joint(model: ET.Element, femur: ET.Element, mesh_dir: Path) -> None:
    """Convert the ankle hinge into the 6-DOF talocrural joint with contact geometry.

    The joint follows whatever axis the hinge currently has (its pos/axis attributes),
    so it works with any axis option in ankle_axis.py.
    """
    bodies = {b.get("name"): b for b in femur.iter("body")}
    tibia, talus = bodies["tibia_r"], bodies["talus_r"]
    talus_pos = np.array([float(c) for c in talus.get("pos").split()])
    hinge = next(j for j in talus.iter("joint") if j.get("name") == "ankle_angle_r")
    axis_pt = np.array([float(c) for c in hinge.get("pos", "0 0 0").split()])     # talus frame
    axis_dir = np.array([float(c) for c in hinge.get("axis").split()])

    # ---- defaults -----------------------------------------------------------
    default = model.find("default")
    play = ET.SubElement(default, "default", {"class": "talar_play"})
    ET.SubElement(play, "joint", damping=f"{TALAR_DAMPING['slide'][0]:g}", armature="0.001", stiffness="0")
    play_rot = ET.SubElement(default, "default", {"class": "talar_play_rot"})
    ET.SubElement(play_rot, "joint", damping=f"{TALAR_DAMPING['hinge'][0]:g}", armature="0.001", stiffness="0")
    cart = ET.SubElement(default, "default", {"class": "cartilage"})
    ET.SubElement(cart, "geom", rgba="0.55 0.8 0.95 0.35", group="2", mass="0", **CONTACT)

    # ---- degrees of freedom -------------------------------------------------
    children = list(talus)
    hinge_idx = next(i for i, c in enumerate(children) if c.tag == "joint" and c.get("name") == "ankle_angle_r")
    for offset, (name, jtype, axis, rng, meaning) in enumerate(EXTRA_DOFS_BEFORE):
        talus.insert(hinge_idx + offset, ET.Element("joint", {
            "name": name, "class": "talar_play", "type": jtype, "pos": "0 0 0", "axis": axis, "range": rng}))
    hinge_idx += len(EXTRA_DOFS_BEFORE)
    for offset, (name, jtype, axis, rng, meaning) in enumerate(EXTRA_DOFS_AFTER, start=1):
        # tilt and axial rotation pivot about the centre of the trochlea
        talus.insert(hinge_idx + offset, ET.Element("joint", {
            "name": name, "class": "talar_play_rot", "type": jtype, "pos": _fmt(axis_pt),
            "axis": axis, "range": rng}))

    # ---- articular geometry -------------------------------------------------
    a = axis_dir / np.linalg.norm(axis_dir)
    e2 = np.array([0.0, 1.0, 0.0]) - a[1] * a           # superior, perpendicular to the axis
    e2 /= np.linalg.norm(e2)
    e1 = np.cross(e2, a)                                 # anterior, perpendicular to the axis
    centre = talus_pos + axis_pt                         # trochlea centre in the tibia frame
    R = plafond_radius(mesh_dir, centre, a)
    hw = _p("trochlea_half_width")
    gap = _p("clearance")
    plate_t = 0.001                                      # plate half-thickness

    # Talar trochlea: cylinder on the ankle axis.
    first_body = next((i for i, c in enumerate(talus) if c.tag == "body"), len(talus))
    talus.insert(first_body, ET.Element("geom", {
        "name": "talar_trochlea_r", "class": "cartilage", "type": "cylinder",
        "size": f"{R - gap:.6g}", "fromto": _fmt(np.r_[axis_pt - hw * a, axis_pt + hw * a]),
        "contype": "2", "conaffinity": "0"}))

    # Tibial plafond: plates tangent to a coaxial arc; malleolar facets: plates
    # normal to the axis. All in the tibia frame, centred on the axis point.
    first_body = next((i for i, c in enumerate(tibia) if c.tag == "body"), len(tibia))
    geoms = []
    n = int(_p("plafond_plates"))
    arc = math.radians(_p("plafond_arc_deg"))
    step = 2 * arc / (n - 1)
    half_tan = R * math.tan(step / 2) + 0.0003
    for i in range(n):
        th = -arc + i * step                              # 0 = superior, + = anterior
        u = math.cos(th) * e2 + math.sin(th) * e1         # outward normal
        t = -math.sin(th) * e2 + math.cos(th) * e1        # tangent
        geoms.append(ET.Element("geom", {
            "name": f"plafond_{i:02d}_r", "class": "cartilage", "type": "box",
            "size": _fmt([half_tan, hw, plate_t]), "pos": _fmt(centre + (R + plate_t) * u),
            "xyaxes": _fmt(np.r_[t, a]), "contype": "0", "conaffinity": "2"}))
    # Malleolar facets: plates normal to the axis covering part of each side of
    # the trochlea (see *_facet_span).
    for side, sign in (("lateral", +1), ("medial", -1)):
        lo, hi = _p(f"{side}_facet_span")
        geoms.append(ET.Element("geom", {
            "name": f"malleolus_{side}_r", "class": "cartilage", "type": "box",
            "size": _fmt([R * 1.1, R * (hi - lo) / 2, plate_t]),
            "pos": _fmt(centre + sign * (hw + gap + plate_t) * a + R * (hi + lo) / 2 * e2),
            "xyaxes": _fmt(np.r_[e1, e2]), "contype": "0", "conaffinity": "2"}))
    for k, g in enumerate(geoms):
        tibia.insert(first_body + k, g)

    # ---- options and contact pairs -----------------------------------------
    # The talus is a child of the tibia, and MuJoCo filters parent-child
    # contacts by default; switch that off and exclude the pairs that should
    # still not collide.
    option = model.find("option")
    if option is None:
        option = ET.Element("option")
        model.insert(1, option)
    ET.SubElement(option, "flag", filterparent="disable", multiccd="enable")
    contact = model.find("contact")
    ET.SubElement(contact, "exclude", body1="calcn_r", body2="toes_r")
    ET.SubElement(contact, "exclude", body1="talus_r", body2="calcn_r")
