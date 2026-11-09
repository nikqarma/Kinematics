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

* talar trochlea: a body of revolution about the ankle axis, wider
  anteriorly (its lateral end inclined by the measured anterior-posterior
  width difference);
* tibial plafond: flat plates tangent to a coaxial arc (radius from the
  tibia mesh);
* medial malleolus: a plate against a small sphere on the talus near the
  top-medial corner (the convex medial facet), so the talus pivots about
  the medial malleolus and cannot slide medially;
* lateral malleolus: a plate inclined to match the trochlea, carried with
  the lateral-ligament origins on a sprung distal-fibula body (lateral
  translation and external rotation).

At zero translation/tilt/rotation the geometry is congruent, so the joint
reproduces the original hinge; under load the talus can slide, roll out of
congruence, tilt and rotate, and the fibula can give way laterally.
The anatomical features can be switched off (medial_pivot_radius = 0,
trochlear_width_difference = 0, fibula_mobile = False) to recover the
earlier idealised cylinder-and-plates mortise.

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
    # Medial: TUNED, not anatomical (the anatomical facet covers the upper third
    # to half of the talar side, i.e. down to about 0.3 R to -0.1 R here).
    # With stiff malleolar plates a full facet jammed a tilting cylinder
    # (~1 deg tilt whatever was cut), so it was cut back to a thin
    # superior rim (0.85-1.0 R). That rim leaves the talus free to slide
    # medially (up to ~16 mm) and tip under body-weight load: with 700 N axial
    # load, 3 N*m inversion and the lateral ligaments cut it dislocates to
    # 45 deg on the Inman axis, where Stormont 1985 found the loaded joint
    # surfaces alone give full inversion stability (KNOWN in verify).
    # Tried 2026-11-08, none kept (README, "Loaded stability"):
    # - facet down to 0.75 R or lower: loaded-stable, but axial rotation locks
    #   again (2.5 deg intact, unchanged by cutting the ATFL; Rasmussen 7 -> 18);
    # - shorter facets front to back (medial_facet_ap_half / lateral_facet_ap_half):
    #   rotation frees but is no longer ligament-limited;
    # - limiting the medial-lateral slide to +-2-4 mm: loaded-stable and the
    #   trapdoor ATFL load drops, but tilt after cutting the CFL and PTFL stops
    #   at 8-14 deg (Rasmussen 19 and 46).
    # The idealised cylinder-and-plates mortise cannot give both unloaded laxity
    # and loaded stability; that needs anatomical surfaces (pivot about the
    # medial malleolus, trochlear wedge, fibular mobility).
    "medial_facet_span": ((0.85, 1.0), "R", "tuned: avoids geometric jamming in tilt (see note)"),
    "lateral_facet_span": ((-1.2, 1.0), "R", "assumed: lateral malleolus covers near full height"),
    # Anterior-posterior half-length of each facet plate, in R.
    "medial_facet_ap_half": (1.1, "R", "assumed: covers the trochlea's full length"),
    "lateral_facet_ap_half": (1.1, "R", "assumed: covers the trochlea's full length"),
    # Malleolar facet compliance (MuJoCo contact solref time constant, s).
    # TUNED. With the stiff default (0.004 s) the trochlear cylinder jams between
    # the flat malleolar plates in axial rotation (~85 N on each at 1.5 N*m), so
    # internal rotation is locked at ~2 deg whatever ligaments are cut. Real
    # mortises allow ~7 deg (Rasmussen 1982) through fibular mobility, cartilage
    # and a talus wider anteriorly than posteriorly. Softening both plates frees
    # rotation abruptly between 0.03 and 0.04 s, after which it is ligament-limited
    # and nearly insensitive to the value (6.3-6.6 deg intact over 0.04-0.1 s).
    # 0.08 s is twice the threshold; it is not fitted to Rasmussen's numbers.
    "malleolar_solref_timeconst": (0.08, "s", "tuned: 2x the value that unlocks axial rotation (see note)"),
    # ---- anatomical mortise features (0 / False = off: idealised mortise) ----
    # Medial pivot: the medial talar facet is convex (comma-shaped) and rides on
    # the flat medial malleolar facet. Modelled as a sphere on the talus at the
    # trochlea's medial end, against a medial plate covering the anatomical span.
    # A sphere on a plane allows tilt and axial rotation about its centre but no
    # medial slide, so the talus pivots about the medial malleolus.
    "medial_pivot_radius": (0.006, "m", "assumed: radius of the convex medial facet's contact"),
    # Height of the pivot sphere's centre above the ankle axis, in R. Clinical
    # talar tilt opens laterally about the top-medial corner of the mortise.
    "medial_pivot_height": (0.7, "R", "assumed: near the top-medial corner; at 0 R (mid-height) tilt and rotation "
                                     "lock at ~3 deg whatever is cut"),
    "medial_pivot_facet_span": ((-0.3, 1.0), "R", "anatomical: upper third to half of the talar side"),
    # Trochlear taper: the trochlea is wider anteriorly than posteriorly. The
    # width difference is spread over the plafond arc's chord and applied on the
    # lateral side only (lateral facet and lateral malleolus inclined to match at
    # neutral): ~9 deg on the Inman build. Wedging in dorsiflexion pushes the
    # mobile fibula laterally; the narrow posterior part leaves room in PF.
    "trochlear_width_difference": (0.0047, "m", "measured: CT talar-dome morphometry, J Orthop Surg Res 2017, "
                                               "anterior 29.9 vs posterior 25.2 mm (search summary)"),
    # Mobile fibula: the lateral malleolus (facet plate and lateral-ligament
    # origins) on its own body, sprung to the tibia: lateral translation along
    # the ankle axis and external rotation about the vertical.
    "fibula_mobile": (True, "-", "on"),
    "fibula_mass": (0.25, "kg", "assumed: distal part of a ~0.6 kg fibula"),
    # Stiffnesses are assumed, bounded by weight-bearing fibular travel of about
    # 1.7 mm side to side and 9 deg of rotation over activities (in vivo
    # stereoradiography, search summary). Syndesmosis stiffness data still wanted.
    "fibula_ml_stiffness": (1.0e5, "N/m", "assumed"),
    "fibula_ml_damping": (500.0, "N*s/m", "assumed"),
    "fibula_er_stiffness": (20.0, "N*m/rad", "assumed"),
    "fibula_er_damping": (0.5, "N*m*s/rad", "assumed"),
    # With the anatomical features the malleolar plates no longer need the tuned
    # compliance above: the sphere pivot and the mobile fibula free axial
    # rotation (ligament-limited: 9.4 -> 20.7 deg with the ATFL cut), and stiff
    # plates keep the talus seated at 350 N as well as 700 N. Both use the
    # default cartilage contact; None falls back to malleolar_solref_timeconst
    # (the idealised mortise).
    "lateral_solref_timeconst": (0.004, "s", "default contact (see note)"),
    "medial_solref_timeconst": (0.004, "s", "default contact (see note)"),
}

LATERAL_LIGAMENTS = ("ATFL_r", "ATFLi_r", "CFL_r", "PTFL_r")  # fibular origins (site <name>-P1)

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

    r_piv = _p("medial_pivot_radius")
    taper = _p("trochlear_width_difference") / (2 * R * math.sin(math.radians(_p("plafond_arc_deg"))))
    sol = {s: _p(f"{s}_solref_timeconst") or _p("malleolar_solref_timeconst") for s in ("lateral", "medial")}

    # Talar trochlea: cylinder on the ankle axis, or (with a taper) a convex mesh
    # whose lateral end is inclined so the trochlea is wider anteriorly.
    first_body = next((i for i, c in enumerate(talus) if c.tag == "body"), len(talus))
    if taper == 0.0:
        talus.insert(first_body, ET.Element("geom", {
            "name": "talar_trochlea_r", "class": "cartilage", "type": "cylinder",
            "size": f"{R - gap:.6g}", "fromto": _fmt(np.r_[axis_pt - hw * a, axis_pt + hw * a]),
            "contype": "2", "conaffinity": "0"}))
    else:
        verts = []
        n_seg = 64
        r_vert = (R - gap) / math.cos(math.pi / n_seg)   # circumscribed: the polygon's faces touch R - gap
        for k in range(n_seg):
            phi = 2 * math.pi * k / n_seg
            rad = r_vert * (math.cos(phi) * e2 + math.sin(phi) * e1)
            x_ant = (R - gap) * math.sin(phi)
            verts += [axis_pt + rad - hw * a, axis_pt + rad + (hw + taper * x_ant) * a]
        asset = model.find("asset")
        ET.SubElement(asset, "mesh", {"name": "talar_trochlea_mesh_r",
                                      "vertex": " ".join(_fmt(v) for v in verts)})
        talus.insert(first_body, ET.Element("geom", {
            "name": "talar_trochlea_r", "class": "cartilage", "type": "mesh", "mesh": "talar_trochlea_mesh_r",
            "contype": "2", "conaffinity": "0"}))
    if r_piv > 0:
        # Convex medial facet: collides only with the medial malleolus (bit 8).
        talus.insert(first_body, ET.Element("geom", {
            "name": "talar_medial_facet_r", "class": "cartilage", "type": "sphere", "size": f"{r_piv:.6g}",
            "pos": _fmt(axis_pt - (hw - r_piv) * a + _p("medial_pivot_height") * R * e2),
            "contype": "8", "conaffinity": "0"}))

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
    lateral_plate = None
    for side, sign in (("lateral", +1), ("medial", -1)):
        lo, hi = _p("medial_pivot_facet_span") if (side == "medial" and r_piv > 0) else _p(f"{side}_facet_span")
        x_axis = e1 + (taper * a if side == "lateral" else 0.0)
        x_axis = x_axis / np.linalg.norm(x_axis)
        g = ET.Element("geom", {
            "name": f"malleolus_{side}_r", "class": "cartilage", "type": "box",
            "size": _fmt([R * _p(f"{side}_facet_ap_half"), R * (hi - lo) / 2, plate_t]),
            "pos": _fmt(centre + sign * (hw + gap + plate_t) * a + R * (hi + lo) / 2 * e2),
            "xyaxes": _fmt(np.r_[x_axis, e2]), "contype": "0",
            "conaffinity": "8" if (side == "medial" and r_piv > 0) else "2",
            "solref": f"{sol[side]:g} 1"})
        if side == "lateral":
            lateral_plate = g
        geoms.append(g)
    for k, g in enumerate(geoms):
        tibia.insert(first_body + k, g)

    # Mobile fibula: move the lateral facet plate and the lateral-ligament
    # origins onto a sprung body at the lateral malleolus.
    if _p("fibula_mobile"):
        f_origin = centre + (hw + gap) * a
        fib = ET.Element("body", {"name": "fibula_distal_r", "pos": _fmt(f_origin)})
        # Inertia: the distal fibula (~0.25 kg) as the spring's moving mass. With 50 g the 100 N/mm spring's
        # period (~4 ms) approached the 2 ms time step and the simulation diverged.
        ET.SubElement(fib, "inertial", pos="0 0 0", mass=f"{_p('fibula_mass'):g}", diaginertia="2e-4 2e-4 2e-4")
        ET.SubElement(fib, "joint", {
            "name": "fibula_ml_r", "type": "slide", "pos": "0 0 0", "axis": _fmt(a), "range": "-0.004 0.004",
            "stiffness": f"{_p('fibula_ml_stiffness'):g}", "damping": f"{_p('fibula_ml_damping'):g}",
            "armature": "0.05"})
        ET.SubElement(fib, "joint", {
            "name": "fibula_er_r", "type": "hinge", "pos": "0 0 0", "axis": "0 1 0", "range": "-0.2 0.2",
            "stiffness": f"{_p('fibula_er_stiffness'):g}", "damping": f"{_p('fibula_er_damping'):g}",
            "armature": "0.0001"})
        tibia.remove(lateral_plate)
        p0 = np.array([float(c) for c in lateral_plate.get("pos").split()])
        lateral_plate.set("pos", _fmt(p0 - f_origin))
        fib.append(lateral_plate)
        for site in list(tibia.findall("site")):
            if site.get("name") in {f"{n}-P1" for n in LATERAL_LIGAMENTS}:
                tibia.remove(site)
                ps = np.array([float(c) for c in site.get("pos").split()])
                site.set("pos", _fmt(ps - f_origin))
                fib.insert(0, site)
        tibia.append(fib)

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
