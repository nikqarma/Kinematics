"""6-DOF subtalar (talocalcaneonavicular) joint with idealised articular contact.

MS-Human-700 models the subtalar joint as a 1-DOF hinge. A hinge turns freely
about its own oblique axis and not at all about any other, so it cannot show
what a cadaver rig measures when it rotates the heel about a front-to-back
axis (Pellegrini 2016), and the calcaneofibular ligament, which runs nearly
parallel to the axis, cannot restrain it. Here the calcaneus additionally gets
three translations and two rotations relative to the talus. None has a spring
or a stop of its own: they are restrained by articular contact and ligaments.

Idealisation: two convex surfaces, each seated in a cup
-------------------------------------------------------
The bone meshes are low-poly visualisation meshes that overlap at the joint, so
the surfaces are primitives, as in talocrural.py:

* talar head: a sphere on the talus seated in a cup on the foot body (the
  "acetabulum pedis": navicular, spring ligament, anterior and middle
  calcaneal facets; the model's foot body is rigid from calcaneus to
  metatarsals, so the talonavicular joint is part of this articulation);
* posterior facet: a convex sphere on the calcaneus under a concave cap on the
  talar body (Kleipool 2019 fits a cylinder of radius 21 mm; a sphere of that
  radius is used so the two centres define one axis).

With both seated, the only free motion is rotation about the line through the
two centres, so that line is the subtalar axis. Its direction is taken from
Isman & Inman 1969 and it passes through the talar-head centre; the posterior
centre is the point on it one facet radius from the posterior facet. Any other
rotation lifts one surface out of its cup against the ligaments and the load.

Every parameter carries a ``source`` tag, as in ligaments.py.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

PARAMS = {
    # name: (value, unit, source)
    "axis_inclination_deg": (42.0, "deg", "measured (secondary): Isman & Inman 1969, 42 +/- 9 deg above horizontal"),
    "axis_medial_deviation_deg": (23.0, "deg", "measured (secondary): Isman & Inman 1969, 23 +/- 11 deg medial of "
                                               "the foot midline (taken as the foot body's x axis)"),
    "posterior_facet_radius": (0.0210, "m", "measured: Kleipool 2019, fitted cylinder diameter 42.0 +/- 7.7 mm "
                                            "(n = 40 feet, weight-bearing CT; abstract)"),
    # Centre of the posterior facet on the talus underside (talus frame: x anterior, y superior, z lateral).
    "posterior_facet_point": ((-0.003, -0.010, 0.005), "m", "mesh: underside of the talar body (~100-vertex mesh, "
                                                            "a few mm)"),
    # Talar head: sphere fitted to the talus mesh vertices anterior of this x (see head_sphere()).
    "talar_head_min_x": (0.018, "m", "mesh: talar head and neck vertices"),
    "clearance": (0.0002, "m", "assumed: unloaded joint clearance"),
    # Cup coverage: half-angles of the rings of plates about each cup's central direction.
    "socket_direction": ((1.0, -0.7, -0.3), "-", "assumed: the cup faces the head from anterior (navicular), "
                                                 "inferior (calcaneal facets) and medial (spring ligament)"),
    "socket_rings_deg": ((30.0, 55.0), "deg", "assumed: cup covers about +-70 deg of the head"),
    "facet_rings_deg": ((18.0, 34.0), "deg", "assumed: facet ~30 x 22 mm on a 21 mm radius, about +-40 deg"),
    # The socket has two parts. Plates within this angle of straight plantar are the anterior and middle
    # calcaneal facets: bone, rigid with the calcaneus. The rest (anterior, medial, dorsal, lateral) is the
    # navicular and the spring ligament, which are joined to the calcaneus only through the midtarsal joint
    # and its ligaments (Astion 1997: with the subtalar joint fused the talonavicular joint keeps 26% of its
    # motion and the calcaneocuboid joint 56%).
    "socket_calcaneal_cone_deg": (45.0, "deg", "assumed: extent of the calcaneal facets under the talar head"),
    # How the navicular part of the socket is held to the calcaneus: math.inf = rigid (the model's foot body
    # is one piece; an upper bound on the constraint), 0 = absent (a lower bound), otherwise an isotropic
    # translational stiffness in N/m.
    "navicular_stiffness": (math.inf, "N/m", "bound: rigid; no measured midtarsal stiffness found"),
    "navicular_damping": (200.0, "N*s/m", "numerical"),
    "navicular_mass": (0.3, "kg", "assumed: the midfoot bones that move with the navicular"),
    # A sprung socket is a stiff spring on a light body: explicit integration needs stiffness < 4 m / dt^2
    # (4800 N/mm with these values; only 300 N/mm at MuJoCo's default 2 ms).
    "navicular_timestep": (0.0005, "s", "numerical: used only when the socket is sprung"),
}

CONTACT = {
    "condim": "1",
    "solref": "0.004 1",
    "solimp": "0.99 0.999 0.0005 0.5 2",
    "margin": "0",
}

EXTRA_SLIDES = [
    ("subtalar_tx_r", "1 0 0", "+ = calcaneus anterior"),
    ("subtalar_ty_r", "0 1 0", "+ = calcaneus superior"),
    ("subtalar_tz_r", "0 0 1", "+ = calcaneus lateral"),
]
EXTRA_HINGES = ["subtalar_flex_r", "subtalar_twist_r"]   # about two axes perpendicular to the subtalar axis
EXTRA_DOF_NAMES = [s[0] for s in EXTRA_SLIDES] + EXTRA_HINGES


def _p(name: str):
    return PARAMS[name][0]


def _fmt(v) -> str:
    return " ".join(f"{c:.6g}" for c in v)


def head_sphere(mesh_dir: Path) -> tuple[np.ndarray, float]:
    """Least-squares sphere through the talar head vertices: (centre in the talus frame, radius)."""
    import ligaments as LIG
    tal = np.unique(np.round(LIG.load_stl(mesh_dir / "r_talus.stl"), 6), axis=0)
    P = tal[tal[:, 0] > _p("talar_head_min_x")]
    A = np.c_[2 * P, np.ones(len(P))]
    sol = np.linalg.lstsq(A, (P ** 2).sum(1), rcond=None)[0]
    c = sol[:3]
    return c, float(math.sqrt(sol[3] + c @ c))


def axis_direction() -> np.ndarray:
    """Unit subtalar axis in the talus frame, pointing anterior, superior and medial (+ rotation = inversion)."""
    inc, dev = math.radians(_p("axis_inclination_deg")), math.radians(_p("axis_medial_deviation_deg"))
    return np.array([math.cos(inc) * math.cos(dev), math.sin(inc), -math.cos(inc) * math.sin(dev)])


def geometry(mesh_dir: Path) -> dict:
    """Centres, radii and axis of the idealised joint, in the talus frame."""
    a = axis_direction()
    c_h, r_h = head_sphere(mesh_dir)
    f = np.array(_p("posterior_facet_point"))
    r_p = _p("posterior_facet_radius")
    # Posterior centre: on the axis behind the head, one facet radius from the facet point (the farther root,
    # so the centre lies below the facet, inside the calcaneus).
    w = c_h - f
    disc = (w @ a) ** 2 - (w @ w - r_p * r_p)
    if disc < 0:
        raise ValueError("posterior facet point is more than one facet radius from the subtalar axis")
    t = (w @ a) + math.sqrt(disc)
    c_p = c_h - t * a
    return {"axis": a, "head_centre": c_h, "head_radius": r_h, "posterior_centre": c_p, "posterior_radius": r_p,
            "facet_normal": (f - c_p) / r_p}


def _ring_directions(centre: np.ndarray, rings_deg, counts) -> list[np.ndarray]:
    """Unit vectors: the centre direction plus rings of directions at the given half-angles around it."""
    c = centre / np.linalg.norm(centre)
    u = np.cross(c, [0.0, 0.0, 1.0])
    u /= np.linalg.norm(u)
    v = np.cross(c, u)
    out = [c]
    for ang, n in zip(rings_deg, counts):
        th = math.radians(ang)
        for k in range(n):
            ph = 2 * math.pi * (k + 0.5 * (n % 2)) / n
            out.append(math.cos(th) * c + math.sin(th) * (math.cos(ph) * u + math.sin(ph) * v))
    return out


def _cup(prefix: str, centre: np.ndarray, radius: float, dirs, half: float, conaffinity: str) -> list[ET.Element]:
    """Plates tangent to a sphere (centre, radius) from outside, one per direction: a concave cup."""
    plate_t = 0.001
    geoms = []
    for i, n in enumerate(dirs):
        x = np.cross(n, [0.0, 1.0, 0.0])
        if np.linalg.norm(x) < 1e-6:
            x = np.array([1.0, 0.0, 0.0])
        x /= np.linalg.norm(x)
        y = np.cross(n, x)
        geoms.append(ET.Element("geom", {
            "name": f"{prefix}_{i:02d}_r", "class": "cartilage", "type": "box",
            "size": _fmt([half, half, plate_t]), "pos": _fmt(centre + (radius + plate_t) * n),
            "xyaxes": _fmt(np.r_[x, y]), "contype": "0", "conaffinity": conaffinity}))
    return geoms


def add_joint(model: ET.Element, femur: ET.Element, mesh_dir: Path) -> dict:
    """Turn the subtalar hinge into a 6-DOF joint with contact. Needs talocrural.add_joint() first
    (its default classes and option flags are reused). Returns the joint geometry."""
    bodies = {b.get("name"): b for b in femur.iter("body")}
    talus, calcn = bodies["talus_r"], bodies["calcn_r"]
    calc_off = np.array([float(c) for c in calcn.get("pos").split()])      # calcn origin in the talus frame
    g = geometry(mesh_dir)
    a, c_h, r_h, c_p, r_p = g["axis"], g["head_centre"], g["head_radius"], g["posterior_centre"], g["posterior_radius"]
    gap = _p("clearance")

    # ---- degrees of freedom -------------------------------------------------
    hinge = next(j for j in calcn.iter("joint") if j.get("name") == "subtalar_angle_r")
    hinge.set("pos", _fmt(c_p - calc_off))
    hinge.set("axis", _fmt(a))
    b = np.array([0.0, 0.0, 1.0]) - a[2] * a          # mostly lateral
    b /= np.linalg.norm(b)
    c = np.cross(a, b)
    idx = list(calcn).index(hinge)
    for k, (name, axis, _) in enumerate(EXTRA_SLIDES):
        calcn.insert(idx + k, ET.Element("joint", {
            "name": name, "class": "talar_play", "type": "slide", "pos": "0 0 0", "axis": axis,
            "range": "-0.015 0.015"}))
    idx += len(EXTRA_SLIDES)
    for k, (name, axis) in enumerate(zip(EXTRA_HINGES, (b, c)), start=1):
        calcn.insert(idx + k, ET.Element("joint", {
            "name": name, "class": "talar_play_rot", "type": "hinge", "pos": _fmt(c_p - calc_off),
            "axis": _fmt(axis), "range": "-0.7 0.7"}))

    # ---- articular geometry -------------------------------------------------
    first = next((i for i, e in enumerate(talus) if e.tag == "body"), len(talus))
    talar = [ET.Element("geom", {
        "name": "talar_head_r", "class": "cartilage", "type": "sphere", "size": f"{r_h - gap:.6g}",
        "pos": _fmt(c_h), "contype": "16", "conaffinity": "0"})]
    rings = _p("facet_rings_deg")
    talar += _cup("talar_posterior_facet", c_p, r_p, _ring_directions(g["facet_normal"], rings, (6, 10)),
                  r_p * math.tan(math.radians(rings[0]) / 2) + 0.001, "32")
    for k, e in enumerate(talar):
        talus.insert(first + k, e)

    first = next((i for i, e in enumerate(calcn) if e.tag == "body"), len(calcn))
    calc = [ET.Element("geom", {
        "name": "calcaneal_posterior_facet_r", "class": "cartilage", "type": "sphere", "size": f"{r_p - gap:.6g}",
        "pos": _fmt(c_p - calc_off), "contype": "32", "conaffinity": "0"})]
    rings = _p("socket_rings_deg")
    dirs = _ring_directions(np.array(_p("socket_direction")), rings, (6, 8))
    half = r_h * math.tan(math.radians(rings[0]) / 2) + 0.001
    plates = _cup("acetabulum_pedis", c_h - calc_off, r_h, dirs, half, "16")
    cone = math.cos(math.radians(_p("socket_calcaneal_cone_deg")))
    is_calcaneal = [-n[1] >= cone - 1e-9 for n in dirs]
    navicular = [e for e, c_ in zip(plates, is_calcaneal) if not c_]
    k_nav = _p("navicular_stiffness")
    # Rigid: every plate stays on the calcaneus, in its original order (free-foot transients are sensitive to it).
    calc += plates if math.isinf(k_nav) else [e for e, c_ in zip(plates, is_calcaneal) if c_]
    for k, e in enumerate(calc):
        calcn.insert(first + k, e)
    if 0 < k_nav < math.inf:
        # Navicular part of the socket on its own body, sprung to the calcaneus in translation.
        option = model.find("option")
        if option is None:
            option = ET.SubElement(model, "option")
        option.set("timestep", f"{_p('navicular_timestep'):g}")
        centre = c_h - calc_off
        nav = ET.Element("body", {"name": "navicular_socket_r", "pos": _fmt(centre)})
        ET.SubElement(nav, "inertial", pos="0 0 0", mass=f"{_p('navicular_mass'):g}", diaginertia="1e-5 1e-5 1e-5")
        for nm, ax in (("navicular_tx_r", "1 0 0"), ("navicular_ty_r", "0 1 0"), ("navicular_tz_r", "0 0 1")):
            ET.SubElement(nav, "joint", {"name": nm, "type": "slide", "pos": "0 0 0", "axis": ax,
                                         "range": "-0.015 0.015", "limited": "true", "stiffness": f"{k_nav:g}",
                                         "damping": f"{_p('navicular_damping'):g}", "armature": "0"})
        for e in navicular:
            pos = np.array([float(v) for v in e.get("pos").split()]) - centre
            e.set("pos", _fmt(pos))
            nav.append(e)
        calcn.append(nav)

    # The talus and calcaneus must now collide (their bone meshes do not: class "bone").
    contact = model.find("contact")
    for ex in list(contact.findall("exclude")):
        if {ex.get("body1"), ex.get("body2")} == {"talus_r", "calcn_r"}:
            contact.remove(ex)
    return g
