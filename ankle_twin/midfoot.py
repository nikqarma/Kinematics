"""Opt-in midfoot (midtarsal / Chopart) joint: build_ankle_model.py --midfoot.  Jira AT-13.

MS-Human-700's foot is one rigid body (calcn_r) from the heel to the metatarsal heads. This module splits
it at the Chopart joint line into the calcaneus (calcn_r) and one midfoot body (midfoot_r: navicular,
cuboid, cuneiforms and metatarsals; the toes hang from it). The navicular and cuboid moving as one unit
follows Wolf 2008 (bone pins in walking). The split is exact: r_foot.stl is made of separate bone
shells, and the most posterior one is the calcaneus.

Joint: two hinges in series through the centre of the talar head (Hicks 1953: the subtalar and both
midtarsal axes pass near it, and both midtarsal axes can move at once):
  oblique axis       Hicks's surface landmarks on the KU Leuven CT bones, rotated into this model's
                     frame by registration (tools/kul_midfoot_register.py)
  front-to-back axis same
Ranges: Hicks 1953 Table 2 totals (oblique 22 deg, front-to-back 8 deg) in one specimen, split
symmetrically about neutral (assumed: Hicks gives no neutral position). These hard stops stand for the
bone and capsule, which the model does not otherwise have; --midfoot-guard widens them to +-30 deg to
see what the ligaments alone allow.

Ligaments: paths from the KU Leuven foot model (Malaquias et al. 2016, CC BY 3.0), mapped onto this
model's bones. Stiffness, as tension-only linear springs, slack at midtarsal neutral (assumed):
  measured        spring ligament complex (Davis 1996), dorsal calcaneocuboid (Hofstede 1999, scaled
                  from its 7 mm test strip to the ligament's full 17 mm width)
  estimated       bifurcate (both limbs), dorsal talonavicular, short and long plantar: no measured
                  stiffness found. Scaled from Hofstede 1999's nine dorsal tarsal ligaments, tested the
                  same way as 7 mm strips: k = C * width / length with C = mean(k * span) / 7 mm =
                  130 N/mm (spread 31%), using each ligament's published width and length.
These estimates are model inputs, not fits: nothing here is tuned to a benchmark. Each estimated group
can be scaled for sensitivity studies (--midfoot-scale dorsal=2).

Contact mode (--midfoot contact; Jira AT-26, AT-16; Confluence "Design: midfoot contact and navicular
socket"): the midfoot gets six degrees of freedom relative to the calcaneus (three slides, then three hinges
about the oblique, front-to-back and a third axis, none with a working stop) and is held by articular
contact and the same ligaments. Hicks's axes then only name the coordinates; the motion is the output.
  talonavicular    the talar head as a sphere on the talus in a cup of plates on the midfoot covering
                   the navicular facet; head centre, radius and coverage from the KU Leuven CT bones
                   (midfoot_facets.py, all mapped by one registration of the anterior calcaneus)
  calcaneocuboid   22 contact pairs, each a small sphere on the cuboid facet facing a plate on the calcaneus
                   at its closest point (midfoot_facets.py); joint space 0.5-3.1 mm
  spring ligament  its three bundles wrap over the talar head (a sling the head can load in tension)
Contact is through explicit pairs only (frictionless, the cartilage contact of talocrural.py), so nothing
else in the model collides differently.
"""
from __future__ import annotations

import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import ligaments as LIG
import midfoot_facets as FA

# ---- geometry (tools/kul_midfoot_register.py output, 2026-11-18) -----------------------------------
# Talar-head centre: sphere fitted to the anterior quarter of this model's talus (radius 16.4 mm).
# (The KU Leuven head centre, mapped across, lands 15 mm further anterior, about at the front of this
# model's coarse talus mesh, so this model's own fit is used: the midfoot must turn about its own bones.)
HEAD_CENTRE_TALUS = np.array([0.012782, -0.000464, -0.003511])
# Axis directions in the calcn_r frame (= talus frame at neutral). Hicks landmarks on the KU Leuven bones:
# oblique 49.6 deg elevation / 29.0 deg medial, front-to-back 14.0 / 27.8; after registration (5.3 deg
# rotation) 46.5 / 29.6 and 11.1 / 25.6 here.
OBLIQUE_AXIS = np.array([0.59856, 0.72541, -0.33986])
FRONT_TO_BACK_AXIS = np.array([0.88512, 0.19285, -0.42351])
RANGE_DEG = {"midtarsal_oblique_r": 11.0, "midtarsal_ap_r": 4.0}   # Hicks 1953: 22 and 8 deg in total
GUARD_DEG = 30.0

TALUS_IN_CALCN = -np.array([-0.04877, -0.04195, 0.00792])   # talus_r origin in the calcn_r frame
HEAD_CENTRE = HEAD_CENTRE_TALUS + TALUS_IN_CALCN               # midfoot_r origin in the calcn_r frame
# The contact midfoot in development (Jira AT-26, AT-16) uses midfoot_facets.py instead: talar head, navicular
# coverage and calcaneocuboid contact pairs from the KU Leuven CT bones, all mapped by one registration of the
# anterior calcaneus. Its head centre lies 8 mm from HEAD_CENTRE_TALUS, which this hinge joint keeps.

MODES = ("hinge", "contact")
# Contact mode: the midfoot pivots about the KU Leuven talar head mapped onto this model (midfoot_facets.py).
CONTACT_HEAD_TALUS = np.array(FA.HEAD_CENTRE_TALUS)
CONTACT_CENTRE = CONTACT_HEAD_TALUS + TALUS_IN_CALCN          # midfoot_r origin in the calcn_r frame
CONTACT = {"condim": "1", "solref": "0.004 1", "solimp": "0.99 0.999 0.0005 0.5 2"}   # as talocrural.CONTACT
TN_CLEARANCE = 0.0002            # m; assumed, as subtalar.py's unloaded clearance
CUP_RINGS = ((0.0, 1), (14.0, 6), (28.0, 12), (42.0, 18), (56.0, 24), (66.0, 28))   # polar deg, plates
PLATE_HALF_THICKNESS = 0.0005
CC_SPHERE_RADIUS = 0.0015        # m; small, so each sphere follows the facet point it stands for
CC_PLATE_HALF = 0.0023           # m; plates 4.6 mm square at a 4 mm spacing, so neighbours overlap
CC_PAIR_REACH = 0.008            # m; each cuboid sphere is paired with every calcaneal plate this close
SLIDE_RANGE = 0.005              # m; safety limit only (the contacts hold the joint well inside it)
PLAY_DAMPING = 200.0             # N*s/m on the slides; as talocrural.TALAR_DAMPING (numerical/capsular)
SPRING_LIGAMENTS = ("SCN1_r", "SCN2_r", "SCN3_r")
CONTACT_TIMESTEP = 0.00025       # s; numerical (see add_joint)

# ---- ligament stiffness ------------------------------------------------------------------------------
# Hofstede 1999 Table 1 / 3: (shortest span, longest span, width) mm and stiffness N/mm per 7 mm strip,
# nine dorsal tarsal ligaments and the anterior tibiofibular ligament.
HOFSTEDE1999 = [(3, 11, 18, 93), (4, 10, 7, 96), (2, 10, 9, 67), (4, 16, 15, 113), (7, 21, 17, 55),
                (4, 17, 12, 127), (3, 14, 15, 124), (3, 12, 12, 123), (3, 15, 9, 125), (4, 17, 12, 100)]
STRIP_MM = 7.0
C_KL_PER_WIDTH = float(np.mean([k * (a + b) / 2 for a, b, _, k in HOFSTEDE1999])) / STRIP_MM   # N/mm
DORSAL_MEAN_WIDTH_MM = float(np.mean([w for _, _, w, _ in HOFSTEDE1999[:9]]))


def estimate_k(width_mm: float, length_mm: float) -> float:
    """Estimated stiffness (N/m) of a ligament of this width and length from Hofstede's class."""
    return C_KL_PER_WIDTH * width_mm / length_mm * 1e3


# name -> (full name, KU Leuven path(s), group, total stiffness N/m, source)
#   groups: "measured", "dorsal" (estimated), "plantar" (estimated)
_BIF_CN = estimate_k((7.12 + 4.9) / 2, (22.7 + 20.8) / 2)
_BIF_CC = estimate_k((7.11 + 4.7) / 2, (10.9 + 10.5) / 2)
_DTN = estimate_k(DORSAL_MEAN_WIDTH_MM, 11.6)
_SPL_DEEP = estimate_k(6.3, 17.7)
_SPL_SUP = estimate_k(11.6, 23.7)
_LPL = estimate_k(14.2, 44.3)
_SPRING = 45.1e3
_DCC = 55.0e3 * 17.0 / STRIP_MM

SPECS = [
    ("SCN1_r", "spring ligament (calcaneonavicular), bundle 1", ["CalcaneoNavicularPlantar1Mus_r"], "measured",
     _SPRING / 3, "measured: Davis1996 (whole complex 45.1 +/- 19.1 N/mm, intact; split equally over 3 bundles)"),
    ("SCN2_r", "spring ligament (calcaneonavicular), bundle 2", ["CalcaneoNavicularPlantar2Mus_r"], "measured",
     _SPRING / 3, "measured: Davis1996 (whole complex, split equally over 3 bundles)"),
    ("SCN3_r", "spring ligament (calcaneonavicular), bundle 3", ["CalcaneoNavicularPlantar3Mus_r"], "measured",
     _SPRING / 3, "measured: Davis1996 (whole complex, split equally over 3 bundles)"),
    ("DCC_r", "dorsal calcaneocuboid", ["CalcaneoCuboidDorsalMus_r"], "measured",
     _DCC, "measured: Hofstede1999 (55 +/- 14 N/mm on a 7 mm strip; x 17/7 for the full width)"),
    ("BCN_r", "bifurcate, calcaneonavicular limb", ["CalcaneoNavicularBifurcateMus_r"], "dorsal",
     _BIF_CN, "estimated: Hofstede1999 class x width 6.0 mm / length 21.8 mm (Kafka2019, Edama2018)"),
    ("BCC_r", "bifurcate, calcaneocuboid limb", ["CalcaneoCuboidBifurcateMus_r"], "dorsal",
     _BIF_CC, "estimated: Hofstede1999 class x width 5.9 mm / length 10.7 mm (Kafka2019, Edama2018)"),
    ("DTN1_r", "dorsal talonavicular, band 1", ["TaloNavicularDorsal1Mus_r"], "dorsal",
     _DTN / 2, "estimated: Hofstede1999 class x mean dorsal width 12.7 mm (no published width) / KUL length "
               "11.6 mm; split over 2 bands"),
    ("DTN2_r", "dorsal talonavicular, band 2", ["TaloNavicularDorsal2Mus_r"], "dorsal",
     _DTN / 2, "estimated: as band 1"),
    ("SPLd_r", "short plantar (plantar calcaneocuboid), deep", ["CalcaneoCuboidPlantar1Mus_r"], "plantar",
     _SPL_DEEP, "estimated: Hofstede1999 class x width 6.3 mm / length 17.7 mm (Edama2021, type I-b deep)"),
    ("SPLs_r", "short plantar (plantar calcaneocuboid), superficial", ["CalcaneoCuboidPlantar2Mus_r"], "plantar",
     _SPL_SUP, "estimated: Hofstede1999 class x width 11.6 mm / length 23.7 mm (Edama2021, type I-b superficial)"),
] + [
    (f"LPL{i}_r", f"long plantar, band {i}", [f"LongPlantar{i}_r"], "plantar",
     _LPL / 4, "estimated: Hofstede1999 class x width 14.2 mm / length 44.3 mm (Edama2021, type II); "
               "split equally over 4 bands")
    for i in (1, 2, 3, 4)
]

# KU Leuven paths mapped onto this model (tools/kul_midfoot_register.py): (body, point in that body's
# frame; "midfoot" points are in the calcn_r frame and are moved to midfoot_r when built).
KUL_PATHS = {
    "LongPlantar1_r": [("calcn_r", (0.02677, 0.00148, 0.00176)), ("midfoot", (0.08095, 0.00685, 0.00442)), ("midfoot", (0.11156, 0.01603, -0.00242))],
    "LongPlantar2_r": [("calcn_r", (0.02677, 0.00148, 0.00176)), ("midfoot", (0.08048, 0.00794, 0.00710)), ("midfoot", (0.10983, 0.01573, 0.00635))],
    "LongPlantar3_r": [("calcn_r", (0.02677, 0.00148, 0.00176)), ("midfoot", (0.07925, 0.00961, 0.01405)), ("midfoot", (0.10096, 0.01174, 0.01740))],
    "LongPlantar4_r": [("calcn_r", (0.02677, 0.00148, 0.00176)), ("midfoot", (0.07654, 0.00696, 0.02394)), ("midfoot", (0.10060, 0.00904, 0.02705))],
    "CalcaneoNavicularPlantar1Mus_r": [("calcn_r", (0.05197, 0.02714, -0.02483)), ("midfoot", (0.07561, 0.03250, -0.02137))],
    "CalcaneoNavicularPlantar2Mus_r": [("calcn_r", (0.05179, 0.02491, -0.02035)), ("midfoot", (0.07164, 0.02878, -0.01564))],
    "CalcaneoNavicularPlantar3Mus_r": [("calcn_r", (0.05949, 0.02743, -0.01595)), ("midfoot", (0.06980, 0.02997, -0.00891))],
    "CalcaneoCuboidPlantar1Mus_r": [("calcn_r", (0.05099, 0.01097, 0.00566)), ("midfoot", (0.07444, 0.00812, 0.00572))],
    "CalcaneoCuboidPlantar2Mus_r": [("calcn_r", (0.05247, 0.01048, -0.00196)), ("midfoot", (0.07490, 0.00988, 0.00310))],
    "CalcaneoCuboidDorsalMus_r": [("calcn_r", (0.05537, 0.02270, 0.01468)), ("midfoot", (0.08617, 0.02764, 0.02016))],
    "TaloNavicularDorsal1Mus_r": [("talus_r", (0.02631, 0.00571, 0.00203)), ("midfoot", (0.08641, 0.03827, -0.01633))],
    "TaloNavicularDorsal2Mus_r": [("talus_r", (0.02086, 0.00739, -0.00304)), ("midfoot", (0.08641, 0.03827, -0.01633))],
    "CalcaneoNavicularBifurcateMus_r": [("calcn_r", (0.06003, 0.03299, 0.01137)), ("midfoot", (0.07818, 0.03182, 0.00036))],
    "CalcaneoCuboidBifurcateMus_r": [("calcn_r", (0.06003, 0.03299, 0.01137)), ("midfoot", (0.07232, 0.02932, 0.01772))],
}

# Ligament sites already on calcn_r that belong to the midfoot. Only the tibionavicular inserts beyond
# the joint line. (The tibiospring stays: it inserts on the spring ligament. The cervical ligament's
# insertion vertex happens to lie on the cuboid shell of the coarse mesh, but it is a calcaneal
# ligament and stays on calcn_r.)
LIGAMENT_SITES_TO_MIDFOOT = {"TNL_r-P2"}
GROUPS = ("dorsal", "plantar", "measured")


def ligaments(scale: dict[str, float] | None = None) -> list[LIG.Ligament]:
    """Midfoot ligaments as LIG.Ligament records (body2 = midfoot_r), stiffness scaled per group."""
    scale = scale or {}
    out = []
    for name, full, kul, group, k, src in SPECS:
        body1 = KUL_PATHS[kul[0]][0][0]
        f = scale.get(group, 1.0)
        out.append(LIG.Ligament(name, full, body1, "midfoot_r", ("linear", k * f),
                                src + (f" [x{f:g} for sensitivity]" if f != 1.0 else ""), (0.0,),
                                "assumed: slack at midtarsal neutral (no zero-strain data found)",
                                taut_joint="midtarsal_oblique_r", reference_source="Malaquias2016",
                                notes=f"group {group}; path KU Leuven {kul[0]}"))
    return out


# ---- mesh handling -----------------------------------------------------------------------------------
def _read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    return np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                         count=n, offset=84)["v"].astype(float)


def _write_stl(path: Path, tri: np.ndarray) -> None:
    with open(path, "wb") as f:
        f.write(b"midfoot.py split of MS-Human-700 r_foot.stl".ljust(80, b" "))
        f.write(struct.pack("<I", len(tri)))
        for t in tri:
            nrm = np.cross(t[1] - t[0], t[2] - t[0])
            nn = np.linalg.norm(nrm)
            f.write(struct.pack("<12fH", *(nrm / nn if nn > 0 else nrm), *t.reshape(-1), 0))


def split_foot(tri: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(calcaneus triangles, the rest): the calcaneus is the most posterior connected shell."""
    v, idx = np.unique(np.round(tri.reshape(-1, 3), 7), axis=0, return_inverse=True)
    idx = idx.reshape(-1, 3)
    parent = np.arange(len(v))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b, c in idx:
        for x, y in ((a, b), (b, c)):
            rx, ry = root(x), root(y)
            if rx != ry:
                parent[rx] = ry
    roots = np.array([root(i) for i in range(len(v))])
    shell = roots[idx[:, 0]]
    ids = np.unique(shell)
    calc = ids[np.argmin([tri[shell == s].reshape(-1, 3)[:, 0].mean() for s in ids])]
    return tri[shell == calc], tri[shell != calc]


def volume(tri: np.ndarray) -> float:
    return abs(float(np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum()) / 6.0)


def _fmt(v) -> str:
    return " ".join(f"{c:.6g}" for c in v)


def _unit(v) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _cup_directions() -> list[np.ndarray]:
    """Plate directions over the navicular facet's coverage of the talar head (calcn_r frame)."""
    c = _unit(FA.NAVICULAR_DIRECTION)
    e1 = _unit(np.cross(c, [0.0, 1.0, 0.0]))
    e2 = np.cross(c, e1)
    az_tab = np.array([a for a, _ in FA.NAVICULAR_OUTLINE_DEG] + [FA.NAVICULAR_OUTLINE_DEG[0][0] + 360.0])
    po_tab = np.array([p for _, p in FA.NAVICULAR_OUTLINE_DEG] + [FA.NAVICULAR_OUTLINE_DEG[0][1]])
    out = []
    for polar, n in CUP_RINGS:
        for k in range(n):
            az = -180.0 + 360.0 * (k + 0.5) / n
            limit = np.interp(az if az >= az_tab[0] else az + 360.0, az_tab, po_tab)
            if polar <= limit:
                th, ph = math.radians(polar), math.radians(az)
                out.append(math.cos(th) * c + math.sin(th) * (math.cos(ph) * e1 + math.sin(ph) * e2))
    return out


def _plate(name: str, pos, normal, half: float) -> ET.Element:
    return ET.Element("geom", {"name": name, "class": "midfoot_cartilage", "type": "box",
                               "size": _fmt([half, half, PLATE_HALF_THICKNESS]), "pos": _fmt(pos),
                               "zaxis": _fmt(normal)})


def _add_contact(model: ET.Element, bodies: dict, mf: ET.Element, C: np.ndarray) -> None:
    """Talonavicular and calcaneocuboid contact geometry, explicit contact pairs, spring-ligament wrap."""
    talus, calcn = bodies["talus_r"], bodies["calcn_r"]
    r_h = FA.HEAD_RADIUS

    def first_body(b):
        return next((i for i, e in enumerate(b) if e.tag == "body"), len(b))

    # talonavicular: head on the talus, cup of plates on the midfoot (origin = head centre at neutral)
    talus.insert(first_body(talus), ET.Element("geom", {
        "name": "talar_head_tn_r", "class": "midfoot_cartilage", "type": "sphere", "size": f"{r_h:.6g}",
        "pos": _fmt(CONTACT_HEAD_TALUS)}))
    dirs = _cup_directions()
    half = (r_h * math.radians(14.0)) / 2 + 0.0008
    cup = []
    for i, n in enumerate(dirs):
        g = _plate(f"navicular_cup_{i:02d}_r", (r_h + TN_CLEARANCE + PLATE_HALF_THICKNESS) * n, -n, half)
        mf.insert(first_body(mf), g)
        cup.append(g.get("name"))

    # calcaneocuboid: plates on the calcaneus, small spheres on the cuboid facet
    cal = np.array(FA.CC_CALCANEAL)
    cub = np.array(FA.CC_CUBOID)
    plates = []
    for i, row in enumerate(cal):
        g = _plate(f"cc_calcaneal_{i:02d}_r", row[:3] - PLATE_HALF_THICKNESS * row[3:], row[3:], CC_PLATE_HALF)
        calcn.insert(first_body(calcn), g)
        plates.append(g.get("name"))
    spheres = []
    for i, row in enumerate(cub):
        # The CT joint space (bone to bone) is cartilage: the sphere touches the calcaneal plate at neutral,
        # with the unloaded clearance, instead of standing at the cuboid bone surface 0.5-3.1 mm away.
        cal_n = cal[i, 3:]
        centre = cal[i, :3] + (TN_CLEARANCE + CC_SPHERE_RADIUS) * cal_n
        g = ET.Element("geom", {"name": f"cc_cuboid_{i:02d}_r", "class": "midfoot_cartilage", "type": "sphere",
                                "size": f"{CC_SPHERE_RADIUS:g}", "pos": _fmt(centre - C)})
        mf.insert(first_body(mf), g)
        spheres.append(g.get("name"))

    contact = model.find("contact")
    if contact is None:
        contact = ET.SubElement(model, "contact")
    # Skin capsules on neighbouring foot bodies overlap at neutral; with MuJoCo's parent filter off (6-DOF
    # builds) or across the new body chain they would push the free midfoot. Excludes only affect
    # automatic contacts, not the explicit pairs below.
    for b1, b2 in (("calcn_r", "midfoot_r"), ("midfoot_r", "toes_r"), ("calcn_r", "toes_r"), ("talus_r", "midfoot_r")):
        ET.SubElement(contact, "exclude", body1=b1, body2=b2)
    for name in cup:
        ET.SubElement(contact, "pair", geom1="talar_head_tn_r", geom2=name, **CONTACT)
    for i, sname in enumerate(spheres):
        # plates near this facet point, facing it, with the point clearly on their open side (at the
        # facet's curled rim a neighbouring plate can otherwise reach behind it)
        near = ((np.linalg.norm(cal[:, :3] - cub[i, :3], axis=1) < CC_PAIR_REACH)
                & (np.einsum("ij,j->i", cal[:, 3:], -cub[i, 3:]) > 0.5)
                & (np.einsum("ij,ij->i", cal[i, :3] + TN_CLEARANCE * cal[i, 3:] - cal[:, :3], cal[:, 3:])
                   > 0.5 * TN_CLEARANCE))
        for j in np.flatnonzero(near):
            ET.SubElement(contact, "pair", geom1=sname, geom2=plates[j], **CONTACT)

    # spring ligament: wraps over the talar head, on the side its path runs (plantar-medial)
    talus.insert(first_body(talus), ET.Element("geom", {
        "name": "talar_head_wrap_r", "class": "wrap", "type": "sphere", "size": f"{r_h:.6g}",
        "pos": _fmt(CONTACT_HEAD_TALUS)}))
    mids = np.mean([np.mean([np.array(q) for _, q in KUL_PATHS[spec[2][0]]], axis=0)
                    for spec in SPECS if spec[0] in SPRING_LIGAMENTS], axis=0)       # calcn_r frame
    side = CONTACT_HEAD_TALUS + (r_h + 0.004) * _unit(mids - CONTACT_CENTRE)
    talus.insert(first_body(talus), ET.Element("site", {"name": "spring_wrap_side_r", "pos": _fmt(side)}))
    for t in model.find("tendon").findall("spatial"):
        if t.get("name") in SPRING_LIGAMENTS:
            sites = t.findall("site")
            t.insert(list(t).index(sites[0]) + 1,
                     ET.Element("geom", {"geom": "talar_head_wrap_r", "sidesite": "spring_wrap_side_r"}))


def add_joint(model: ET.Element, femur: ET.Element, mesh_dir: Path, scale: dict[str, float] | None = None,
              guard: bool = False, mode: str = "hinge") -> list[LIG.Ligament]:
    """Split calcn_r into calcn_r + midfoot_r with the midtarsal joint and its ligaments.

    mode "hinge": two hinges through the talar head with Hicks's stops; "contact": six degrees of freedom
    held by articular contact (see the module docstring). Call after the ankle ligaments have been added.
    Returns the midfoot ligaments (for slack lengths)."""
    if mode not in MODES:
        raise ValueError(f"midfoot mode {mode!r}: expected one of {MODES}")
    bodies = {b.get("name"): b for b in femur.iter("body")}
    calcn = bodies["calcn_r"]
    C = HEAD_CENTRE if mode == "hinge" else CONTACT_CENTRE

    # meshes
    hind, mid = split_foot(_read_stl(mesh_dir / "r_foot.stl"))
    _write_stl(mesh_dir / "r_calcaneus.stl", hind)
    _write_stl(mesh_dir / "r_midfoot.stl", mid)
    asset = model.find("asset")
    for m in ("r_calcaneus", "r_midfoot"):
        ET.SubElement(asset, "mesh", name=m, content_type="model/stl", file=f"{m}.stl")

    # mass: MS-Human-700's foot mass split by bone volume; inertia from the meshes (uniform density)
    inertial = calcn.find("inertial")
    m_total = float(inertial.get("mass"))
    v_h, v_m = volume(hind), volume(mid)
    m_h, m_m = m_total * v_h / (v_h + v_m), m_total * v_m / (v_h + v_m)
    calcn.remove(inertial)

    foot_geom = next(g for g in calcn.findall("geom") if g.get("name") == "r_foot")
    foot_geom.set("name", "r_calcaneus")
    foot_geom.set("mesh", "r_calcaneus")
    foot_geom.set("mass", f"{m_h:.6g}")

    toes = bodies["toes_r"]
    mf = ET.Element("body", name="midfoot_r", pos=_fmt(C))
    if mode == "hinge":
        rng = (lambda j: GUARD_DEG if guard else RANGE_DEG[j])
        for jname, ax in (("midtarsal_oblique_r", OBLIQUE_AXIS), ("midtarsal_ap_r", FRONT_TO_BACK_AXIS)):
            r = math.radians(rng(jname))
            ET.SubElement(mf, "joint", name=jname, **{"class": "ankle"}, pos="0 0 0", axis=_fmt(ax),
                          range=f"{-r:.6g} {r:.6g}")
    else:
        # A 0.3 kg midfoot on ~580 N/mm of ligaments: explicit integration needs a short step. Hinge
        # subtalar: unstable at 2 ms, drifts to its slide limits at 1 ms, 0.5 and 0.25 ms agree. With
        # research-twin's 6-DOF subtalar (talus, calcaneus and midfoot all floating): unstable at 0.5 ms,
        # 0.25 and 0.125 ms agree to 0.01 mm under gravity.
        option = model.find("option")
        if option is None:
            option = ET.Element("option")
            model.insert(1, option)
        option.set("timestep", f"{CONTACT_TIMESTEP:g}")
        default = model.find("default")
        play = ET.SubElement(default, "default", {"class": "midfoot_play"})
        ET.SubElement(play, "joint", damping=f"{PLAY_DAMPING:g}", armature="0.001", stiffness="0")
        cart = ET.SubElement(default, "default", {"class": "midfoot_cartilage"})
        ET.SubElement(cart, "geom", contype="0", conaffinity="0", rgba="0.55 0.8 0.95 0.35", group="2", mass="0")
        for jname, ax in (("midtarsal_tx_r", "1 0 0"), ("midtarsal_ty_r", "0 1 0"), ("midtarsal_tz_r", "0 0 1")):
            ET.SubElement(mf, "joint", name=jname, type="slide", **{"class": "midfoot_play"}, pos="0 0 0",
                          axis=ax, range=f"{-SLIDE_RANGE:g} {SLIDE_RANGE:g}")
        third = _unit(np.cross(OBLIQUE_AXIS, FRONT_TO_BACK_AXIS))
        r = math.radians(GUARD_DEG)
        for jname, ax in (("midtarsal_oblique_r", OBLIQUE_AXIS), ("midtarsal_ap_r", FRONT_TO_BACK_AXIS),
                          ("midtarsal_third_r", third)):
            ET.SubElement(mf, "joint", name=jname, **{"class": "ankle"}, pos="0 0 0", axis=_fmt(ax),
                          range=f"{-r:.6g} {r:.6g}")
    ET.SubElement(mf, "geom", name="r_midfoot", **{"class": "bone"}, type="mesh", mesh="r_midfoot",
                  pos=_fmt(-C), mass=f"{m_m:.6g}")

    # move elements beyond the joint line: muscle sites and skin capsules nearest a midfoot shell, the
    # tibionavicular insertion, and the toes
    hv, mv = hind.reshape(-1, 3), mid.reshape(-1, 3)
    for el in list(calcn):
        if el.tag == "geom" and el.get("class") == "skin":
            el.set("mass", "0")
        if el.tag not in ("site", "geom") or el.get("name") == "r_calcaneus":
            continue
        p = np.array([float(c) for c in el.get("pos", "0 0 0").split()])
        if el.get("class") == "ligament":
            move = el.get("name") in LIGAMENT_SITES_TO_MIDFOOT
        else:
            move = np.linalg.norm(mv - p, axis=1).min() < np.linalg.norm(hv - p, axis=1).min()
        if move:
            calcn.remove(el)
            el.set("pos", _fmt(p - C))
            mf.append(el)
    calcn.remove(toes)
    toes.set("pos", _fmt(np.array([float(c) for c in toes.get("pos").split()]) - C))
    mf.append(toes)
    calcn.append(mf)
    bodies["midfoot_r"] = mf

    # ligaments
    ligs = ligaments(scale)
    tendon = model.find("tendon")
    tendon.append(ET.Comment(
        " Midfoot ligaments (midfoot.py): paths from the KU Leuven foot model (Malaquias et al. 2016, "
        "CC BY 3.0) mapped onto these bones; tension-only linear springs "))
    for lig, spec in zip(ligs, SPECS):
        path = KUL_PATHS[spec[2][0]]
        sites = []
        for i, (body, p) in enumerate(path, start=1):
            p = np.array(p)
            bname = "midfoot_r" if body == "midfoot" else body
            if bname == "midfoot_r":
                p = p - C
            b = bodies[bname]
            first_child = next((j for j, c in enumerate(b) if c.tag == "body"), len(b))
            sname = f"{lig.name}-P{i}"
            b.insert(first_child, ET.Element("site", {"name": sname, "class": "ligament", "pos": _fmt(p)}))
            sites.append(sname)
        tendon.append(ET.Comment(f" {lig.full_name}: linear k={lig.curve[1] / 1000:.1f} N/mm "
                                 f"({lig.stiffness_source}); {lig.taut_source} "))
        sp = ET.SubElement(tendon, "spatial", {"name": lig.name, "class": "ligament",
                                               "stiffness": "0", "springlength": "0 0"})
        for s in sites:
            ET.SubElement(sp, "site", site=s)
    if mode == "contact":
        _add_contact(model, bodies, mf, C)
    return ligs
