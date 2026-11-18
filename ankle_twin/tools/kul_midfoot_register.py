"""Map the KU Leuven foot model's midtarsal axes and midfoot ligament paths onto this model's bones.

Source: KU Leuven 8-DOF foot model (Malaquias T., Silveira C., Aerts W., De Groote F., Dereymaeker G.,
Vander Sloten J., Jonkers I. (2016) Extended foot-ankle musculoskeletal models for application in movement
analysis. Comput Methods Biomech Biomed Engin), SimTK project kul_footmodel, CC BY 3.0. The KU Leuven
files are not redistributed here; download them and pass their folder.

Method
  1. Similarity ICP (rotation, translation, one scale) of the KU Leuven calcaneus, midfoot and forefoot
     meshes onto this model's r_foot mesh (calcn_r frame), pairing calcaneus with calcaneus and the rest
     with the rest. The same for the talus (talus_r frame).
  2. Hicks 1953's surface landmarks give the two midtarsal axes on the KU Leuven bones (see
     hicks_landmark_axes); their directions are rotated into this model's calcn_r frame.
  3. Each KU Leuven ligament attachment is mapped and moved to the nearest vertex of the right bone
     (calcaneus, talus, or the midfoot/forefoot shells) of this model.
  4. The talar-head centre is a sphere fitted to the anterior quarter of this model's talus.

Prints the constants used by midfoot.py.

With --facets OUT.py it also writes the articular facet geometry for the contact midfoot (Jira AT-26,
AT-16; Confluence "Design: midfoot contact and navicular socket"):
  5. Facet patches on the KU Leuven bones: calcaneocuboid = calcaneus and cuboid vertices within 3 mm of
     each other and over 6 mm from the talus; talonavicular = talus and navicular vertices within 4 mm.
  6. A second similarity registration, of the anterior 40% of the KU Leuven calcaneus onto the anterior
     40% of this model's calcaneus. The whole-foot registration leaves the calcaneal facet about 7 mm off
     this model's calcaneus; this one puts it on the surface (median 2 mm; this model's mesh is coarse).
     Both calcaneocuboid facets are mapped by it, so they stay congruent; the cuboid facet then sits a
     few mm proximal of this model's cuboid mesh, whose joint space is wider (5 mm).
  7. The talar head (sphere through its articular patch) and the navicular facet's coverage are mapped
     by the same transform, so the whole joint set comes from one subject in one pose. The mapped head
     lies on this model's talus surface (median 2 mm) but 8 mm from midfoot.HEAD_CENTRE_TALUS, a sphere
     fit to the coarse talus mesh that the hinge midfoot keeps; with that centre the calcaneocuboid facet
     would cut into the head.
  8. Calcaneocuboid contact pairs: cuboid facet vertices about 4 mm apart (farthest-point sampling),
     each paired with its closest point on the KU Leuven calcaneus surface (triangles). The pair's
     direction is the calcaneal surface normal there, and its length the local joint space, so every
     pair is congruent and open at neutral by construction. (Height fields over one mid-plane do not
     work: the saddle facets curl at their edges.)

Usage: python tools/kul_midfoot_register.py <kul_footmodel folder> [--facets midfoot_facets.py]
"""
from __future__ import annotations

import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
import ligaments as LIG  # noqa: E402

# KU Leuven ligaments spanning the midtarsal joint (all bundles of each).
KUL_LIGAMENTS = [
    "CalcaneoNavicularPlantar1Mus_r", "CalcaneoNavicularPlantar2Mus_r", "CalcaneoNavicularPlantar3Mus_r",
    "CalcaneoCuboidPlantar1Mus_r", "CalcaneoCuboidPlantar2Mus_r", "CalcaneoCuboidDorsalMus_r",
    "TaloNavicularDorsal1Mus_r", "TaloNavicularDorsal2Mus_r",
    "CalcaneoNavicularBifurcateMus_r", "CalcaneoCuboidBifurcateMus_r",
    "LongPlantar1_r", "LongPlantar2_r", "LongPlantar3_r", "LongPlantar4_r",
]


def load_any(path: Path) -> np.ndarray:
    """Unique vertices of a binary or ASCII STL."""
    data = path.read_bytes()
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        a = np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data), dtype=float)
    else:
        a = LIG.load_stl(path)
    return np.unique(np.round(a, 6), axis=0)


def shells(path: Path) -> list[np.ndarray]:
    """Vertices of each connected shell of a binary STL."""
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    tri = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                        count=n, offset=84)["v"].astype(float)
    v, idx = np.unique(np.round(tri.reshape(-1, 3), 6), axis=0, return_inverse=True)
    parent = list(range(len(v)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b, c in idx.reshape(-1, 3):
        for x, y in ((a, b), (b, c)):
            rx, ry = root(x), root(y)
            if rx != ry:
                parent[rx] = ry
    roots = np.array([root(i) for i in range(len(v))])
    return [v[roots == r] for r in np.unique(roots)]


def umeyama(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """s, R, t minimising |s R src + t - dst|."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(b.T @ a / len(src))
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1
    R = U @ D @ Vt
    s = float(np.trace(np.diag(S) @ D) / (a ** 2).sum(1).mean())
    return s, R, mu_d - s * R @ mu_s


def icp(groups_src: list[np.ndarray], groups_dst: list[np.ndarray], iters: int = 60):
    """Similarity ICP with correspondences found within matching groups (dst -> nearest src)."""
    allsrc, alldst = np.vstack(groups_src), np.vstack(groups_dst)
    s = (np.ptp(alldst, 0).mean() / np.ptp(allsrc, 0).mean())
    R, t = np.eye(3), alldst.mean(0) - s * allsrc.mean(0)
    for _ in range(iters):
        P, Q = [], []
        for gs, gd in zip(groups_src, groups_dst):
            ts = s * gs @ R.T + t
            d = ((gd[:, None, :] - ts[None, :, :]) ** 2).sum(2)
            P.append(gs[d.argmin(1)])
            Q.append(gd)
        s, R, t = umeyama(np.vstack(P), np.vstack(Q))
    res = []
    for gs, gd in zip(groups_src, groups_dst):
        ts = s * gs @ R.T + t
        res.append(np.sqrt(((gd[:, None, :] - ts[None, :, :]) ** 2).sum(2).min(1)).mean())
    return s, R, t, res


def subsample(v: np.ndarray, n: int = 2500, seed: int = 0) -> np.ndarray:
    if len(v) <= n:
        return v
    return v[np.random.default_rng(seed).choice(len(v), n, replace=False)]


def sphere_fit(p: np.ndarray) -> tuple[np.ndarray, float]:
    A = np.c_[2 * p, np.ones(len(p))]
    sol = np.linalg.lstsq(A, (p ** 2).sum(1), rcond=None)[0]
    c = sol[:3]
    return c, float(math.sqrt(sol[3] + c @ c))


def talar_head(tal: np.ndarray) -> tuple[np.ndarray, float]:
    span = np.ptp(tal[:, 0])
    return sphere_fit(tal[tal[:, 0] > tal[:, 0].max() - 0.25 * span])


def hicks_landmark_axes(tal, cal, mid):
    """Hicks 1953's oblique and front-to-back midtarsal axes from surface landmarks (unit vectors,
    pointing anterior). Same rules as the scratch study of 2026-11-17."""
    x, y, z = 0, 1, 2
    span = np.ptp(tal[:, x])
    head = tal[tal[:, x] > tal[:, x].max() - 0.25 * span]
    sm_head = head[np.argmax(head[:, y] - head[:, z])]
    cspan = np.ptp(cal[:, x])
    heel = cal[cal[:, x] < cal[:, x].min() + 0.35 * cspan]
    pl_corner = heel[np.argmax(-heel[:, x] + heel[:, z])]
    band = cal[np.abs(cal[:, x] - (pl_corner[x] + 0.025)) < 0.003]
    low = band[band[:, y] < band[:, y].min() + 0.006]
    inf_lat = low[np.argmax(low[:, z])]
    band2 = cal[np.abs(cal[:, y] - (pl_corner[y] + 0.025)) < 0.003]
    band2 = band2[band2[:, x] < band2[:, x].min() + 0.012]
    post_lat = band2[np.argmax(band2[:, z])]
    x0 = tal[:, x].max()
    nav = mid[(mid[:, x] > x0 + 0.002) & (mid[:, x] < x0 + 0.015)]
    fore = mid[(mid[:, x] > x0 + 0.04) & (mid[:, x] < x0 + 0.06)]
    zt = 0.5 * (nav[:, z].min() + fore[np.argmax(fore[:, y]), z])
    cand = nav[np.abs(nav[:, z] - zt) < 0.004]
    sup_nav = cand[np.argmax(cand[:, y])]

    def unit(v):
        v = v / np.linalg.norm(v)
        return v if v[0] > 0 else -v

    return unit(sm_head - inf_lat), unit(sup_nav - post_lat)


def near_dist(a: np.ndarray, b: np.ndarray, cut: float = 0.008) -> np.ndarray:
    """Distance from each point of a to the nearest point of b, exact up to cut (beyond it: cut)."""
    cells: dict[tuple, list[int]] = {}
    for i, k in enumerate(map(tuple, np.floor(b / cut).astype(int))):
        cells.setdefault(k, []).append(i)
    out = np.full(len(a), cut)
    for i, k in enumerate(np.floor(a / cut).astype(int)):
        idx = [j for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
               for j in cells.get((k[0] + dx, k[1] + dy, k[2] + dz), ())]
        if idx:
            out[i] = min(cut, float(np.sqrt(((b[idx] - a[i]) ** 2).sum(1)).min()))
    return out


def load_triangles(path: Path) -> np.ndarray:
    """Triangles (n, 3, 3) of a binary or ASCII STL."""
    data = path.read_bytes()
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        return np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data), dtype=float).reshape(-1, 3, 3)
    n = int.from_bytes(data[80:84], "little")
    return np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                         count=n, offset=84)["v"].astype(float)


def closest_on_triangles(q: np.ndarray, tri: np.ndarray) -> np.ndarray:
    """Closest point to q on a set of triangles (Ericson, Real-Time Collision Detection 5.1.5)."""
    best, bd = None, np.inf
    for a, b, c in tri:
        ab, ac, ap = b - a, c - a, q - a
        d1, d2 = ab @ ap, ac @ ap
        if d1 <= 0 and d2 <= 0:
            x = a
        else:
            bp = q - b
            d3, d4 = ab @ bp, ac @ bp
            cp = q - c
            d5, d6 = ab @ cp, ac @ cp
            vc, vb, va = d1 * d4 - d3 * d2, d5 * d2 - d1 * d6, d3 * d6 - d5 * d4
            if d3 >= 0 and d4 <= d3:
                x = b
            elif d6 >= 0 and d5 <= d6:
                x = c
            elif vc <= 0 and d1 >= 0 and d3 <= 0:
                x = a + d1 / (d1 - d3) * ab
            elif vb <= 0 and d2 >= 0 and d6 <= 0:
                x = a + d2 / (d2 - d6) * ac
            elif va <= 0 and d4 - d3 >= 0 and d5 - d6 >= 0:
                x = b + (d4 - d3) / ((d4 - d3) + (d5 - d6)) * (c - b)
            else:
                den = 1.0 / (va + vb + vc)
                x = a + ab * vb * den + ac * vc * den
        d = float(((q - x) ** 2).sum())
        if d < bd:
            best, bd = x, d
    return best


def contact_pairs(cub: np.ndarray, cal_tri: np.ndarray, spacing: float = 0.004):
    """Cuboid points ~spacing apart and their closest points on the calcaneal triangles (same frame)."""
    keep = [int(np.argmin(np.linalg.norm(cub - cub.mean(0), axis=1)))]
    dmin = np.linalg.norm(cub - cub[keep[0]], axis=1)
    while dmin.max() > spacing:
        i = int(np.argmax(dmin))
        keep.append(i)
        dmin = np.minimum(dmin, np.linalg.norm(cub - cub[i], axis=1))
    near = cal_tri[np.linalg.norm(cal_tri.mean(1)[:, None, :] - cub[None, :, :], axis=2).min(1) < 0.008]
    q = cub[keep]
    p = np.array([closest_on_triangles(x, near) for x in q])
    n = (q - p) / np.linalg.norm(q - p, axis=1)[:, None]
    return p, n, q


FACETS_TEMPLATE = '''"""Articular facet geometry for the contact midfoot. GENERATED by tools/kul_midfoot_register.py --facets;
do not edit by hand.

Source: KU Leuven foot model CT bones (Malaquias T., Silveira C., Aerts W., De Groote F., Dereymaeker G.,
Vander Sloten J., Jonkers I. (2016) Extended foot-ankle musculoskeletal models for application in movement
analysis. Comput Methods Biomech Biomed Engin; SimTK kul_footmodel), CC BY 3.0. One subject. Mapped onto
this model by a similarity registration of the anterior calcaneus (scale {s:.3f}, mean surface distance
{res:.1f} mm); see the tool's docstring for the method.

Frame: calcn_r, metres, at midtarsal neutral. Facet rows: point x, y, z, then outward unit normal.
"""

# Talar head: sphere through the KU Leuven talar head's articular patch ({nhead} vertices, rms {hrms:.2f} mm,
# r {hr:.2f} mm), mapped. Centre in the talus_r frame (talus_r origin at midfoot.TALUS_IN_CALCN at neutral);
# {hoff:.1f} mm from midfoot.HEAD_CENTRE_TALUS, which the hinge midfoot keeps.
HEAD_CENTRE_TALUS = ({h0:.6f}, {h1:.6f}, {h2:.6f})
HEAD_RADIUS = {head_radius:.5f}

# Navicular facet's coverage of the talar head: central direction (calcn_r frame) and, per 30 deg azimuth
# sector about it (azimuth 0 along cross(direction, +y)), the largest polar angle covered, deg.
NAVICULAR_DIRECTION = ({c0:.4f}, {c1:.4f}, {c2:.4f})
NAVICULAR_OUTLINE_DEG = {outline}

# Calcaneocuboid contact pairs: row i of CC_CUBOID is a cuboid facet point (about 4 mm apart), row i of
# CC_CALCANEAL its closest point on the calcaneus; the normal is the pair's direction (calcaneal surface
# normal), pointing to the cuboid in CC_CALCANEAL and to the calcaneus in CC_CUBOID. Joint space: KU Leuven
# before scaling min {gmin:.2f}, median {gmed:.2f} mm; the pairs, mapped: {nmin:.2f} to {nmax:.2f} mm (median {nmed:.2f}).
CC_SPACING = 0.004
CC_CALCANEAL = (
{cal_rows}
)
CC_CUBOID = (
{cub_rows}
)
'''


def facets(kul: Path, osim, o_cal: np.ndarray, out: Path) -> None:
    """Steps 5-7: write the contact midfoot's facet geometry module."""
    def loc(joint):
        j = next(e for e in osim.iter("CustomJoint") if e.get("name") == joint)
        return np.array([float(c) for c in j.find(".//location_in_parent").text.split()])

    import midfoot as MF  # noqa: E402

    tal = load_any(kul / "model_r_talus.stl") - loc("Subtalar_r")          # KU Leuven calcn frame
    cal = load_any(kul / "model_r_calcaneus.stl")
    mid = load_any(kul / "model_r_midfoot.stl") + loc("Chopart_r")
    d_cal_mid, d_cal_tal = near_dist(cal, mid), near_dist(cal, tal)
    d_mid_cal, d_mid_tal = near_dist(mid, cal), near_dist(mid, tal)
    on_cal = (d_cal_mid < 0.003) & (d_cal_tal > 0.006)
    cc_cal, gap = cal[on_cal], d_cal_mid[on_cal]
    cc_cub = mid[(d_mid_cal < 0.003) & (d_mid_tal > 0.006)]
    head = tal[near_dist(tal, mid) < 0.004]
    nav = mid[d_mid_tal < 0.004]
    hc, hr = sphere_fit(head)
    hrms = float(np.sqrt(np.mean((np.linalg.norm(head - hc, axis=1) - hr) ** 2)))

    ka = cal[cal[:, 0] > cal[:, 0].max() - 0.4 * np.ptp(cal[:, 0])]
    oa = o_cal[o_cal[:, 0] > o_cal[:, 0].max() - 0.4 * np.ptp(o_cal[:, 0])]
    s, R, t, res = icp([subsample(ka)], [oa])

    def m(p):
        return s * p @ R.T + t

    cal_tri = load_triangles(kul / "model_r_calcaneus.stl")
    p_cal, n_cal, p_cub = contact_pairs(m(cc_cub), m(cal_tri.reshape(-1, 3)).reshape(-1, 3, 3))
    n_cub = -n_cal
    node_gap = np.linalg.norm(p_cub - p_cal, axis=1)
    h_calcn = m(hc)
    h_talus = h_calcn - MF.TALUS_IN_CALCN

    dirs = ((nav - hc) / np.linalg.norm(nav - hc, axis=1)[:, None]) @ R.T
    c = dirs.mean(0)
    c /= np.linalg.norm(c)
    e1 = np.cross(c, [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(c, e1)
    az = np.degrees(np.arctan2(dirs @ e2, dirs @ e1))
    polar = np.degrees(np.arccos(np.clip(dirs @ c, -1, 1)))
    outline = []
    for a0 in range(-180, 180, 30):
        sel = (az >= a0) & (az < a0 + 30)
        outline.append((a0 + 15, round(float(polar[sel].max()), 1) if sel.any() else 0.0))

    centre = 0.5 * (p_cal.mean(0) + p_cub.mean(0))
    print(f"# facets: anterior-calcaneus registration scale {s:.3f}, mean surface distance {res[0] * 1e3:.1f} mm")
    print(f"# calcaneocuboid: {len(cc_cal)} / {len(cc_cub)} KU Leuven vertices -> {len(p_cal)} / {len(p_cub)} "
          f"samples; KU Leuven gap min {gap.min() * 1e3:.2f}, median {np.median(gap) * 1e3:.2f} mm; centre "
          f"{np.round(centre * 1e3, 1)} mm calcn frame")
    print(f"# contact-pair joint space {node_gap.min() * 1e3:.2f} to {node_gap.max() * 1e3:.2f} mm (median "
          f"{np.median(node_gap) * 1e3:.2f}); facet clear of the head sphere by "
          f"{(np.linalg.norm(np.r_[p_cal, p_cub] - h_calcn, axis=1).min() - hr * s) * 1e3:.1f} mm")
    print(f"# talar head: KU Leuven articular sphere r {hr * 1e3:.1f} mm (rms {hrms * 1e3:.2f}) -> {hr * s * 1e3:.1f} mm, "
          f"centre {np.round(h_talus * 1e3, 2)} mm talus frame, {np.linalg.norm(h_calcn - MF.HEAD_CENTRE) * 1e3:.1f} mm "
          f"from HEAD_CENTRE_TALUS; navicular facet {len(nav)} vertices, up to {max(p for _, p in outline):.0f} deg "
          f"from its centre direction")

    def rows(p, n):
        return "\n".join(f"    ({a[0]:.5f}, {a[1]:.5f}, {a[2]:.5f}, {b[0]:+.4f}, {b[1]:+.4f}, {b[2]:+.4f}),"
                         for a, b in zip(p, n))

    out.write_text(FACETS_TEMPLATE.format(
        s=s, res=res[0] * 1e3, nhead=len(head), hrms=hrms * 1e3, hr=hr * 1e3, head_radius=hr * s,
        h0=h_talus[0], h1=h_talus[1], h2=h_talus[2], hoff=np.linalg.norm(h_calcn - MF.HEAD_CENTRE) * 1e3,
        nmin=node_gap.min() * 1e3, nmax=node_gap.max() * 1e3, nmed=float(np.median(node_gap)) * 1e3,
        c0=c[0], c1=c[1], c2=c[2], outline=tuple(outline), gmin=gap.min() * 1e3, gmed=float(np.median(gap)) * 1e3,
        cal_rows=rows(p_cal, n_cal), cub_rows=rows(p_cub, n_cub)), encoding="utf-8", newline="\n")
    print(f"wrote {out}")


def describe(u):
    return (f"elevation {math.degrees(math.asin(u[1])):+.1f} deg, "
            f"medial {math.degrees(math.atan2(-u[2], u[0])):+.1f} deg")


def main(kul: Path, facets_out: Path | None = None) -> None:
    osim = ET.parse(kul / "KULeuven_8DoF.osim").getroot()

    def loc(joint):
        j = next(e for e in osim.iter("CustomJoint") if e.get("name") == joint)
        return np.array([float(c) for c in j.find(".//location_in_parent").text.split()])

    calcn_in_talus = loc("Subtalar_r")
    mid_in_calcn = loc("Chopart_r")
    fore_in_calcn = mid_in_calcn + loc("Tarsometatarsal_r")
    to_calcn = {"calcn_r": np.zeros(3), "midfoot_r": mid_in_calcn, "forefoot_r": fore_in_calcn,
                "talus_r": -calcn_in_talus}

    k_tal = load_any(kul / "model_r_talus.stl")
    k_cal = load_any(kul / "model_r_calcaneus.stl")
    k_rest = np.vstack([load_any(kul / "model_r_midfoot.stl") + mid_in_calcn,
                        load_any(kul / "model_r_forefoot.stl") + fore_in_calcn])

    md = HERE / "model" / "meshes"
    sh = shells(md / "r_foot.stl")
    i_cal = int(np.argmin([s[:, 0].mean() for s in sh]))     # the most posterior shell
    o_cal = sh[i_cal]
    o_rest = np.vstack([s for i, s in enumerate(sh) if i != i_cal])
    o_tal = LIG.load_stl(md / "r_talus.stl")

    s_f, R_f, t_f, res_f = icp([subsample(k_cal), subsample(k_rest)], [o_cal, o_rest])
    s_t, R_t, t_t, res_t = icp([subsample(k_tal)], [o_tal])
    rot = math.degrees(math.acos(np.clip((np.trace(R_f) - 1) / 2, -1, 1)))
    print(f"# foot: scale {s_f:.3f}, rotation {rot:.1f} deg, mean surface distance "
          f"calcaneus {res_f[0] * 1e3:.1f} mm, midfoot/forefoot {res_f[1] * 1e3:.1f} mm")
    print(f"# talus: scale {s_t:.3f}, mean surface distance {res_t[0] * 1e3:.1f} mm")

    # Hicks axes on the KU Leuven bones (talus frame = calcn frame rotation), rotated into ours.
    obl_k, ap_k = hicks_landmark_axes(k_tal, k_cal + calcn_in_talus, k_rest + calcn_in_talus)
    obl, ap = R_f @ obl_k, R_f @ ap_k
    print(f"# KUL oblique {describe(obl_k)}; front-to-back {describe(ap_k)}")
    print(f"# mapped oblique {describe(obl)}; front-to-back {describe(ap)}")

    T = LIG.BODY_ORIGIN_IN_TIBIA
    talus_in_calcn = T["talus_r"] - T["calcn_r"]
    head_c, head_r = talar_head(o_tal)
    k_head_c, _ = talar_head(k_tal)
    k_head_mapped = s_t * R_t @ k_head_c + t_t
    print(f"# talar head (ours, sphere fit): centre {np.round(head_c * 1e3, 2)} mm talus frame, "
          f"radius {head_r * 1e3:.1f} mm; KUL head mapped: {np.round(k_head_mapped * 1e3, 2)} mm")
    print(f"HEAD_CENTRE_TALUS = ({head_c[0]:.6f}, {head_c[1]:.6f}, {head_c[2]:.6f})")
    print(f"OBLIQUE_AXIS = ({obl[0]:.5f}, {obl[1]:.5f}, {obl[2]:.5f})")
    print(f"FRONT_TO_BACK_AXIS = ({ap[0]:.5f}, {ap[1]:.5f}, {ap[2]:.5f})")

    print("KUL_PATHS = {   # body, point (m) in that body's frame: calcn_r / talus_r / midfoot (calcn_r frame)")
    for lig in osim.iter("Ligament"):
        name = lig.get("name").strip()
        if name not in KUL_LIGAMENTS:
            continue
        pts = []
        for pp in lig.iter("PathPoint"):
            body = pp.find("body").text.strip()
            p = np.array([float(c) for c in pp.find("location").text.split()])
            if body == "talus_r":
                q = s_t * R_t @ p + t_t
                q = o_tal[np.argmin(((o_tal - q) ** 2).sum(1))]
                pts.append(("talus_r", q))
            else:
                q = s_f * R_f @ (p + to_calcn[body]) + t_f
                tgt = o_cal if body == "calcn_r" else o_rest
                q = tgt[np.argmin(((tgt - q) ** 2).sum(1))]
                pts.append(("calcn_r" if body == "calcn_r" else "midfoot", q))
        rest = float(lig.find("resting_length").text)
        txt = ", ".join(f'("{b}", ({q[0]:.5f}, {q[1]:.5f}, {q[2]:.5f}))' for b, q in pts)
        print(f'    "{name}": [{txt}],  # KUL resting length {rest * 1e3:.1f} mm')
    print("}")
    _ = talus_in_calcn
    if facets_out is not None:
        facets(kul, osim, o_cal, facets_out)


if __name__ == "__main__":
    args = sys.argv[1:]
    out = None
    if "--facets" in args:
        i = args.index("--facets")
        out = Path(args[i + 1])
        del args[i:i + 2]
    main(Path(args[0]), out)
