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

Usage: python tools/kul_midfoot_register.py <kul_footmodel folder>
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


def describe(u):
    return (f"elevation {math.degrees(math.asin(u[1])):+.1f} deg, "
            f"medial {math.degrees(math.atan2(-u[2], u[0])):+.1f} deg")


def main(kul: Path) -> None:
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


if __name__ == "__main__":
    main(Path(sys.argv[1]))
