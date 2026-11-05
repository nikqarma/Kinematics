"""Ankle ligament definitions for the MS-Human-700 ankle-foot model.

Attachment sites are not hand-typed coordinates. Each one is found on the
MS-Human-700 bone meshes by an explicit landmark rule (see ``attachments``),
anchored where possible to published distances from bony landmarks. The
meshes are low-poly (~100-2000 triangles per bone), so expect a few
millimetres of error. Treat these as a generic-anatomy first pass, not
subject-specific insertions.

Every number below carries a ``source`` tag:
    measured     - taken from a cited experimental study
    qualitative  - direction of action from anatomy; numeric value assumed
    placeholder  - no verified source yet; replace before drawing conclusions

Frames follow OpenSim/MS-Human-700: x anterior, y superior, z lateral
(right side). Distances are in metres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# --------------------------------------------------------------------------
# References
# --------------------------------------------------------------------------
REFS = {
    "Rochelle2020": "Rochelle D. et al. (2020) Mechanical characterisation of the lateral collateral "
                    "ligament complex of the ankle at realistic sprain-like strain rates. J Mech Behav "
                    "Biomed Mater 102:103473. doi:10.1016/j.jmbbm.2019.103473",
    "Takao2021": "Takao M. et al. (2021) Strain patterns in normal anterior talofibular and "
                 "calcaneofibular ligaments and after anatomical reconstruction. BMC Musculoskelet "
                 "Disord 22:558",
    "Clanton2015": "Clanton T.O. et al. (2015) Radiographic identification of the lateral ankle "
                   "ligaments (J Bone Joint Surg Am), footprint distances as summarised by ACFAS "
                   "Scientific Literature Review, June 2015 - original paper not yet checked",
    "Campbell2014": "Campbell K.J. et al. (2014) The ligament anatomy of the deltoid complex of the ankle: "
                    "a qualitative and quantitative anatomical study. J Bone Joint Surg Am 96(8):e62 "
                    "(n=14; Table I footprint offsets and lengths, read in full text)",
    "Takao2020": "Takao M. et al. (2020) Strain pattern of each ligamentous band of the superficial "
                 "deltoid ligament: a cadaver study. BMC Musculoskelet Disord 21:289 (n=6, full text)",
    "Funk2000": "Funk J.R. et al. (2000) Linear and quasi-linear viscoelastic characterization of ankle "
                "ligaments. J Biomech Eng 122:15-22; force-strain fits T = a(exp(b*eps)-1) as tabulated "
                "in Xu C. et al. (2019) J Orthop Surg Res 14:114 (PMC6489249), Table 1 - Funk's own "
                "tables not yet checked",
    "Siegler1988": "Siegler S., Block J., Schneck C.D. (1988) The mechanical characteristics of the "
                   "collateral ligaments of the human ankle joint. Foot Ankle 8:234-242; tibionavicular "
                   "stiffness 39.1 N/mm and failure load 120 N as cited by Shin J. et al. (2012) Ann "
                   "Biomed Eng (automotive FE foot model) and Xu 2019 - original tables not yet checked",
}

# Piecewise-linear representation of exponential force-strain curves: the
# curve is matched at these strains (secant slopes between them) and
# continued linearly with its tangent slope beyond the last one. Each extra
# segment is a parallel tension-only spring on the same path whose deadband
# starts at that strain, so the stack is still native MuJoCo.
EXP_BREAKS = (0.0, 0.02, 0.04, 0.06, 0.08, 0.10)


@dataclass(frozen=True)
class Ligament:
    name: str
    full_name: str
    body1: str               # proximal attachment body
    body2: str               # distal attachment body
    # Force-strain curve: ("linear", k [N/m]) or ("exp", a [N], b [-]) with
    # T(eps) = a * (exp(b * eps) - 1), eps relative to the slack length.
    curve: tuple
    stiffness_source: str
    # Ankle angle(s) (+ dorsiflexion) at which the ligament is reported to become
    # taut. The slack length is the longest model length over these angles, so
    # the ligament is slack between them and taut beyond the first one reached.
    taut_ankle_deg: tuple[float, ...]
    taut_source: str
    failure_load: float | None = None  # N
    notes: str = ""
    reference_length: float | None = None  # m, published centre-to-centre length at neutral


TNL_K = 39.1e3  # N/m, Siegler1988 (secondary)

LIGAMENTS = [
    # ---- lateral collateral complex --------------------------------------
    Ligament("ATFL_r", "anterior talofibular", "tibia_r", "talus_r",
             ("linear", 44.7e3), "measured: Rochelle2020", (0.0,), "measured: Takao2021 (tenses from 0 deg PF)",
             failure_load=263.6),
    Ligament("CFL_r", "calcaneofibular", "tibia_r", "calcn_r",
             ("linear", 45.8e3), "measured: Rochelle2020", (-20.0, 5.0),
             "measured: Takao2021 (biphasic: tenses from 20 deg PF and from ~5 deg DF)",
             failure_load=367.8,
             notes="nearly isometric over ankle flexion in this geometry; the straight-line path "
                   "on a 1-DOF hinge reproduces the PF branch but not DF tensioning"),
    Ligament("PTFL_r", "posterior talofibular", "tibia_r", "talus_r",
             ("linear", 59.0e3), "measured: Rochelle2020", (10.0,),
             "qualitative: taut in dorsiflexion; 10 deg onset assumed",
             failure_load=351.4),
    # ---- deltoid complex (attachments: Campbell2014) ----------------------
    Ligament("TNL_r", "tibionavicular (superficial deltoid)", "tibia_r", "calcn_r",
             ("linear", TNL_K), "measured (secondary): Siegler1988", (-10.0,),
             "measured: Takao2020 (tenses from 10 deg PF)",
             failure_load=120.0, reference_length=0.0336),
    Ligament("TSL_r", "tibiospring (superficial deltoid)", "tibia_r", "calcn_r",
             ("linear", TNL_K), "assumed: equal to TNL (no per-band value found; Siegler1988 reports "
                                "the tibiospring among the strongest bands, so this is likely low)",
             (-15.0,), "measured: Takao2020 (tenses from 15 deg PF)",
             reference_length=0.0250,
             notes="inserts on the spring ligament, which is part of the rigid calcn_r body here"),
    Ligament("TCL_r", "tibiocalcaneal (superficial deltoid)", "tibia_r", "calcn_r",
             ("exp", 0.51, 45.99), "measured (secondary): Funk2000", (0.0,),
             "measured: Takao2020 (tenses from 0 deg into DF)",
             reference_length=0.0288),
    Ligament("dATTL_r", "deep anterior tibiotalar", "tibia_r", "talus_r",
             ("exp", 2.06, 20.11), "measured (secondary): Funk2000 'anterior tibiotalar'", (0.0,),
             "qualitative: taut in plantarflexion; onset at neutral assumed",
             reference_length=0.0120),
    Ligament("dPTTL_r", "deep posterior tibiotalar", "tibia_r", "talus_r",
             ("exp", 1.34, 28.65), "measured (secondary): Funk2000 'posterior tibiotalar'", (0.0,),
             "qualitative: taut in dorsiflexion; onset at neutral assumed",
             reference_length=0.0103,
             notes="with Campbell2014 geometry this band lies anterior of the model's ankle axis and "
                   "so slackens in dorsiflexion - the model's axis sits ~7 mm posterior and ~8 mm "
                   "inferior of the intercollicular groove"),
]


def curve_force(lig: Ligament, strain: float) -> float:
    """Tension (N) on the ligament's published force-strain curve; slack length L0 is not needed
    for 'exp' curves, and 'linear' curves are k * strain * L0 so take L0 via linear_force."""
    if lig.curve[0] != "exp":
        raise ValueError("use spring_segments for linear ligaments")
    a, b = lig.curve[1], lig.curve[2]
    return a * (np.exp(b * max(strain, 0.0)) - 1.0)


def spring_segments(lig: Ligament, L0: float) -> list[tuple[float, float]]:
    """Parallel tension-only springs reproducing the ligament curve: [(strain_start, k_increment N/m)].

    Summed, the springs give a force that is piecewise linear in length, matching
    the curve at EXP_BREAKS and continuing with the tangent slope beyond them.
    """
    if lig.curve[0] == "linear":
        return [(0.0, float(lig.curve[1]))]
    a, b = lig.curve[1], lig.curve[2]
    T = lambda e: a * (np.exp(b * e) - 1.0)  # noqa: E731
    slopes = [(T(e1) - T(e0)) / ((e1 - e0) * L0) for e0, e1 in zip(EXP_BREAKS[:-1], EXP_BREAKS[1:])]
    slopes.append(a * b * np.exp(b * EXP_BREAKS[-1]) / L0)
    segs, prev = [], 0.0
    for e, s in zip(EXP_BREAKS, slopes):
        segs.append((e, float(s - prev)))
        prev = s
    return segs

# Viscous damping on every ligament (N*s/m). MuJoCo tendon damping also acts
# while the ligament is slack, so it is kept small; it is mainly there for
# numerical stability. Not from the literature.
LIGAMENT_DAMPING = 100.0


# --------------------------------------------------------------------------
# Mesh landmarks
# --------------------------------------------------------------------------
def load_stl(path: Path) -> np.ndarray:
    """Unique vertices of a binary STL (already in the body frame for these meshes)."""
    data = path.read_bytes()
    n = int.from_bytes(data[80:84], "little")
    tri = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                        count=n, offset=84)
    return np.unique(tri["v"].reshape(-1, 3).astype(float), axis=0)


def _pick(v: np.ndarray, mask: np.ndarray, score: np.ndarray) -> np.ndarray:
    """Vertex with the highest score among those selected by mask."""
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        raise ValueError("landmark rule selected no vertices")
    return v[idx[np.argmax(score[idx])]]


# Body origins in the tibia_r frame at neutral (all joints zero), from
# MS-Human-700 Body_Leg_Foot_r.xml. The builder checks these against the XML.
BODY_ORIGIN_IN_TIBIA = {
    "tibia_r": np.zeros(3),
    "talus_r": np.array([-0.01, -0.4, 0.0]),
    "calcn_r": np.array([-0.01, -0.4, 0.0]) + np.array([-0.04877, -0.04195, 0.00792]),
}

# Campbell2014 Table I, per deltoid band:
#   origin offset from the distal centre of the intercollicular groove (mm; +anterior, +superior, +lateral),
#   orientation origin->insertion (deg; angle to the coronal / transverse / sagittal plane with
#   sign +anterior, +inferior, +lateral), centre-to-centre length (mm).
# The three orientation angles are read as each plane's normal component
# (sin(angle)); for five bands the components form a unit vector to within 6%.
# The tibionavicular row does not (|v| = 1.21), so its insertion uses the
# navicular landmark instead.
CAMPBELL2014 = {
    "TNL_r": dict(origin=(13.8, 4.9, 5.7), orient=(44.7, 77.4, -5.5), length=33.6),
    "TSL_r": dict(origin=(10.8, 6.9, 1.8), orient=(8.6, 75.6, 11.2), length=25.0),
    "TCL_r": dict(origin=(5.5, -0.4, 0.6), orient=(-7.0, 69.1, 20.3), length=28.8),
    "dATTL_r": dict(origin=(9.0, -3.6, 4.9), orient=(22.5, 59.9, 23.8), length=12.0),
    "dPTTL_r": dict(origin=(-0.7, -2.2, 6.4), orient=(8.5, 56.0, 31.9), length=10.3),
}


def campbell_direction(ant_deg: float, inf_deg: float, lat_deg: float) -> tuple[np.ndarray, float]:
    """Unit origin->insertion vector (tibia frame) from Campbell2014 angles, and the pre-normalisation norm."""
    v = np.array([math.sin(math.radians(ant_deg)), -math.sin(math.radians(inf_deg)),
                  math.sin(math.radians(lat_deg))])
    n = float(np.linalg.norm(v))
    return v / n, n


def attachments(mesh_dir: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Return {ligament: (point on body1, point on body2)} in each body's own frame."""
    fib = load_stl(mesh_dir / "r_fibula.stl")   # tibia_r frame
    tib = load_stl(mesh_dir / "r_tibia.stl")    # tibia_r frame
    tal = load_stl(mesh_dir / "r_talus.stl")    # talus_r frame
    foot = load_stl(mesh_dir / "r_foot.stl")    # calcn_r frame
    x, y, z = 0, 1, 2

    # Lateral malleolus: distal tip of the fibula.
    lm_tip = fib[fib[:, y].argmin()]

    def fib_anterior(h: float) -> np.ndarray:
        """Anterior border of the fibula, h metres proximal to the malleolar tip."""
        band = np.abs(fib[:, y] - (lm_tip[y] + h)) < 0.004
        return _pick(fib, band, fib[:, x])

    # Medial malleolus: most distal tibial point on the medial side.
    medial = tib[:, z] < -0.010
    # Deltoid reference point: distal centre of the intercollicular groove
    # (Campbell2014's landmark in its text and figures; its Table I header says
    # "inferior tip of the medial malleolus"). Taken as the midpoint of the
    # anterior and posterior colliculus tips: the most anterior and most
    # posterior of the distal tibial vertices lying medial to the talus's
    # medial wall. (The plain "most distal medial vertex" lands on the plafond
    # corner, inside the talus.)
    talus_medial_wall = tal[:, z].min() + BODY_ORIGIN_IN_TIBIA["talus_r"][z]
    malleolus = tib[:, z] < talus_medial_wall
    mm_lowest = _pick(tib, malleolus, -tib[:, y])
    distal_malleolus = malleolus & (tib[:, y] < mm_lowest[y] + 0.006)
    dcig = 0.5 * (_pick(tib, distal_malleolus, tib[:, x]) + _pick(tib, distal_malleolus, -tib[:, x]))
    mm_tip = _pick(tib, medial, -tib[:, y])

    def mm_band(h: float, score: np.ndarray, zmax: float = -0.010) -> np.ndarray:
        band = (tib[:, z] < zmax) & (np.abs(tib[:, y] - (mm_tip[y] + h)) < 0.004)
        return _pick(tib, band, score)

    # Talus regions.
    tal_lat = tal[:, z] > tal[:, z].max() - 0.006
    tal_med = tal[:, z] < tal[:, z].min() + 0.006

    # Calcaneus/midfoot regions (calcn_r frame; origin near the posterior heel).
    def foot_band(x0: float, x1: float) -> np.ndarray:
        return (foot[:, x] > x0) & (foot[:, x] < x1)

    # Sustentaculum tali: most medial calcaneal point on the shelf directly
    # beneath the talus (talar underside sits at y ~ 0.032 in this frame).
    sust_region = (foot[:, y] > 0.015) & (foot[:, y] < 0.035)
    sustentaculum = _pick(foot, foot_band(0.045, 0.060) & (foot[:, y] > 0.020) & (foot[:, y] < 0.033),
                          -foot[:, z])
    # Posterior point of the sustentaculum: most posterior vertex of the medial
    # shelf (within 6 mm of the sustentaculum's medial extent).
    sust_posterior = _pick(foot, foot_band(0.030, 0.065) & sust_region
                           & (foot[:, z] < sustentaculum[z] + 0.006), -foot[:, x])
    # Navicular tuberosity: most medial midfoot point.
    navicular_tub = _pick(foot, foot_band(0.070, 0.100), -foot[:, z])
    # Talar landmarks: anteromedial corner of the trochlea and the
    # posteromedial talar tubercle.
    tal_upper = tal[:, y] > 0.004
    am_trochlea = _pick(tal, tal_upper, tal[:, x] - tal[:, z])
    pm_tubercle = _pick(tal, tal[:, y] < 0.0, -tal[:, x] - tal[:, z])

    def unit(*v):
        v = np.asarray(v, float)
        return v / np.linalg.norm(v)

    def campbell(name: str) -> tuple[np.ndarray, np.ndarray]:
        """Deltoid origin (tibia frame) and the insertion implied by Campbell2014's orientation
        and length, still in the tibia frame at neutral."""
        row = CAMPBELL2014[name]
        o = dcig + np.array(row["origin"]) / 1000.0
        u, _ = campbell_direction(*row["orient"])
        return o, o + u * row["length"] / 1000.0

    def to_body(p_tibia: np.ndarray, body: str) -> np.ndarray:
        return p_tibia - BODY_ORIGIN_IN_TIBIA[body]

    pts = {
        # ATFL: fibular footprint 13.8 mm from the tip on the anterior border
        # (Clanton2015); talar insertion on the lateral talar body, anterior to
        # the lateral malleolar facet (most anterior vertex of the lateral face).
        "ATFL_r": (fib_anterior(0.0138), _pick(tal, tal_lat, tal[:, x])),
        # CFL: fibular footprint 5.3 mm from the tip on the anterior border
        # (Clanton2015); calcaneal insertion on the lateral calcaneal wall,
        # posterior-inferior to the fibular tip.
        "CFL_r": (fib_anterior(0.0053),
                  _pick(foot, foot_band(0.015, 0.040) & (foot[:, y] < 0.025), foot[:, z])),
        # PTFL: malleolar fossa (posteromedial distal fibula) to the lateral
        # tubercle of the talar posterior process.
        "PTFL_r": (_pick(fib, (fib[:, y] > lm_tip[y] + 0.004) & (fib[:, y] < lm_tip[y] + 0.012),
                         -fib[:, x] - fib[:, z]),
                   _pick(tal, tal[:, z] > 0, -tal[:, x])),
    }
    # ---- Deltoid: origins and insertions from Campbell2014 Table I ----------
    # Origins: Table I offsets from the inferior tip of the medial malleolus.
    # Insertions: origin + published length along the published orientation
    # (this does not depend on the coarse talus/foot meshes) - except the
    # tibionavicular, whose orientation row is internally inconsistent; its
    # navicular footprint is 9.7 mm from the tuberosity along the
    # talonavicular joint line (taken as dorsal) and 3.4 mm distal to it.
    for name in CAMPBELL2014:
        lig = next(L for L in LIGAMENTS if L.name == name)
        o, ins = campbell(name)
        if name == "TNL_r":
            pts[name] = (o, navicular_tub + np.array([0.0034, 0.0097, 0.0]))
        else:
            pts[name] = (o, to_body(ins, lig.body2))
    return pts


def landmark_report(mesh_dir: Path) -> dict[str, float]:
    """Distance (m) from each attachment point to the nearest vertex of its bone mesh."""
    meshes = {"tibia_r": np.vstack([load_stl(mesh_dir / "r_tibia.stl"), load_stl(mesh_dir / "r_fibula.stl")]),
              "talus_r": load_stl(mesh_dir / "r_talus.stl"), "calcn_r": load_stl(mesh_dir / "r_foot.stl")}
    pts = attachments(mesh_dir)
    out = {}
    for lig in LIGAMENTS:
        p1, p2 = pts[lig.name]
        out[lig.name] = max(np.linalg.norm(meshes[lig.body1] - p1, axis=1).min(),
                            np.linalg.norm(meshes[lig.body2] - p2, axis=1).min())
    return out
