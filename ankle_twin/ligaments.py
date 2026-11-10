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
    "Ozeki2002": "Ozeki S. et al. (2002) Simultaneous strain measurement with determination of a zero "
                 "strain reference for the medial and lateral ligaments of the ankle. Foot Ankle Int "
                 "23(9):825-832 (n=12, zero-strain reference n=5); values from the author's thesis summary "
                 "(Hokkaido Univ. 2002) - journal tables not yet checked",
    "Bahr1997": "Bahr R. et al. (1997) Mechanics of the anterior drawer and talar tilt tests. Acta Orthop "
                "Scand 68(5):435-441 (n=8, buckle transducers; Tables 2-3, read in full text)",
    "Michels2022": "Michels F. et al. (2022) The different subtalar ligaments show significant differences in "
                   "their mechanical properties. Foot Ankle Surg 28(7):1014-1020 (18 pairs, bone-ligament-bone "
                   "uniaxial tests; accepted manuscript read: stiffness, failure load, elongation at failure "
                   "CL 3.0 +/- 1.1 mm, ACaL-ITCL 2.6 +/- 0.8 mm; in-situ lengths CL 13.9 +/- 2.1 mm, ACaL-ITCL "
                   "11.0 +/- 2.5 mm, measured at maximal passive tension with the ankle neutral)",
    "Knudson1997": "Knudson G.A., Kitaoka H.B., Lu C.-L., Luo Z.P., An K.-N. (1997) Subtalar joint stability: "
                   "talocalcaneal interosseous ligament function studied in cadaver specimens. Acta Orthop Scand "
                   "68(5):442-446 (n=6, tibia and talus fixed, ~0.2 N*m circumferential load on the calcaneus, "
                   "no axial load; full text)",
    "Poonja2017": "Poonja A.J. et al. (2017) Anatomical study of the cervical and interosseous talocalcaneal "
                  "ligaments of the foot with surgical relevance. Cureus 9(6):e1382 (n=26, formalin-fixed; "
                  "full text)",
    "Choisne2013": "Choisne J., Hoch M., Bawab S., Ringleb S.I. (2013) Effect of a semi-rigid ankle stabilizer "
                   "on a simulated isolated subtalar joint instability. ISB 2013 congress abstract, Table 1 "
                   "(n=9, end-range manual inversion, intact / CFL cut / CFL+cervical+ITCL cut)",
    "Edama2017": "Edama M. et al. (2017) The effects on calcaneofibular ligament function of differences in the "
                 "angle of the calcaneofibular ligament with respect to the long axis of the fibula: a simulation "
                 "study. J Foot Ankle Res 10:60 (81 feet, goniometer at neutral; open access, full text)",
    "Colville1990": "Colville M.R. et al. (1990) Strain measurement in lateral ankle ligaments. Am J Sports "
                    "Med 18(2):196-200 (n=10, subtalar fixed with screws, 3 N*m moments; full text)",
    "Vega2020": "Vega J., Malagelada F., Manzanares Cespedes M.C., Dalmau-Pastor M. (2020) The lateral "
                "fibulotalocalcaneal ligament complex: an ankle stabilizing isometric structure. Knee Surg "
                "Sports Traumatol Arthrosc 28:8-17 (n=30 fresh-frozen; accepted manuscript read, Table 1 missing)",
    "Rasmussen1982": "Rasmussen O., Tovborg-Jensen I. (1982) Mobility of the ankle joint. Acta Orthop Scand "
                     "53:155-160 (n=12, 1.5 N*m on the talus, subtalar fixed; full text)",
    "Pellegrini2016": "Pellegrini M.J. et al. (2016) Systematic quantification of stabilizing effects of subtalar "
                      "joint soft-tissue constraints in a novel cadaveric model. J Bone Joint Surg Am 98:842-848 "
                      "(n=16, 4 N*m, 36 N axial load; full text, appendix tables not seen)",
    "Siegler1988":"Siegler S., Block J., Schneck C.D. (1988) The mechanical characteristics of the "
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
    # Angle(s) of taut_joint (ankle: + dorsiflexion; subtalar: + inversion) at
    # which the ligament is reported to become taut. The slack length is the
    # longest model length over these angles, so the ligament is slack between
    # them and taut beyond the first one reached.
    taut_ankle_deg: tuple[float, ...]
    taut_source: str
    failure_load: float | None = None  # N
    notes: str = ""
    reference_length: float | None = None  # m, published centre-to-centre length at neutral
    reference_source: str = "Campbell2014"
    taut_joint: str = "ankle_angle_r"


TNL_K = 39.1e3  # N/m, Siegler1988 (secondary)

# Angle of the CFL behind the fibular long axis, lateral view, ankle neutral.
# Edama2017 sorted 80 single-band CFLs into 10 deg classes: 20-29 deg 14 feet,
# 30-39 deg 22, 40-49 deg 29 (the modal class), 50-59 deg 15; the mean of the
# class midpoints is 40.6 deg. Ruth 1961 (via Edama2017): 10-45 deg in 75%.
CFL_ANGLE_DEG = 40.0

LIGAMENTS = [
    # ---- lateral collateral complex --------------------------------------
    # Taut onsets: Ozeki2002's zero-strain references (cadaver strain gauges
    # calibrated by a tensile test of each bone-ligament-bone unit). They replace
    # Takao2021's onsets (ATFL 0 deg; CFL biphasic 20 PF / 5 DF), which were
    # strain changes without a zero-strain reference. Takeuchi 2021 (strain
    # probe) also has the ATFL beginning to tense near 20 deg PF.
    # The ATFL is two bands (Vega2020, two-band in 30/30 specimens). ATFL_r is the
    # superior band: it lengthens 6.6 mm from DF to PF (Vega2020: 12.6 -> 19.2 mm),
    # which matches this model's ATFL and Ozeki's 16 deg PF onset. ATFLi_r is the
    # inferior band: isometric (10.6 mm in both PF and DF), sharing its fibular
    # origin with the CFL. Rochelle2020 tested the whole ATFL; with no per-band
    # widths (Vega2020 Table 1 not seen), stiffness and failure load are split
    # equally between the bands - an assumption.
    Ligament("ATFL_r", "anterior talofibular, superior band", "tibia_r", "talus_r",
             ("linear", 22.35e3), "assumed: half of Rochelle2020's whole-ATFL 44.7 N/mm", (-16.2,),
             "measured: Ozeki2002 (zero strain at 16.2 +/- 2.6 deg PF)",
             failure_load=131.8, reference_length=0.0198, reference_source="Ozeki2002",
             notes="model neutral length is shorter than Ozeki2002 (19.8 mm, cadaver caliper) and "
                   "de Asla 2009 (16.3 mm, in vivo); its length change over flexion matches de Asla's "
                   "in-vivo centroid distance and Vega2020's superior band, not Ozeki's midsubstance "
                   "gauge (README). Failure load: half of Rochelle2020's 263.6 N (assumed split)"),
    Ligament("ATFLi_r", "anterior talofibular, inferior band", "tibia_r", "talus_r",
             ("linear", 22.35e3), "assumed: half of Rochelle2020's whole-ATFL 44.7 N/mm", (0.0,),
             "assumed: zero strain at neutral (Vega2020: isometric through flexion, no tension value)",
             failure_load=131.8, reference_length=0.0106, reference_source="Vega2020",
             notes="fibular origin shared with the CFL in Vega2020; here it still uses the older anterior-border "
                   "point, 18.7 mm from the tip (see fib_anterior_band); talar insertion below the superior "
                   "band's. Failure load: half of Rochelle2020's 263.6 N (assumed split)"),
    Ligament("CFL_r", "calcaneofibular", "tibia_r", "calcn_r",
             ("linear", 45.8e3), "measured: Rochelle2020", (17.8,),
             "measured: Ozeki2002 (zero strain at 17.8 +/- 6.4 deg DF)",
             failure_load=367.8, reference_length=0.0299, reference_source="Ozeki2002",
             notes="tightens in dorsiflexion (5.4 mm/rad on the Inman axes), so with Ozeki's onset it is "
                   "about 1.6 mm slack at neutral; loaded by dorsiflexion, subtalar inversion and (6-DOF "
                   "build) talar tilt"),
    Ligament("PTFL_r", "posterior talofibular", "tibia_r", "talus_r",
             ("linear", 59.0e3), "measured: Rochelle2020", (18.0,),
             "measured: Ozeki2002 (zero strain at 18.0 +/- 7.5 deg DF)",
             failure_load=351.4, reference_length=0.0237, reference_source="Ozeki2002"),
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
    # ---- subtalar (sinus tarsi / tarsal canal) ----------------------------
    # Without these, subtalar inversion was stopped only by the CFL and the hard
    # +-19.5 deg joint limit: every inversion test ended on the limit.
    # Cervical taut onset: Knudson1997's unloaded cadaver feet rotate 13-14 +/- 7
    # deg into supination under ~0.2 N*m before reaching a stable zone, so the
    # subtalar ligaments are slack over that range. Their screw axis is within
    # ~13 deg of the model's hinge, so the hinge angle is taken as that rotation.
    # Taut from neutral (the earlier assumption) made the CL fail first in every
    # sprain load case at ~12 deg subtalar inversion and 4-8 N*m (2026-11-07
    # probe): its 163.5 N failure load sits only 2.6 mm (13 deg) past its onset.
    Ligament("CL_r", "cervical (talocalcaneal)", "talus_r", "calcn_r",
             ("linear", 63.9e3), "measured: Michels2022 (63.9 +/- 38.0 N/mm)", (14.0,),
             "measured (indirect): Knudson1997 (supination stable zone at 13-14 +/- 7 deg, ~0.2 N*m)",
             failure_load=163.5, reference_length=0.0111, reference_source="Poonja2017",
             taut_joint="subtalar_angle_r",
             notes="reference length = mean of Poonja2017's anterior (8.3 mm) and posterior (14.0 mm) "
                   "heights of a near-vertical ligament. Its length at the 14 deg onset (15.8 mm) is within 1 SD "
                   "of Michels2022's 13.9 +/- 2.1 mm measured at maximal passive tension"),
    Ligament("ITCL_r", "interosseous talocalcaneal", "talus_r", "calcn_r",
             ("linear", 150.0e3), "measured: Michels2022 (ACaL-ITCL complex, 150 +/- 51 N/mm)", (0.0,),
             "assumed: taut from subtalar neutral (no zero-strain data found)",
             failure_load=382.5, reference_length=0.0110, reference_source="Michels2022",
             taut_joint="subtalar_angle_r",
             notes="Michels2022 tested it together with the anterior capsular ligament (ACaL-ITCL, 11.0 mm). "
                   "With this geometry it lengthens in eversion: the model's subtalar axis runs ~11 mm "
                   "above the tarsal canal, through the talar body. Knudson1997 found that cutting it adds "
                   "~4 deg of supination and no pronation, i.e. the opposite direction; Smith 1958 and "
                   "Kjaersgaard-Andersen 1988 (as cited by Knudson) put it in eversion"),
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

    def fib_anterior_band(h: float) -> np.ndarray:
        """Most anterior fibular vertex in a band h metres above the malleolar tip (ATFL origins only: this
        lands 22.5 mm from the tip, not Clanton2015's 13.8 mm, but with this mesh's talar insertion it gives a
        near-horizontal ATFL of the right length; moving it is a separate, open question)."""
        band = np.abs(fib[:, y] - (lm_tip[y] + h)) < 0.004
        return _pick(fib, band, fib[:, x])

    def fib_anterior(d: float) -> np.ndarray:
        """Point on the anterior border of the lateral malleolus, d metres (straight line) from its tip.

        The border is the antero-inferior outline of the distal fibula in the sagittal plane: the lower
        convex hull of its vertices from the tip forwards. The point is interpolated along that outline,
        so it does not snap to a vertex. (The earlier rule took the most anterior vertex in a height band
        and put the CFL origin 18.7 mm and the ATFL origin 22.5 mm from the tip instead of 5.3 and 13.8;
        the ATFL still uses it, see fib_anterior_band.)
        """
        distal = fib[(fib[:, y] < lm_tip[y] + 0.025) & (fib[:, x] >= lm_tip[x])]
        distal = distal[np.argsort(distal[:, x])]
        hull = [lm_tip]
        for v in distal:
            if v[x] <= hull[-1][x] + 1e-6:
                continue
            while len(hull) >= 2 and ((hull[-1][x] - hull[-2][x]) * (v[y] - hull[-2][y])
                                      - (hull[-1][y] - hull[-2][y]) * (v[x] - hull[-2][x])) <= 0:
                hull.pop()
            hull.append(v)
        for a, b in zip(hull[:-1], hull[1:]):
            da, db = np.linalg.norm(a - lm_tip), np.linalg.norm(b - lm_tip)
            if da <= d <= db:
                # |a + s (b - a) - tip| = d
                u, w = b - a, a - lm_tip
                A, B, C = u @ u, 2 * (u @ w), w @ w - d * d
                s = (-B + math.sqrt(B * B - 4 * A * C)) / (2 * A)
                return a + s * u
        raise ValueError(f"anterior fibular border shorter than {d * 1000:.1f} mm")

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
        "ATFL_r": (fib_anterior_band(0.0138), _pick(tal, tal_lat, tal[:, x])),
        # CFL: fibular footprint 5.3 mm from the tip on the anterior border
        # (Clanton2015). Calcaneal insertion: the vertex of the lateral calcaneal
        # wall nearest the point one in-situ length (Ozeki2002, 29.9 mm) from
        # that origin, along a line CFL_ANGLE_DEG behind the fibular long axis
        # in the sagittal plane (Edama2017). The coarse foot mesh does not
        # resolve the peroneal tubercle, Clanton2015's landmark for it.
        "CFL_r": (fib_anterior(0.0053), None),
    }
    cfl_o = pts["CFL_r"][0]
    fc = fib - fib.mean(axis=0)
    f_axis = np.linalg.eigh(fc.T @ fc)[1][:, -1]
    f_axis = f_axis * np.sign(f_axis[y])                       # fibular long axis, pointing proximally
    back = np.array([-1.0, 0.0, 0.0]) - (np.array([-1.0, 0.0, 0.0]) @ f_axis) * f_axis
    back[z] = 0.0
    back /= np.linalg.norm(back)
    a = math.radians(CFL_ANGLE_DEG)
    cfl_ref = next(L for L in LIGAMENTS if L.name == "CFL_r").reference_length
    target = cfl_o + cfl_ref * (-math.cos(a) * f_axis + math.sin(a) * back) - BODY_ORIGIN_IN_TIBIA["calcn_r"]
    wall = foot[:, z] > target[z] - 0.008                      # lateral wall, not the far side of the heel
    pts["CFL_r"] = (cfl_o, _pick(foot, wall, -np.linalg.norm(foot - target, axis=1)))
    # PTFL: malleolar fossa (posteromedial distal fibula) to the posterior
    # process of the talus. The coarse talus mesh does not resolve the lateral
    # tubercle, so the insertion is the posterior-process vertex (posterior of
    # x = -10 mm and below the talar origin, talus frame) whose neutral length
    # is closest to Ozeki2002's in-situ length (23.7 mm). The previous rule
    # (most posterior vertex of the lateral half) gave 14.6 mm.
    ptfl_o = _pick(fib, (fib[:, y] > lm_tip[y] + 0.004) & (fib[:, y] < lm_tip[y] + 0.012), -fib[:, x] - fib[:, z])
    ptfl_ref = next(L for L in LIGAMENTS if L.name == "PTFL_r").reference_length
    process = (tal[:, x] < -0.010) & (tal[:, y] < 0.0)
    gap = np.linalg.norm(tal + BODY_ORIGIN_IN_TIBIA["talus_r"] - ptfl_o, axis=1)
    pts["PTFL_r"] = (ptfl_o, _pick(tal, process, -np.abs(gap - ptfl_ref)))
    # ATFL inferior band: fibular origin shared with the CFL (Vega2020). The
    # coarse talus mesh has no vertex that is both ~10.6 mm from that origin and
    # isometric, so the insertion is constructed like the deltoid insertions:
    # a band stays isometric over flexion when its talar end lies in the plane
    # through the ankle axis and its origin. The insertion is the point in that
    # plane (landmark-registered Inman axis, used for every build), at the
    # superior band's depth along the axis, Vega2020's 10.6 mm from the origin,
    # on the outer (talar-surface) side. It lands ~4 mm below the superior
    # insertion and ~4 mm from the nearest mesh vertex.
    import ankle_axis as AX  # local import: ankle_axis imports this module
    T = BODY_ORIGIN_IN_TIBIA["talus_r"]
    atfli_o = fib_anterior_band(0.0053)
    c_ax, u_ax = AX.axis("inman", mesh_dir)
    C = c_ax + T
    o_rel = atfli_o - C
    o_rad = o_rel - (o_rel @ u_ax) * u_ax
    r_o, e_rad = float(np.linalg.norm(o_rad)), o_rad / np.linalg.norm(o_rad)
    a_o, a_p = float(o_rel @ u_ax), float((pts["ATFL_r"][1] + T - C) @ u_ax)
    atfli_ref = next(L for L in LIGAMENTS if L.name == "ATFLi_r").reference_length
    radial = r_o + math.sqrt(max(atfli_ref ** 2 - (a_p - a_o) ** 2, 0.0))
    pts["ATFLi_r"] = (atfli_o, C + a_p * u_ax + radial * e_rad - T)
    # ---- Subtalar: sinus tarsi and tarsal canal --------------------------------
    # Talar points in the talus frame; calcaneal candidates are expressed in the
    # talus frame at neutral (calcn origin offset) to measure lengths, then
    # returned in the calcn frame. The talus mesh has ~100 vertices, so these
    # are approximate to a few millimetres.
    calc_in_talus = BODY_ORIGIN_IN_TIBIA["calcn_r"] - BODY_ORIGIN_IN_TIBIA["talus_r"]
    calc_t = foot + calc_in_talus
    ref = {L.name: L.reference_length for L in LIGAMENTS}

    def closest_length(p: np.ndarray, mask: np.ndarray, length: float) -> np.ndarray:
        return _pick(calc_t, mask, -np.abs(np.linalg.norm(calc_t - p, axis=1) - length)) - calc_in_talus

    underside = tal[:, y] < -0.005
    # Cervical: talar origin on the inferolateral talar neck (most lateral
    # underside vertex 8-20 mm anterior of the talar origin); calcaneal
    # insertion on the anterolateral floor of the sinus tarsi - anterior,
    # inferior and lateral of the origin (Poonja2017) - at the vertex whose
    # length is closest to the reference length.
    cl_o = _pick(tal, underside & (tal[:, x] > 0.008) & (tal[:, x] < 0.020), tal[:, z])
    cl_mask = ((calc_t[:, x] > cl_o[x]) & (calc_t[:, x] < cl_o[x] + 0.012) & (calc_t[:, y] < cl_o[y] - 0.003)
               & (calc_t[:, z] > cl_o[z] - 0.004) & (calc_t[:, z] < cl_o[z] + 0.008))
    pts["CL_r"] = (cl_o, closest_length(cl_o, cl_mask, ref["CL_r"]))
    # ITCL: talar origin in the sulcus tali (highest underside vertex under the
    # neck, medial half of the canal); fibres run superomedial to inferolateral
    # (Poonja2017), so the calcaneal insertion is lateral and inferior of it.
    itcl_o = _pick(tal, underside & (tal[:, x] > 0.005) & (tal[:, x] < 0.012) & (np.abs(tal[:, z] + 0.0025) < 0.0075),
                   tal[:, y])
    itcl_mask = ((calc_t[:, x] > itcl_o[x] - 0.004) & (calc_t[:, x] < itcl_o[x] + 0.008)
                 & (calc_t[:, y] < itcl_o[y] - 0.003) & (calc_t[:, z] > itcl_o[z] + 0.002))
    pts["ITCL_r"] = (itcl_o, closest_length(itcl_o, itcl_mask, ref["ITCL_r"]))
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
