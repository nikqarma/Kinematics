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
    "Boonstra2015": "Radiographic identification of the deltoid ligament complex of the medial ankle "
                    "(PubMed 26443536), distances as summarised in search abstracts - original paper "
                    "not yet checked",
}

# Placeholder stiffness for the deltoid bands: mean of the three lateral
# ligaments in Rochelle2020. No verified per-band deltoid stiffness was found.
DELTOID_PLACEHOLDER_K = (44.7 + 45.8 + 59.0) / 3 * 1000  # N/m


@dataclass(frozen=True)
class Ligament:
    name: str
    full_name: str
    body1: str               # proximal attachment body
    body2: str               # distal attachment body
    stiffness: float         # N/m, linear region
    stiffness_source: str
    # Ankle angle(s) (+ dorsiflexion) at which the ligament is reported to become
    # taut. The slack length is the longest model length over these angles, so
    # the ligament is slack between them and taut beyond the first one reached.
    taut_ankle_deg: tuple[float, ...]
    taut_source: str
    failure_load: float | None = None  # N
    notes: str = ""


LIGAMENTS = [
    # ---- lateral collateral complex --------------------------------------
    Ligament("ATFL_r", "anterior talofibular", "tibia_r", "talus_r",
             44.7e3, "measured: Rochelle2020", (0.0,), "measured: Takao2021 (tenses from 0 deg PF)",
             failure_load=263.6),
    Ligament("CFL_r", "calcaneofibular", "tibia_r", "calcn_r",
             45.8e3, "measured: Rochelle2020", (-20.0, 5.0),
             "measured: Takao2021 (biphasic: tenses from 20 deg PF and from ~5 deg DF)",
             failure_load=367.8,
             notes="nearly isometric over ankle flexion in this geometry; the straight-line path "
                   "on a 1-DOF hinge reproduces the PF branch but not DF tensioning"),
    Ligament("PTFL_r", "posterior talofibular", "tibia_r", "talus_r",
             59.0e3, "measured: Rochelle2020", (10.0,),
             "qualitative: taut in dorsiflexion; 10 deg onset assumed",
             failure_load=351.4),
    # ---- deltoid complex -------------------------------------------------
    Ligament("TNL_r", "tibionavicular (superficial deltoid)", "tibia_r", "calcn_r",
             DELTOID_PLACEHOLDER_K, "placeholder", (0.0,),
             "qualitative: taut in plantarflexion; onset at neutral assumed"),
    Ligament("TSL_r", "tibiospring (superficial deltoid)", "tibia_r", "calcn_r",
             DELTOID_PLACEHOLDER_K, "placeholder", (0.0,),
             "qualitative: restrains eversion; onset at neutral assumed",
             notes="inserts on the spring ligament, which is part of the rigid calcn_r body here"),
    Ligament("TCL_r", "tibiocalcaneal (superficial deltoid)", "tibia_r", "calcn_r",
             DELTOID_PLACEHOLDER_K, "placeholder", (0.0,),
             "qualitative: reported to tense from 0 deg DF (abstract only); onset at neutral"),
    Ligament("dATTL_r", "deep anterior tibiotalar", "tibia_r", "talus_r",
             DELTOID_PLACEHOLDER_K, "placeholder", (0.0,),
             "qualitative: taut in plantarflexion; onset at neutral assumed"),
    Ligament("dPTTL_r", "deep posterior tibiotalar", "tibia_r", "talus_r",
             DELTOID_PLACEHOLDER_K, "placeholder", (0.0,),
             "qualitative: taut in dorsiflexion; onset at neutral assumed"),
]

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
    sustentaculum = _pick(foot, foot_band(0.045, 0.060) & (foot[:, y] > 0.020) & (foot[:, y] < 0.033),
                          -foot[:, z])
    # Navicular tuberosity: most medial midfoot point.
    navicular_tub = _pick(foot, foot_band(0.070, 0.100), -foot[:, z])

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
        # Deltoid: anterior colliculus (~12 mm above the tip, Boonstra2015) to
        # the navicular tuberosity.
        "TNL_r": (mm_band(0.012, tib[:, x]), navicular_tub),
        # Anterior colliculus to the spring ligament, taken as the midpoint of
        # the sustentaculum and the navicular tuberosity (the spring ligament
        # spans these two).
        "TSL_r": (mm_band(0.010, tib[:, x] - tib[:, z]), 0.5 * (sustentaculum + navicular_tub)),
        # Medial malleolus to the sustentaculum tali.
        "TCL_r": (mm_band(0.004, -tib[:, z]), sustentaculum),
        # Deep anterior tibiotalar: anterior colliculus to the medial talar neck.
        "dATTL_r": (mm_band(0.006, tib[:, x]), _pick(tal, tal_med, tal[:, x])),
        # Deep posterior tibiotalar: posterior colliculus, ~8 mm posterior-
        # superior of the tip (Boonstra2015), to the posteromedial talus.
        "dPTTL_r": (mm_band(0.008, -tib[:, x], zmax=-0.015), _pick(tal, tal_med, -tal[:, x])),
    }
    return pts
