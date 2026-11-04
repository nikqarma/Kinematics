# Ankle–foot model (right side, MuJoCo)

A standalone shank–ankle–foot model, extracted from
[MS-Human-700](../MS-Human-700) and extended with the main ankle ligaments. It is the first building block of the LigaSys ankle digital twin.

<img src="docs/ankle_foot_r.png" width="300" alt="Right shank, ankle and foot with muscle paths">
<img src="docs/ligaments_lateral.png" width="300" alt="Lateral view: ATFL, CFL and PTFL in blue">
<img src="docs/ligaments_medial.png" width="300" alt="Medial view: deltoid bands in blue">

## What it is

A bench-top rig. The femur is welded in place and the knee is locked at a chosen angle (0° by default). The foot hangs free. It is driven by the 11 muscles that cross the ankle and restrained by 8 ligament bundles.

| Part | Contents |
|---|---|
| Bodies | `femur_r` (fixed), `tibia_r` (tibia + fibula), `talus_r`, `calcn_r`, `toes_r` |
| Joints | `ankle_angle_r` (talocrural hinge, oblique axis, −39° … +30°, + = dorsiflexion), `subtalar_angle_r` (±19.5°, + = inversion), `mtp_angle_r` (±29°); knee DOF plus 5 coupled translation/rotation DOFs |
| Muscles | gastrocnemius med/lat, soleus, tibialis posterior, FDL, FHL, peroneus longus/brevis, tibialis anterior, EDL, EHL |
| Muscle model | MuJoCo Hill-type muscle, **rigid tendon**, parameters unchanged from MS-Human-700 (Rajagopal-lineage values) |
| Ligaments | ATFL, CFL, PTFL; deltoid: tibionavicular, tibiospring, tibiocalcaneal, deep anterior and deep posterior tibiotalar |
| Ligament model | straight-line spatial tendon with a **tension-only linear spring** (MuJoCo `springlength` deadband) plus light damping |

Geometry, joint axes, muscle paths and muscle strengths are copied **unchanged** from MS-Human-700. The six bone meshes are copied into `model/meshes/`, so the model folder is self-contained.

## Ligaments

All ligament data lives in [ligaments.py](ligaments.py), each value tagged with its source. Attachments are not hand-typed coordinates. Each one is found on the bone meshes by an explicit landmark rule, for example "anterior fibular border, 13.8 mm proximal to the malleolar tip". Slack length is the model length at the ankle angle where the ligament is reported to become taut.

| Ligament | Stiffness (N/mm) | Taut from | Confidence |
|---|---|---|---|
| ATFL | 44.7 | 0° (neutral) | measured: stiffness Rochelle 2020, onset Takao 2021 |
| CFL | 45.8 | 20° PF and 5° DF | measured: stiffness Rochelle 2020, onset Takao 2021 |
| PTFL | 59.0 | 10° DF | stiffness measured (Rochelle 2020); onset **assumed** |
| Deltoid (5 bands) | 49.8 | 0° (neutral) | **placeholder**: no verified per-band stiffness found; value is the mean of the lateral three; onsets assumed |

Attachment landmarks use footprint distances from Clanton 2015 (lateral) and a radiographic deltoid study (PubMed 26443536). For both, I only had secondary summaries; the original papers still need checking.

## Use

```bash
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python build_ankle_model.py              # knee locked at 0°, with ligaments
.venv\Scripts\python build_ankle_model.py --knee-angle 30
.venv\Scripts\python build_ankle_model.py --free-knee
.venv\Scripts\python build_ankle_model.py --no-ligaments
.venv\Scripts\python verify_ankle_model.py
```

Experimental 6-DOF talocrural joint (see below):

```bash
.venv\Scripts\python build_ankle_model.py --six-dof --out model_6dof
.venv\Scripts\python verify_ankle_model.py model_6dof\ankle_foot_r.xml
```

[bench.py](bench.py) is a small test rig for either build. It can hold the ankle and subtalar angles the way an examiner's hands would, apply heel, sole and foot loads, cut ligaments, and read out talar motion, ligament strain and contact force.

To view the model, drag `model/ankle_foot_r.xml` into MuJoCo's `simulate` viewer (`python -m mujoco.viewer`). Ligament sites are in site group 4.

`model/ankle_foot_r.xml` is generated. Change `build_ankle_model.py` or `ligaments.py` and rebuild instead of editing the XML.

## Verification (`verify_ankle_model.py`)

| Check | Result |
|---|---|
| Muscle–tendon lengths vs. the full 700-muscle model, 420 poses | identical (max diff 4e-16 m) |
| Muscle moment arms at neutral | triceps surae −41 to −42 mm, tibialis anterior +43 mm; peroneals evert, tibialis posterior/FDL/FHL invert |
| Ligament springs are tension-only | matches k·max(0, L−L0) to 4e-15 N·m |
| Ligament length trends | ATFL, TNL, dATTL lengthen in plantarflexion; PTFL, dPTTL in dorsiflexion; CFL in inversion; TNL, TSL, TCL in eversion |
| Passive behaviour | the unloaded foot now rests at −0.9° instead of on the −39° stop; ligaments resist both end ranges |
| Forward dynamics | stable; knee lock holds; soleus plantarflexes, tibialis anterior dorsiflexes |

## Experimental: 6-DOF talocrural joint (`--six-dof`)

<img src="docs/talocrural_6dof.png" width="300" alt="Idealised talocrural joint: cylindrical talar trochlea (yellow) under plafond plates, lateral malleolar plate, thin medial rim, ligaments in blue">

**What changes.** The talus gains 3 translations, a tilt (+ = inversion) and an axial rotation (+ = internal), named `talus_tx/ty/tz_r`, `talus_tilt_r` and `talus_rot_r`. None of these has a spring of its own. They are restrained only by articular contact, the ligaments and the muscles.

**Why the contact is idealised.** The MS-Human-700 meshes are visualisation meshes:
- The talus sits 6–10 mm inside the plafond and overlaps the fibula.
- MuJoCo collides meshes as convex hulls, which would fill the mortise.

So [talocrural.py](talocrural.py) builds the joint from primitives around the model's own ankle axis:
- a cylindrical trochlea;
- 10 plafond plates on a 20 mm arc (radius taken from the tibia mesh, ±45°);
- a full-height lateral malleolar plate;
- a thin medial rim.

At zero offsets the joint is exactly congruent, so it reproduces the hinge. Contact is frictionless. Under 700 N the penetration is 0.03 mm.

**What it does well** (asserted in the verification):
- The talus stays seated unloaded under gravity, under 700 N axial load, and under forefoot pushes and pulls.
- Sectioning behaves in the right direction:
  - cutting the ATFL increases anterior drawer (2.2 → 7.5 mm at 150 N, coupled with ~20° internal rotation);
  - talar tilt under 3 N·m inversion grows 6.4° → 7.1° → 10.2° as the ATFL, then the CFL, are cut;
  - axial compression nearly abolishes drawer (2.2 → 0.07 mm).

**Why it is not the default:**
1. **It does not fix the end-range strain problem.** A cylinder coaxial with the hinge keeps flexion about the same fixed axis. With the talus free to tilt, the placeholder deltoid bands pull it into ~15° of inversion and 5 mm medially at 35° plantarflexion, and ATFL strain gets *worse* (74% vs 42%).
2. **The medial facet is tuned, not anatomical.** A flat plate covering the anatomical medial facet jams a tilting cylinder: tilt was ~1° whatever was cut. Only a thin superior rim gives ligament-controlled tilt.
3. **The magnitudes are unvalidated.** Drawer and tilt values have not been compared with clinical or cadaver data, and they depend on the placeholder deltoid.
4. **It is not robust to non-physical loads.** A pure generalized torque on `ankle_angle_r` with no joint compression subluxes the talus. Apply loads as forces (muscles, sole or heel forces), which also compress the joint.
5. **The extra DOFs carry assumed damping.** 200 N·s/m and 1 N·m·s/rad suppress an overshoot instability; this is not taken from literature.

Real articular surfaces (segmented cartilage, contact on non-convex geometry) and verified deltoid properties would fix 1–3.

## Known limitations: read before using ligament forces

1. **Ligament strains at end range are too high.** On the fixed-axis hinge, several ligaments reach non-physiological strain:
   - 35° plantarflexion: ATFL ≈ 42%, dATTL ≈ 70%.
   - 25° dorsiflexion: PTFL ≈ 13%.

   For comparison, measured strain changes are under ~10%. At full plantarflexion the model ATFL carries about its reported failure load (~260 N). The cause is the joint, not the attachments: in a real ankle the talus also translates and rotates, guided by these ligaments, which keeps them near-isometric. **Use ligament length/strain *trends* and onset; don't use force magnitudes near end range yet.**
2. **CFL dorsiflexion branch is missing.** The CFL is nearly isometric in flexion here (−2 mm/rad). It tightens in plantarflexion but not in dorsiflexion (Takao 2021 reports both).
3. **The deltoid is placeholder.** Its stiffness and onset angles are unsourced, yet the deltoid bands, with their long levers, supply ~90% of the passive plantarflexion moment (20.3 of 22.3 N·m at 25° PF; tibionavicular alone ~470 N).
4. **The talocrural joint is a 1-DOF hinge** in the default build. Ligaments attached to the talus can't sense inversion or eversion, and there is no anterior drawer or talar tilt. The experimental `--six-dof` build adds these.
5. **The ligament model is simplified.** It is linear (no toe region), has no pre-strain distribution across fibres, and uses straight-line paths with no wrapping over bone.
6. **Rigid tendons and no contact.** There is no Achilles compliance, no cartilage contact and no ground contact (the floor is visual only).
7. Right side only, generic adult anatomy, low-poly meshes (landmark error of a few mm).

The experimental 6-DOF joint (above) addresses limitation 4. It does **not** fix 1 or 2: with idealised congruent surfaces, flexion still happens about the fixed axis. Fixing them needs real articular geometry and better-founded ligament data, in particular the deltoid, slack lengths and toe region.

## References

- Rochelle D. et al. (2020) *J Mech Behav Biomed Mater* 102:103473. doi:10.1016/j.jmbbm.2019.103473: ATFL/CFL/PTFL stiffness and failure loads at 100 %/s.
- Takao M. et al. (2021) *BMC Musculoskelet Disord* 22:558: ATFL and CFL strain-onset angles.
- Clanton T.O. et al. (2015) *J Bone Joint Surg Am*: lateral ligament footprint distances (via secondary summary).
- Radiographic identification of the deltoid ligament complex, PubMed 26443536 (via secondary summary).

## Licence

Model data derived from MS-Human-700 (© LNS Group, Apache-2.0). Cite Zuo et al., ICRA 2024 (see `../MS-Human-700/README.md`).
