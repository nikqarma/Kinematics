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
| Joints | `ankle_angle_r` (talocrural flexion, −39° … +30°, + = dorsiflexion) about a moving axis, via the coupled `ankle_axis_tilt_r` (see "Ankle axis"); `subtalar_angle_r` (±19.5°, + = inversion); `mtp_angle_r` (±29°); knee DOF plus 5 coupled translation/rotation DOFs |
| Muscles | gastrocnemius med/lat, soleus, tibialis posterior, FDL, FHL, peroneus longus/brevis, tibialis anterior, EDL, EHL |
| Muscle model | MuJoCo Hill-type muscle, **rigid tendon**, parameters unchanged from MS-Human-700 (Rajagopal-lineage values) |
| Ligaments | ATFL, CFL, PTFL; deltoid: tibionavicular, tibiospring, tibiocalcaneal, deep anterior and deep posterior tibiotalar |
| Ligament model | straight-line spatial tendon with a **tension-only spring** (MuJoCo `springlength` deadband) plus light damping; linear, or an exponential toe-region curve built from parallel springs |

Geometry, joint axes, muscle paths and muscle strengths are copied **unchanged** from MS-Human-700. The six bone meshes are copied into `model/meshes/`, so the model folder is self-contained.

## Ligaments

All ligament data lives in [ligaments.py](ligaments.py), each value tagged with its source. Slack length is the model length at the ankle angle where the ligament is reported to become taut.

| Ligament | Force–strain | Taut from | Sources |
|---|---|---|---|
| ATFL | linear 44.7 N/mm | 16.2° PF | stiffness Rochelle 2020; onset Ozeki 2002 |
| CFL | linear 45.8 N/mm | 17.8° DF | stiffness Rochelle 2020; onset Ozeki 2002 |
| PTFL | linear 59.0 N/mm | 18.0° DF | stiffness Rochelle 2020; onset Ozeki 2002 |
| TNL (tibionavicular) | linear 39.1 N/mm | 10° PF | stiffness Siegler 1988 (secondary); onset Takao 2020 |
| TSL (tibiospring) | linear 39.1 N/mm | 15° PF | stiffness **assumed = TNL**; onset Takao 2020 |
| TCL (tibiocalcaneal) | 0.51·(e^(45.99ε) − 1) N | 0° | curve Funk 2000 (secondary); onset Takao 2020 |
| dATTL (deep anterior tibiotalar) | 2.06·(e^(20.11ε) − 1) N | 0° | curve Funk 2000 (secondary); onset **assumed** |
| dPTTL (deep posterior tibiotalar) | 1.34·(e^(28.65ε) − 1) N | 0° | curve Funk 2000 (secondary); onset **assumed** |

**Lateral onsets.** Ozeki et al. 2002 calibrated each strain gauge with a tensile test of its own bone–ligament–bone unit, so their onsets are true zero-strain angles. They replace Takao 2021 (ATFL from 0°, CFL biphasic), whose strain changes had no zero-strain reference. Takeuchi 2021 also has the ATFL beginning to tense near 20° PF. With these onsets every lateral ligament is slack between about 16° PF and 18° DF.

**Exponential curves.** Each is a stack of parallel tension-only springs on the same path, with deadbands starting at 0, 2, 4, 6, 8 and 10% strain. The stack follows the published curve to within 5–14% up to 10% strain and continues linearly beyond. This keeps the model pure MuJoCo, with no callbacks, so it still runs on GPU.

**Secondary sources.** "Secondary" means the value was read from a later paper that cites the original:
- Funk's curve fits come from Table 1 of Xu et al. 2019.
- The tibionavicular 39.1 N/mm comes from both Shin et al. 2012 and Xu et al. 2019, each citing Siegler.

The original tables still need checking. The tibiospring stiffness is the one remaining assumption. Siegler 1988 reports the tibiospring among the strongest bands, so 39.1 N/mm is probably low.

**Attachments.**
- **Lateral ligaments:** found on the bone meshes by landmark rules (e.g. "anterior fibular border, 13.8 mm proximal to the malleolar tip", Clanton 2015 via secondary summary). The coarse talus mesh does not resolve the lateral tubercle, so the PTFL insertion is the posterior-process vertex whose neutral length matches Ozeki 2002 (23.7 mm; the old rule gave 14.6 mm). Neutral lengths vs Ozeki: CFL −1%, PTFL 0%, ATFL **−29%** (14.1 vs 19.8 mm; de Asla 2009 measured 16.3 mm in vivo).
- **Deltoid:** placed from Campbell et al. 2014, Table I, read in full. Origins are the published offsets from the distal centre of the intercollicular groove. Insertions follow the published length and orientation, so TSL, TCL, dATTL and dPTTL reproduce Campbell's lengths exactly at neutral.
- **Tibionavicular exception:** its orientation row in Campbell's table is internally inconsistent (the direction components don't form a unit vector), so its insertion uses the navicular landmark instead. Its length is 31.5 mm vs the published 33.6 mm.
- **Mesh mismatch:** deltoid attachment points sit 4–9 mm from the nearest mesh vertex, a mismatch between published anatomy and the coarse MS-Human-700 meshes.

## Use

```bash
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python build_ankle_model.py              # moving ankle axis, knee locked at 0°, with ligaments
.venv\Scripts\python build_ankle_model.py --knee-angle 30
.venv\Scripts\python build_ankle_model.py --free-knee
.venv\Scripts\python build_ankle_model.py --no-ligaments
.venv\Scripts\python verify_ankle_model.py
```

Other ankle axes and the experimental 6-DOF joint (described below):

```bash
.venv\Scripts\python build_ankle_model.py --swing-deg 20 --out model_swing20     # larger axis swing
.venv\Scripts\python build_ankle_model.py --axis inman --out model_inman         # fixed Inman axis
.venv\Scripts\python build_ankle_model.py --axis original --out model_original   # MS-Human-700's own axis
.venv\Scripts\python build_ankle_model.py --six-dof --out model_6dof             # 6-DOF joint, fixed Inman axis
.venv\Scripts\python verify_ankle_model.py model_inman\ankle_foot_r.xml
```

The 6-DOF joint needs a fixed axis (its trochlea is a cylinder about it), so `--six-dof` defaults to `--axis inman` and refuses `--axis moving`.

[bench.py](bench.py) is a small test rig for either build. It can hold the ankle and subtalar angles the way an examiner's hands would, apply heel, sole and foot loads, cut ligaments, and read out talar motion, ligament strain and contact force.

To view the model, drag `model/ankle_foot_r.xml` into MuJoCo's `simulate` viewer (`python -m mujoco.viewer`). Ligament sites are in site group 4.

`model/ankle_foot_r.xml` is generated. Change `build_ankle_model.py` or `ligaments.py` and rebuild instead of editing the XML.

## Verification (`verify_ankle_model.py`)

| Check | Result |
|---|---|
| Muscle–tendon lengths vs. the full 700-muscle model | identical (max diff 4e-16 m); over 420 poses with `--axis original`, over the 60 neutral-ankle poses otherwise (the muscles rotate about a different axis by design) |
| Muscle moment arms at neutral | printed against Klein 1996 with an RMS difference; signs checked. With the Inman-based axes, peroneus brevis is reported as a **KNOWN** discrepancy (see below) instead of failing the run |
| Moving axis (default build) | instantaneous axis inclination 11.3° → 18.1° → 22.7° from 35° PF through neutral to 25° DF; equals the Inman axis at neutral |
| Ligament springs are tension-only | matches k·max(0, L−L0) to 4e-15 N·m |
| Exponential spring stacks | follow the published Funk curves within 5–14% over 0.5–10% strain |
| Ligament lengths at neutral | TSL, TCL, dATTL, dPTTL equal Campbell 2014; TNL −6%; CFL and PTFL match Ozeki 2002; ATFL −29% |
| Ligament length trends | ATFL, TNL, TSL, dATTL lengthen in plantarflexion; PTFL in dorsiflexion; CFL in inversion; TNL, TSL, TCL in eversion |
| Passive behaviour | the hanging foot rests at about 14° PF instead of on the −39° stop; ligaments resist both end ranges (2.7 N·m at 25° PF, 0.9 N·m at 25° DF) |
| Bahr 1997 benchmark (6-DOF builds) | ATFL and CFL force increases under the talar tilt (5.7 N·m) and drawer (80 N) tests at 10° DF – 20° PF; flexion trends asserted, magnitudes reported (see the 6-DOF section) |
| Forward dynamics | stable; knee lock holds; soleus plantarflexes, tibialis anterior dorsiflexes |

## Ankle axis (`--axis moving | inman | original`)

[ankle_axis.py](ankle_axis.py) offers three talocrural axes. The neutral pose is identical in all three; only how the talus rotates changes.

- **`moving` (default):** the Inman axis at neutral, with a coronal-plane inclination that changes continuously with flexion. The lateral end is lower in dorsiflexion than in plantarflexion, as Lundberg et al. 1989 found (quasi-static, 7 of 8 subjects).
  - **Mechanism:** a second hinge, `ankle_axis_tilt_r`, points inferiorly, perpendicular to the axis, through the same centre. It is coupled to the flexion angle by `tilt = c·flexion²`, the same polynomial-equality mechanism MS-Human-700 uses for its knee. Two hinges in series add their angular velocities, so the instantaneous axis tilts linearly with flexion. It is pure MuJoCo, so it still runs on GPU.
  - **Swing size:** a total of 14.4° across the flexion range (`--swing-deg`) gives 11.3° at 35° PF, 18.1° at neutral and 22.7° at 25° DF. External rotation dips by 2–4° at both ends as a second-order effect.
  - **Where 14.4° comes from:** it is **illustrative**. It is the difference between morphology-predicted dorsiflexion and plantarflexion axes for one participant in Wolf et al. 2022 (Fig. 5); Lundberg's own magnitudes were not available to me.
  - **Conflicting walking data:** Wolf et al. 2022, using bone pins during walking, found the **opposite** direction in 4 of 5 participants. Lundberg's pattern is quasi-static, so treat the moving axis as a quasi-static model.
- **`inman`:** the line through points 5 mm distal to the medial malleolar tip, and 3 mm distal + 8 mm anterior to the lateral tip (Inman 1976, secondary), on this model's own meshes. It passes through the midpoint of the tips, consistent with Lundberg 1989.
- **`original`:** MS-Human-700's own hinge (from the OpenSim gait models). It passes 2.5 mm from the midpoint of the tips but 13.8 mm posterior of the medial tip.

| | `original` | `inman` | `moving` (default) |
|---|---|---|---|
| External rotation / frontal inclination at neutral | 6.1° / 10.1° | 22.5° / 18.1° | 22.5° / 18.1° (inclination 11.3° → 22.7° over the range) |
| Muscle moment arms vs Klein 1996 (RMS) | 8.5 mm | 8.9 mm | 8.9 mm (same as `inman` at neutral) |
| Peroneus brevis at neutral | −6.0 mm (plantarflexor, as measured) | **+1.3 mm (weak dorsiflexor; Klein −9.9 mm)** | same as `inman` |
| Tibialis anterior / posterior moment arm (Klein 32.0 / −8.0) | 42.6 / −8.9 mm | 30.4 / −19.0 mm | same as `inman` |
| Ligaments over failure load within 25° PF – 20° DF | TNL at 25° PF (231 N vs 120 N) | none | none |
| Strain at 35° PF: ATFL / TNL / dATTL / dPTTL / TCL | 22 / 27 / 82 / 41 / 19% | 26 / 12 / 30 / 4 / 0% | 24 / 13 / 35 / 7 / 0% |
| PTFL strain at 25° DF | 2% | 4% | 5% |
| Ligaments that should tighten in DF but don't | CFL, TCL, dPTTL | TCL, dPTTL (near-isometric, < 1 mm/rad) | same as `inman` |
| Passive ligament moment at 25° PF | 12.8 N·m | 2.5 N·m | 2.7 N·m |

**Reading it.**
- **Ligaments favour the Inman-based axes.** In the normal walking range nothing exceeds its failure load, and the medial bands become near-isometric.
- **Moving vs fixed Inman.** Within the walking range the moving axis is nearly identical to fixed Inman: the coupled tilt is only ~1° at 25° PF. Toward the end of range it trades ATFL strain (26 → 24%) for deep anterior tibiotalar strain (30 → 35%).
- **Muscles are a wash.** Overall fit to Klein is similar, but tibialis anterior improves while tibialis posterior and the peroneals get worse.
- **Peroneus brevis is a known discrepancy.** Its sign flip is a real disagreement with Klein. The verification prints it as **KNOWN** for the Inman-based builds rather than failing the run.
- **No version brings end-range ATFL strain under ~20%.** Limitation 1 explains why part of this gap is about how strain is defined, not a model error.

## Experimental: 6-DOF talocrural joint (`--six-dof`)

<img src="docs/talocrural_6dof.png" width="300" alt="Idealised talocrural joint: cylindrical talar trochlea (yellow) under plafond plates, lateral malleolar plate, thin medial rim, ligaments in blue">

**What changes.** The talus gains 3 translations, a tilt (+ = inversion) and an axial rotation (+ = internal), named `talus_tx/ty/tz_r`, `talus_tilt_r` and `talus_rot_r`. None of these has a spring of its own. They are restrained only by articular contact, the ligaments and the muscles.

**Why the contact is idealised.** The MS-Human-700 meshes are visualisation meshes:
- The talus sits 6–10 mm inside the plafond and overlaps the fibula.
- MuJoCo collides meshes as convex hulls, which would fill the mortise.

So [talocrural.py](talocrural.py) builds the joint from primitives around the model's own ankle axis:
- a cylindrical trochlea;
- 10 plafond plates on an arc about the chosen ankle axis (radius measured from the tibia mesh about that axis: 20.7 mm for `original`, 22.1 mm for `inman`; ±45°);
- a full-height lateral malleolar plate;
- a thin medial rim.

At zero offsets the joint is exactly congruent, so it reproduces the hinge. Contact is frictionless. Under 700 N the penetration is 0.03 mm.

**What it does well** (asserted in the verification):
- The talus stays seated under 700 N axial load and under forefoot pushes and pulls. Unloaded under gravity it drifts 0.6 mm medially (reported as **KNOWN**). With Ozeki's onsets the lateral ligaments are slack in the hanging foot, while the deep anterior deltoid, whose onset at neutral is assumed, is not.
- Sectioning behaves in the right direction (values for each fixed axis; `inman` is the default with `--six-dof`):

  | Test | `--axis original` | `inman` (default) |
  |---|---|---|
  | Anterior drawer of the lateral talus at 150 N, intact → ATFL cut | 7.0 → 13.3 mm | 8.4 → 18.6 mm |
  | Talar tilt under 3 N·m inversion, intact → ATFL cut → ATFL+CFL cut | 6.5° → 6.5° → 31.6° | 12.2° → 13.2° → 32.6° |
  | Anterior drawer with 700 N axial compression | 1.6 mm | 1.6 mm |

  Cutting the ATFL also brings 20–30° of internal rotation (anterolateral rotatory instability). Drawer is scored at the ATFL's talar insertion because the talar centre barely moves once the talus rotates. As Bahr 1997 found, cutting the ATFL alone barely changes talar tilt at neutral, while also cutting the CFL increases it sharply.

**Bahr 1997 benchmark.** Increase in ligament force (N) with the talocrural flexion held at each angle, `inman` axis. Bahr values are mean ± SD (n = 8):

| Test | Ankle | ATFL model | ATFL Bahr | CFL model | CFL Bahr |
|---|---|---|---|---|---|
| Talar tilt, 5.7 N·m supination | 10° DF | 0 | 26 ± 11 | 214 | 123 ± 54 |
| | 0° | 0 | 36 ± 17 | 238 | 81 ± 45 |
| | 10° PF | 47 | 64 ± 51 | 223 | 60 ± 31 |
| | 20° PF | 87 | 123 ± 107 | 174 | 32 ± 34 |
| Anterior drawer, 80 N | 10° DF | 100 | 33 ± 17 | 26 | 40 ± 17 |
| | 0° | 112 | 23 ± 14 | 37 | 15 ± 14 |
| | 10° PF | 43 | 34 ± 17 | 26 | 6 ± 8 |
| | 20° PF | 44 | 53 ± 68 | 11 | −3 ± 6 |

7 of 16 values fall within Bahr's mean ± 2 SD (10 of 16 on `--axis original`). Both flexion trends are right: ATFL load rises and CFL load falls from DF to PF. What's wrong:
- **The CFL is overloaded under inversion, about 2–5× Bahr.** In every tilt test the subtalar joint sits at its hard ±19.5° range limit, and the CFL is the only ligament that spans it. Real subtalar inversion is shared with the interosseous and cervical talocalcaneal ligaments, which the model lacks. A sweep of the ATFL and CFL onsets across their published ranges (ATFL 0° to 16° PF, CFL 5° to 24° DF) left the CFL at about 165–240 N in every case. **This is the main obstacle to a biofidelic inversion sprain** (limitation 8).
- **The ATFL takes no tilt load at neutral or in DF**, where Bahr measured 26–36 N. With Takao's 0° onset it would, but only by compensating for the missing subtalar restraint, so Ozeki's measured onset is kept.
- **Drawer at neutral and in DF loads the ATFL 3–5× too much.** The rig differs from Bahr's: Bahr let the tibia rotate and the foot plate translate, while here the tibia is fixed.

**Why it is not the default:**
1. **It does not fix the end-range strain problem.** A cylinder coaxial with the hinge keeps flexion about the same fixed axis. On the `original` axis the deltoid tilts the talus ~6° into inversion at 35° plantarflexion, and ATFL strain rises above the hinge value (33% vs 22%); on the `inman` axis it is 1.5° and 25% (hinge 26%).
2. **The medial facet is tuned, not anatomical.** A flat plate covering the anatomical medial facet jams a tilting cylinder: tilt was ~1° whatever was cut. Only a thin superior rim gives ligament-controlled tilt.
3. **The magnitudes are only partly validated.** Ligament forces are benchmarked against Bahr 1997 (above), with mixed agreement. Talar displacements have not been compared with cadaver data; the intact drawer (8 mm at 150 N) may be large for a healthy ankle.
4. **It is not robust to non-physical loads.** A pure generalized torque on `ankle_angle_r` with no joint compression subluxes the talus. Apply loads as forces (muscles, sole or heel forces), which also compress the joint.
5. **The extra DOFs carry assumed damping.** 200 N·s/m and 1 N·m·s/rad suppress an overshoot instability; this is not taken from literature.

Real articular surfaces (segmented cartilage, contact on non-convex geometry) would fix 1–3.

## Known limitations: read before using ligament forces

1. **Ligament strains at end range are still too high** (default moving axis).
   - **At 35° plantarflexion:** ATFL ≈ 24%, dATTL ≈ 35%, TNL ≈ 13%.
   - **At 25° dorsiflexion:** PTFL ≈ 5%.
   - **Forces:** at full plantarflexion the TNL exceeds its reported failure load. Within 25° PF – 20° DF no ligament does. (With `--axis original`, the TNL already exceeds its failure load at 25° PF.)

   **Two kinds of measurement disagree.**
   - **Attachment-to-attachment length (de Asla 2009, in vivo):** the ATFL lengthens about 7 mm from full DF to full PF. The model matches this (7.4 mm).
   - **Midsubstance strain gauges (Ozeki 2002; Renström 1988):** only 1.6 mm (ATFL) and 1.2 mm (PTFL) over the whole range.

   A single straight line through the footprint centres reproduces the first and so overstates fibre strain. Recruiting fibres in sequence, with superior and inferior ATFL bands each having its own onset, would reconcile them. That is deferred until per-band data are in hand (Colville 1990 and the 2023 fascicle study). **Use ligament length/strain *trends* and onsets; treat force magnitudes near end range with caution.**
2. **Structures that should tighten in dorsiflexion:** with the Inman-based axes, TCL (Takao 2020) and dPTTL (anatomy) are near-isometric (< 1 mm/rad) rather than tightening. With `--axis original`, the CFL also slackens. The verification prints this as a NOTE.
3. **Axis choice drives most of 1 and 2.** On the original axis (13.8 mm posterior of the medial malleolar tip), the short medial bands lie *anterior* of the axis and are stretched hard in plantarflexion.
   - **The Inman-based axes fix the medial side**, at the cost of different muscle moment arms: peroneus brevis flips sign, a KNOWN discrepancy.
   - **The moving axis adds only the coronal tilt Lundberg describes.** It doesn't resolve the ATFL/PTFL end-range strain. That likely needs a richer axis motion (translation, transverse rotation) or subject-specific geometry. The experimental 6-DOF joint does not help with this.
4. **The talocrural joint is a hinge** (with a moving axis) in the default build. Ligaments attached to the talus can't sense inversion or eversion, and there is no anterior drawer or talar tilt. The experimental `--six-dof` build adds these, on a fixed axis.
5. **The ligament model is simplified.**
   - The lateral ligaments and TNL/TSL are linear, with no toe region (TCL, dATTL and dPTTL have Funk's exponential toe region).
   - There is no pre-strain distribution across fibres.
   - Paths are straight lines with no wrapping over bone.
   - The superficial posterior tibiotalar band, present in 79% of Campbell's specimens, is not modelled.
6. **Rigid tendons and no contact.** There is no Achilles compliance, no cartilage contact and no ground contact (the floor is visual only).
7. Right side only, generic adult anatomy, low-poly meshes (landmark error of a few mm).
8. **The subtalar joint is not fit for inversion sprain.** It is MS-Human-700's 1-DOF hinge with a hard ±19.5° range limit and no subtalar ligaments (interosseous talocalcaneal, cervical). The CFL therefore carries all subtalar inversion load, 2–5× Bahr 1997's values, and the foot cannot reach injury-event inversion (mean peak about 67°, Lysdal 2022). Fixing this is the next step for sprain biofidelity.

The experimental 6-DOF joint (above) addresses limitation 4. It does **not** fix 1 or 2: with idealised congruent surfaces, flexion still happens about the fixed axis.

## References

- Rochelle D. et al. (2020) *J Mech Behav Biomed Mater* 102:103473. doi:10.1016/j.jmbbm.2019.103473: ATFL/CFL/PTFL stiffness and failure loads at 100 %/s.
- Takao M. et al. (2021) *BMC Musculoskelet Disord* 22:558: ATFL and CFL strain changes (onsets superseded by Ozeki 2002).
- Ozeki S. et al. (2002) *Foot Ankle Int* 23(9):825–832: zero-strain onset angles (ATFL 16.2° PF, CFL 17.8° DF, PTFL 18.0° DF), in-situ lengths and length changes (author's thesis summary; journal tables not yet checked).
- Bahr R. et al. (1997) *Acta Orthop Scand* 68(5):435–441: ATFL/CFL force increases in drawer and talar-tilt tests (Tables 2–3), read in full text.
- de Asla R.J. et al. (2009) *J Orthop Surg Res* 4:7: in-vivo ATFL/CFL attachment-to-attachment lengths at five positions, read in full text.
- Takeuchi Y. et al. (2021) *BMC Musculoskelet Disord* 22:208: ATFL strain probe; tension begins near 20° PF, peaks in inversion.
- Lysdal F.G. et al. (2022) *Sports Biomech*: pooled kinematics of 24 recorded ankle sprains (abstract).
- Takao M. et al. (2020) *BMC Musculoskelet Disord* 21:289: superficial deltoid strain onsets (TNL 10° PF, TSL 15° PF, TCL 0° into DF), read in full text.
- Campbell K.J. et al. (2014) *J Bone Joint Surg Am* 96(8):e62: deltoid band footprints, orientations and lengths (Table I), read in full text.
- Funk J.R. et al. (2000) *J Biomech Eng* 122:15–22: exponential force–strain fits for TCL, anterior and posterior tibiotalar (via Table 1 of Xu C. et al. 2019, *J Orthop Surg Res* 14:114).
- Siegler S., Block J., Schneck C.D. (1988) *Foot Ankle* 8:234–242: tibionavicular stiffness 39.1 N/mm, failure 120 N (via Shin et al. 2012 and Xu et al. 2019).
- Clanton T.O. et al. (2015) *J Bone Joint Surg Am*: lateral ligament footprint distances (via secondary summary).
- Inman V.T. (1976) *The Joints of the Ankle*: empirical axis 5 mm distal to the medial malleolar tip; 3 mm distal and 8 mm anterior to the lateral tip (via a functional-anatomy text).
- Lundberg A. et al. (1989) *J Bone Joint Surg Br* 71(1):94–99: axes lie close to the midpoint of the line between the malleolar tips, change through the range of motion, are oblique downward and laterally in dorsiflexion, and differ between individuals (abstract).
- Wolf P. et al. (2022) *Sci Rep* 12:13957: bone-pin walking data; the dorsiflexion/plantarflexion inclination difference runs opposite to Lundberg in 4 of 5 participants; source of the illustrative 14.4° swing (Fig. 5, morphology-based axes for one participant).
- Klein P., Mattys S., Rooze M. (1996) *J Biomech* 29:21–30: in vitro ankle and subtalar muscle moment arms (via Brookbush Institute review).

## Licence

Model data derived from MS-Human-700 (© LNS Group, Apache-2.0). Cite Zuo et al., ICRA 2024 (see `../MS-Human-700/README.md`).
