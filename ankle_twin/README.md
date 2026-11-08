# Ankle–foot model (right side, MuJoCo)

A standalone shank–ankle–foot model, extracted from
[MS-Human-700](../MS-Human-700) and extended with the main ankle ligaments. It is the first building block of the LigaSys ankle digital twin.

**Branch: `ligasys-sprain`.** This branch models the inversion-sprain mechanism, to study how a RiteFoot device could intervene. It accepts disclosed, fitted parameters where the anatomy is not yet modelled. Currently that is one: the subtalar inversion stiffness, fitted to Pellegrini 2016 (see "Subtalar ligaments"). The `research-twin` branch develops a research-grade model without such fits.

<img src="docs/ankle_foot_r.png" width="300" alt="Right shank, ankle and foot with muscle paths">
<img src="docs/ligaments_lateral.png" width="300" alt="Lateral view: ATFL, CFL and PTFL in blue">
<img src="docs/ligaments_medial.png" width="300" alt="Medial view: deltoid bands in blue">

## What it is

A bench-top rig. The femur is welded in place and the knee is locked at a chosen angle (0° by default). The foot hangs free. It is driven by the 11 muscles that cross the ankle and restrained by 8 ligament bundles.

| Part | Contents |
|---|---|
| Bodies | `femur_r` (fixed), `tibia_r` (tibia + fibula), `talus_r`, `calcn_r`, `toes_r` |
| Joints | `ankle_angle_r` (talocrural flexion, −39° … +30°, + = dorsiflexion) about a moving axis, via the coupled `ankle_axis_tilt_r` (see "Ankle axis"); `subtalar_angle_r` (±19.5° bony stop, + = inversion; resisted in inversion by a **fitted** 12 N·m/rad stiffness, see "Subtalar ligaments"); `mtp_angle_r` (±29°); knee DOF plus 5 coupled translation/rotation DOFs |
| Muscles | gastrocnemius med/lat, soleus, tibialis posterior, FDL, FHL, peroneus longus/brevis, tibialis anterior, EDL, EHL |
| Muscle model | MuJoCo Hill-type muscle, **rigid tendon**, parameters unchanged from MS-Human-700 (Rajagopal-lineage values) |
| Ligaments | ATFL (superior and inferior bands), CFL, PTFL; deltoid: tibionavicular, tibiospring, tibiocalcaneal, deep anterior and deep posterior tibiotalar; subtalar: cervical, interosseous talocalcaneal |
| Ligament model | straight-line spatial tendon with a **tension-only spring** (MuJoCo `springlength` deadband) plus light damping; linear, or an exponential toe-region curve built from parallel springs |

Geometry, joint axes, muscle paths and muscle strengths are copied **unchanged** from MS-Human-700. The six bone meshes are copied into `model/meshes/`, so the model folder is self-contained.

## Ligaments

All ligament data lives in [ligaments.py](ligaments.py), each value tagged with its source. Slack length is the model length at the ankle angle where the ligament is reported to become taut.

| Ligament | Force–strain | Taut from | Sources |
|---|---|---|---|
| ATFL, superior band | linear 22.35 N/mm | 16.2° PF | **half** of Rochelle 2020's whole-ATFL stiffness (split assumed); onset Ozeki 2002 |
| ATFL, inferior band | linear 22.35 N/mm | 0° | **half** of Rochelle 2020 (split assumed); isometric (Vega 2020), zero strain at neutral **assumed** |
| CFL | linear 45.8 N/mm | 17.8° DF | stiffness Rochelle 2020; onset Ozeki 2002 |
| PTFL | linear 59.0 N/mm | 18.0° DF | stiffness Rochelle 2020; onset Ozeki 2002 |
| TNL (tibionavicular) | linear 39.1 N/mm | 10° PF | stiffness Siegler 1988 (secondary); onset Takao 2020 |
| TSL (tibiospring) | linear 39.1 N/mm | 15° PF | stiffness **assumed = TNL**; onset Takao 2020 |
| TCL (tibiocalcaneal) | 0.51·(e^(45.99ε) − 1) N | 0° | curve Funk 2000 (secondary); onset Takao 2020 |
| dATTL (deep anterior tibiotalar) | 2.06·(e^(20.11ε) − 1) N | 0° | curve Funk 2000 (secondary); onset **assumed** |
| dPTTL (deep posterior tibiotalar) | 1.34·(e^(28.65ε) − 1) N | 0° | curve Funk 2000 (secondary); onset **assumed** |
| CL (cervical) | linear 63.9 N/mm | subtalar 14° inversion | stiffness and failure (163.5 N) Michels 2022; onset Knudson 1997 (indirect: unloaded lax zone) |
| ITCL (interosseous talocalcaneal) | linear 150 N/mm | subtalar 0° | stiffness, failure (382.5 N) and length (11.0 mm) Michels 2022, tested with the anterior capsular ligament; onset **assumed** |

**Two-band ATFL.** Vega 2020 found two ATFL bands in all 30 fresh-frozen specimens:
- **Superior band:** lengthens from 12.6 mm in DF to 19.2 mm in PF. The model's ATFL already behaved this way, so it keeps Ozeki's 16° PF onset.
- **Inferior band:** isometric at 10.6 mm. It shares its fibular origin with the CFL, to which it is joined by arciform fibres. In the model it runs from the CFL's fibular site to the lateral talar vertex below the superior insertion whose length is closest to 10.6 mm, and it is taut from neutral.

Without it, the ATFL was slack at neutral and cutting it changed nothing, where Rasmussen 1982 measured internal rotation rising from 7° to 18°. Rochelle 2020 tested the whole ATFL, and Vega's Table 1 (band widths) was not available, so stiffness and failure load are split equally between the bands. The arciform coupling to the CFL is not modelled.

**Subtalar ligaments.** Before these were added, every inversion test ended on MS-Human-700's hard ±19.5° subtalar stop, which is now kept as the bony limit. Sources and gaps:
- **Stiffness, failure loads, elongation at failure and lengths** are from Michels 2022 (accepted manuscript, 18 pairs).
- **CL insertion:** the calcaneal vertex whose length is closest to Poonja 2017's mean height of 11.1 mm. The model length is 12.3 mm at neutral.
- **CL onset, 14° inversion:** Knudson 1997's unloaded cadaver feet rotate 13–14 ± 7° into supination under about 0.2 N·m before reaching a stable zone, so the subtalar ligaments are slack over that range. Their screw axis is within about 13° of the model's hinge. At this onset the CL is 15.8 mm long, within 1 SD of Michels' 13.9 ± 2.1 mm measured at maximal passive tension. Taut from neutral (the previous assumption), the CL failed first in every sprain load case, at about 12° and 4–8 N·m: its failure load sits only 2.6 mm (13°) past its onset, and onsets up to 9° did not change the failure torque.
- **ITCL insertion:** the vertex closest to Michels' 11.0 mm (ACaL–ITCL complex); taut from neutral (assumed).
- **ITCL direction:** in this geometry it lengthens in eversion, because MS-Human-700's subtalar axis runs about 11 mm above the tarsal canal, through the talar body. Knudson 1997 found that cutting it adds about 4° of supination and no pronation, i.e. the opposite direction; Tochigi 2004 also puts it in inversion; older descriptions put it in eversion. Moving the axis through the canal was tried: it doubles the CL's moment arm and makes it fail even earlier.

**Subtalar inversion stiffness (fitted).** With the subtalar ligaments slack to 14°, the hinge ran to its 19.5° stop at 4 N·m, far laxer than Pellegrini 2016's feet, which carried a 36 N axial load. A real subtalar joint stiffens under axial load as its facets seat. The 1-DOF hinge has no subtalar contact, so a one-sided rotational spring stands in for that and the capsule: a fixed tendon on `subtalar_angle_r` with a deadband, active in inversion from neutral.
- **Value:** 12 N·m/rad (0.21 N·m/°). It is the single parameter **fitted** to Pellegrini's Table II: subtalar and ankle inversion at four flexion angles on both 6-DOF builds, 16 values, minimum summed squared z-score over a 6–14 N·m/rad grid. Onsets of 0° and 1.5° fit equally, so the onset stays at neutral.
- **Consequence:** Pellegrini is calibration on this branch, not validation; its checks now guard the fit.
- **Why not a rigid stop:** a hard stop at 7–13° fits the magnitude equally well. But it holds the joint still, so cutting the CFL changes nothing and Pellegrini's flexion trend is lost. The spring keeps both: subtalar inversion rises from 3.9° at 10° DF to 7.7° at 20° PF (Pellegrini 3.8 → 10.8°), and cutting the CFL adds 4.4° (Pellegrini 5.7–6.2°).
- **Limits:** the stiffness does not change with axial load as the real joint's would, and it is identical in every build, including the hinge builds.
- **The CL now never becomes taut** in the intact joint under these loads. The spring and the CFL hold the subtalar joint below its 14° onset.

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
| Ligament springs are tension-only | matches k·max(0, L−L0) to 4e-15 N·m, including the one-sided subtalar inversion stiffness |
| Exponential spring stacks | follow the published Funk curves within 5–14% over 0.5–10% strain |
| Ligament lengths at neutral | TSL, TCL, dATTL, dPTTL equal Campbell 2014; TNL −6%; CFL and PTFL match Ozeki 2002; ATFL −29% |
| Ligament length trends | ATFL, TNL, TSL, dATTL lengthen in plantarflexion; PTFL in dorsiflexion; CFL and CL in inversion; TNL, TSL, TCL in eversion (ITCL not asserted; see "Subtalar ligaments") |
| Passive behaviour | the hanging foot rests at about 14° PF instead of on the −39° stop; ligaments resist both end ranges (2.7 N·m at 25° PF, 0.9 N·m at 25° DF) |
| Bahr 1997 benchmark (6-DOF builds) | ATFL and CFL force increases under the talar tilt (5.7 N·m) and drawer (80 N) tests at 10° DF – 20° PF; flexion trends asserted, magnitudes reported (see the 6-DOF section) |
| ATFL bands | superior band lengthens 4.8–5.8 mm from 20° DF to 30° PF; inferior band changes ≤ 1.2 mm (Vega 2020: 6.6 mm and isometric) |
| Rasmussen 1982 benchmark (6-DOF builds) | talar tilt and internal rotation at 1.5 N·m after serial ATFL/CFL/PTFL cuts. Intact tilt in range; tilt grows with each cut; cutting the ATFL raises internal rotation 6.5° → 17.6° (Rasmussen 7° → 18°) |
| Pellegrini 2016 benchmark (6-DOF builds) | **calibration on this branch** (the subtalar inversion stiffness is fitted to it). Subtalar inversion at neutral, ankle share, CFL vs CL + ITCL sectioning: all pass |
| Sprain order (6-DOF builds) | quasi-static supination (inversion + internal rotation) ramped 1 N·m at a time at 30, 20 and 10° PF; each ligament is cut when it reaches its reported failure load. An ATFL band must fail first. Passes on `inman`; KNOWN at 30° PF on `--axis original`, where holding the ankle there already loads the TNL past its failure load |
| Colville 1990 benchmark (6-DOF builds) | lateral-ligament length change with flexion, 3 N·m inversion and 3 N·m internal rotation, subtalar locked. All pass on the Inman axis; two KNOWN on `--axis original` (DF subluxation) |
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
  | Anterior drawer of the lateral talus at 150 N, intact → ATFL (both bands) cut | 6.5 → 13.1 mm | 7.7 → 18.3 mm |
  | Talar tilt under 3 N·m inversion, intact → ATFL cut → ATFL+CFL cut | 8.2° → 9.0° → 33.1° | 11.5° → 13.9° → 33.0° |
  | Anterior drawer with 700 N axial compression | 1.6 mm | 1.6 mm |

  Cutting the ATFL also brings 20–30° of internal rotation (anterolateral rotatory instability). Drawer is scored at the ATFL's talar insertion because the talar centre barely moves once the talus rotates. As Bahr 1997 found, cutting the ATFL alone barely changes talar tilt at neutral, while also cutting the CFL increases it sharply.

**Bahr 1997 benchmark.** Increase in ligament force (N) with the talocrural flexion held at each angle, `inman` axis. Bahr values are mean ± SD (n = 8):

| Test | Ankle | ATFL model | ATFL Bahr | CFL model | CFL Bahr |
|---|---|---|---|---|---|
| Talar tilt, 5.7 N·m supination | 10° DF | 20 | 26 ± 11 | 205 | 123 ± 54 |
| | 0° | 67 | 36 ± 17 | 203 | 81 ± 45 |
| | 10° PF | 93 | 64 ± 51 | 189 | 60 ± 31 |
| | 20° PF | 93 | 123 ± 107 | 168 | 32 ± 34 |
| Anterior drawer, 80 N | 10° DF | 100 | 33 ± 17 | 29 | 40 ± 17 |
| | 0° | 90 | 23 ± 14 | 18 | 15 ± 14 |
| | 10° PF | 73 | 34 ± 17 | 5 | 6 ± 8 |
| | 20° PF | 53 | 53 ± 68 | −19 | −3 ± 6 |

The model ATFL is both bands summed, as Bahr's transducer measured the whole ligament. 9 of 16 values fall within Bahr's mean ± 2 SD (12 of 16 on `--axis original`). Both flexion trends are right. What's wrong:
- **The CFL is overloaded under inversion, about 2–5× Bahr.** The two-band ATFL and the compliant mortise now load the ATFL in tilt (20–95 N vs Bahr 26–123 N), but the CFL barely unloads. Rasmussen 1982 and Pellegrini 2016 (below) find the intact tilt and its split with the subtalar joint about right. Bahr's buckle transducers may under-read, so the excess may be partly in the reference. Not resolved.
- **Drawer at neutral and in DF loads the ATFL 3–5× too much.** Bahr let the tibia rotate and the foot plate translate; here the tibia is fixed.

**Rasmussen 1982 benchmark.** 1.5 N·m applied to the foot, ankle and subtalar held at neutral. Rasmussen loaded the talus directly with the subtalar joint nailed (n = 6 per plane, mean):

| | Talar tilt, `inman` | Talar tilt, Rasmussen | Internal rotation, `inman` | Internal rotation, `original` | Internal rotation, Rasmussen |
|---|---|---|---|---|---|
| Intact | 8.7° | 6° (2–10°) | 6.5° | 3.2° | 7° |
| ATFL cut | 9.9° | 11° | 17.6° | 15.1° | 18° |
| + CFL cut | 27.8° | 19° | 18.7° | 18.4° | 21° |
| + PTFL cut | 40.1° | 46° | 18.7° | 20.3° | 26° |

- **Matches:** intact tilt, the growth of tilt with each cut, and the ATFL as the main internal-rotation restraint.
- **Over-reads:** tilt after the CFL cut is too high (27.8° vs 19°).
- **Before the mortise change (below):** internal rotation was locked at 1.7° whatever was cut. The trochlear cylinder jammed between stiff, flat malleolar plates, so the ATFL never engaged.

**Compliant mortise (tuned).** The malleolar contact plates use a softer contact time constant, 0.08 s instead of 0.004 s. It stands in for fibular mobility, cartilage and a talus that is wider at the front than the back, none of which the idealised geometry has.
- **How the value was chosen:** rotation unlocks abruptly between 0.03 and 0.04 s; beyond that it is ligament-limited and nearly insensitive to the value (6.3–6.6° intact from 0.04 to 0.1 s). 0.08 s is twice the threshold, not fitted to Rasmussen.
- **Cost:** malleolar penetration under a 700 N axial load rises to about 0.36 mm (limit 0.5 mm).
- **Side effect:** the two Colville failures at 30° PF cleared, confirming they were caused by the rotation lock.
- **`--axis original` at 20° DF:** with no load the talus sits 3–8 mm anterior, held by the PTFL and dPTTL. This also happened with the stiff mortise, where 3 N·m of inversion dislocated it 12.6 mm. The Colville CFL-with-inversion check is therefore KNOWN on that axis.

**Pellegrini 2016 benchmark (calibration on this branch).** 4 N·m inversion, ankle flexion held, subtalar free. The model's oblique subtalar hinge angle is projected onto the coronal plane (×0.79) to compare with Pellegrini's calcaneal rotation (n = 16, mean ± SD). The subtalar inversion stiffness is fitted to this table:

| Ankle | Subtalar, `inman` | Subtalar, `original` | Subtalar, Pellegrini | Ankle (talar tilt), `inman` | Ankle, `original` | Ankle, Pellegrini |
|---|---|---|---|---|---|---|
| 10° DF | 3.9° | 5.7° | 3.8 ± 1.4° | 12.3° | 7.0° | 6.4 ± 2.2° |
| 0° | 5.0° | 5.5° | 7.2 ± 1.7° | 10.9° | 7.6° | 10.4 ± 3.1° |
| 10° PF | 6.6° | 6.6° | 8.0 ± 2.2° | 8.1° | 5.6° | 13.1 ± 3.6° |
| 20° PF | 7.7° | 8.3° | 10.8 ± 2.6° | 6.3° | 4.2° | 14.7 ± 3.4° |

- **Split at neutral:** the ankle carries 69% (`original`: 58%) against Pellegrini's 59%.
- **Subtalar flexion trend is right:** it rises into PF, as Pellegrini's does.
- **Ankle flexion trend is wrong:** talar tilt falls into PF where Pellegrini's rises. This is on the talocrural side and not addressed by the fit.
- **Sectioning at neutral:** CFL cut +4.4° (`original` +5.1°), CL + ITCL cut +0.0°. Pellegrini: CFL +5.7–6.2°, CL + ITCL not significant. This was KNOWN before the fit, when the CL restrained the hinge from neutral. With the CL slack to 14°, the CFL's 5 mm moment arm about the hinge is enough, but this agreement depends on the fitted stiffness.

**Colville 1990 benchmark.** Strain gauges, subtalar joint screwed, 3 N·m moments; full text read. Model ATFL here is the superior band. The model reproduces:
- ATFL lengthening from 20° DF to 30° PF (+32% here; Colville +15%).
- CFL lengthening with inversion at every angle.

With the compliant mortise, the ATFL also lengthens with inversion and with internal rotation at every angle, as Colville found. It misses:
- **`--axis original`, ATFL at 20° DF and CFL:** KNOWN, caused by the unloaded DF subluxation above.
- **CFL in flexion:** near-isometric (limitation 2), where Colville found it lengthens toward DF.
- **PTFL with inversion:** lengthens 1–8%; Colville found a slight decrease.

**Why it is not the default:**
1. **It does not fix the end-range strain problem.** A cylinder coaxial with the hinge keeps flexion about the same fixed axis. On the `original` axis the deltoid tilts the talus ~6° into inversion at 35° plantarflexion, and ATFL strain rises above the hinge value (33% vs 22%); on the `inman` axis it is 1.5° and 25% (hinge 26%).
2. **The mortise is tuned, not anatomical.**
   - **Medial facet:** a flat plate covering the anatomical medial facet jams a tilting cylinder; tilt was about 1° whatever was cut. Only a thin superior rim gives ligament-controlled tilt.
   - **Malleolar contact compliance:** stiff plates jam axial rotation, so it is set to twice the value that unlocks rotation (see "Compliant mortise").
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

   A single straight line through the footprint centres reproduces the first and so overstates fibre strain. Recruiting fibres in sequence would reconcile them. The ATFL is now two bands (Vega 2020). The superior band still carries the centroid strain, about 24% at 35° PF. Per-band stiffness, from Vega's Table 1 or Dalmau-Pastor 2023, would refine this further. **Use ligament length/strain *trends* and onsets; treat force magnitudes near end range with caution.**
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
8. **Inversion is not yet biofidelic.**
   - **Subtalar joint:** still MS-Human-700's 1-DOF hinge (axis through the talar body, about 11 mm above the tarsal canal). Its stiffness in inversion is a **fitted** spring standing in for facet seating and capsule (see "Subtalar ligaments"). It does not vary with axial load, so a weight-bearing sprain sees the stiffness of Pellegrini's 36 N load. The `research-twin` branch replaces this with subtalar contact.
   - **Sprain order is right in plantarflexed supination.** The ATFL fails first at 8–15 N·m (30–10° PF), then its inferior band, then the TNL. Under pure inversion the CFL fails first by about 1 N·m, at 20° PF as well as neutral. The test is quasi-static: no strain-rate effects, no tearing model, and ligaments fail instantly at their mean failure load.
   - **Talar tilt (6-DOF build) is about right.** Intact tilt is 8.7° at 1.5 N·m (Rasmussen 1982: 2–10°), and the ankle/subtalar split is 69% vs 59% (Pellegrini 2016, here a calibration target). An earlier "too lax" conclusion rested on Choisne 2013 and is withdrawn.
   - **Internal rotation now matches Rasmussen 1982** (6.5° intact, 17.6° with the ATFL cut, against 7° and 18°). This comes from the two-band ATFL plus a **tuned** compliant mortise, which stands in for fibular mobility, cartilage and the wedge-shaped talus. A segmented, anatomically shaped mortise would replace the tuned value.
   - **The CFL is still 2–5× Bahr 1997 in talar-tilt tests,** and tilt after the CFL is cut over-reads Rasmussen (27.8° vs 19°).
   - **The subtalar joint is the wrong kind of joint for the CFL and ITCL.** In a 1-DOF oblique hinge, the near-parallel CFL has only a 5 mm moment arm, and the ITCL works in the wrong direction (Knudson 1997). The CFL now ranks as the main restraint (Pellegrini) only because the CL is slack and the fitted spring carries the rest. A subtalar joint with calcaneal rotation about the AP axis, translation and contact is the structural fix (`research-twin`).
   - **Inferior ATFL band assumptions:** stiffness and failure load are split equally with the superior band, and it has zero strain at neutral. It stretches 1.5–2.5 mm at the extremes of flexion on the moving axis.
   - **Injury-event inversion:** the hindfoot cannot reach it (mean peak about 67° foot-to-shank, Lysdal 2022). The model has no midfoot joint to carry the rest.

The experimental 6-DOF joint (above) addresses limitation 4. It does **not** fix 1 or 2: with idealised congruent surfaces, flexion still happens about the fixed axis.

## References

- Rochelle D. et al. (2020) *J Mech Behav Biomed Mater* 102:103473. doi:10.1016/j.jmbbm.2019.103473: ATFL/CFL/PTFL stiffness and failure loads at 100 %/s.
- Takao M. et al. (2021) *BMC Musculoskelet Disord* 22:558: ATFL and CFL strain changes (onsets superseded by Ozeki 2002).
- Ozeki S. et al. (2002) *Foot Ankle Int* 23(9):825–832: zero-strain onset angles (ATFL 16.2° PF, CFL 17.8° DF, PTFL 18.0° DF), in-situ lengths and length changes (author's thesis summary; journal tables not yet checked).
- Bahr R. et al. (1997) *Acta Orthop Scand* 68(5):435–441: ATFL/CFL force increases in drawer and talar-tilt tests (Tables 2–3), read in full text.
- Colville M.R. et al. (1990) *Am J Sports Med* 18(2):196–200: lateral-ligament strain vs flexion with 3 N·m inversion, eversion and rotation moments, subtalar fixed (Figs 1–3), read in full text.
- Michels F. et al. (2022) *Foot Ankle Surg* 28(7):1014–1020: stiffness, failure load, elongation at failure and in-situ length of the CFL, cervical ligament and ACaL–ITCL complex (accepted manuscript).
- Knudson G.A. et al. (1997) *Acta Orthop Scand* 68(5):442–446: subtalar stable and transition zones under ~0.2 N·m circumferential load, before and after ITCL section (full text).
- Poonja A.J. et al. (2017) *Cureus* 9(6):e1382: cervical ligament dimensions; ITCL fibre direction (full text).
- Vega J. et al. (2020) *Knee Surg Sports Traumatol Arthrosc* 28:8–17: superior ATFL band lengthens 12.6 → 19.2 mm from DF to PF; inferior band isometric (10.6 mm), sharing its fibular origin with the CFL (accepted manuscript read; Table 1 missing).
- Rasmussen O., Tovborg-Jensen I. (1982) *Acta Orthop Scand* 53:155–160: talar tilt and rotation at 1.5 N·m after serial lateral-ligament cuts, read in full text.
- Pellegrini M.J. et al. (2016) *J Bone Joint Surg Am* 98:842–848: subtalar vs ankle share of hindfoot motion at 4 N·m; CFL/CL/ITCL sectioning, read in full text.
- Choisne J. et al. (2013) ISB congress abstract (superseded as a benchmark by Pellegrini 2016): subtalar and talocrural inversion at end range, intact and after CFL / cervical / ITCL sectioning (Table 1).
- Tochigi Y. et al. (2004) *Foot Ankle Int* 25(8):588–596: ITCL keeps the subtalar joint apposed and resists lateral-to-medial drawer (abstract).
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
