# Ankle–foot model (right side, MuJoCo)

A standalone shank–ankle–foot model, extracted from
[MS-Human-700](../MS-Human-700) and extended with the main ankle ligaments. It is the first building block of the LigaSys ankle digital twin.

**Branch: `research-twin`.** This branch develops a research-grade model for publication and open access. Validation benchmarks stay independent of any calibration. Its 6-DOF builds replace the 1-DOF subtalar hinge with a contact joint about an anatomical axis (see "6-DOF subtalar joint"), tested in a reproduction of Pellegrini 2016's rig with nothing fitted.

## Branches

`main` is the **shared core**: the model builder, ligament data, the 6-DOF joint, the bench rig, the verifier and the benchmarks both lines of work use. Two long-lived branches build on it and are never merged back as a whole:
- **`ligasys-sprain`:** the inversion-sprain mechanism and Smart Ligament design. Disclosed, fitted parameters are allowed (currently a subtalar inversion stiffness fitted to Pellegrini 2016) along with the trapdoor rig.
- **`research-twin`:** a research-grade model for publication. No fits used as validation; its 6-DOF builds use a subtalar contact joint in place of the hinge.

**Workflow.**
- **Shared changes:** branch off `main`, PR into `main`, then merge `main` into both long-lived branches.
- **Branch-only changes:** branch off that branch and PR back into it.
- **Never open a PR from `ligasys-sprain` or `research-twin` into `main`.** It would carry branch-only fits into the shared core. Shared work done on a branch is cherry-picked onto a branch off `main`.
- **`docs/papers_needed.md`** is owned by each branch and is not changed on `main`.
- **Generated model XMLs:** on merge conflicts, rebuild them rather than editing.

**What `main` lacks on its own:** the cervical ligament is slack to 14° of subtalar inversion (Knudson 1997), and nothing else stiffens the subtalar hinge in inversion below that. At 4 N·m it sits near its 19.5° stop, so two Pellegrini checks are KNOWN on `main`. Each branch adds the missing stiffness. On this branch that turned out to be a property of the test rig as much as of the joint (see "6-DOF subtalar joint").

<img src="docs/ankle_foot_r.png" width="300" alt="Right shank, ankle and foot with muscle paths">
<img src="docs/ligaments_lateral.png" width="300" alt="Lateral view: ATFL, CFL and PTFL in blue">
<img src="docs/ligaments_medial.png" width="300" alt="Medial view: deltoid bands in blue">

## What it is

A bench-top rig. The femur is welded in place and the knee is locked at a chosen angle (0° by default). The foot hangs free. It is driven by the 11 muscles that cross the ankle and restrained by 8 ligament bundles.

| Part | Contents |
|---|---|
| Bodies | `femur_r` (fixed), `tibia_r` (tibia + fibula), `talus_r`, `calcn_r`, `toes_r` |
| Joints | `ankle_angle_r` (talocrural flexion, −39° … +30°, + = dorsiflexion) about a moving axis, via the coupled `ankle_axis_tilt_r` (see "Ankle axis"); `subtalar_angle_r` (±19.5°, + = inversion; the stop now stands in for the articular and capsular limit, see "Subtalar ligaments"); `mtp_angle_r` (±29°); knee DOF plus 5 coupled translation/rotation DOFs |
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
- **Inferior band:** isometric at 10.6 mm. It shares its fibular origin with the CFL, to which it is joined by arciform fibres. In the model it runs from the anterior fibular border, 18.7 mm from the malleolar tip (the CFL's site before its correction; see "Attachment geometry" for why the ATFL origins have not moved), to the lateral talar vertex below the superior insertion whose length is closest to 10.6 mm, and it is taut from neutral.

Without it, the ATFL was slack at neutral and cutting it changed nothing, where Rasmussen 1982 measured internal rotation rising from 7° to 18°. Rochelle 2020 tested the whole ATFL, and Vega's Table 1 (band widths) was not available, so stiffness and failure load are split equally between the bands. The arciform coupling to the CFL is not modelled.

**Subtalar ligaments.** MS-Human-700's hard ±19.5° subtalar stop is kept as the bony limit. Sources and gaps:
- **Stiffness, failure loads, elongation at failure and lengths** are from Michels 2022 (accepted manuscript, 18 pairs).
- **CL insertion:** the calcaneal vertex whose length is closest to Poonja 2017's mean height of 11.1 mm. The model length is 12.3 mm at neutral.
- **CL onset, 14° inversion:** Knudson 1997's unloaded cadaver feet rotate 13–14 ± 7° into supination under about 0.2 N·m before reaching a stable zone, so the subtalar ligaments are slack over that range. Their screw axis is within about 13° of the model's hinge. At this onset the CL is 15.8 mm long, within 1 SD of Michels' 13.9 ± 2.1 mm measured at maximal passive tension. Taut from neutral (the previous assumption), the CL failed first in every sprain load case, at about 12° and 4–8 N·m: its failure load sits only 2.6 mm (13°) past its onset.
- **ITCL insertion:** the vertex closest to Michels' 11.0 mm (ACaL–ITCL complex); taut from neutral (assumed).
- **ITCL direction:** in this geometry it lengthens in eversion, because MS-Human-700's subtalar axis runs about 11 mm above the tarsal canal, through the talar body. Knudson 1997 found that cutting it adds about 4° of supination and no pronation, i.e. the opposite direction; Tochigi 2004 also puts it in inversion; older descriptions put it in eversion.
- **Subtalar inversion stiffness below 14°:** the 1-DOF hinge has none. The 6-DOF builds on this branch get it from articular contact (see "6-DOF subtalar joint").

**Lateral onsets.** Ozeki et al. 2002 calibrated each strain gauge with a tensile test of its own bone–ligament–bone unit, so their onsets are true zero-strain angles. They replace Takao 2021 (ATFL from 0°, CFL biphasic), whose strain changes had no zero-strain reference. Takeuchi 2021 also has the ATFL beginning to tense near 20° PF. With these onsets every lateral ligament is slack between about 16° PF and 18° DF.

**Exponential curves.** Each is a stack of parallel tension-only springs on the same path, with deadbands starting at 0, 2, 4, 6, 8 and 10% strain. The stack follows the published curve to within 5–14% up to 10% strain and continues linearly beyond. This keeps the model pure MuJoCo, with no callbacks, so it still runs on GPU.

**Secondary sources.** "Secondary" means the value was read from a later paper that cites the original:
- Funk's curve fits come from Table 1 of Xu et al. 2019.
- The tibionavicular 39.1 N/mm comes from both Shin et al. 2012 and Xu et al. 2019, each citing Siegler.

The original tables still need checking. The tibiospring stiffness is the one remaining assumption. Siegler 1988 reports the tibiospring among the strongest bands, so 39.1 N/mm is probably low.

**Attachments.**
- **Lateral ligaments:** found on the bone meshes by landmark rules, anchored to published distances where the mesh allows (Clanton 2015 via secondary summary).
  - **CFL:** the fibular origin is the point on the antero-inferior outline of the lateral malleolus 5.3 mm from its tip (Clanton 2015). The calcaneal insertion is the lateral-wall vertex nearest the point one in-situ length (29.9 mm, Ozeki 2002) from the origin, 40° behind the fibular long axis (Edama 2017: the modal 40–49° class of 80 feet; mean of the class midpoints 40.6°). The result is 31.2 mm long at 42°. It tightens in dorsiflexion (5.4 mm/rad on the Inman axes, 2.2 mm/rad on `original`), as Colville 1990, Ozeki 2002 and Sarrafian describe. The rate is about twice Colville's midsubstance gauge (about 6% strain from 20° PF to 20° DF against the model's 11%). With Ozeki's zero strain at 17.8° DF the CFL is 1.6 mm slack at neutral.
  - **Earlier CFL path:** until November 2026 the origin rule took the most anterior fibular vertex in a height band, which put it 18.7 mm from the tip. The ankle axis then passed through the middle of the ligament, so it was near-isometric over flexion.
  - **ATFL origins still use that band rule** and sit 22.5 mm (superior band) and 18.7 mm (inferior band) from the tip, against Clanton's 13.8 mm and a shared origin with the CFL (Vega 2020). Moving them tilts the ATFL 51° upwards with this mesh's talar insertion, so the talar footprint or the mesh's malleolar tip is also off. Open.
  - **Sensitivity:** whether the CFL tightens in dorsiflexion depends on where the ankle axis exits relative to its origin, within Inman's own spread (8 ± 5 mm anterior of the tip). Edama 2017, simulating an axis through the malleolar tips, found 20–39° ligaments tighten in dorsiflexion and 40–59° ones slacken.
  - **PTFL:** the coarse talus mesh does not resolve the lateral tubercle, so the insertion is the posterior-process vertex whose neutral length matches Ozeki 2002 (23.7 mm; the old rule gave 14.6 mm). Neutral lengths vs Ozeki: CFL +4%, PTFL 0%, ATFL **−29%** (14.1 vs 19.8 mm; de Asla 2009 measured 16.3 mm in vivo).
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
.venv\Scripts\python build_ankle_model.py --six-dof --out model_6dof             # 6-DOF ankle and subtalar joints, fixed Inman axis
.venv\Scripts\python build_ankle_model.py --six-dof --subtalar hinge --out model_6dof_hinge   # 6-DOF ankle, 1-DOF subtalar hinge
.venv\Scripts\python verify_ankle_model.py model_inman\ankle_foot_r.xml
```

The 6-DOF joint needs a fixed axis (its trochlea is a cylinder about it), so `--six-dof` defaults to `--axis inman` and refuses `--axis moving`.

[bench.py](bench.py) is a small test rig for either build. It can hold the ankle and subtalar angles the way an examiner's hands would, apply heel, sole and foot loads, cut ligaments, and read out talar motion, ligament strain and contact force. Holding the subtalar joint on a 6-DOF build fixes all six of its degrees of freedom, as a screw across the joint does. `HindfootRig` in the same file reproduces Pellegrini 2016's cradle rig.

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
| Ligament lengths at neutral | TSL, TCL, dATTL, dPTTL equal Campbell 2014; TNL −6%; PTFL matches Ozeki 2002, CFL +4%; ATFL −29% |
| Ligament length trends | ATFL, TNL, TSL, dATTL lengthen in plantarflexion; PTFL and CFL in dorsiflexion; CL in inversion (the CFL's change in subtalar inversion is reported, not asserted: it depends on the ligament's angle, Edama 2017); TNL, TSL, TCL in eversion (ITCL not asserted; see "Subtalar ligaments") |
| Passive behaviour | the hanging foot rests at about 14° PF instead of on the −39° stop; ligaments resist both end ranges (2.7 N·m at 25° PF, 0.9 N·m at 25° DF) |
| Bahr 1997 benchmark (6-DOF builds) | ATFL and CFL force increases under the talar tilt (5.7 N·m) and drawer (80 N) tests at 10° DF – 20° PF; flexion trends asserted, magnitudes reported (see the 6-DOF section) |
| ATFL bands | superior band lengthens 4.8–5.8 mm from 20° DF to 30° PF; inferior band changes ≤ 1.2 mm (Vega 2020: 6.6 mm and isometric) |
| Rasmussen 1982 benchmark (6-DOF builds) | talar tilt and internal rotation at 1.5 N·m after serial ATFL/CFL/PTFL cuts. Tilt grows with each cut; cutting the ATFL raises internal rotation 9.9° → 24.4° (Rasmussen 7° → 18°). Intact tilt on `inman` is 11.3°, 1.3° over Rasmussen's range (**KNOWN**) |
| Pellegrini 2016 benchmark (6-DOF builds) | in a reproduction of the rig: subtalar and ankle inversion at 4 N·m at four flexion angles, eversion, rotation and 67 N drawer at neutral, and CFL vs CL + ITCL sectioning. Intact inversion within 2 SD, the ankle's share and the CFL as the main restraint in dorsiflexion are asserted. The too-small flexion trend is KNOWN |
| Sprain order (6-DOF builds) | quasi-static supination ramped 1 N·m at a time at 30, 20 and 10° PF, subtalar free; each ligament is cut at its reported failure load. An ATFL band must fail first |
| Stormont 1985 loaded stability (6-DOF builds) | 700 N axial load, 3 N·m inversion, lateral ligaments cut: the talus must stay seated. Passes on both axes with the anatomical mortise (0.1° tilt) |
| Colville 1990 benchmark (6-DOF builds) | lateral-ligament length change with flexion, 3 N·m inversion and 3 N·m internal rotation, subtalar locked. All pass on both axes |
| Astion 1997 benchmark (all builds with ligaments) | subtalar range of motion from full supination to full pronation under 36 N / 11 N tibialis posterior and peroneus brevis pulls, ankle screwed (`bench.TendonPullRig`). Asserted within 2 SD of 20.4 ± 8°: 17.8° on the hinge builds, 30.9° with the 6-DOF contact joint |
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
| Ligaments that should tighten in DF but don't | TCL, dPTTL | TCL, dPTTL (near-isometric, < 1 mm/rad) | same as `inman` |
| Passive ligament moment at 25° PF | 12.8 N·m | 2.5 N·m | 2.7 N·m |

**Reading it.**
- **Ligaments favour the Inman-based axes.** In the normal walking range nothing exceeds its failure load, and the medial bands become near-isometric.
- **Moving vs fixed Inman.** Within the walking range the moving axis is nearly identical to fixed Inman: the coupled tilt is only ~1° at 25° PF. Toward the end of range it trades ATFL strain (26 → 24%) for deep anterior tibiotalar strain (30 → 35%).
- **Muscles are a wash.** Overall fit to Klein is similar, but tibialis anterior improves while tibialis posterior and the peroneals get worse.
- **Peroneus brevis is a known discrepancy.** Its sign flip is a real disagreement with Klein. The verification prints it as **KNOWN** for the Inman-based builds rather than failing the run.
- **No version brings end-range ATFL strain under ~20%.** Limitation 1 explains why part of this gap is about how strain is defined, not a model error.

## Experimental: 6-DOF talocrural joint (`--six-dof`)

<img src="docs/talocrural_6dof.png" width="300" alt="Idealised talocrural joint (earlier version): cylindrical talar trochlea (yellow) under plafond plates, lateral malleolar plate, thin medial rim, ligaments in blue">

**What changes.** The talus gains 3 translations, a tilt (+ = inversion) and an axial rotation (+ = internal), named `talus_tx/ty/tz_r`, `talus_tilt_r` and `talus_rot_r`. None of these has a spring of its own. They are restrained only by articular contact, the ligaments and the muscles. The distal fibula also becomes mobile (below). The image shows the earlier idealised geometry.

**Why the contact is built from primitives.** The MS-Human-700 meshes are visualisation meshes:
- The talus sits 6–10 mm inside the plafond and overlaps the fibula.
- MuJoCo collides meshes as convex hulls, which would fill the mortise.

**Anatomical mortise.** [talocrural.py](talocrural.py) builds the joint around the model's own ankle axis, with three anatomical features that each can be switched off:
- **Trochlea, wider in front.** A body of revolution about the ankle axis (radius measured from the tibia mesh: 20.7 mm for `original`, 22.1 mm for `inman`), 28 mm wide, with its lateral face inclined so the front is wider than the back. Hayes 2006 measured 29.9 ± 2.6 mm anteriorly and 25.2 ± 3.7 mm posteriorly (3D-CT, n = 21) on coronal sections 30° either side of the centre of a 20.7 mm dome, i.e. 20.7 mm apart: a 12.8° taper, all on the lateral side.
- **Pivot about the medial malleolus.** A 6 mm sphere on the talus near the top-medial corner (0.7 R above the axis) bears on a medial malleolar plate covering the anatomical facet. A sphere on a plane allows tilt and axial rotation about its centre but no medial slide. Radius and height are assumptions; at mid-height the joint locked (about 3° of tilt and rotation whatever was cut).
- **Mobile fibula.** The lateral malleolar plate, inclined to match the trochlea, sits on a distal-fibula body (0.25 kg) together with the ATFL, ATFLi, CFL and PTFL origins. It can translate along the ankle axis (100 N/mm) and rotate externally about the vertical (20 N·m/rad). These stiffnesses are assumptions bounded by in-vivo fibular travel (about 1.7 mm side to side and 9° of rotation across activities); syndesmosis stiffness data are still wanted.
- **Plafond:** 10 plates on a ±45° arc about the axis.

At zero offsets the joint is congruent, so it reproduces the hinge. Contact is frictionless, and all plates now use the default stiff cartilage contact (700 N: 0.04 mm penetration). The tuned compliant malleolar plates of the earlier idealised mortise are no longer needed.

**What it does well** (asserted in the verification; `inman` is the default with `--six-dof`):
- **Seating:** the talus stays seated under 700 N axial load (0.3 mm) and, once settled, under forefoot pushes and pulls. During the first second a free foot swings into its ankle and subtalar range stops and the talus moves up to 3.2 mm; that transient is reported, not asserted. At the plantarflexion stop it settles 2.7 mm off centre (`original`: 2.1 mm), both **KNOWN**: the overstretched deltoid (limitation 1) tilts it into inversion and the narrow back of the trochlea gives it room. Unloaded under gravity it moves 1.5 mm (`original`: 1.4 mm), also **KNOWN**: the lateral ligaments are slack in the hanging foot, so the deep anterior deltoid tilts the talus into inversion until the ATFL catches. The model has no capsule or muscle tone to hold it.
- **Loaded stability (Stormont 1985):** with 700 N or 350 N axial load and the lateral ligaments cut, 3 N·m inversion tilts the talus 0.1°. With the idealised mortise it slid 16 mm medially and tipped to 45°.
- **Sectioning** moves in the right direction:

  | Test | `--axis original` | `inman` (default) |
  |---|---|---|
  | Anterior drawer of the lateral talus at 150 N, intact → ATFL (both bands) cut | 8.4 → 16.8 mm | 10.8 → 16.7 mm |
  | Talar tilt under 3 N·m inversion, intact → ATFL cut → ATFL+CFL cut | 10.1° → 16.7° → 32.4° | 13.6° → 18.2° → 36.0° |
  | Anterior drawer with 700 N axial compression | 1.6 mm | 1.8 mm |

**Rasmussen 1982 benchmark.** 1.5 N·m applied to the foot, ankle flexion held at neutral and the subtalar joint fixed. Rasmussen loaded the talus directly with the subtalar joint nailed (n = 6 per plane, mean):

| | Talar tilt, `inman` | Talar tilt, `original` | Talar tilt, Rasmussen | Internal rotation, `inman` | Internal rotation, `original` | Internal rotation, Rasmussen |
|---|---|---|---|---|---|---|
| Intact | 11.3° | 8.1° | 6° (2–10°) | 9.9° | 9.3° | 7° |
| ATFL cut | 16.1° | 14.6° | 11° | 24.4° | 27.8° | 18° |
| + CFL cut | 29.6° | 23.8° | 19° | 24.3° | 28.4° | 21° |
| + PTFL cut | 40.1° | 40.2° | 46° | 24.3° | 28.3° | 26° |

- **Matches:** the size of the ATFL-cut step in tilt (+5–6°, Rasmussen +5°), and the ATFL as the main internal-rotation restraint.
- **Intact tilt on `inman` over-reads (KNOWN):** 11.3°, above Rasmussen's 2–10° range (9.1° before the CFL correction). The corrected CFL is 1.6 mm slack at neutral, so the talus tilts about 2° further before it engages, and the model has no capsule to take up that slack.
- **Over-reads:** tilt after the CFL cut (24–30° vs 19°), and internal rotation by 2–7°.

**Bahr 1997 benchmark.** Increase in ligament force (N) with the talocrural flexion held at each angle and the subtalar joint free, `inman` axis. Bahr values are mean ± SD (n = 8):

| Test | Ankle | ATFL model | ATFL Bahr | CFL model | CFL Bahr |
|---|---|---|---|---|---|
| Talar tilt, 5.7 N·m supination | 10° DF | 40 | 26 ± 11 | 141 | 123 ± 54 |
| | 0° | 76 | 36 ± 17 | 135 | 81 ± 45 |
| | 10° PF | 110 | 64 ± 51 | 124 | 60 ± 31 |
| | 20° PF | 138 | 123 ± 107 | 108 | 32 ± 34 |
| Anterior drawer, 80 N | 10° DF | 63 | 33 ± 17 | 45 | 40 ± 17 |
| | 0° | 90 | 23 ± 14 | 3 | 15 ± 14 |
| | 10° PF | 100 | 34 ± 17 | 0 | 6 ± 8 |
| | 20° PF | 85 | 53 ± 68 | 0 | −3 ± 6 |

The model ATFL is both bands summed, as Bahr's transducer measured the whole ligament. 11 of 16 values fall within Bahr's mean ± 2 SD on both axes (9 before the CFL correction). Both flexion trends are right, and the CFL's drawer forces now match at all four angles. What's wrong:
- **The CFL is still overloaded under inversion, 1.1–3.4× Bahr** (it was 1.6–5×), most in plantarflexion. Bahr's buckle transducers may under-read, so the excess may be partly in the reference. Not resolved.
- **Drawer loads the ATFL 2–4× too much at neutral and in DF.** Bahr let the tibia rotate and the foot plate translate; here the tibia is fixed.

### 6-DOF subtalar joint (`subtalar.py`, this branch)

**What changes.** On `research-twin`, `--six-dof` also turns the subtalar hinge into a contact joint (`--subtalar hinge` keeps MS-Human-700's hinge). The calcaneus gains 3 translations (`subtalar_tx/ty/tz_r`) and 2 rotations (`subtalar_flex_r`, `subtalar_twist_r`) relative to the talus, with no springs or stops of their own. The 1-DOF builds, including the committed `model/`, are unchanged.

**Why the hinge could not match the cadaver data.** Two readings of the same joint looked contradictory:
- **Knudson 1997:** an unloaded talocalcaneal joint turns 13–14° under about 0.2 N·m.
- **Pellegrini 2016:** an intact subtalar joint turns only 7.2° under 4 N·m.

The difference is the rig. Pellegrini's heel sits in a cradle that can turn only about the foot's front-to-back axis, with tibial rotation locked. The subtalar joint's own axis is oblique, so the cradle forces a motion the joint surfaces do not allow, and they cam apart against the ligaments. A hinge turns freely about its own axis and not at all about any other, so it cannot show this, and the earlier benchmark (a free torque on the foot) was not the experiment Pellegrini ran. With a free 4 N·m torque the new joint still turns about 13°, as Knudson's lax zone suggests it should.

**Idealisation: two convex surfaces, each in a cup.** The bone meshes overlap at this joint, so the surfaces are primitives:
- **Talar head:** a sphere fitted to the talus mesh (radius 15.2 mm), seated in a cup of plates on the foot body. The cup stands for the navicular, the spring ligament and the anterior and middle calcaneal facets; the model's foot is rigid from calcaneus to metatarsals, so the talonavicular joint is part of this articulation.
- **Posterior facet:** a convex sphere on the calcaneus (radius 21.0 mm; Kleipool 2019 fitted a cylinder of diameter 42.0 ± 7.7 mm) under a concave cap of plates on the talar body.
- **Axis:** with both seated, the only free motion is rotation about the line through the two centres. That line is set to Isman & Inman's mean subtalar axis (42° above horizontal, 23° medial) through the talar-head centre. The posterior centre is the point on it one facet radius from the posterior facet. This places the facet facing up and 30° forward, as it does anatomically, which the three numbers were not chosen to produce.
- **Moved axis:** the subtalar hinge is re-registered to this axis. MS-Human-700's own axis has a similar direction (38°, 9°) but runs 10–15 mm higher, through the talar body. Muscle paths are unchanged; their subtalar moment arms change (tibialis anterior drops to about 1 mm).

Nothing here is fitted to a benchmark. Cup coverage, the facet point on the mesh and the clearance are assumptions (tagged in `subtalar.py`).

**Pellegrini 2016 benchmark.** `bench.HindfootRig` reproduces the rig: heel welded to a cradle on a free translation platform, 36 N axial load, 4 N·m about the front-to-back axis (rotation locked) or about the vertical (inversion locked), 67 N drawer, ankle pinned or freed. Model on the `inman` axis against Pellegrini's intact specimens (n = 16, mean ± SD):

| 4 N·m inversion | Subtalar (ankle pinned), model | Pellegrini | Ankle (freed − pinned), model | Pellegrini |
|---|---|---|---|---|
| 10° DF | 6.0° | 3.8 ± 1.4° | 4.6° | 6.4 ± 2.2° |
| 0° | 6.5° | 7.2 ± 1.7° | 7.5° | 10.4 ± 3.1° |
| 10° PF | 7.0° | 8.0 ± 2.2° | 10.5° | 13.1 ± 3.6° |
| 20° PF | 7.0° | 10.8 ± 2.6° | 13.7° | 14.7 ± 3.4° |

| Subtalar joint at neutral, ankle pinned | Model | Pellegrini |
|---|---|---|
| Eversion, 4 N·m | 5.6° | 4.6 ± 3.3° |
| Internal rotation, 4 N·m | 8.6° | 5.1 ± 2.4° |
| External rotation, 4 N·m | 5.9° | 4.8 ± 1.1° |
| Anterior / posterior drawer, 67 N | 1.4 / 1.5 mm | 1.3 ± 0.7 / 0.8 ± 0.3 mm |
| Medial / lateral drawer, 67 N | 2.4 / 1.9 mm | 1.5 ± 0.7 / 1.2 ± 0.4 mm |

- **Matches:** 14 of 15 values are within Pellegrini's mean ± 2 SD on both axes (`original` was 11 of 15 before the CFL correction); the miss is the posterior drawer. At neutral the ankle takes 54% of the inversion (Pellegrini 59%; `original` 45%).
- **The ankle's share now follows flexion:** 4.6° at 10° DF rising to 13.7° at 20° PF (Pellegrini 6.4° to 14.7°). Before the CFL correction it was nearly flat (6.8° to 8.4°). The CFL tightens in dorsiflexion and limits talar tilt there.
- **Sectioning:**

  | | Intact | CFL cut | CL + ITCL cut | All three cut |
  |---|---|---|---|---|
  | 10° DF | 6.0° | 7.0° | 6.4° | 27.3° |
  | Neutral | 6.5° | 7.0° | 7.0° | 27.2° |

  Pellegrini: +5.9° at 10° DF as soon as the CFL was cut; at neutral, significant only once the CL was cut too (+5.7°); CL + ITCL alone not significant at any angle. The model has the same pattern (the CFL matters in dorsiflexion, not at neutral, and more than the other two there) at about a sixth of the size.
- **Why the CFL's effect is small:** at neutral the talar-head cup carries about 2.9 of the rig's 4 N·m and the CFL 0.9 N·m. With the CFL cut the cup stops the joint at 7.0°, where Pellegrini's joints reached 10° or more. The cup's coverage and direction are assumed, and its infero-medial part is the spring ligament in reality, soft tissue represented here by rigid plates.
- **Over-reads the last cut:** with all three cut the model reaches 27°; Pellegrini's reached 13–19° across flexion angles. The model has no lateral or posterior talocalcaneal ligaments, capsule or retinaculum to take over.
- **The socket has a rigid part and a part that should not be rigid.** The talar head sits in a socket made of the anterior and middle calcaneal facets (bone, below the head) and the navicular with the spring ligament (in front and medially). In the model's one-piece foot both are rigid with the calcaneus. In a real foot the navicular is joined to the calcaneus only through the midtarsal joint and its ligaments: with the subtalar joint fused, the talonavicular joint keeps 26% of its motion and the calcaneocuboid joint 56% (Astion 1997). Pellegrini clamped only the calcaneus. The plate that carries the rig's inversion torque is antero-medial, on the navicular part.
- **Bracket (`--navicular-stiffness`, N/mm):** the plates within 45° of straight plantar are kept on the calcaneus; the rest can be rigid (default), sprung in translation, or absent.

  | Navicular part of the socket | Intact, 10° DF / 0 / 10° PF / 20° PF | CFL cut, 10° DF | CL + ITCL cut, 10° DF | Eversion / internal / external rotation | Posterior drawer |
  |---|---|---|---|---|---|
  | Rigid (default) | 6.0 / 6.5 / 7.0 / 7.0° | 7.0° | 6.4° | 5.6 / 8.6 / 5.9° | 1.5 mm |
  | 1000 N/mm | 6.6 / 7.1 / 7.8 / 8.1° | 8.0° | 6.9° | 5.9 / 9.5 / 6.1° | 1.6 mm |
  | 300 N/mm | 7.3 / 8.1 / 8.9 / 9.6° | 9.7° | 7.7° | 6.3 / 11.0 / 6.5° | 1.7 mm |
  | 100 N/mm | 8.3 / 9.4 / 10.5 / 11.4° | 12.4° | 8.8° | 6.9 / 13.6 / 7.0° | 2.1 mm |
  | Absent | 9.5 / 11.1 / 12.2 / 13.1° | 17.9° | 9.9° | 8.2 / 19.9 / 8.2° | 3.9 mm |
  | Pellegrini | 3.8 / 7.2 / 8.0 / 10.8° | about 9.7° | not significant | 4.6 / 5.1 / 4.8° | 0.8 mm |

  Pellegrini's inversion values at neutral and in plantarflexion, and his CFL-cut value, lie between the two bounds. Around 300 N/mm the CFL-cut value and the plantarflexed values match and the flexion-trend check passes, but internal rotation moves further out (11° against 5.1 ± 2.4°), and no stiffness reaches his 3.8° at 10° DF.
- **Why the default stays rigid:** no measured stiffness for the navicular's attachment to the calcaneus was found (Davis 1996 tested the spring ligament; its numbers are not in the abstract). Choosing the value that matches Pellegrini would be a fit, which this branch does not use as validation. The sprung socket is in the code for when a measured value, or a midfoot joint, exists. A sprung build runs at a 0.5 ms timestep (set in the XML) because the spring is stiff for its mass.
- **What the bracket does not explain:** with all three ligaments cut the joint still opens to 27° at any stiffness, and the 3.8° at 10° DF needs more restraint in dorsiflexion than the CFL gives.
- **Flexion trend too small (KNOWN):** Pellegrini's subtalar inversion triples from 10° DF to 20° PF (3.8° to 10.8°); the model's rises from 6.0° to 7.0°. With the ankle pinned only the CFL can carry that dependence. It now tightens in dorsiflexion and gives the right direction, but the cup caps the range at 7.0°. A more vertical CFL gives a stronger trend (4.8° to 6.3° at 14° behind the fibula), as Edama 2017's angle classes suggest.
- **Sensitivity (run before the CFL correction, not repeated since):** intact inversion at neutral stayed between 4.0° and 7.0° when each uncertain input was moved in turn: the axis angles by ± 1 SD, the facet radius by ± 1 SD, the facet point by ± 3 mm, cup coverage and direction, and clearance from 0 to 0.5 mm. That is inside Pellegrini's ± 2 SD throughout. Internal rotation is the sensitive output (4–14°, driven by axis inclination).

**Colville 1990 benchmark.** Strain gauges, subtalar joint screwed, 3 N·m moments; full text read. Model ATFL here is the superior band. On both axes the model reproduces:
- ATFL lengthening from 20° DF to 30° PF (+25% on `inman`; Colville +15%);
- the ATFL lengthening with inversion and with internal rotation at every angle;
- CFL lengthening with inversion at every angle.

The `original` axis's earlier failures at 20° DF (the talus sat 3–8 mm anterior with no load) are gone with the anatomical mortise. It misses:
- **CFL in flexion:** on the hinge path the CFL now lengthens toward DF as Colville found, but in this test the talus is free and the deltoid tilts it into inversion in plantarflexion, which lengthens the CFL again (**KNOWN**). Colville also saw a slight rise at 30° PF.
- **PTFL with inversion:** lengthens; Colville found a slight decrease.

**Astion 1997 benchmark (all builds).** `bench.TendonPullRig` reproduces Astion's rig: ankle screwed at neutral, the foot free, the tibialis posterior (36 N) with the peroneus brevis (11 N) supinating it and the reverse pronating it; Astion found larger loads gave no more motion, so these are end ranges (n = 10). The subtalar range is the calcaneus's rotation relative to the talus between the two poses.

| | Hinge, `inman` / `moving` | Hinge, `original` | 6-DOF contact joint, `inman` | 6-DOF, `original` | Astion |
|---|---|---|---|---|---|
| Subtalar range | 17.8° | 17.6° | 30.9° | 31.1° | 20.4 ± 8° |
| Supination, stopped by | +14.9°, cervical ligament (18 N) | +14.8°, cervical ligament and CFL | +15.7°, cervical ligament (18 N) | +15.7°, cervical ligament | |
| Pronation, stopped by | −2.8°, ITCL (74 N) | −2.8°, ITCL (74 N) | −15.2°, ITCL (136 N) | −15.5°, ITCL (139 N) | |

- **The contact joint is within 2 SD but at the loose end** (30.9°, +1.3 SD), and symmetric: about 15° each way, close to Knudson 1997's unloaded lax zone of 13–14° on each side. Pronation still ends on the ITCL.
- **The hinge range passes but is lopsided.** Almost all of it is supination. Pronation stops after 2.8° because the ITCL tightens in eversion with this geometry, where Knudson 1997 found it restrains supination (see "Subtalar ligaments").
- **Not tested yet:** the talonavicular (36.7 ± 13°) and calcaneocuboid (14.4 ± 6°) ranges, and what each joint keeps after the others are fused. They need a midfoot joint; the model's foot is one body from calcaneus to metatarsals.
- **Calcaneal load:** Astion loaded the calcaneus but did not report how much, and found it made no difference. The rig applies none by default; a load of 40 N or more at the calcaneal origin inverts the foot on its own, so a larger load would need a better-placed point.

**Why it is not the default:**
1. **It does not fix the end-range strain problem.** Flexion still happens about the same fixed axis. On the `inman` axis ATFL strain at 35° plantarflexion is about 25% (hinge 26%).
2. **Some of the mortise is assumed.** The pivot sphere's radius and height and the fibula's stiffnesses are assumptions; the taper is measured.
3. **The magnitudes are only partly validated.** Ligament forces are benchmarked against Bahr 1997 (above), with mixed agreement. Talar displacements have not been compared with cadaver data; the intact drawer (11 mm at 150 N) may be large for a healthy ankle.
4. **It is not robust to non-physical loads.** A pure generalized torque on `ankle_angle_r` with no joint compression subluxes the talus. Apply loads as forces (muscles, sole or heel forces), which also compress the joint.
5. **The extra DOFs carry assumed damping.** 200 N·s/m and 1 N·m·s/rad suppress an overshoot instability; this is not taken from literature.

**History: the idealised mortise and why it was replaced.** The previous joint was a plain cylinder between flat plates: a full-height lateral plate and a thin medial rim (0.85–1.0 R), both made compliant (contact time constant 0.08 s instead of 0.004 s) because stiff plates jammed axial rotation. It matched Rasmussen's unloaded tests but failed Stormont: under 700 N with the lateral ligaments cut, the talus slid about 16 mm medially under the rim and tipped to 45°. On the `ligasys-sprain` branch that mechanism drove the trapdoor rig to 26° of talar tilt with the ATFL and CFL past failure. Three geometric fixes each traded one benchmark for another (`inman`; unloaded tilt at 1.5 N·m intact / ATFL / +CFL / +PTFL cut; Rasmussen 6/11/19/46°):

| Change | Unloaded tilt | Internal rotation, intact → ATFL cut (Rasmussen 7 → 18°) | 700 N, lateral cut |
|---|---|---|---|
| Thin rim (0.85–1.0 R) | 8.7/9.9/27.8/40.1° | 6.5 → 17.6° | 45° (dislocated) |
| Medial facet to 0.70 R | 7.6/7.7/28.5/37.4° | 2.5 → 2.6° (locked) | 0.0° |
| Medial facet 0.70 R, both facets short front to back | 8.3/9.3/27.8/40.1° | 6.0 → 6.1° (not ligament-limited) | 0.0° |
| Medial-lateral slide limited to ±3 mm | 8.7/9.9/11.1/11.1° | 6.5 → 12.4° | 0.0° |
| **Anatomical mortise (current, with the corrected CFL)** | **11.3/16.1/29.6/40.1°** | **9.9 → 24.4°** | **0.1°** |

Each feature alone fell short: the pivot at mid-height locked the joint; pivot alone near the top kept the ATFL from controlling rotation (5.5 → 6.5°); fibula and taper without the pivot tipped the loaded intact talus to 25°. All three together give unloaded laxity and loaded stability. Setting `medial_pivot_radius = 0`, `trochlear_width_difference = 0` and `fibula_mobile = False` recovers the idealised mortise.

## Known limitations: read before using ligament forces

1. **Ligament strains at end range are still too high** (default moving axis).
   - **At 35° plantarflexion:** ATFL ≈ 24%, dATTL ≈ 35%, TNL ≈ 13%.
   - **At 25° dorsiflexion:** PTFL ≈ 5%.
   - **Forces:** at full plantarflexion the TNL exceeds its reported failure load. Within 25° PF – 20° DF no ligament does. (With `--axis original`, the TNL already exceeds its failure load at 25° PF.)

   **Two kinds of measurement disagree.**
   - **Attachment-to-attachment length (de Asla 2009, in vivo):** the ATFL lengthens about 7 mm from full DF to full PF. The model matches this (7.4 mm).
   - **Midsubstance strain gauges (Ozeki 2002; Renström 1988):** only 1.6 mm (ATFL) and 1.2 mm (PTFL) over the whole range.

   A single straight line through the footprint centres reproduces the first and so overstates fibre strain. Recruiting fibres in sequence would reconcile them. The ATFL is now two bands (Vega 2020). The superior band still carries the centroid strain, about 24% at 35° PF. Per-band stiffness, from Vega's Table 1 or Dalmau-Pastor 2023, would refine this further. **Use ligament length/strain *trends* and onsets; treat force magnitudes near end range with caution.**
2. **Structures that should tighten in dorsiflexion:** with the Inman-based axes, TCL (Takao 2020) and dPTTL (anatomy) are near-isometric (< 1 mm/rad) rather than tightening. The verification prints this as a NOTE. (The CFL was on this list until its attachments were corrected.)
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
   - **Subtalar joint:** the default build still has MS-Human-700's 1-DOF hinge (axis through the talar body, about 11 mm above the tarsal canal). The 6-DOF builds on this branch replace it with a contact joint that reproduces Pellegrini 2016's intact subtalar and ankle motion without fitting (see "6-DOF subtalar joint"). Its surfaces are two spheres in cups, not segmented facets, and the cup coverage is assumed.
   - **Talar tilt (6-DOF build) is slightly lax.** Intact tilt is 11.3° at 1.5 N·m on `inman` (Rasmussen 1982: 2–10°; `original` 8.1°), because the CFL is slack at neutral and the model has no capsule. An earlier "too lax" conclusion rested on Choisne 2013 and is withdrawn.
   - **Internal rotation follows Rasmussen 1982** (9.9° intact, 24.4° with the ATFL cut, against 7° and 18°) with the anatomical mortise: a pivot about the medial malleolus, a trochlea wider in front and a mobile fibula. The pivot geometry and the fibula's stiffnesses are assumptions; segmented articular surfaces would replace them.
   - **Loaded stability (Stormont 1985) holds:** under body-weight axial load the talus stays seated in inversion even with the lateral ligaments cut.
   - **The CFL is still 1.1–3.4× Bahr 1997 in talar-tilt tests,** and tilt after the CFL is cut over-reads Rasmussen (29.6° vs 19°).
   - **The CFL's subtalar role is under-read.** The CFL runs nearly parallel to the subtalar axis (3 mm moment arm), so it cannot resist rotation about that axis; it restrains the motion a rig forces across it. In the contact joint it matters in dorsiflexion, as Pellegrini found, but cutting it there adds 1.0° where the cadaver joints gained about 6°: the talar-head cup, whose coverage is assumed, carries most of the rig's torque and stops the joint at 7.0° without it. The part of the cup that does this is the navicular, which the model's one-piece foot holds rigidly to the calcaneus; with it sprung or absent the CFL cut adds 2.4° to 8.4° (see the bracket under "6-DOF subtalar joint").
   - **Subtalar end range is under-restrained.** With the CFL, CL and ITCL cut the joint opens to 27° (Pellegrini 13–19° across flexion angles), and eversion is limited by the ITCL and the hinge stop; the ITCL reaches 122 N of its 382 N failure load when the unloaded foot is dropped passively (366 N before the CFL correction changed how the foot hangs). The lateral, posterior and medial talocalcaneal ligaments, the capsule and the bony stop at the sinus tarsi are not modelled.
   - **ITCL direction is still wrong:** about the anatomical axis its moment arm halves (5 mm) but it still lengthens in eversion; its attachment points come from a 100-vertex talus mesh.
   - **Inferior ATFL band assumptions:** stiffness and failure load are split equally with the superior band, and it has zero strain at neutral. It stretches 1.5–2.5 mm at the extremes of flexion on the moving axis.
   - **Injury-event inversion:** the hindfoot cannot reach it (mean peak about 67° foot-to-shank, Lysdal 2022). The model has no midfoot joint to carry the rest.

The experimental 6-DOF joint (above) addresses limitation 4. It does **not** fix 1 or 2: flexion still happens about the fixed axis.

## References

- Rochelle D. et al. (2020) *J Mech Behav Biomed Mater* 102:103473. doi:10.1016/j.jmbbm.2019.103473: ATFL/CFL/PTFL stiffness and failure loads at 100 %/s.
- Takao M. et al. (2021) *BMC Musculoskelet Disord* 22:558: ATFL and CFL strain changes (onsets superseded by Ozeki 2002).
- Ozeki S. et al. (2002) *Foot Ankle Int* 23(9):825–832: zero-strain onset angles (ATFL 16.2° PF, CFL 17.8° DF, PTFL 18.0° DF), in-situ lengths and length changes (author's thesis summary; journal tables not yet checked).
- Astion D.J., Deland J.T., Otis J.C., Kenneally S. (1997) *J Bone Joint Surg Am* 79-A(2):241–246: subtalar, talonavicular and calcaneocuboid range under tendon pulls, intact and after simulated arthrodesis, read in full text.
- Bahr R. et al. (1997) *Acta Orthop Scand* 68(5):435–441: ATFL/CFL force increases in drawer and talar-tilt tests (Tables 2–3), read in full text.
- Colville M.R. et al. (1990) *Am J Sports Med* 18(2):196–200: lateral-ligament strain vs flexion with 3 N·m inversion, eversion and rotation moments, subtalar fixed (Figs 1–3), read in full text.
- Michels F. et al. (2022) *Foot Ankle Surg* 28(7):1014–1020: stiffness, failure load, elongation at failure and in-situ length of the CFL, cervical ligament and ACaL–ITCL complex (accepted manuscript).
- Isman R.E., Inman V.T. (1969) Anthropometric studies of the human foot and ankle. *Bull Prosthet Res* 10–11:97–129: subtalar axis 42 ± 9° above horizontal and 23 ± 11° medial of the foot midline (secondary sources).
- Kleipool R.P. et al. (2019) Study on the three-dimensional orientation of the posterior facet of the subtalar joint using simulated weight-bearing CT. *J Orthop Res* 37(1):197–204: cylinder fitted to the posterior facet, diameter 42.0 ± 7.7 mm (20 subjects, both feet; abstract).
- Knudson G.A. et al. (1997) *Acta Orthop Scand* 68(5):442–446: subtalar stable and transition zones under ~0.2 N·m circumferential load, before and after ITCL section (full text).
- Hayes A., Tochigi Y., Saltzman C.L. (2006) Ankle morphometry on 3D-CT images. *Iowa Orthop J* 26:1–4: talar-dome widths 29.9 / 27.9 / 25.2 mm anterior / middle / posterior on ±30° oblique sections, dome radius 20.7 mm (n = 21; full text). An earlier version of this README attributed these values to the wrong DOI.
- Stormont D.M. et al. (1985) *Am J Sports Med* 13(5):295–300: under physiological load the articular surfaces give all inversion-eversion stability (abstract).
- Poonja A.J. et al. (2017) *Cureus* 9(6):e1382: cervical ligament dimensions; ITCL fibre direction (full text).
- Vega J. et al. (2020) *Knee Surg Sports Traumatol Arthrosc* 28:8–17: superior ATFL band lengthens 12.6 → 19.2 mm from DF to PF; inferior band isometric (10.6 mm), sharing its fibular origin with the CFL (accepted manuscript read; Table 1 missing).
- Rasmussen O., Tovborg-Jensen I. (1982) *Acta Orthop Scand* 53:155–160: talar tilt and rotation at 1.5 N·m after serial lateral-ligament cuts, read in full text.
- Astion D.J., Deland J.T., Otis J.C., Kenneally S. (1997) *J Bone Joint Surg Am* 79-A(2):241–246: motion of the subtalar, talonavicular and calcaneocuboid joints (20.4 ± 8°, 36.7 ± 13°, 14.4 ± 6°) and what each keeps after simulated arthrodesis of the others, 10 feet, read in full text.
- Pellegrini M.J. et al. (2016) *J Bone Joint Surg Am* 98:842–848: subtalar and ankle inversion, eversion and rotation at 4 N·m and drawer at 67 N in a cradle rig with 36 N axial load (Tables I and II); CFL/CL/ITCL sectioning, read in full text.
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
- Edama M. et al. (2017) *J Foot Ankle Res* 10:60: angle of the CFL to the fibular long axis in 81 feet, and its simulated strain over flexion and inversion by angle class (open access, full text).
- Inman V.T. (1976) *The Joints of the Ankle*: empirical axis 5 mm distal to the medial malleolar tip; 3 mm distal and 8 mm anterior to the lateral tip (via a functional-anatomy text).
- Lundberg A. et al. (1989) *J Bone Joint Surg Br* 71(1):94–99: axes lie close to the midpoint of the line between the malleolar tips, change through the range of motion, are oblique downward and laterally in dorsiflexion, and differ between individuals (abstract).
- Wolf P. et al. (2022) *Sci Rep* 12:13957: bone-pin walking data; the dorsiflexion/plantarflexion inclination difference runs opposite to Lundberg in 4 of 5 participants; source of the illustrative 14.4° swing (Fig. 5, morphology-based axes for one participant).
- Klein P., Mattys S., Rooze M. (1996) *J Biomech* 29:21–30: in vitro ankle and subtalar muscle moment arms (via Brookbush Institute review).

## Licence

Model data derived from MS-Human-700 (© LNS Group, Apache-2.0). Cite Zuo et al., ICRA 2024 (see `../MS-Human-700/README.md`).
