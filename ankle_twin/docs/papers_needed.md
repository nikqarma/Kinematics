# Papers needed: `ligasys-sprain` branch

This branch models the inversion-sprain mechanism and the Smart Ligament (external wearable, semi-active lock).
It lists only what this branch needs, ranked by impact on the sprain simulation and device design. The master list
covering both branches is `ankle_twin/literature/papers_needed.md` (gitignored, local); drop PDFs into that folder.
When a paper arrives, update the master list and this file, plus `research-twin`'s copy if it needs the paper too.

Started 2026-11-05; split by branch 2026-11-08.


Status key: *wanted* (none of it was readable), *abstract only* (just the abstract was read),
*not reached* (the site blocked access), *have* (a PDF is in `ankle_twin/literature/`). Rows marked
"paper to identify" are needs without a chosen source yet.

Priority: **A** blocks the next modelling step or replaces an assumed value; **B** replaces a secondary or
abstract-only value; **C** extra validation or background; **-** not needed on that branch.

Tickets: the Jira issues (project AT) a paper would inform. AT-1 is the shared-core epic, AT-2 `research-twin`, AT-3 `ligasys-sprain`. Use the table below to pick papers for the tickets in a sprint.

## By Jira ticket

Open tickets in project AT, each with the papers that would inform it, highest priority first. A paper can appear under several tickets. **have** marks a paper already in hand whose copy is incomplete or not yet in `literature/`.

| Ticket | Papers to look for (priority, paper, what we have now) |
|---|---|
| AT-5 ATFL fibular origin | **A** Vega J. et al. (2020), published Table 1: have manuscript only<br>**B** Cordier G. et al. (2021): wanted<br>**B** Edama M. et al. (2018): abstract only<br>**B** Open, licence-compatible segmented ankle and hindfoot...: wanted<br>**C** Clanton T.O. et al. (2015): secondary summary only<br>**have** Vega J., Malagelada F., Manzanares Céspedes M.C., Dalmau-Pastor M. (2020): accepted manuscript; Table 1 and figures missing |
| AT-6 Peroneus brevis moment-arm sign | **B** Inman V.T. (1976): secondary source only<br>**B** Inman V.T. (1976), trochlear chapter: wanted<br>**B** Klein P., Mattys S., Rooze M. (1996): secondary source only |
| AT-7 Talar tilt: CFL force and tilt after cuts | **A** Ankle and subtalar joint capsule: stiffness or contribution...: wanted<br>**A** Bahr R. et al. (1998): wanted<br>**A** Dalmau-Pastor M. et al. (2023): abstract only<br>**A** Robotic ATFL in-situ force under inversion /...: not reached<br>**A** Stormont D.M., Morrey B.F., An K.-N., Cass J.R. (1985): abstract only<br>**A** Syndesmosis / distal tibiofibular stiffness (lateral and rotational): wanted<br>**A** The role of ankle ligaments and articular geometry in...: abstract only<br>**A** Weight-bearing biplane/stereoradiography of fibular motion...: search summary only<br>**B** MRI quantification of the impact of ankle position on...: abstract only<br>**B** Ozeki S. et al. (2002): thesis summary only<br>**B** Talar tilt angle vs inversion torque at several torques,...: wanted |
| AT-8 End-range ligament strain, fibres, toe regions | **A** Attarian D.E. et al. (1985): wanted<br>**A** Dalmau-Pastor M. et al. (2023): abstract only<br>**A** Funk J.R. et al. (2000): wanted<br>**A** Ozeki S. et al. (2006): abstract only<br>**A** Siegler S., Block J., Schneck C.D. (1988): wanted<br>**A** Vega J. et al. (2020), published Table 1: have manuscript only<br>**B** Cordier G. et al. (2021): wanted<br>**B** Edama M. et al. (2018): abstract only<br>**B** Ozeki S. et al. (2002): thesis summary only<br>**C** Edama M. et al. (2019): abstract only<br>**C** Renstrom P. et al. (1988): abstract only<br>**have** Vega J., Malagelada F., Manzanares Céspedes M.C., Dalmau-Pastor M. (2020): accepted manuscript; Table 1 and figures missing |
| AT-9 Medial bands that should tighten in dorsiflexion | **B** Inman V.T. (1976): secondary source only<br>**B** Inman V.T. (1976), trochlear chapter: wanted |
| AT-10 Moving ankle axis swing | **B** Lundberg A. et al. (1989): secondary source only<br>**C** Wolf P. et al. (2022): figure read |
| AT-11 6-DOF talocrural seating | **A** Ankle and subtalar joint capsule: stiffness or contribution...: wanted<br>**A** Syndesmosis / distal tibiofibular stiffness (lateral and rotational): wanted<br>**A** Weight-bearing biplane/stereoradiography of fibular motion...: search summary only<br>**B** MRI quantification of the impact of ankle position on...: abstract only |
| AT-12 Segmented bone and joint geometry | **A** The role of ankle ligaments and articular geometry in...: abstract only<br>**B** Inman V.T. (1976), trochlear chapter: wanted<br>**B** Open, licence-compatible segmented ankle and hindfoot...: wanted |
| AT-13 Midfoot joint | **A** Lysdal F.G. et al. (2022): abstract only<br>**B** Midtarsal (talonavicular / calcaneocuboid): wanted<br>**have** Astion D.J., Deland J.T., Otis J.C., Kenneally S. (1997): read online; no local PDF in `literature/` |
| AT-14 Verify secondary-source values | **A** Funk J.R. et al. (2000): wanted<br>**A** Lysdal F.G. et al. (2022): abstract only<br>**A** Siegler S., Block J., Schneck C.D. (1988): wanted<br>**A** Stormont D.M., Morrey B.F., An K.-N., Cass J.R. (1985): abstract only<br>**A** Vega J. et al. (2020), published Table 1: have manuscript only<br>**B** Inman V.T. (1976): secondary source only<br>**B** Inman V.T. (1976), trochlear chapter: wanted<br>**B** Klein P., Mattys S., Rooze M. (1996): secondary source only<br>**B** Lundberg A. et al. (1989): secondary source only<br>**B** Ozeki S. et al. (2002): thesis summary only<br>**C** Clanton T.O. et al. (2015): secondary summary only<br>**C** Pellegrini M.J. et al. (2016), online appendix: wanted |
| AT-15 Missing passive structures (capsule, tendons) | **A** Achilles and peroneal tendon compliance (paper to identify): wanted<br>**A** Ankle and subtalar joint capsule: stiffness or contribution...: wanted |
| AT-21 Trapdoor rig passive result | **A** Ankle and subtalar joint capsule: stiffness or contribution...: wanted<br>**A** Bahr R. et al. (1998): wanted<br>**A** Fong D.T. et al. (2009): not reached<br>**A** Stormont D.M., Morrey B.F., An K.-N., Cass J.R. (1985): abstract only<br>**A** Trapdoor inversion velocity and time to peak inversion at...: wanted<br>**B** Segment masses and inertias for the shank and foot (paper to identify): wanted |
| AT-22 Peroneal reflex | **A** Achilles and peroneal tendon compliance (paper to identify): wanted<br>**A** Peroneal reaction times and eversion motor response in...: abstract only<br>**A** Peroneal reflex amplitude and electromechanical delay on...: wanted<br>**A** The peroneus reaction time during sudden inversion test:...: abstract only |
| AT-23 Fitted subtalar spring | **A** Subtalar stiffness vs axial load (paper to identify): wanted<br>**C** Pellegrini M.J. et al. (2016), online appendix: wanted<br>**have** Pellegrini M.J. et al. (2016): online appendix not included |
| AT-24 Smart Ligament device model | **A** Ankle brace / tape mechanical restraint of inversion, with...: wanted<br>**A** Skin, soft-tissue and shoe-upper compliance under strap...: wanted |
| AT-25 Sprain mechanism and load cases | **A** Attarian D.E. et al. (1985): wanted<br>**A** Fong D.T. et al. (2009): not reached<br>**A** Funk J.R. et al. (2000): wanted<br>**A** Kristianslund E. et al. (2011): not reached<br>**A** Lysdal F.G. et al. (2022): abstract only<br>**A** Ozeki S. et al. (2006): abstract only<br>**B** Bonner T.J. et al. (2015): wanted<br>**B** Fong D.T. et al. (2012): open access PDF located, not yet read<br>**B** Gehring D. et al. (2013): wanted |

## Have

| Paper | Status | Content | Role on this branch | Tickets |
|---|---|---|---|---|
| Colville M.R. et al. (1990) Strain measurement in lateral ankle ligaments. *Am J Sports Med* 18(2):196-200 | have | Colville benchmark in `verify_ankle_model.py`. It gauges the whole ATFL only, with no per-band data | validation | AT-7 |
| Edama M. et al. (2017) The effects on calcaneofibular ligament function of differences in the angle of the calcaneofibular ligament with respect to the long axis of the fibula: a simulation study. *J Foot Ankle Res* 10:60 (open access) | have (read online; no local PDF) | In use: CFL angle to the fibular long axis (modal class 40-49 deg of 80 feet) sets the calcaneal insertion. Its simulated strains by angle class are why the CFL's change in subtalar inversion is reported, not asserted | model input | AT-4 |
| Astion D.J., Deland J.T., Otis J.C., Kenneally S. (1997) Motion of the hindfoot after simulated arthrodesis. *J Bone Joint Surg Am* 79-A(2):241-246 | have (read online; no local PDF in `literature/`) | Subtalar, talonavicular and calcaneocuboid motion (20.4, 36.7, 14.4 deg) and what each keeps when the others are fused (talonavicular keeps 26% with the subtalar joint fused). Shows the navicular is not rigid with the calcaneus; a target for a midfoot joint | background | AT-13 |
| Edama M. et al. (2025) Differences in ankle stabilizing function between the upper and lower fiber bundles of the ATFL. *Sci Rep* 15:26126 (open access, CC BY-NC-ND) | have | Qualitative per-band check only, no onsets or stiffness (see note below) | qualitative | AT-8 |
| Vega J., Malagelada F., Manzanares Céspedes M.C., Dalmau-Pastor M. (2020) The lateral fibulotalocalcaneal ligament complex: an ankle stabilizing isometric structure. *Knee Surg Sports Traumatol Arthrosc* 28:8-17 | have (accepted manuscript; **Table 1 and figures missing**) | Per-band ATFL lengths in PF/DF: the superior band lengthens (12.6 -> 19.2 mm), the inferior band is isometric (10.6 mm). Basis for the two-band ATFL | model input | AT-5, AT-8 |
| Rasmussen O., Tovborg-Jensen I. (1982) Mobility of the ankle joint. *Acta Orthop Scand* 53:155-160 | have | Talar tilt and internal rotation at 1.5 N·m, subtalar fixed, after serial ATFL/CFL/PTFL cuts | validation | AT-7 |
| Pellegrini M.J. et al. (2016) Systematic quantification of stabilizing effects of subtalar joint soft-tissue constraints in a novel cadaveric model. *J Bone Joint Surg Am* 98:842-848 | have (online appendix not included) | Subtalar vs ankle share of hindfoot inversion at 4 N·m; CFL/CL/ITCL sectioning | **calibration** (subtalar inversion stiffness fitted to it) | AT-23 |
| Michels F. et al. (2022) The different subtalar ligaments show significant differences in their mechanical properties. *Foot Ankle Surg* 28(7):1014-1020 | have (accepted manuscript) | CL and ACaL-ITCL stiffness, failure load, elongation at failure (CL 3.0, ACaL-ITCL 2.6 mm) and in-situ length (CL 13.9, ACaL-ITCL 11.0 mm, now the ITCL reference length). No zero-strain angle | model input | - |
| Knudson G.A. et al. (1997) Subtalar joint stability: talocalcaneal interosseous ligament function. *Acta Orthop Scand* 68(5):442-446 | have | Unloaded feet rotate 13-14 +/- 7 deg into supination at ~0.2 N·m (sets the CL onset); cutting the ITCL adds ~4 deg supination, no pronation. Screw axis within ~13 deg of the model hinge | model input | - |

**Edama 2025 per-band findings:**
- **Setup:** 10 Thiel-embalmed feet from donors of about 90 years, 5 per group. A Telos device applied 120 N, and ultrasound measured the malleolus–talus distance.
- **Drawer:** cutting either band increased drawer at 30 PF only.
- **Inversion:** cutting the lower band increased inversion opening at 0 deg. The abstract says it was the upper band, but Table 3 and the results say lower; Table 3 is taken as correct.
- **No group difference:** the upper-cut and lower-cut groups never differed significantly. SDs are often larger than the effects.
- **Use:** supports a two-band ATFL with the lower band acting near neutral in inversion, coupled to the CFL by arciform fibres. It cannot set band onsets, stiffness or geometry.

Findings and model comparison are in `docs/lateral_ligament_literature.md`.

## Lateral ligaments: geometry, bands and load sharing

| Priority | Tickets | Paper | Status | What it would give |
|---|---|---|---|---|
| A | AT-5, AT-8, AT-14 | Vega J. et al. (2020), published version or its Table 1 | have manuscript only | Per-band widths (to split the whole-ATFL stiffness between bands) and footprint figures |
| A | AT-8, AT-7 | Dalmau-Pastor M. et al. (2023) Clinical relevance and function of ATFL superior and inferior fascicles: a robotic study. *Am J Sports Med* 51(8):2169-2175 (PMID 37232327) | abstract only | 6-DOF robot, 10 cadavers, serial sectioning superior -> inferior: per-band contribution to drawer, internal rotation and inversion through flexion. The inferior band was never cut alone |
| A | AT-8, AT-25 | Ozeki S. et al. (2006) Ankle ligament tensile forces at the end points of passive circumferential rotating motion. *Foot Ankle Int* 27(11):965-969 | abstract only | Peak ATFL/CFL/PTFL/TCL forces at end range (8 cadavers): ATFL 108-130 N in supination + PF |
| A | AT-7 | Robotic ATFL in-situ force under inversion / internal-rotation torque, *Foot Ankle Orthop* 2018 | not reached (DOAJ 403) | ATFL force vs applied torque, a direct check on talar-tilt load sharing |
| A | AT-7, AT-21 | Bahr R. et al. (1998) *Knee Surg Sports Traumatol Arthrosc* 6:115-121 | wanted | ATFL/CFL forces across positions with 375 N axial compression (weight-bearing tilt behaviour) |
| A | AT-7, AT-21, AT-14 | Stormont D.M., Morrey B.F., An K.-N., Cass J.R. (1985) Stability of the loaded ankle. *Am J Sports Med* 13(5):295-300 | abstract only | Under physiological load the articular surfaces give 100% of inversion-eversion stability (30% of rotational). The trapdoor rig shows the model's mortise lacks this |
| A | AT-7, AT-12 | The role of ankle ligaments and articular geometry in stabilizing the ankle (2012, Mayo Clinic group; authors to confirm) | abstract only | Loaded ankle: articular geometry gives 100% of translational and 60% of rotational stability |
| A | AT-12 | Hayes A., Tochigi Y., Saltzman C.L. (2006) Ankle morphometry on 3D-CT images. *Iowa Orthop J* 26:1-4 | have | Trochlear width anterior 29.9 +/- 2.6, middle 27.9 +/- 3.0, posterior 25.2 +/- 3.7 mm on +-30 deg oblique sections; dome radius 20.7 +/- 2.6 mm (n=21, 3D-CT): sets the 12.8 deg trochlear taper. Replaces a mis-attributed DOI (10.1186/s13018-017-0610-3, a surgical case study) |
| A | AT-11, AT-7 | Weight-bearing biplane/stereoradiography of fibular motion (paper to confirm; search summary: rotation 9.3 +/- 3.5 deg, ML 1.7 +/- 1.0 mm, AP 3.3 +/- 2.2 mm, SI 2.5 +/- 0.9 mm ROM) | search summary only | Bounds for the mobile fibula's travel |
| A | AT-7, AT-11 | Syndesmosis / distal tibiofibular stiffness (lateral and rotational), intact (paper to identify) | wanted | Spring constants for the mobile fibula (assumed now) |
| B | AT-5, AT-8 | Edama M. et al. (2018) Morphological features of the ATFL by the number of fiber bundles. *Ann Anat* 216:69-74 | abstract only | Per-band length, width and angle (81 legs; two-band type 57%) |
| B | AT-8, AT-5 | Cordier G. et al. (2021) Connecting fibers between ATFL's inferior fascicle and CFL transmit tension between both ligaments. *Knee Surg Sports Traumatol Arthrosc* 29:2511-2516 | wanted | Whether the inferior band and CFL should share load (coupled springs) |
| B | AT-12, AT-6, AT-9, AT-14 | Inman V.T. (1976) *The Joints of the Ankle*, trochlear shape chapter | wanted | Original trochlear wedge and facet-curvature data (medial facet radius smaller than lateral) |
| B | AT-11, AT-7 | MRI quantification of the impact of ankle position on syndesmosis anatomy (PubMed 27733557) | abstract only (search summary) | Fibular external rotation and lateral translation with dorsiflexion |
| B | AT-7 | Talar tilt angle vs inversion torque at several torques, subtalar fixed (paper to identify) | wanted | A talocrural moment-angle curve; Rasmussen 1982 gives one point (6 deg at 1.5 N·m) |
| C | AT-8 | Edama M. et al. (2019) Effect of the number of ATFL fiber bundles on ankle braking function: a simulation study. *Surg Radiol Anat* 41(1):69-73 | abstract only | Per-band length change in 20 DF / 30 PF / 20 inversion; could set band onsets |

## Subtalar joint and its ligaments

| Priority | Tickets | Paper | Status | What it would give |
|---|---|---|---|---|
| A | AT-23 | Subtalar stiffness vs axial load (paper to identify) | wanted | How hindfoot inversion stiffness rises with compression; would check or replace the fitted stiffness |
| B | - | Fujii T. et al. (2005) Analysis of ankle-hindfoot stability in multiple planes: an in vitro study. *Foot Ankle Int* 26(8):633-637 | wanted | Ankle vs hindfoot stiffness in several planes (cited by Pellegrini) |
| B | AT-13 | Midtarsal (talonavicular / calcaneocuboid) joint stiffness or load-displacement data (paper to identify) | wanted | Same purpose, and the input for a midfoot joint |
| C | AT-14, AT-23 | Pellegrini M.J. et al. (2016) *J Bone Joint Surg Am* 98:842-848, online appendix (jbjs.org data supplement) | wanted | Per-condition subtalar angulation and drawer tables after each cut |
| C | - | Tochigi Y. et al. (2004) *Foot Ankle Int* 25(8):588-596 | abstract only | ITCL sectioning: laxity and flexibility numbers; the ITCL's inversion vs eversion role |

## Ligament material, rate dependence and onsets

| Priority | Tickets | Paper | Status | What it would give |
|---|---|---|---|---|
| A | AT-8, AT-14, AT-25 | Funk J.R. et al. (2000) *J Biomech Eng* 122(1):15-22 | wanted (core.ac.uk copy blocked) | Original QLV / rate-dependent curves for all 8 ankle ligaments; model uses secondary fits (Xu 2019) |
| A | AT-8, AT-14 | Siegler S., Block J., Schneck C.D. (1988) *Foot Ankle* 8(5):234-242 | wanted | Failure strain and load for 120 ligaments; tibionavicular value (secondary now); tibiospring stiffness (assumed now) |
| A | AT-25, AT-8 | Attarian D.E. et al. (1985) *Foot Ankle* 6(2):54-58 | wanted | Failure vs loading rate; ATFL/CFL strain at failure. The one source reporting significant rate effects around 100 %/s |
| B | AT-25 | Bonner T.J. et al. (2015) Strain-rate sensitivity of the lateral collateral ligament of the knee. *J Mech Behav Biomed Mater* 41:261-270 | wanted | Basis for Rochelle 2020's claim that rate effects are negligible above 100 %/s |
| B | AT-14, AT-7, AT-8 | Ozeki S. et al. (2002) *Foot Ankle Int* 23(9):825-832 | thesis summary only | Journal tables to confirm the lateral zero-strain onsets |
| C | AT-8 | Renstrom P. et al. (1988) *Foot Ankle* 9:59-63 | abstract only | ATFL / CFL strain vs flexion |
| C | AT-5, AT-14 | Clanton T.O. et al. (2015) *J Bone Joint Surg Am* (lateral ligament radiographic landmarks) | secondary summary only | Original footprint distances used for ATFL/CFL attachments |

## Sprain events and landing rig

| Priority | Tickets | Paper | Status | What it would give |
|---|---|---|---|---|
| A | AT-25, AT-21 | Fong D.T. et al. (2009) *Am J Sports Med* 37(4):822-827 | not reached (Loughborough repository 403) | Lab-captured grade I ATFL sprain: kinematics and the ~632 deg/s inversion-velocity threshold |
| A | AT-25 | Kristianslund E. et al. (2011) *J Biomech* 44:2576 | not reached (NIH Brage) | Lab sprain during cutting with joint moments; peak inversion 130-180 ms after contact |
| A | AT-25, AT-13, AT-14 | Lysdal F.G. et al. (2022) *Sports Biomech* (DOI 10.1080/14763141.2022.2035801) | abstract only (AAU copy 403) | Pooled kinematics of 24 sprains: peak inversion 67.5 deg, velocity 974 deg/s (468-1752) |
| A | AT-22 | The peroneus reaction time during sudden inversion test: systematic review (authors to confirm) | abstract only (search summary) | Peroneus longus latency on trapdoors in healthy adults (median 63 ms; brevis 66 ms in one study). Sets the rig's reflex onset |
| A | AT-22 | Peroneal reaction times and eversion motor response in healthy and unstable ankles (authors to confirm) | abstract only (search summary) | 81 subjects, 50 deg inversion: total inversion time 105-110 ms; latency similar with and without instability |
| A | AT-22 | Peroneal reflex amplitude and electromechanical delay on sudden inversion (paper to identify) | wanted | How much eversion force arrives, and when; the rig's outcome depends strongly on it |
| A | AT-21 | Trapdoor inversion velocity and time to peak inversion at 20-30 deg (paper to identify) | wanted | Direct validation of the rig's 416 deg/s at 24 deg |
| B | AT-25 | Gehring D. et al. (2013) How to sprain your ankle: a biomechanical case report of an inversion trauma. *J Biomech* | wanted | Lab sprain kinematics and kinetics |
| B | AT-25 | Fong D.T. et al. (2012) Kinematics analysis of ankle inversion ligamentous sprain injuries in sports: five cases from televised tennis competitions. *Am J Sports Med* | open access PDF located, not yet read | Video-based sprain kinematics; plantarflexion not required for a lateral sprain |

## Smart Ligament (external wearable, semi-active lock)

| Priority | Tickets | Paper | Status | What it would give |
|---|---|---|---|---|
| A | AT-24 | Ankle brace / tape mechanical restraint of inversion, with stiffness values (paper to identify) | wanted | Benchmark for an external restraint's load path and effectiveness |
| A | AT-24 | Skin, soft-tissue and shoe-upper compliance under strap loads (paper to identify) | wanted | Series compliance between a shank cuff / heel counter and bone |

## Body segment parameters

| Priority | Tickets | Paper | Status | What it would give |
|---|---|---|---|---|
| B | AT-21 | Segment masses and inertias for the shank and foot (paper to identify) | wanted | Body-weight loading and inertia in the landing rig |

## Bone geometry and passive structures

| Priority | Tickets | Paper | Status | What it would give |
|---|---|---|---|---|
| A | AT-15, AT-7, AT-11, AT-21 | Ankle and subtalar joint capsule: stiffness or contribution to talar tilt and distraction (paper to identify) | wanted | The slack the CFL leaves at neutral has nothing to take it up: intact talar tilt 11 deg (Rasmussen 2-10) and the hanging foot's talus shifts 1.6 mm |
| A | AT-15, AT-22 | Achilles and peroneal tendon compliance (paper to identify) | wanted | Rigid tendons now; matters for landing dynamics and for when reflex force arrives |
| B | AT-12, AT-5 | Open, licence-compatible segmented ankle and hindfoot (bones, ideally cartilage) dataset (to identify) | wanted | Replaces the low-poly meshes and the primitive joint surfaces; re-derives every attachment |
| C | AT-10 | Wolf P. et al. (2022) Human ankle joint movements during walking are probably not determined by talar morphology. *Sci Rep* 12:13957 (open access) | figure read | Source of the illustrative 14.4 deg axis swing (one participant, Fig. 5); full data would size it properly |

## Background (low priority; only secondary sources read so far)

| Paper | What it would confirm | Tickets |
|---|---|---|
| Inman V.T. (1976) *The Joints of the Ankle* | Empirical ankle-axis landmarks (via a functional-anatomy text now) | AT-6, AT-9, AT-14 |
| Lundberg A. et al. (1989) *J Bone Joint Surg Br* 71(1):94-99 | Moving-axis description (abstract only) | AT-10, AT-14 |
| Klein P., Mattys S., Rooze M. (1996) *J Biomech* 29:21-30 | Muscle moment arms used in verification (via Brookbush Institute review) | AT-6, AT-14 |
