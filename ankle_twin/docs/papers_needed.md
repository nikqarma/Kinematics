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

## Have

| Paper | Status | Content | Role on this branch |
|---|---|---|---|
| Colville M.R. et al. (1990) Strain measurement in lateral ankle ligaments. *Am J Sports Med* 18(2):196-200 | have | Colville benchmark in `verify_ankle_model.py`. It gauges the whole ATFL only, with no per-band data | validation |
| Edama M. et al. (2017) The effects on calcaneofibular ligament function of differences in the angle of the calcaneofibular ligament with respect to the long axis of the fibula: a simulation study. *J Foot Ankle Res* 10:60 (open access) | have (read online; no local PDF) | In use: CFL angle to the fibular long axis (modal class 40-49 deg of 80 feet) sets the calcaneal insertion. Its simulated strains by angle class are why the CFL's change in subtalar inversion is reported, not asserted | model input |
| Edama M. et al. (2025) Differences in ankle stabilizing function between the upper and lower fiber bundles of the ATFL. *Sci Rep* 15:26126 (open access, CC BY-NC-ND) | have | Qualitative per-band check only, no onsets or stiffness (see note below) | qualitative |
| Vega J., Malagelada F., Manzanares Céspedes M.C., Dalmau-Pastor M. (2020) The lateral fibulotalocalcaneal ligament complex: an ankle stabilizing isometric structure. *Knee Surg Sports Traumatol Arthrosc* 28:8-17 | have (accepted manuscript; **Table 1 and figures missing**) | Per-band ATFL lengths in PF/DF: the superior band lengthens (12.6 -> 19.2 mm), the inferior band is isometric (10.6 mm). Basis for the two-band ATFL | model input |
| Rasmussen O., Tovborg-Jensen I. (1982) Mobility of the ankle joint. *Acta Orthop Scand* 53:155-160 | have | Talar tilt and internal rotation at 1.5 N·m, subtalar fixed, after serial ATFL/CFL/PTFL cuts | validation |
| Pellegrini M.J. et al. (2016) Systematic quantification of stabilizing effects of subtalar joint soft-tissue constraints in a novel cadaveric model. *J Bone Joint Surg Am* 98:842-848 | have (online appendix not included) | Subtalar vs ankle share of hindfoot inversion at 4 N·m; CFL/CL/ITCL sectioning | **calibration** (subtalar inversion stiffness fitted to it) |
| Michels F. et al. (2022) The different subtalar ligaments show significant differences in their mechanical properties. *Foot Ankle Surg* 28(7):1014-1020 | have (accepted manuscript) | CL and ACaL-ITCL stiffness, failure load, elongation at failure (CL 3.0, ACaL-ITCL 2.6 mm) and in-situ length (CL 13.9, ACaL-ITCL 11.0 mm, now the ITCL reference length). No zero-strain angle | model input |
| Knudson G.A. et al. (1997) Subtalar joint stability: talocalcaneal interosseous ligament function. *Acta Orthop Scand* 68(5):442-446 | have | Unloaded feet rotate 13-14 +/- 7 deg into supination at ~0.2 N·m (sets the CL onset); cutting the ITCL adds ~4 deg supination, no pronation. Screw axis within ~13 deg of the model hinge | model input |

**Edama 2025 per-band findings:**
- **Setup:** 10 Thiel-embalmed feet from donors of about 90 years, 5 per group. A Telos device applied 120 N, and ultrasound measured the malleolus–talus distance.
- **Drawer:** cutting either band increased drawer at 30 PF only.
- **Inversion:** cutting the lower band increased inversion opening at 0 deg. The abstract says it was the upper band, but Table 3 and the results say lower; Table 3 is taken as correct.
- **No group difference:** the upper-cut and lower-cut groups never differed significantly. SDs are often larger than the effects.
- **Use:** supports a two-band ATFL with the lower band acting near neutral in inversion, coupled to the CFL by arciform fibres. It cannot set band onsets, stiffness or geometry.

Findings and model comparison are in `docs/lateral_ligament_literature.md`.

## Lateral ligaments: geometry, bands and load sharing

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| A | Vega J. et al. (2020), published version or its Table 1 | have manuscript only | Per-band widths (to split the whole-ATFL stiffness between bands) and footprint figures |
| A | Dalmau-Pastor M. et al. (2023) Clinical relevance and function of ATFL superior and inferior fascicles: a robotic study. *Am J Sports Med* 51(8):2169-2175 (PMID 37232327) | abstract only | 6-DOF robot, 10 cadavers, serial sectioning superior -> inferior: per-band contribution to drawer, internal rotation and inversion through flexion. The inferior band was never cut alone |
| A | Ozeki S. et al. (2006) Ankle ligament tensile forces at the end points of passive circumferential rotating motion. *Foot Ankle Int* 27(11):965-969 | abstract only | Peak ATFL/CFL/PTFL/TCL forces at end range (8 cadavers): ATFL 108-130 N in supination + PF |
| A | Robotic ATFL in-situ force under inversion / internal-rotation torque, *Foot Ankle Orthop* 2018 | not reached (DOAJ 403) | ATFL force vs applied torque, a direct check on talar-tilt load sharing |
| A | Bahr R. et al. (1998) *Knee Surg Sports Traumatol Arthrosc* 6:115-121 | wanted | ATFL/CFL forces across positions with 375 N axial compression (weight-bearing tilt behaviour) |
| A | Stormont D.M., Morrey B.F., An K.-N., Cass J.R. (1985) Stability of the loaded ankle. *Am J Sports Med* 13(5):295-300 | abstract only | Under physiological load the articular surfaces give 100% of inversion-eversion stability (30% of rotational). The trapdoor rig shows the model's mortise lacks this |
| A | The role of ankle ligaments and articular geometry in stabilizing the ankle (2012, Mayo Clinic group; authors to confirm) | abstract only | Loaded ankle: articular geometry gives 100% of translational and 60% of rotational stability |
| A | Hayes A., Tochigi Y., Saltzman C.L. (2006) Ankle morphometry on 3D-CT images. *Iowa Orthop J* 26:1-4 | have | Trochlear width anterior 29.9 +/- 2.6, middle 27.9 +/- 3.0, posterior 25.2 +/- 3.7 mm on +-30 deg oblique sections; dome radius 20.7 +/- 2.6 mm (n=21, 3D-CT): sets the 12.8 deg trochlear taper. Replaces a mis-attributed DOI (10.1186/s13018-017-0610-3, a surgical case study) |
| A | Weight-bearing biplane/stereoradiography of fibular motion (paper to confirm; search summary: rotation 9.3 +/- 3.5 deg, ML 1.7 +/- 1.0 mm, AP 3.3 +/- 2.2 mm, SI 2.5 +/- 0.9 mm ROM) | search summary only | Bounds for the mobile fibula's travel |
| A | Syndesmosis / distal tibiofibular stiffness (lateral and rotational), intact (paper to identify) | wanted | Spring constants for the mobile fibula (assumed now) |
| B | Edama M. et al. (2018) Morphological features of the ATFL by the number of fiber bundles. *Ann Anat* 216:69-74 | abstract only | Per-band length, width and angle (81 legs; two-band type 57%) |
| B | Cordier G. et al. (2021) Connecting fibers between ATFL's inferior fascicle and CFL transmit tension between both ligaments. *Knee Surg Sports Traumatol Arthrosc* 29:2511-2516 | wanted | Whether the inferior band and CFL should share load (coupled springs) |
| B | Inman V.T. (1976) *The Joints of the Ankle*, trochlear shape chapter | wanted | Original trochlear wedge and facet-curvature data (medial facet radius smaller than lateral) |
| B | MRI quantification of the impact of ankle position on syndesmosis anatomy (PubMed 27733557) | abstract only (search summary) | Fibular external rotation and lateral translation with dorsiflexion |
| B | Talar tilt angle vs inversion torque at several torques, subtalar fixed (paper to identify) | wanted | A talocrural moment-angle curve; Rasmussen 1982 gives one point (6 deg at 1.5 N·m) |
| C | Edama M. et al. (2019) Effect of the number of ATFL fiber bundles on ankle braking function: a simulation study. *Surg Radiol Anat* 41(1):69-73 | abstract only | Per-band length change in 20 DF / 30 PF / 20 inversion; could set band onsets |

## Subtalar joint and its ligaments

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| A | Subtalar stiffness vs axial load (paper to identify) | wanted | How hindfoot inversion stiffness rises with compression; would check or replace the fitted stiffness |
| B | Fujii T. et al. (2005) Analysis of ankle-hindfoot stability in multiple planes: an in vitro study. *Foot Ankle Int* 26(8):633-637 | wanted | Ankle vs hindfoot stiffness in several planes (cited by Pellegrini) |
| C | Pellegrini M.J. et al. (2016) *J Bone Joint Surg Am* 98:842-848, online appendix (jbjs.org data supplement) | wanted | Per-condition subtalar angulation and drawer tables after each cut |
| C | Tochigi Y. et al. (2004) *Foot Ankle Int* 25(8):588-596 | abstract only | ITCL sectioning: laxity and flexibility numbers; the ITCL's inversion vs eversion role |

## Ligament material, rate dependence and onsets

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| A | Funk J.R. et al. (2000) *J Biomech Eng* 122(1):15-22 | wanted (core.ac.uk copy blocked) | Original QLV / rate-dependent curves for all 8 ankle ligaments; model uses secondary fits (Xu 2019) |
| A | Siegler S., Block J., Schneck C.D. (1988) *Foot Ankle* 8(5):234-242 | wanted | Failure strain and load for 120 ligaments; tibionavicular value (secondary now); tibiospring stiffness (assumed now) |
| A | Attarian D.E. et al. (1985) *Foot Ankle* 6(2):54-58 | wanted | Failure vs loading rate; ATFL/CFL strain at failure. The one source reporting significant rate effects around 100 %/s |
| B | Bonner T.J. et al. (2015) Strain-rate sensitivity of the lateral collateral ligament of the knee. *J Mech Behav Biomed Mater* 41:261-270 | wanted | Basis for Rochelle 2020's claim that rate effects are negligible above 100 %/s |
| B | Ozeki S. et al. (2002) *Foot Ankle Int* 23(9):825-832 | thesis summary only | Journal tables to confirm the lateral zero-strain onsets |
| C | Renstrom P. et al. (1988) *Foot Ankle* 9:59-63 | abstract only | ATFL / CFL strain vs flexion |
| C | Clanton T.O. et al. (2015) *J Bone Joint Surg Am* (lateral ligament radiographic landmarks) | secondary summary only | Original footprint distances used for ATFL/CFL attachments |

## Sprain events and landing rig

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| A | Fong D.T. et al. (2009) *Am J Sports Med* 37(4):822-827 | not reached (Loughborough repository 403) | Lab-captured grade I ATFL sprain: kinematics and the ~632 deg/s inversion-velocity threshold |
| A | Kristianslund E. et al. (2011) *J Biomech* 44:2576 | not reached (NIH Brage) | Lab sprain during cutting with joint moments; peak inversion 130-180 ms after contact |
| A | Lysdal F.G. et al. (2022) *Sports Biomech* (DOI 10.1080/14763141.2022.2035801) | abstract only (AAU copy 403) | Pooled kinematics of 24 sprains: peak inversion 67.5 deg, velocity 974 deg/s (468-1752) |
| A | The peroneus reaction time during sudden inversion test: systematic review (authors to confirm) | abstract only (search summary) | Peroneus longus latency on trapdoors in healthy adults (median 63 ms; brevis 66 ms in one study). Sets the rig's reflex onset |
| A | Peroneal reaction times and eversion motor response in healthy and unstable ankles (authors to confirm) | abstract only (search summary) | 81 subjects, 50 deg inversion: total inversion time 105-110 ms; latency similar with and without instability |
| A | Peroneal reflex amplitude and electromechanical delay on sudden inversion (paper to identify) | wanted | How much eversion force arrives, and when; the rig's outcome depends strongly on it |
| A | Trapdoor inversion velocity and time to peak inversion at 20-30 deg (paper to identify) | wanted | Direct validation of the rig's 416 deg/s at 24 deg |
| B | Gehring D. et al. (2013) How to sprain your ankle: a biomechanical case report of an inversion trauma. *J Biomech* | wanted | Lab sprain kinematics and kinetics |
| B | Fong D.T. et al. (2012) Kinematics analysis of ankle inversion ligamentous sprain injuries in sports: five cases from televised tennis competitions. *Am J Sports Med* | open access PDF located, not yet read | Video-based sprain kinematics; plantarflexion not required for a lateral sprain |

## Smart Ligament (external wearable, semi-active lock)

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| A | Ankle brace / tape mechanical restraint of inversion, with stiffness values (paper to identify) | wanted | Benchmark for an external restraint's load path and effectiveness |
| A | Skin, soft-tissue and shoe-upper compliance under strap loads (paper to identify) | wanted | Series compliance between a shank cuff / heel counter and bone |

## Body segment parameters

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| B | Segment masses and inertias for the shank and foot (paper to identify) | wanted | Body-weight loading and inertia in the landing rig |

## Background (low priority; only secondary sources read so far)

| Paper | What it would confirm |
|---|---|
| Inman V.T. (1976) *The Joints of the Ankle* | Empirical ankle-axis landmarks (via a functional-anatomy text now) |
| Lundberg A. et al. (1989) *J Bone Joint Surg Br* 71(1):94-99 | Moving-axis description (abstract only) |
| Klein P., Mattys S., Rooze M. (1996) *J Biomech* 29:21-30 | Muscle moment arms used in verification (via Brookbush Institute review) |
