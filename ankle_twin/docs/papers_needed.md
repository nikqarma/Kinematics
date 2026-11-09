# Papers needed for the ankle twin

This is a running log of papers that could not be read in full and would improve the model. Drop PDFs into `ankle_twin/literature/`,
which is gitignored so copyrighted papers are never committed; a working copy of this log also lives there.

The **Priority** column ranks by impact on inversion-sprain biofidelity:

- **A**: blocks the next modelling step, or replaces an assumed value.
- **B**: replaces a secondary or abstract-only value already in the model.
- **C**: extra validation or background.

Started 2026-11-05. Status key: *wanted* (none of it was readable), *abstract only* (just the abstract was read),
*not reached* (the site blocked access), *have* (a PDF is in this folder).

## Have

| Paper | Status | Used for |
|---|---|---|
| Colville M.R. et al. (1990) Strain measurement in lateral ankle ligaments. *Am J Sports Med* 18(2):196-200 | have | Colville benchmark in `verify_ankle_model.py`. It gauges the whole ATFL only, with no per-band data |
| Edama M. et al. (2025) Differences in ankle stabilizing function between the upper and lower fiber bundles of the ATFL. *Sci Rep* 15:26126 (open access, CC BY-NC-ND) | have | Qualitative per-band check only, no onsets or stiffness (see note below) |

**Edama 2025 per-band findings:**
- **Setup:** 10 Thiel-embalmed feet from donors of about 90 years, 5 per group. A Telos device applied 120 N, and ultrasound measured the malleolus–talus distance.
- **Drawer:** cutting either band increased drawer at 30 PF only.
- **Inversion:** cutting the lower band increased inversion opening at 0 deg. The abstract says it was the upper band, but Table 3 and the results say lower; Table 3 is taken as correct.
- **No group difference:** the upper-cut and lower-cut groups never differed significantly. SDs are often larger than the effects.
- **Use:** supports a two-band ATFL with the lower band acting near neutral in inversion, coupled to the CFL by arciform fibres. It cannot set band onsets, stiffness or geometry.

| Paper | Status | Used for |
|---|---|---|
| Vega J., Malagelada F., Manzanares Céspedes M.C., Dalmau-Pastor M. (2020) The lateral fibulotalocalcaneal ligament complex: an ankle stabilizing isometric structure. *Knee Surg Sports Traumatol Arthrosc* 28:8-17 | have (accepted manuscript; **Table 1 and figures missing**) | Per-band ATFL lengths in PF/DF: the superior band lengthens (12.6 -> 19.2 mm), the inferior band is isometric (10.6 mm). Basis for the two-band ATFL |
| Rasmussen O., Tovborg-Jensen I. (1982) Mobility of the ankle joint. *Acta Orthop Scand* 53:155-160 | have | Talar tilt and internal rotation at 1.5 N·m, subtalar fixed, after serial ATFL/CFL/PTFL cuts. Shows the model's ATFL does nothing at neutral |
| Pellegrini M.J. et al. (2016) Systematic quantification of stabilizing effects of subtalar joint soft-tissue constraints in a novel cadaveric model. *J Bone Joint Surg Am* 98:842-848 | have (online appendix tables not included) | Subtalar vs ankle share of hindfoot motion at 4 N·m; CFL/CL/ITCL sectioning. Contradicts Choisne 2013; shows the CFL, not the CL, is the main subtalar restraint |

Findings and model comparison are in `docs/lateral_ligament_literature.md` under "Talar-tilt step (2026-11-06)".

## Next step: talar-tilt restraint (6-DOF build)

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| A | Vega J. et al. (2020), published version or its Table 1 | have manuscript only | Per-band widths (to split the whole-ATFL stiffness between bands) and footprint figures |
| A | Dalmau-Pastor M. et al. (2023) Clinical relevance and function of ATFL superior and inferior fascicles: a robotic study. *Am J Sports Med* 51(8):2169-2175 (PMID 37232327) | abstract only | 6-DOF robot, 10 cadavers, serial sectioning superior -> inferior: per-band contribution to drawer, internal rotation and inversion through flexion. The best source for a two-band ATFL. Note: the inferior band was never cut alone |
| A | Edama M. et al. (2018) Morphological features of the ATFL by the number of fiber bundles. *Ann Anat* 216:69-74 | abstract only | Per-band length, width and angle (81 legs; two-band type 57%). Sets band geometry |
| B | Edama M. et al. (2019) Effect of the number of ATFL fiber bundles on ankle braking function: a simulation study. *Surg Radiol Anat* 41(1):69-73 | abstract only | Per-band length change in 20 DF / 30 PF / 20 inversion: the upper band stretches in DF and PF, the lower band shortens in PF and stretches in DF (per Edama 2025). Could set band onsets |
| B | Ozeki S. et al. (2006) Ankle ligament tensile forces at the end points of passive circumferential rotating motion. *Foot Ankle Int* 27(11):965-969 | abstract only | Peak ATFL/CFL/PTFL/TCL forces at end range (8 cadavers): ATFL 108-130 N in supination + PF. A force-magnitude benchmark |
| B | Cordier G. et al. (2021) Connecting fibers between ATFL's inferior fascicle and CFL transmit tension between both ligaments. *Knee Surg Sports Traumatol Arthrosc* 29:2511-2516 | wanted | Whether the inferior band and CFL should share load (coupled springs) |
| C | Kakegawa A. et al. (2022) Relationship between inferior fascicle of ATFL and articular capsule. *Surg Radiol Anat* 44:253-259 | wanted | Inferior band–capsule attachment, relevant to capsule restraint of tilt |
| B | Pellegrini M.J. et al. (2016) *J Bone Joint Surg Am* 98:842-848, online appendix (jbjs.org data supplement) | wanted | Per-condition subtalar angulation and drawer tables after each cut; needed to benchmark the sectioning results quantitatively |
| B | Fujii T. et al. (2005) Analysis of ankle-hindfoot stability in multiple planes: an in vitro study. *Foot Ankle Int* 26(8):633-637 | wanted | Ankle vs hindfoot stiffness in several planes (cited by Pellegrini) |
| A | Robotic ATFL in-situ force under inversion / internal-rotation torque, *Foot Ankle Orthop* 2018 | not reached (DOAJ 403) | ATFL force vs applied torque, a direct check on talar-tilt load sharing |
| A | Bahr R. et al. (1998) *Knee Surg Sports Traumatol Arthrosc* 6:115-121 | wanted | ATFL/CFL forces across positions with 375 N axial compression (loaded tilt behaviour) |
| A | Choisne J. et al. (2012) *J Biomech* 45:46-52 | not reached | Probably the applied inversion torques and kinematics method behind the Choisne 2013 benchmark |
| B | Choisne J. et al. (2013) *J Orthop Res* 31:1869-1875 | not reached | Full paper version of the ISB 2013 abstract used now (ankle vs subtalar inversion after CFL / intrinsic ligament cuts, 9 feet) |

Partly answered by Rasmussen 1982 (one point: 6 deg at 1.5 N·m): **intact talar tilt angle vs applied inversion
torque** at several torques (a moment-angle curve for the talocrural joint alone, subtalar fixed) is still wanted.

## Subtalar ligaments (CL, ITCL)

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| A | Jotoku T. et al. (2006) Anatomy of ligamentous structures in the tarsal sinus and canal. *Foot Ankle Int* 27(7):533-538 | wanted | ITCL and anterior capsular ligament dimensions; would replace the 10 mm ITCL placeholder |
| A | Michels F. et al. (2022) *Foot Ankle Surg* 28(7):1014-1020 | abstract only | Full text: lengths, elongation / strain at failure, which could set the CL onset (the CL currently fails at about 13 deg inversion) |
| B | Tochigi Y. et al. (2004) *Foot Ankle Int* 25(8):588-596 | abstract only | ITCL sectioning: laxity and flexibility numbers; settles the ITCL's inversion vs eversion role |
| B | Kjaersgaard-Andersen P. et al. (1988) *Am J Sports Med* 16(5):512-516 | abstract only | Hindfoot ROM changes after cutting the cervical ligament and ITCL, with the applied torques |
| C | Weindel S. et al. (2010) Subtalar instability: a biomechanical cadaver study. *Arch Orthop Trauma Surg* 130(3):313-319 | not reached | Subtalar ROM after sequential sectioning |
| B | Kjaersgaard-Andersen P., Wethelund J.O., Nielsen S. (1987) Lateral talocalcaneal instability following section of the calcaneofibular ligament. *Foot Ankle* 7(6):355-361 | wanted | Subtalar motion after CFL section: checks the CFL as the main subtalar restraint (Pellegrini) |
| B | Knudson G.A. et al. (1997) Subtalar joint stability: talocalcaneal interosseous ligament function. *Acta Orthop Scand* 68(5):442-446 | wanted (likely open access, like Rasmussen 1982 in the same journal) | ITCL function with measured loads |
| C | Tochigi Y. et al. (2000) Influence of the ITCL injury on stability of the ankle-subtalar joint complex. *Foot Ankle Int* 21(6):486-491 | wanted | ITCL role in combined ankle-subtalar stability |
| C | Leardini A., Stagni R., O'Connor J.J. (2001) Mobility of the subtalar joint in the intact ankle complex. *J Biomech* 34(6):805-809 | wanted | Subtalar mobility with the ankle intact |

## Lateral ligament material and onsets

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| A | Funk J.R. et al. (2000) *J Biomech Eng* 122(1):15-22 | wanted (core.ac.uk copy blocked) | Original QLV / rate-dependent curves for all 8 ankle ligaments; model uses secondary fits (Xu 2019 Table 1) |
| A | Siegler S., Block J., Schneck C.D. (1988) *Foot Ankle* 8(5):234-242 | wanted | Failure strain and load for 120 ligaments; original tibionavicular value (secondary now); tibiospring stiffness (assumed now) |
| B | Ozeki S. et al. (2002) *Foot Ankle Int* 23(9):825-832 | thesis summary only | Journal tables to confirm the zero-strain onsets the lateral ligaments now use |
| B | Attarian D.E. et al. (1985) *Foot Ankle* 6(2):54-58 | wanted | Failure vs loading rate; ATFL/CFL strain at failure (rupture criteria for sprain simulation) |
| C | Renstrom P. et al. (1988) *Foot Ankle* 9:59-63 | abstract only | ATFL / CFL strain vs flexion (cross-check of Ozeki) |
| C | Clanton T.O. et al. (2015) *J Bone Joint Surg Am* (lateral ligament radiographic landmarks) | secondary summary only | Original footprint distances used for ATFL/CFL attachments |

## Sprain-event kinematics (validation targets)

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| B | Fong D.T. et al. (2009) *Am J Sports Med* 37(4):822-827 | not reached (Loughborough repository 403) | Lab-captured grade I ATFL sprain kinematics |
| B | Kristianslund E. et al. (2011) *J Biomech* 44:2576 | not reached (NIH Brage) | Sprain reconstruction with joint moments |
| C | Lysdal F.G. et al. (2022) *Sports Biomech* | abstract only | Full pooled kinematics of 24 sprains (peak inversion, velocity, timing) |

## Background (low priority; only secondary sources read so far)

| Paper | What it would confirm |
|---|---|
| Inman V.T. (1976) *The Joints of the Ankle* | Empirical ankle-axis landmarks (via a functional-anatomy text now) |
| Lundberg A. et al. (1989) *J Bone Joint Surg Br* 71(1):94-99 | Moving-axis description (abstract only) |
| Klein P., Mattys S., Rooze M. (1996) *J Biomech* 29:21-30 | Muscle moment arms used in verification (via Brookbush Institute review) |
