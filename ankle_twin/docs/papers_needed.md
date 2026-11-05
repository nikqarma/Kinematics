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

## Next step: talar-tilt restraint (6-DOF build)

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| A | ATFL superior/inferior fascicle robotic study, 2023 (PMID 37232327) | abstract only | Per-band function and onsets, needed for a two-band ATFL (its inferior band may restrain tilt near neutral) |
| A | Robotic ATFL in-situ force under inversion / internal-rotation torque, *Foot Ankle Orthop* 2018 | not reached (DOAJ 403) | ATFL force vs applied torque, a direct check on talar-tilt load sharing |
| A | Bahr R. et al. (1998) *Knee Surg Sports Traumatol Arthrosc* 6:115-121 | wanted | ATFL/CFL forces across positions with 375 N axial compression (loaded tilt behaviour) |
| A | Choisne J. et al. (2012) *J Biomech* 45:46-52 | not reached | Probably the applied inversion torques and kinematics method behind the Choisne 2013 benchmark |
| B | Choisne J. et al. (2013) *J Orthop Res* 31:1869-1875 | not reached | Full paper version of the ISB 2013 abstract used now (ankle vs subtalar inversion after CFL / intrinsic ligament cuts, 9 feet) |
| B | "Systematic quantification of stabilizing effects of subtalar joint soft-tissue constraints in a novel cadaveric model" (found via repositorio.uchile.cl; journal and citation to confirm) | not reached | Sequential-sectioning subtalar constraint data |

Data still wanted with no specific paper identified: **intact talar tilt angle vs applied inversion torque**
(a moment-angle curve for the talocrural joint alone, subtalar fixed).

## Subtalar ligaments (CL, ITCL)

| Priority | Paper | Status | What it would give |
|---|---|---|---|
| A | Jotoku T. et al. (2006) Anatomy of ligamentous structures in the tarsal sinus and canal. *Foot Ankle Int* 27(7):533-538 | wanted | ITCL and anterior capsular ligament dimensions; would replace the 10 mm ITCL placeholder |
| A | Michels F. et al. (2022) *Foot Ankle Surg* 28(7):1014-1020 | abstract only | Full text: lengths, elongation / strain at failure, which could set the CL onset (the CL currently fails at about 13 deg inversion) |
| B | Tochigi Y. et al. (2004) *Foot Ankle Int* 25(8):588-596 | abstract only | ITCL sectioning: laxity and flexibility numbers; settles the ITCL's inversion vs eversion role |
| B | Kjaersgaard-Andersen P. et al. (1988) *Am J Sports Med* 16(5):512-516 | abstract only | Hindfoot ROM changes after cutting the cervical ligament and ITCL, with the applied torques |
| C | Weindel S. et al. (about 2010) "Subtalar instability: a biomechanical cadaver study", *Arch Orthop Trauma Surg* (volume to confirm) | not reached | Subtalar ROM after sequential sectioning |

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
