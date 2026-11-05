# Lateral ligament literature: step 1 of the recalibration

Purpose: calibrate ATFL / CFL / PTFL in the ankle twin for **inversion-sprain biofidelity**
(LigaSys device simulation). Compiled 2026-11-04. Status of each source: **read** (full text),
**abstract** (abstract / secondary only), **blocked** (open access but not reachable from this
environment), **paywalled**.

## 1. Model vs literature (default `moving`-axis build, commit 7084230)

Model lengths are straight-line attachment-to-attachment; strain is relative to the current
zero-strain (slack) length. Positive angle = dorsiflexion.

| Angle | ATFL mm (strain) | CFL mm (strain) | PTFL mm (strain) |
|---|---|---|---|
| 40 PF | 21.3 (+51%) | 28.6 (-4%) | 11.2 (slack) |
| 20 PF | 17.0 (+20%) | 29.4 (-1%) | 11.5 (slack) |
| 0 | 14.1 (0%) | 29.7 (0%) | 14.6 (slack) |
| 20 DF | 14.3 (+1%) | 29.8 (+0.3%) | 18.9 (+14%) |
| 30 DF | 15.7 (+11%) | 29.9 (+0.5%) | 21.3 (+28%) |

Subtalar inversion of 19 deg (the joint's limit) lengthens the CFL by about 9% but **does not change the ATFL
at all**: the ATFL spans fibula to talus, and the hinge-based model has no talar tilt, internal
rotation or anterior translation.

| Quantity | Model | Literature |
|---|---|---|
| ATFL taut onset | 0 deg (Takao 2021) | **16.2 +/- 2.6 deg PF** (Ozeki 2002, zero-strain reference, n=5); "begins to tense near 20 deg PF" (Takeuchi 2021) |
| PTFL taut onset | 10 deg DF (assumed) | **18.0 +/- 7.5 deg DF** (Ozeki 2002) |
| CFL taut onset | biphasic 20 PF / 5 DF (Takao 2021) | **17.8 +/- 6.4 deg DF** (Ozeki 2002) |
| TCL (deltoid) onset | 0 deg | 9.5 +/- 8.9 deg PF length; taut when dorsiflexed from there (Ozeki 2002) |
| ATFL length, neutral | 14.1 mm | 16.3 +/- 3.0 (de Asla 2009, in vivo); 19.8 +/- 1.9 (Ozeki 2002, cadaver); 12.6 +/- 0.9 dissected fibres (Chapman 2019) |
| CFL length, neutral | 29.7 mm | 28.0 +/- 2.9 (de Asla); 29.9 +/- 4.2 (Ozeki) |
| PTFL length, neutral | 14.6 mm | **23.7 +/- 3.1** (Ozeki) |
| ATFL length change over ROM | 7.4 mm (10 DF to 40 PF) | 6.9 mm centroid distance, max DF to max PF (de Asla); **1.56 +/- 0.76 mm / 7.9%** midsubstance (Ozeki, 40 PF to 30 DF); +3.3% strain 10 DF to 40 PF (Renstrom 1988) |
| PTFL length change over ROM | 10 mm | **1.17 +/- 0.41 mm / 5.9%** (Ozeki) |
| CFL length change over ROM | 1.3 mm | 1.47 +/- 0.65 mm / 5.3% (Ozeki); 3.3 mm (de Asla); near-isometric (Renstrom) |

### Interpretation

1. **The ATFL centroid geometry is plausible.** Its length change matches de Asla's in-vivo centroid
   distance. The problem is the strain reference: with slack at neutral, end-range strain
   reaches 51%. Moving the onset to Ozeki's 16 deg PF lowers strain at 40 PF to about 31%. That is
   still far above the midsubstance strains that transducers measure (under about 8% over the
   full ROM). Centroid distance overstates fibre strain, because the fan-shaped ATFL has fascicles
   that recruit sequentially. A two-fascicle ATFL (superior and inferior bands, as in the 2023
   robotic study, PMID 37232327) with per-fascicle onsets is the likely fix.
2. **The PTFL was too short (14.6 vs 23.7 mm).** Its insertion has been re-placed on the
   posterior process to match Ozeki's length. Its 1.2 mm midsubstance length change cannot
   be matched by any posterior-process vertex: the minimum is 8 mm, the same centroid-vs-gauge
   gap as the ATFL. So this was not a mis-registration.
3. **The CFL is good in flexion.** The Ozeki DF onset should replace the biphasic onset.
4. **Sprain biofidelity needs the 6-DOF talocrural joint.** Inversion sprain loads the ATFL
   through talar internal rotation, anterior translation and tilt (Bahr 1997, Colville 1990,
   Fong 2009). The hinge build cannot produce any of these. The subtalar range (+/-19.5 deg) is also well
   below injury-event inversion (mean peak about 67 deg foot-to-shank, Lysdal 2022), so it will need
   widening, with a contact or ligament stop instead of a range limit.

### Outcome of the recalibration (2026-11-05)

- ATFL/CFL/PTFL onsets are set to Ozeki 2002. The PTFL insertion is re-placed (23.6 mm at neutral).
- End-range strain at 35 PF: ATFL 43% -> 24%. At 25 DF: PTFL 21% -> 5%.
- The Bahr 1997 benchmark is now in `verify_ankle_model.py` (6-DOF builds); 7/16 values fall within 2 SD.
- An onset sweep (ATFL 0..16.2 PF x CFL 5..24.2 DF) gave sum z^2 between 75 and 149, with no good fit
  anywhere: the CFL stays at 165-240 N in talar tilt. The cause is structural. The subtalar hinge sits on its
  +/-19.5 deg hard limit, and no subtalar ligaments share the load. Fixing the subtalar joint
  comes before any further onset tuning.
- The two-band ATFL is deferred until there is per-band data.

### Subtalar step (2026-11-05): the diagnosis above was only partly right

- Adding the cervical ligament (CL) and the ITCL took the intact subtalar joint off its stop: at 5.7 N·m and neutral
  it sits at 7.8 deg, where it used to sit on the 19.5 deg stop.
- **The CFL did not unload: 227 N, was 238 N.** The inversion moved into talar tilt instead (7.9 -> 13.4 deg).
- Choisne 2013 measured the opposite split in cadavers: subtalar 13.5 deg, talar 3.4 deg. So the CFL overload comes
  from too-lax talar tilt. At neutral the CFL is the only tilt restraint: the ATFL is slack (Ozeki onset), there is no
  capsule, and the medial facet is a thin rim. The subtalar joint was not the cause.
- **What is next:** restrain talar tilt. Candidate fixes are anatomical facet and capsule restraint, or a two-band
  ATFL whose inferior band is taut near neutral.
- The subtalar stop (+/-19.5 deg) is kept. With the intrinsic ligaments cut, Choisne measured 17.7 +/- 4.3 deg, so the
  stop now models what is left: articular surfaces and capsule. A trial with a 40 deg guard ran to the guard at 2 N·m
  once the CFL, CL and ITCL were cut.
- Open: the CL is taut from subtalar neutral, an assumption, and reaches its 163.5 N failure load at about 13 deg
  subtalar inversion, i.e. normal end range. It needs a zero-strain onset that has not been found.

## 2. Validation targets for inversion

**Bahr 1997** (Acta Orthop Scand 68(5):435-441, **read**). 8 cadavers. Ligament force by buckle
transducer, as change from reference. The tibia is free in internal/external rotation; the foot plate
is free in AP and ML translation.

| Flexion | Talar tilt test, 5.7 N.m supination: ATFL N | CFL N | Drawer test, 80 N anterior: ATFL N | CFL N |
|---|---|---|---|---|
| 10 DF | 26 (4) | 123 (19) | 33 (6) | 40 (6) |
| 0 | 36 (6) | 81 (16) | 23 (5) | 15 (5) |
| 10 PF | 64 (18) | 60 (11) | 34 (6) | 6 (3) |
| 20 PF | 123 (38) | 32 (12) | 53 (24) | -3 (2) |

Mean (SE). Cutting the ATFL adds about 2 mm anterior translation in PF. Also cutting the CFL adds 2-4 mm more,
and supination increases significantly. Motion magnitudes are in bar charts only (Figs 3 and 4).

**Colville 1990** (Am J Sports Med 18(2):196-200, **read**; the PDF stays local in `literature/`):
- **Specimens and setup:** 10 cadaver ankles. The subtalar joint was fixed with two Schanz screws.
- **Gauges:** mercury strain gauges at midsubstance, zeroed at each ligament's most lax position. Only the long PTFL
  fibres were gauged, and the whole ATFL (no per-band data).
- **Protocol:** the ankle was swept from 20 DF to 30 PF, unloaded and with 3 N·m inversion, eversion, internal and
  external rotation moments on the talus.
- **ATFL:** +15% over the 50 deg range, steepest between neutral and 20 PF. Inversion and internal rotation increase it
  at every angle, most in PF. Eversion and external rotation decrease it.
- **CFL:** decreases from 20 DF to about 20 PF, then rises slightly. Inversion and external rotation increase it at all
  angles. Strains stay low.
- **PTFL:** increases toward both ends of the range. External rotation increases it; inversion slightly decreases it.
- **Figure scale caveat:** the printed y-axis ticks do not fit the text. Read with the tick spacing, the ATFL rise is
  about 10%, but the text says 15%. Treat the figures as trend data.

**Choisne 2013** (ISB congress abstract, Table 1, **read**). 9 specimens, manual inversion to end range (torque not
reported). Mean (SD), deg:

| | Subtalar inversion | Subtalar eversion | Talocrural inversion |
|---|---|---|---|
| Intact, neutral | 13.46 (3.48) | 8.55 (3.74) | 3.43 (3.08) |
| Intact, DF | 10.31 (2.91) | 8.18 (1.84) | 1.85 (1.56) |
| Intact, PF | 12.16 (2.94) | 4.31 (1.73) | 3.35 (1.72) |
| CFL cut, neutral | 15.18 (3.81) | 9.52 (3.94) | 8.22 (4.68) |
| CFL + cervical + ITCL cut, neutral | 17.73 (4.29) | 9.12 (3.43) | 8.64 (5.44) |

**Subtalar ligament material** (Michels 2022, Foot Ankle Surg 28:1014, abstract, 18 pairs):

| Ligament | Stiffness (N/mm) | Failure load (N) |
|---|---|---|
| CFL | 55.8 +/- 23.0 | 320.4 +/- 122.0 |
| CL | 63.9 +/- 38.0 | 163.5 +/- 58.0 |
| ACaL-ITCL complex | 150 +/- 51 | 382.5 +/- 158 |

The CFL values sit close to Rochelle 2020's, which the model keeps.

**Subtalar ligament anatomy:** Poonja 2017 (Cureus 9:e1382, full text, n=26).
- **CL dimensions:** height 8.3 (anterior) / 14.0 (posterior) mm; width 9.2 (superior) / 11.9 (inferior) mm;
  thickness about 1 mm.
- **ITCL:** no numbers. Its fibres run superomedial to inferolateral across the full width of the sinus.
- **Other sources:** Jotoku 2006 (Foot Ankle Int 27:533) has dimensions but was not accessible. Tochigi 2004 (n=6,
  abstract): the ITCL keeps the joint apposed and resists lateral-to-medial drawer.

**Sprain-event kinematics:** Lysdal 2022 review (24 cases). Mean peak inversion about 67.5 deg; peak
inversion velocity about 974 deg/s; mean over the event about 337 deg/s (range 117-1400). Inversion is
dominant, followed by internal rotation and plantarflexion.

**Material:** Chapman 2019 / UCLan (**read**; the same data the model already uses as
"Rochelle2020"). Tested at 100%/s. Stiffness ATFL 44.7, CFL 45.8, PTFL 59.0 N/mm. Failure 263.6 / 367.8 / 351.4 N.
Dissected lengths 12.6 / 20.0 / 13.4 mm. No failure strain was reported.

## 3. Sources

| Source | Status | What it gives |
|---|---|---|
| de Asla 2009, J Orthop Surg Res 4:7 | read | In-vivo ATFL/CFL centroid lengths at 5 positions (n=4) |
| Ozeki 2002, Foot Ankle Int 23(9):825-832 | abstract (thesis summary, Hokkaido) | Zero-strain onsets, lengths, ROM length change (above) |
| Bahr 1997, Acta Orthop Scand 68:435 | read | Drawer and talar-tilt ligament forces |
| Chapman 2019 (UCLan) | read | Rate-matched stiffness and failure |
| Takeuchi 2021, BMC Musculoskelet Disord 22:208 (MLPP) | read | ATFL strain normalised to max; peaks in inversion; tenses about 20 deg PF |
| Renstrom 1988, Foot Ankle 9:59-63 | abstract | ATFL +3.3% from 10 DF to 40 PF; CFL isometric |
| Colville 1990, Am J Sports Med 18(2):196-200 | read (user-supplied PDF, kept local) | ATFL/CFL/PTFL strain vs flexion, inversion and IR (10 cadavers); whole-ATFL only |
| Choisne 2013, ISB congress abstract | read | Subtalar vs talocrural inversion, intact and after CFL/CL/ITCL sectioning |
| Michels 2022, Foot Ankle Surg 28:1014 | abstract | CL and ACaL-ITCL stiffness and failure |
| Poonja 2017, Cureus 9:e1382 | read | Cervical ligament dimensions |
| Tochigi 2004, Foot Ankle Int 25:588 | abstract | ITCL function (drawer, distraction) |
| Jotoku 2006, Foot Ankle Int 27:533 | **paywalled** | ITCL and anterior capsular ligament dimensions |
| Kjaersgaard-Andersen 1988, Am J Sports Med 16:512 | abstract | Hindfoot ROM after CL/ITCL sectioning (max +2.6 deg) |
| Choisne 2012, J Biomech 45:46-52 | not reached | Subtalar/ankle kinematics method; likely has applied torques |
| Bahr 1998, KSSTA 6:115-121 | **paywalled** | ATFL/CFL force across positions with 375 N compression |
| Funk 2000, J Biomech Eng 122(1):15-22 | **paywalled** (core.ac.uk copy blocked) | QLV (rate-dependent) curves for all 8 ankle ligaments |
| Siegler 1988, Foot Ankle 8(5):234-242 | **paywalled** | Failure strain and load, 120 ligaments |
| Attarian 1985, Foot Ankle 6(2):54-58 | **paywalled** | Failure vs loading rate; ATFL/CFL strain at failure |
| Fong 2009, Am J Sports Med 37(4):822-827 | **blocked** (Loughborough repository 403) | Lab-captured grade I ATFL sprain kinematics |
| Kristianslund 2011, J Biomech 44:2576 | **blocked** (NIH Brage unreachable) | Sprain reconstruction with joint moments |
| Robotic ATFL in-situ force, Foot Ankle Orthop 2018 | **blocked** (DOAJ 403) | In-situ ATFL force under inversion/IR torque |
| ATFL fascicles robotic study 2023 (PMID 37232327) | abstract | Superior vs inferior fascicle function |
| Lysdal 2022, Sports Biomech | abstract | Pooled sprain kinematics |
