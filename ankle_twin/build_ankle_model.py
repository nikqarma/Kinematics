"""Extract a standalone right shank-ankle-foot model from MS-Human-700.

The output is a MuJoCo model of a bench-top rig: the femur is welded in
place, the knee is locked at a chosen angle (or left free), and the
talocrural, subtalar and MTP joints are driven by the 11 muscles that
cross the ankle. Geometry, joint axes, muscle paths and muscle parameters
are copied unchanged from MS-Human-700. Nothing is re-tuned.

Ankle ligaments (see ligaments.py) are added as tension-only linear springs
on straight-line spatial tendons. Each ligament's slack length is its length
at the ankle angle where it is reported (or assumed) to become taut.

Usage:
    python build_ankle_model.py                    # knee locked at 0 deg
    python build_ankle_model.py --knee-angle 30    # knee locked at 30 deg flexion
    python build_ankle_model.py --free-knee        # knee kept as a passive DOF
    python build_ankle_model.py --no-ligaments     # muscles only, as extracted
    python build_ankle_model.py --six-dof --out model_6dof   # EXPERIMENTAL 6-DOF talocrural joint
"""

from __future__ import annotations

import argparse
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

import ankle_axis as AX
import couplings as CPL
import ligaments as LIG
import talocrural as TC

HERE = Path(__file__).resolve().parent
DEFAULT_SRC = HERE.parent / "MS-Human-700"

# Muscles whose paths cross the ankle (talocrural and/or subtalar joints).
ANKLE_MUSCLES = [
    "gasmed_r", "gaslat_r", "soleus_r",          # triceps surae
    "tibpost_r", "fdl_r", "fhl_r",               # deep posterior compartment
    "perlong_r", "perbrev_r",                    # lateral compartment
    "tibant_r", "edl_r", "ehl_r",                # anterior compartment
]

MESHES = ["r_femur", "r_tibia", "r_fibula", "r_talus", "r_foot", "r_bofoot"]

HIP_JOINTS = {"hip_flexion_r", "hip_adduction_r", "hip_rotation_r"}
# Knee DOFs coupled to knee_angle_r by polynomial equality constraints.
KNEE_COUPLED = [
    "knee_angle_r_translation1", "knee_angle_r_translation2",
    "knee_angle_r_translation3", "knee_angle_r_rotation2",
    "knee_angle_r_rotation3",
]

# Same orientation as the MS-Human-700 pelvis: OpenSim Y-up -> MuJoCo Z-up.
MOUNT_QUAT = "0.707107 0.707107 0 0"
MOUNT_HEIGHT = 1.0  # m; puts the foot clear of the (visual-only) floor.

# Subtalar range (rad, + = inversion): MS-Human-700's hard stop at +-0.34 rad
# (19.5 deg), kept on purpose as the bony limit. Before the cervical and
# interosseous ligaments were added, every inversion test ended on this stop.
# Inside it, inversion is resisted by the ligaments and by the fitted
# inversion stiffness below. Choisne 2013 (manual end range, n=9): 13.5 +/- 3.5 deg
# intact, 17.7 +/- 4.3 deg with CFL + cervical + ITCL cut; eversion 8.6 +/- 3.7.
# A 40 deg guard was tried: with CFL + cervical + ITCL cut the joint then ran to
# the guard at 2 N*m, i.e. nothing else in the model limits it.
SUBTALAR_RANGE = (-0.34, 0.34)

# Subtalar inversion stiffness (ligasys-sprain branch; FITTED, not measured).
# A one-sided rotational spring on subtalar_angle_r, active in inversion from
# neutral (a fixed tendon with a deadband). It stands in for what the 1-DOF
# hinge lacks: the talocalcaneal facets seating under axial load, plus the
# capsule. Without it the subtalar ligaments are slack to 14 deg (Knudson 1997,
# unloaded feet) and the joint ran to the 19.5 deg stop at 4 N*m, far laxer than
# Pellegrini 2016's axially loaded feet. The stiffness is the single parameter
# fitted to Pellegrini 2016 Table II (subtalar and ankle inversion at 4 N*m, four
# flexion angles, both 6-DOF builds: 16 values, minimum sum of squared z-scores
# at 12 N*m/rad over a 6-14 grid; onsets of 0 and 1.5 deg fit equally, so the
# onset is left at neutral). Pellegrini is therefore calibration on this branch,
# not validation. It does not depend on axial load, unlike the real joint.
SUBTALAR_INVERSION_STIFFNESS = 12.0  # N*m/rad


def add_subtalar_endrange(model: ET.Element) -> None:
    tendon = model.find("tendon")
    tendon.append(ET.Comment(
        f" Subtalar inversion stiffness {SUBTALAR_INVERSION_STIFFNESS:g} N*m/rad from neutral: FITTED to "
        "Pellegrini 2016 (articular/capsular stand-in; build_ankle_model.SUBTALAR_INVERSION_STIFFNESS) "))
    fixed = ET.SubElement(tendon, "fixed", {"name": "subtalar_inversion_stiffness",
                                            "stiffness": f"{SUBTALAR_INVERSION_STIFFNESS:g}",
                                            "springlength": "-10 0"})
    ET.SubElement(fixed, "joint", joint="subtalar_angle_r", coef="1")


def load(path: Path) -> ET.Element:
    return ET.parse(path).getroot()


def _fmt(v) -> str:
    return " ".join(f"{c:.6g}" for c in v)


def add_ligaments(model: ET.Element, femur: ET.Element, mesh_dir: Path) -> None:
    """Add attachment sites and tension-only ligament tendons (slack lengths and stiffness set later)."""
    bodies = {b.get("name"): b for b in femur.iter("body")}
    # The attachment code places some insertions via the tibia frame; make sure
    # its body offsets still match the source model.
    talus_pos = np.array([float(c) for c in bodies["talus_r"].get("pos").split()])
    calcn_pos = np.array([float(c) for c in bodies["calcn_r"].get("pos").split()])
    assert np.allclose(LIG.BODY_ORIGIN_IN_TIBIA["talus_r"], talus_pos, atol=1e-6), "talus offset changed"
    assert np.allclose(LIG.BODY_ORIGIN_IN_TIBIA["calcn_r"], talus_pos + calcn_pos, atol=1e-6), "calcn offset changed"
    pts = LIG.attachments(mesh_dir)

    default = model.find("default")
    lig_default = ET.SubElement(default, "default", {"class": "ligament"})
    ET.SubElement(lig_default, "tendon", width="0.0025", rgba="0.35 0.75 0.95 1",
                  damping=f"{LIG.LIGAMENT_DAMPING:g}")
    ET.SubElement(lig_default, "site", size="0.0025", rgba="0.35 0.75 0.95 1", group="4")

    seg_default = ET.SubElement(default, "default", {"class": "ligament_segment"})
    ET.SubElement(seg_default, "tendon", width="0.001", rgba="0 0 0 0", damping="0")

    tendon = model.find("tendon")
    tendon.append(ET.Comment(
        " Ankle ligaments: tension-only springs (springlength deadband). Exponential force-strain "
        "curves are stacks of parallel springs (<name>__sN) with staggered deadbands. "
        "Parameters and sources in ligaments.py "))
    for lig in LIG.LIGAMENTS:
        p1, p2 = pts[lig.name]
        for body, p, tag in ((lig.body1, p1, "P1"), (lig.body2, p2, "P2")):
            # Insert before child bodies so the site belongs to this body's own block.
            b = bodies[body]
            first_child = next((i for i, c in enumerate(b) if c.tag == "body"), len(b))
            b.insert(first_child, ET.Element("site", {"name": f"{lig.name}-{tag}", "class": "ligament",
                                                       "pos": _fmt(p)}))
        onset = " / ".join(f"{a:+g}" for a in lig.taut_ankle_deg)
        if lig.curve[0] == "linear":
            law = f"linear k={lig.curve[1] / 1000:.1f} N/mm"
            n_seg = 1
        else:
            law = f"T = {lig.curve[1]:g}*(exp({lig.curve[2]:g}*strain)-1) N"
            n_seg = len(LIG.EXP_BREAKS)
        tendon.append(ET.Comment(
            f" {lig.full_name}: {law} ({lig.stiffness_source}); "
            f"taut from {onset} deg ankle ({lig.taut_source}) "))
        for j in range(n_seg):
            name, cls = (lig.name, "ligament") if j == 0 else (f"{lig.name}__s{j}", "ligament_segment")
            sp = ET.SubElement(tendon, "spatial", {"name": name, "class": cls,
                                                   "stiffness": "0", "springlength": "0 0"})
            ET.SubElement(sp, "site", site=f"{lig.name}-P1")
            ET.SubElement(sp, "site", site=f"{lig.name}-P2")


def set_slack_lengths(xml_path: Path) -> dict[str, float]:
    """Set each slack length L0 to the longest length over the ligament's taut-onset ankle angles,
    then each spring segment's deadband start (L0 * (1 + strain_j)) and stiffness."""
    m = mujoco.MjModel.from_xml_path(str(xml_path))
    d = mujoco.MjData(m)
    slack = {}
    for lig in LIG.LIGAMENTS:
        lengths = []
        for angle in lig.taut_ankle_deg:
            CPL.pose(m, d, {lig.taut_joint: math.radians(angle)}, forward=False)
            lengths.append(float(d.ten_length[m.tendon(lig.name).id]))
        slack[lig.name] = max(lengths)

    springs = {}
    for lig in LIG.LIGAMENTS:
        L0 = slack[lig.name]
        for j, (strain, k) in enumerate(LIG.spring_segments(lig, L0)):
            springs[lig.name if j == 0 else f"{lig.name}__s{j}"] = (L0 * (1 + strain), k)

    tree = ET.parse(xml_path)
    for sp in tree.getroot().find("tendon").iter("spatial"):
        if sp.get("name") in springs:
            start, k = springs[sp.get("name")]
            sp.set("springlength", f"0 {start:.6g}")
            sp.set("stiffness", f"{k:.6g}")
    header = xml_path.read_text(encoding="utf-8").split("\n", 2)
    ET.indent(tree.getroot(), space="  ")
    xml_path.write_text(header[0] + "\n" + header[1] + "\n" + ET.tostring(tree.getroot(), encoding="unicode") + "\n",
                        encoding="utf-8")
    return slack


def set_ankle_axis(model: ET.Element, femur: ET.Element, kind: str, mesh_dir: Path,
                   swing_deg: float = AX.SWING_DEG) -> None:
    """Point the talocrural hinge at the chosen axis (see ankle_axis.py). The neutral pose is unchanged.

    For the moving axis, a second hinge (ankle_axis_tilt_r) about the inferior
    direction (perpendicular to the axis) through the same centre is added after the flexion hinge and
    coupled to it by tilt = c * flexion**2.
    """
    talus = next(b for b in femur.iter("body") if b.get("name") == "talus_r")
    joint = next(j for j in talus.iter("joint") if j.get("name") == "ankle_angle_r")
    point, u = AX.axis(kind, mesh_dir)
    joint.set("pos", _fmt(point))
    joint.set("axis", _fmt(u))
    if kind != "moving":
        return
    lo, hi = (float(v) for v in joint.get("range").split())
    m_axis, c = AX.moving_axis_coupling(u, (lo, hi), swing_deg)
    idx = list(talus).index(joint)
    # Range is a guard only: the coupling keeps |tilt| <= c * max(q^2) ~ 0.06 rad.
    talus.insert(idx + 1, ET.Element("joint", {
        "name": "ankle_axis_tilt_r", "class": "ankle", "pos": _fmt(point), "axis": _fmt(m_axis),
        "range": "-0.3 0.3"}))
    equality = model.find("equality")
    equality.append(ET.Comment(
        f" Moving talocrural axis (ankle_axis.py): tilt = {c:.6g} * flexion^2 gives a "
        f"{swing_deg:g} deg change in coronal inclination across the flexion range "))
    ET.SubElement(equality, "joint", name="ankle_axis_coupling", joint1="ankle_axis_tilt_r",
                  joint2="ankle_angle_r", polycoef=f"0 0 {c:.6g} 0 0", solimp="0.9999 0.9999")


def build(src: Path, out_dir: Path, knee_angle_deg: float | None, ligaments: bool = True,
          six_dof: bool = False, axis: str = "moving", swing_deg: float = AX.SWING_DEG) -> Path:
    if six_dof and axis == "moving":
        raise ValueError("the 6-DOF contact joint needs a fixed axis (its trochlea is a cylinder about it): "
                         "use axis='inman' or 'original'")
    main = load(src / "MS-Human-700.xml")
    leg = load(src / "Body_Primary" / "Body_Leg_Foot_r.xml")
    tendons_src = load(src / "Tendon" / "Tendon_Leg_r.xml")
    muscles_src = load(src / "Muscle" / "Muscle_Leg_r.xml")
    eq_src = load(src / "Equality" / "Equality_Lowerbody.xml")

    # --- tendons and muscles -------------------------------------------------
    tendon_names = {f"{m}_tendon" for m in ANKLE_MUSCLES}
    tendons = [t for t in tendons_src.find("tendon") if t.get("name") in tendon_names]
    muscles = [a for a in muscles_src.find("actuator") if a.get("name") in ANKLE_MUSCLES]
    assert len(tendons) == len(ANKLE_MUSCLES), "missing tendon definitions"
    assert len(muscles) == len(ANKLE_MUSCLES), "missing muscle definitions"

    used_sites, used_geoms = set(), set()
    for t in tendons:
        for el in t:
            if el.tag == "site":
                used_sites.add(el.get("site"))
            elif el.tag == "geom":
                used_geoms.add(el.get("geom"))
                if el.get("sidesite"):
                    used_sites.add(el.get("sidesite"))

    # --- body tree -----------------------------------------------------------
    femur = leg.find("body")
    assert femur.get("name") == "femur_r"
    femur.set("pos", "0 0 0")

    def prune(body: ET.Element) -> None:
        for child in list(body):
            if child.tag == "joint" and child.get("name") in HIP_JOINTS:
                body.remove(child)  # femur is welded to the rig
            elif child.tag == "body" and child.get("name") == "patella_r":
                body.remove(child)  # only quadriceps use the patella
            elif child.tag == "site" and child.get("name") not in used_sites:
                body.remove(child)
            elif (child.tag == "geom" and child.get("class") == "wrap"
                  and child.get("name") not in used_geoms):
                body.remove(child)
            elif child.tag == "body":
                prune(child)

    prune(femur)

    # --- assemble model ------------------------------------------------------
    model = ET.Element("mujoco", model="MS_Human_700_Ankle_Foot_r")
    compiler = ET.SubElement(model, "compiler", main.find("compiler").attrib)
    compiler.set("meshdir", "meshes")
    model.append(main.find("visual"))
    model.append(main.find("default"))

    asset = ET.SubElement(model, "asset")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient",
                  rgb1="0.9 0.9 0.95", rgb2="0.6 0.65 0.7", width="512", height="512")
    ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker",
                  rgb1="0.75 0.75 0.75", rgb2="0.65 0.65 0.65", width="300", height="300")
    ET.SubElement(asset, "material", name="grid", texture="grid",
                  texrepeat="8 8", reflectance="0.1")
    for m in MESHES:
        ET.SubElement(asset, "mesh", name=m, content_type="model/stl", file=f"{m}.stl")

    world = ET.SubElement(model, "worldbody")
    ET.SubElement(world, "light", pos="0 -1 2.5", dir="0 0.4 -1", directional="true")
    # Visual reference floor only; it does not collide with the foot.
    ET.SubElement(world, "geom", name="floor", type="plane", size="1 1 0.05",
                  material="grid", contype="0", conaffinity="0")
    mount = ET.SubElement(world, "body", name="rig_mount",
                          pos=f"0 0 {MOUNT_HEIGHT}", quat=MOUNT_QUAT)
    mount.append(femur)
    ET.SubElement(world, "camera", name="side", pos="0 -1.4 0.55",
                  xyaxes="1 0 0 0 0.3 1", mode="fixed")
    ET.SubElement(world, "camera", name="front", pos="1.4 0 0.55",
                  xyaxes="0 1 0 -0.3 0 1", mode="fixed")

    contact = ET.SubElement(model, "contact")
    for b1, b2 in [("femur_r", "tibia_r"), ("tibia_r", "calcn_r")]:
        ET.SubElement(contact, "exclude", body1=b1, body2=b2)

    equality = ET.SubElement(model, "equality")
    for eq in eq_src.find("equality"):
        if eq.get("joint1") in KNEE_COUPLED:
            equality.append(eq)
    if knee_angle_deg is not None:
        k = math.radians(knee_angle_deg)
        ET.SubElement(equality, "joint", name="knee_lock", joint1="knee_angle_r",
                      polycoef=f"{k:.6g} 0 0 0 0", solimp="0.9999 0.9999")

    tendon_el = ET.SubElement(model, "tendon")
    tendon_el.extend(tendons)
    actuator = ET.SubElement(model, "actuator")
    actuator.extend(muscles)

    # --- write ---------------------------------------------------------------
    mesh_dir = out_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for m in MESHES:
        shutil.copy2(src / "Geometry" / f"{m}.stl", mesh_dir / f"{m}.stl")

    set_ankle_axis(model, femur, axis, mesh_dir, swing_deg)
    if ligaments:
        add_ligaments(model, femur, mesh_dir)
        subtalar = next(j for j in femur.iter("joint") if j.get("name") == "subtalar_angle_r")
        subtalar.set("range", _fmt(SUBTALAR_RANGE))
        add_subtalar_endrange(model)
    if six_dof:
        TC.add_joint(model, femur, mesh_dir)

    ET.indent(model, space="  ")
    knee = "free" if knee_angle_deg is None else f"locked at {knee_angle_deg:g} deg"
    header = (
        "<!-- Generated by build_ankle_model.py from MS-Human-700 (Apache-2.0).\n"
        f"     Knee: {knee}. Ligaments: {'yes' if ligaments else 'no'}."
        f" Talocrural joint: {'6-DOF with contact' if six_dof else '1-DOF hinge'}, {axis} axis"
        + (f" ({swing_deg:g} deg swing)" if axis == "moving" else "") + "."
        " Do not edit by hand; edit the generator and rebuild. -->\n"
    )
    out = out_dir / "ankle_foot_r.xml"
    out.write_text(header + ET.tostring(model, encoding="unicode") + "\n", encoding="utf-8")
    if ligaments:
        for name, L0 in set_slack_lengths(out).items():
            print(f"  {name:8s} slack length {L0 * 1000:5.1f} mm")
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--src", type=Path, default=DEFAULT_SRC, help="MS-Human-700 directory")
    p.add_argument("--out", type=Path, default=HERE / "model", help="output directory")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--knee-angle", type=float, default=0.0,
                   help="lock knee flexion at this angle in degrees (default 0)")
    g.add_argument("--free-knee", action="store_true", help="leave the knee unlocked")
    p.add_argument("--no-ligaments", action="store_true", help="omit the ankle ligaments")
    p.add_argument("--six-dof", action="store_true",
                   help="EXPERIMENTAL: 6-DOF talocrural joint with articular contact (see talocrural.py)")
    p.add_argument("--axis", choices=AX.KINDS, default=None,
                   help="talocrural axis (see ankle_axis.py): 'moving' = Inman axis at neutral whose "
                        "inclination varies with flexion (default); 'inman' = fixed Inman axis (default "
                        "with --six-dof); 'original' = MS-Human-700's own axis")
    p.add_argument("--swing-deg", type=float, default=AX.SWING_DEG,
                   help=f"moving axis: total change in coronal inclination across the flexion range "
                        f"(default {AX.SWING_DEG:g}, illustrative - see ankle_axis.py)")
    args = p.parse_args()
    if args.six_dof and args.no_ligaments:
        p.error("--six-dof needs the ligaments: without them nothing holds the talus in the mortise")
    axis = args.axis or ("inman" if args.six_dof else "moving")
    if args.six_dof and axis == "moving":
        p.error("--six-dof needs a fixed axis (its trochlea is a cylinder about it): use --axis inman or original")
    out = build(args.src, args.out, None if args.free_knee else args.knee_angle,
                ligaments=not args.no_ligaments, six_dof=args.six_dof, axis=axis, swing_deg=args.swing_deg)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
