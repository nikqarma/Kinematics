"""Check the extracted ankle-foot model against the full MS-Human-700 model.

1. Path equivalence: at a grid of knee/ankle/subtalar/MTP poses, every
   muscle-tendon length in the ankle model must match the full model.
2. Moment arms: the signs about the ankle (talocrural) and subtalar joints
   must match the anatomy (dorsiflexors vs plantarflexors, peroneals evert).
3. Dynamics: a passive drop and single-muscle activations must run
   without NaNs, hold the knee lock, and move the foot the right way.

4. Ligaments: tension-only springs, length trends, passive moments, strain.
5. (6-DOF builds only) talocrural seating, stability and stress tests.

Usage: python verify_ankle_model.py [path/to/model.xml]
Exits non-zero if any check fails.
"""

from __future__ import annotations

import itertools
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ANKLE_XML = HERE / "model" / "ankle_foot_r.xml"
FULL_XML = HERE.parent / "MS-Human-700" / "MS-Human-700.xml"

DORSIFLEXORS = {"tibant_r", "edl_r", "ehl_r"}
EVERTORS = {"perlong_r", "perbrev_r"}

# Expected sign of dL/dq for each ligament (+1: lengthens as q increases).
# Ankle + = dorsiflexion; subtalar + = inversion. None = not asserted
# (no established direction, or the joint is not crossed in this model).
LIGAMENT_FUNCTION = {
    #           ankle  subtalar
    "ATFL_r":  (-1,    None),   # taut in plantarflexion; crosses talocrural only here
    "CFL_r":   (None,  +1),     # taut in inversion; biphasic in flexion (Takao 2021), see note
    "PTFL_r":  (+1,    None),   # taut in dorsiflexion
    "TNL_r":   (-1,    -1),     # taut in plantarflexion; superficial deltoid resists eversion
    "TSL_r":   (None,  -1),     # resists eversion
    "TCL_r":   (None,  -1),     # resists eversion
    "dATTL_r": (-1,    None),   # taut in plantarflexion
    "dPTTL_r": (+1,    None),   # taut in dorsiflexion
}

failures: list[str] = []


def check(ok: bool, msg: str) -> None:
    print(("  PASS  " if ok else "  FAIL  ") + msg)
    if not ok:
        failures.append(msg)


def set_pose(m: mujoco.MjModel, d: mujoco.MjData, pose: dict[str, float]) -> None:
    """Set named joints, fill the knee's coupled DOFs from its polynomials, run kinematics."""
    d.qpos[:] = m.qpos0
    for name, val in pose.items():
        d.qpos[m.joint(name).qposadr[0]] = val
    knee = m.joint("knee_angle_r").id
    k = pose.get("knee_angle_r", 0.0)
    for i in range(m.neq):
        if m.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT and m.eq_obj2id[i] == knee:
            c = m.eq_data[i, :5]
            d.qpos[m.jnt_qposadr[m.eq_obj1id[i]]] = sum(c[p] * k**p for p in range(5))
    mujoco.mj_kinematics(m, d)
    mujoco.mj_comPos(m, d)
    mujoco.mj_tendon(m, d)


def tendon_lengths(m, d, names):
    return np.array([d.ten_length[m.tendon(n).id] for n in names])


def tendon_jacobian(m: mujoco.MjModel, d: mujoco.MjData) -> np.ndarray:
    """Dense (ntendon x nv) tendon Jacobian dL/dq; MuJoCo stores it sparse."""
    if d.ten_J.size == m.ntendon * m.nv:
        return d.ten_J.reshape(m.ntendon, m.nv).copy()
    J = np.zeros((m.ntendon, m.nv))
    src = m if hasattr(m, "ten_J_rownnz") else d  # sparsity pattern moved to mjModel in MuJoCo 3.x
    mujoco.mju_sparse2dense(J, d.ten_J, src.ten_J_rownnz, src.ten_J_rowadr, src.ten_J_colind)
    return J


def check_ligaments(m: mujoco.MjModel, d: mujoco.MjData) -> None:
    print("\n4. Ligaments")
    ligs = [n for n in LIGAMENT_FUNCTION]
    ids = [m.tendon(n).id for n in ligs]
    k = m.tendon_stiffness[ids]
    L0 = m.tendon_lengthspring[ids, 1]
    ja, js = m.joint("ankle_angle_r").dofadr[0], m.joint("subtalar_angle_r").dofadr[0]
    qa, qs = m.joint("ankle_angle_r").qposadr[0], m.joint("subtalar_angle_r").qposadr[0]

    def at(ankle: float, sub: float = 0.0) -> None:
        mujoco.mj_resetData(m, d)
        d.qpos[qa], d.qpos[qs] = ankle, sub
        mujoco.mj_forward(m, d)

    # (a) MuJoCo's springlength deadband must act as a tension-only spring:
    # qfrc_passive must equal sum_i J_i^T * (-k_i * max(0, L_i - L0_i)).
    worst = 0.0
    for a in np.radians(np.arange(-35, 30, 5)):
        for s in np.radians([-15, 0, 15]):
            at(a, s)
            J = tendon_jacobian(m, d)[ids]
            f = -k * np.maximum(0.0, d.ten_length[ids] - L0)
            expected = J.T @ f
            worst = max(worst, np.abs(expected[[ja, js]] - d.qfrc_passive[[ja, js]]).max())
    check(worst < 1e-6, f"springs are tension-only (max mismatch vs k*max(0,L-L0): {worst:.1e} N*m)")

    # (b) length trends match known ligament function.
    at(0.0)
    J = tendon_jacobian(m, d)[ids]
    print(f"  {'ligament':<9}{'L0 mm':>7}{'strain@0':>10}{'dL/d ankle':>12}{'dL/d subtalar':>15}  (mm/rad)")
    for i, n in enumerate(ligs):
        eps = d.ten_length[ids[i]] / L0[i] - 1
        print(f"  {n:<9}{L0[i] * 1000:7.1f}{eps * 100:9.1f}%{J[i, ja] * 1000:12.1f}{J[i, js] * 1000:15.1f}")
    for i, n in enumerate(ligs):
        for col, sign, lab, pos, neg in ((ja, LIGAMENT_FUNCTION[n][0], "ankle", "dorsiflexion", "plantarflexion"),
                                         (js, LIGAMENT_FUNCTION[n][1], "subtalar", "inversion", "eversion")):
            if sign is not None:
                check(np.sign(J[i, col]) == sign, f"{n} lengthens in {pos if sign > 0 else neg}")

    icfl = ligs.index("CFL_r")
    print(f"  NOTE  CFL is nearly isometric over ankle flexion here ({J[icfl, ja] * 1000:+.1f} mm/rad vs "
          f"ATFL {J[ligs.index('ATFL_r'), ja] * 1000:+.1f}); it tightens in plantarflexion but not in "
          "dorsiflexion, so the DF branch reported by Takao 2021 is not reproduced")

    # (c) passive moment-angle curve from the ligaments alone.
    print("\n  Passive ligament moment (N*m; + = dorsiflexing / inverting):")
    print("  ankle deg  " + "".join(f"{a:7d}" for a in range(-35, 31, 10)))
    row = []
    for a in range(-35, 31, 10):
        at(np.radians(min(a, 29)))
        row.append(d.qfrc_passive[ja])
    print("  ankle M    " + "".join(f"{v:7.1f}" for v in row))
    row = []
    for s in range(-15, 16, 5):
        at(0.0, np.radians(s))
        row.append(d.qfrc_passive[js])
    print("  subtalar deg " + "".join(f"{s:7d}" for s in range(-15, 16, 5)))
    print("  subtalar M   " + "".join(f"{v:7.1f}" for v in row))
    at(np.radians(-30)); mpf = d.qfrc_passive[ja]
    at(np.radians(25)); mdf = d.qfrc_passive[ja]
    check(mpf > 0 and mdf < 0, "ligaments resist both end ranges of the ankle "
          f"(+{mpf:.1f} N*m at 30 deg PF, {mdf:.1f} N*m at 25 deg DF)")

    # (c2) strain at the ends of the range of motion. Ligaments typically fail
    # somewhere around 10-30% strain, so anything above ~10% is flagged.
    poses = {"PF -35": (-35, 0), "DF +25": (25, 0), "inv +15": (0, 15), "ev -15": (0, -15)}
    print("\n  Ligament strain at end range (%):")
    print(f"  {'ligament':<9}" + "".join(f"{p:>9}" for p in poses))
    strains = {}
    for name, (a, s) in poses.items():
        at(np.radians(a), np.radians(s))
        strains[name] = d.ten_length[ids] / L0 - 1
    for i, n in enumerate(ligs):
        print(f"  {n:<9}" + "".join(f"{max(0.0, strains[p][i]) * 100:9.1f}" for p in poses))
    high = sorted({ligs[i] for p in poses for i in np.flatnonzero(strains[p] > 0.10)})
    if high:
        print(f"  WARN  strain above 10% at end range: {', '.join(high)} - see README limitations")

    # (d) a passive drop must not load any ligament past its reported failure load.
    mujoco.mj_resetData(m, d)
    peak = np.zeros(len(ids))
    while d.time < 2.0:
        mujoco.mj_step(m, d)
        peak = np.maximum(peak, k * np.maximum(0.0, d.ten_length[ids] - L0))
    for n, f in zip(ligs, peak):
        if f > 1.0:
            print(f"  peak force during passive drop: {n} {f:.1f} N")
    check(peak.max() < 263.6, f"passive drop: peak ligament force {peak.max():.1f} N "
          "is below the weakest reported failure load (ATFL 263.6 N)")


def check_six_dof(xml: Path) -> None:
    """Checks for the experimental 6-DOF talocrural joint (bench tests, foot held by constraints)."""
    from bench import Bench

    print("\n5. 6-DOF talocrural joint (experimental)")
    deg = np.degrees

    def talus(rig):
        return (rig.q("talus_tx_r"), rig.q("talus_ty_r"), rig.q("talus_tz_r"),
                rig.q("talus_tilt_r"), rig.q("talus_rot_r"))

    def run(cuts=(), ankle=0.0, gravity=False, **loads):
        rig = Bench(xml, gravity=gravity)
        rig.cut(*cuts)
        if ankle is not None:
            rig.hold(ankle, 0)
        ok = rig.settle(1.5, **loads)
        return rig, ok

    # (a) seating
    rig, ok = run(ankle=None, gravity=True)
    tx, ty, tz, tilt, rot = talus(rig)
    check(ok and max(abs(tx), abs(ty), abs(tz)) < 0.5e-3,
          f"unloaded under gravity: talus stays seated (max translation {max(abs(tx), abs(ty), abs(tz)) * 1e3:.2f} mm)")
    rig, ok = run(sole_force=[0, 0, 700])
    fn, pen = rig.contact()
    tr = max(abs(v) for v in talus(rig)[:3])
    check(ok and pen < 0.5e-3 and tr < 0.5e-3,
          f"700 N axial load: contact {fn:.0f} N, penetration {pen * 1e3:.3f} mm, max translation {tr * 1e3:.2f} mm")
    # Physically applied loads on the free foot (a push/pull on the forefoot sole)
    # both rotate the ankle and compress the joint; the talus must stay seated.
    for label, local, F in (("forefoot push up 100 N (dorsiflexes)", [0.17, -0.01, 0.0], [0, 0, 100]),
                            ("forefoot pull down 100 N (plantarflexes)", [0.17, 0.02, 0.0], [0, 0, -100])):
        rig = Bench(xml)
        calcn = rig.calcn
        worst = 0.0  # max talar translation during the load
        while rig.d.time < 1.5:
            rig.d.qfrc_applied[:] = 0
            p = rig.d.xpos[calcn] + rig.d.xmat[calcn].reshape(3, 3) @ np.array(local)
            mujoco.mj_applyFT(rig.m, rig.d, np.array(F, float), np.zeros(3), p, calcn, rig.d.qfrc_applied)
            mujoco.mj_step(rig.m, rig.d)
            worst = max(worst, max(abs(v) for v in talus(rig)[:3]))
        check(worst < 2e-3, f"{label}, foot free: talus stays seated "
              f"(max translation {worst * 1e3:.2f} mm, ankle {deg(rig.q('ankle_angle_r')):.1f} deg)")
    # A pure generalized torque on ankle_angle_r with no joint compression is not
    # a physical load; report what it does rather than assert.
    rig = Bench(xml)
    ja = rig.m.joint("ankle_angle_r").dofadr[0]
    worst = 0.0
    while rig.d.time < 1.5:
        rig.d.qfrc_applied[:] = 0
        rig.d.qfrc_applied[ja] = 10.0
        mujoco.mj_step(rig.m, rig.d)
        worst = max(worst, max(abs(v) for v in talus(rig)[:3]))
    if worst > 2e-3:
        print(f"  WARN  a pure +10 N*m torque on ankle_angle_r (no joint compression) subluxes the talus "
              f"({worst * 1e3:.1f} mm): apply loads as forces, not generalized torques, in the 6-DOF build")

    # (b) clinical stress tests: values are reported, only directions are asserted
    print("\n  Stress tests (ankle and subtalar held at 0 deg; values NOT validated against clinical data):")
    print(f"  {'test':<36}{'tx mm':>7}{'ty mm':>7}{'tz mm':>7}{'tilt':>7}{'rot':>7}")
    res = {}
    for name, cuts, loads in [
        ("drawer 150 N", (), dict(heel_force=[150, 0, 0])),
        ("drawer 150 N, ATFL cut", ("ATFL_r",), dict(heel_force=[150, 0, 0])),
        ("drawer 150 N + 700 N axial", (), dict(heel_force=[150, 0, 0], sole_force=[0, 0, 700])),
        ("inversion 3 N*m", (), dict(foot_torque=[3, 0, 0])),
        ("inversion 3 N*m, ATFL cut", ("ATFL_r",), dict(foot_torque=[3, 0, 0])),
        ("inversion 3 N*m, ATFL+CFL cut", ("ATFL_r", "CFL_r"), dict(foot_torque=[3, 0, 0])),
    ]:
        rig, ok = run(cuts, **loads)
        res[name] = talus(rig)
        tx, ty, tz, tilt, rot = res[name]
        print(f"  {name:<36}{tx * 1e3:7.2f}{ty * 1e3:7.2f}{tz * 1e3:7.2f}{deg(tilt):7.1f}{deg(rot):7.1f}"
              + ("" if ok else "  UNSTABLE"))
    check(res["drawer 150 N, ATFL cut"][0] > res["drawer 150 N"][0] + 1e-3,
          "cutting the ATFL increases anterior drawer")
    check(res["drawer 150 N + 700 N axial"][0] < res["drawer 150 N"][0],
          "axial compression reduces anterior drawer")
    t0, t1, t2 = (res[k][3] for k in ("inversion 3 N*m", "inversion 3 N*m, ATFL cut", "inversion 3 N*m, ATFL+CFL cut"))
    check(t0 < t1 < t2, f"talar tilt grows with sectioning ({deg(t0):.1f} -> {deg(t1):.1f} -> {deg(t2):.1f} deg)")

    # (c) strain through flexion, for comparison with the hinge (section 4)
    print("\n  Ligament strain (%) driving the ankle with the subtalar held at 0 (talus free to settle):")
    print(f"  {'ankle':>6}" + "".join(f"{n[:-2]:>7}" for n in LIGAMENT_FUNCTION) + "   tilt deg  tz mm")
    for a in (-35, -15, 0, 15, 25):
        rig = Bench(xml)
        rig.hold(0, 0)
        rig.settle(1.5, ankle_ramp_deg=(0, a))
        s = rig.strains()
        print(f"  {a:6d}" + "".join(f"{max(0.0, s[n]) * 100:7.1f}" for n in LIGAMENT_FUNCTION)
              + f"   {deg(rig.q('talus_tilt_r')):7.1f}  {rig.q('talus_tz_r') * 1e3:5.2f}")
    print("  NOTE  in plantarflexion the placeholder deltoid bands tilt the talus into inversion and shift it"
          " medially,\n        which raises lateral-ligament strain above the hinge values - see README")


def main() -> int:
    global ANKLE_XML
    if len(sys.argv) > 1:
        ANKLE_XML = Path(sys.argv[1]).resolve()
    print(f"model: {ANKLE_XML}")
    am = mujoco.MjModel.from_xml_path(str(ANKLE_XML))
    ad = mujoco.MjData(am)
    fm = mujoco.MjModel.from_xml_path(str(FULL_XML))
    fd = mujoco.MjData(fm)
    muscles = [am.actuator(i).name for i in range(am.nu)]
    tendons = [f"{n}_tendon" for n in muscles]
    print(f"ankle model: {am.nbody} bodies, {am.njnt} joints, {am.nu} muscles")
    print(f"full model:  {fm.nbody} bodies, {fm.njnt} joints, {fm.nu} muscles\n")

    # 1. path equivalence --------------------------------------------------
    print("1. Muscle-tendon lengths vs full model")
    grid = itertools.product(
        np.linspace(0.0, 1.5, 4),      # knee flexion
        np.linspace(-0.68, 0.52, 7),   # ankle (talocrural); + = dorsiflexion
        np.linspace(-0.34, 0.34, 5),   # subtalar
        np.linspace(-0.5, 0.5, 3),     # MTP
    )
    max_err = 0.0
    n = 0
    for k, a, s, t in grid:
        pose = {"knee_angle_r": k, "ankle_angle_r": a, "subtalar_angle_r": s, "mtp_angle_r": t}
        set_pose(am, ad, pose)
        set_pose(fm, fd, pose)
        max_err = max(max_err, np.abs(tendon_lengths(am, ad, tendons) - tendon_lengths(fm, fd, tendons)).max())
        n += 1
    check(max_err < 1e-9, f"{n} poses x {len(tendons)} muscles, max length difference {max_err:.2e} m")

    # 2. moment arms at neutral -----------------------------------------------
    print("\n2. Moment arms at neutral pose (r = -dL/dq, mm)")
    h = 1e-5
    print(f"  {'muscle':<11}{'ankle':>9}{'subtalar':>10}")
    for mus, ten in zip(muscles, tendons):
        r = {}
        for j in ("ankle_angle_r", "subtalar_angle_r"):
            set_pose(am, ad, {j: h}); lp = ad.ten_length[am.tendon(ten).id]
            set_pose(am, ad, {j: -h}); lm = ad.ten_length[am.tendon(ten).id]
            r[j] = -(lp - lm) / (2 * h) * 1000
        print(f"  {mus:<11}{r['ankle_angle_r']:9.1f}{r['subtalar_angle_r']:10.1f}")
        want_df = mus in DORSIFLEXORS
        check((r["ankle_angle_r"] > 0) == want_df,
              f"{mus} is a {'dorsi' if want_df else 'plantar'}flexor")
        if mus in EVERTORS:
            check(r["subtalar_angle_r"] < 0, f"{mus} everts (negative subtalar moment arm)")

    # 3. dynamics ------------------------------------------------------------
    print("\n3. Forward dynamics (2 s each)")
    ia = am.joint("ankle_angle_r").qposadr[0]
    ik = am.joint("knee_angle_r").qposadr[0]

    lo = am.jnt_range[am.joint("ankle_angle_r").id, 0]

    def run(ctrl: dict[str, float], gravity: bool) -> tuple[float, float, bool]:
        mujoco.mj_resetData(am, ad)
        am.opt.gravity[2] = -9.81 if gravity else 0.0
        for name, u in ctrl.items():
            ad.ctrl[am.actuator(name).id] = u
        while ad.time < 2.0:
            mujoco.mj_step(am, ad)
        return ad.qpos[ia], ad.qpos[ik], bool(np.all(np.isfinite(ad.qpos)))

    has_ligs = any(am.tendon(i).name in LIGAMENT_FUNCTION for i in range(am.ntendon))
    a0, k0, ok = run({}, gravity=True)
    check(ok, f"passive under gravity: finite state, ankle settles at {np.degrees(a0):.1f} deg")
    check(abs(k0) < np.radians(1.0), f"passive under gravity: knee lock held (knee = {np.degrees(k0):.2f} deg)")
    if has_ligs:
        check(abs(a0 - lo) > np.radians(2.0),
              f"passive under gravity: ligaments hold the foot off the plantarflexion stop ({np.degrees(lo):.1f} deg)")
    elif abs(a0 - lo) < 0.02:
        print(f"  NOTE  the unloaded foot rests on the plantarflexion stop ({np.degrees(lo):.1f} deg):"
              " the model has no passive ankle stiffness (no ligaments/capsule)")
    # Single-muscle actions are checked without gravity so the foot starts
    # at neutral rather than on a joint stop.
    a1, _, ok = run({"soleus_r": 1.0}, gravity=False)
    check(ok and a1 < -0.05, f"soleus alone plantarflexes from neutral ({np.degrees(a1):.1f} deg)")
    a2, _, ok = run({"tibant_r": 1.0}, gravity=False)
    check(ok and a2 > 0.05, f"tibialis anterior alone dorsiflexes from neutral ({np.degrees(a2):.1f} deg)")
    am.opt.gravity[2] = -9.81

    if has_ligs:
        check_ligaments(am, ad)
    if any(am.joint(i).name == "talus_tx_r" for i in range(am.njnt)):
        check_six_dof(ANKLE_XML)

    print(f"\n{'ALL CHECKS PASSED' if not failures else f'{len(failures)} CHECK(S) FAILED'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
