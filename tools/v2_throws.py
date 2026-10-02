#!/usr/bin/env python3
"""v2 Phase 1, item 4: A's random throws (offline, venv). Needs docs/v2/ready_envelope.json (B's ready pose).

    env -u PYTHONPATH .venv/bin/python tools/v2_throws.py            # targets x angles: A feasible, B catchable
    env -u PYTHONPATH .venv/bin/python tools/v2_throws.py --square   # same, palm pitch chosen for a square arrival
    env -u PYTHONPATH .venv/bin/python tools/v2_throws.py --id       # online inverse dynamics vs library, pinocchio
    (pinocchio --id needs the ROS environment for xacro: source /opt/ros/jazzy/setup.bash && source install/setup.bash)

A: for every target point (the throw passes through it on the way down) and launch angle, the M1 machinery
(tools/feasibility.py): release at (-0.05, y_target, 1.25) in a throw plane parallel to x, best IK branch and
palm pitch rate, toss profile from throw.yaml, joint speed <= 85 %, torque (MuJoCo inverse dynamics) <= 80 %,
position margin, TCP above the table, arm clearance to table / pallets / pedestals.
B: catchable from the ready pose (some point on the descending path reachable in the time left after the
perception decision, tools/v2_ready_envelope.py) with the natural (ready) orientation and with the cup limited
to a 30 deg cone. The random-throw distribution is the intersection.
"""
import argparse
import glob
import multiprocessing as mp
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
import yaml  # noqa: E402

import v2_common as V  # noqa: E402
import v2_ready_envelope as RE  # noqa: E402

A = {}


def _init_a():
    import copy
    c = V.cfg()
    scene, phys = V.build_mjcf.load_cfg()
    A["c"] = c
    A["robots"] = V.F.make_robots(scene)
    A["dyn"] = V.F.Dynamics(scene, phys)
    A["clr"] = V.F.Clearance()
    A["scene"] = scene
    tc = yaml.safe_load(open(os.path.join(V.REPO, "src/catch_throw/config/throw.yaml")))["throw"]
    from catch_throw.profile import ProfileParams
    A["profiles"] = [ProfileParams(**tc["profile"]), ProfileParams(accel=10, decel=16, tau3=0.03)]
    A["q_ready"] = np.array(copy.deepcopy(_ready()["q"]))
    A["z_ready"] = V.ready_rotation(_ready()["elev"])[:, 2]


def _ready():
    import json
    return json.load(open(os.path.join(V.OUT, "ready_envelope.json")))["chosen"]


def arrival_tilt(p, v0, spin, T, z_ready):
    """Rotation [deg] about B's closing axis between the cube (A's toss orientation, spun for T) and B's cup palm
    (cup, 30 deg cone) at the arrival, folded to +-45 deg (cube symmetry)."""
    from catch_kinematics.transforms import axis_angle
    from catch_throw.generator import pitch_axis, toss_orientation
    n = v0 / np.linalg.norm(v0)
    Rc = axis_angle(pitch_axis(n), spin * T) @ toss_orientation(n, [0, 1.0, 0])
    Rp, _ = V.catch_rotation(z_ready, v0 + V.GVEC * T, np.radians(30), "cup")
    Rr = Rp.T @ Rc
    k = int(np.argmax(np.abs(Rr[1])))
    a = np.degrees(np.arctan2(Rr[2, k], Rr[1, k]))
    return (a + 45) % 90 - 45


def square_kappas(p, v, T, z_ready):
    """Palm pitch rates kappa [rad/m] for which the cube arrives square to B's palm: tilt within
    +-throws.square_tol_deg of 0 (mod 90 deg)."""
    V_ = np.linalg.norm(v)
    t0 = arrival_tilt(p, v, 0.0, T, z_ready)
    eps = 1e-3
    sgn = np.sign(arrival_tilt(p, v, eps / T, T, z_ready) - t0) or 1.0
    tol = V.cfg()["throws"]["square_tol_deg"]
    out = []
    for k in range(-3, 4):
        for dt in np.linspace(-tol, tol, 11):
            spin = np.radians(-t0 + 90 * k + dt) * sgn / T
            kap = spin / V_
            if abs(kap) <= 6.0:
                out.append(kap)
    return out


def branch_for_kappas(robot, p, v, kappas):
    """Like feasibility.best_branch_and_kappa, restricted to the given pitch rates."""
    from catch_kinematics.transforms import make_tf
    from catch_throw.generator import pitch_axis, toss_orientation
    Vn = np.linalg.norm(v)
    n = v / Vn
    best = (np.inf, None, None)
    for q in robot.ik_all(make_tf(toss_orientation(n), p)):
        if np.any(np.abs(q) > V.POS_LIMIT - 0.15):
            continue
        J = robot.jacobian(q)
        if abs(np.linalg.det(J)) < 1e-4:
            continue
        Ji = np.linalg.inv(J)
        a0 = Ji @ np.r_[Vn * n, 0, 0, 0]
        a1 = Ji @ np.r_[0, 0, 0, Vn * pitch_axis(n)]
        for kap in kappas:
            r = float(np.max(np.abs(a0 + kap * a1) / V.VEL_LIMIT))
            if r < best[0]:
                best = (r, float(kap), q)
    return best


def eval_throw(args):
    """A side of one (target, angle[, square])."""
    from catch_ballistics import velocity_for_launch_angle
    from catch_throw.generator import InfeasibleThrow, generate_throw
    tgt, deg = args[:2]
    square = len(args) > 2 and args[2]
    c, lim = A["c"], A["c"]["limits"]
    tgt = np.asarray(tgt, float)
    p = np.array([c["throws"]["release_point"][0], tgt[1], c["throws"]["release_point"][2]])
    r = dict(target=tgt.tolist(), deg=deg, p_release=p.tolist(), ok=False)
    try:
        v, T = velocity_for_launch_angle(p, tgt, np.radians(deg))
    except ValueError as e:
        r["why"] = str(e)
        return r
    if (v + V.GVEC * T)[2] >= 0:
        r["why"] = "target on the rising branch"
        return r
    r.update(v_release=v.tolist(), speed=float(np.linalg.norm(v)), t_target=float(T))
    if square:
        r0, kappa, q_r = branch_for_kappas(A["robots"]["a"], p, v, square_kappas(p, v, T, A["z_ready"]))
    else:
        r0, kappa, q_r = V.F.best_branch_and_kappa(A["robots"]["a"], p, v)
    if q_r is None or r0 > lim["vel_ratio"]:
        r["why"] = "release joint speed" if q_r is not None else "no IK at release"
        return r
    last = "generation"
    for prof in A["profiles"]:
        try:
            plan = generate_throw(A["robots"]["a"], p, v, q_r, prof, pitch_rate=kappa)
        except (InfeasibleThrow, ValueError) as e:
            last = f"generation: {e}"
            continue
        m = plan.metrics
        if m["vel_ratio_max"] > lim["vel_ratio"]:
            last = "stroke joint speed"
            continue
        if m["pos_margin_min"] < lim["pos_margin"]:
            last = "position margin"
            continue
        if m["tcp_z_min"] < A["scene"]["table"]["top_z"] + 0.06:
            last = "TCP near table"
            continue
        tau = A["dyn"].torques(plan.q, plan.qd, plan.qdd)
        tr = float(np.max(np.abs(tau) / V.TORQUE_LIMIT))
        if tr > lim["torque_ratio"]:
            last = "torque"
            continue
        dmin = A["clr"].min_distance(plan.q, A["q_ready"])
        if dmin < 0.02:
            last = "arm clearance"
            continue
        r.update(ok=True, why="", square=bool(square), arrival_tilt_deg=float(arrival_tilt(p, v, kappa * np.linalg.norm(v), T, A["z_ready"])),
                 kappa=kappa, spin=float(kappa * np.linalg.norm(v)), vel_ratio=m["vel_ratio_max"],
                 torque_ratio=tr, pos_margin=m["pos_margin_min"], clearance=float(dmin), accel=prof.accel,
                 decel=prof.decel, q_release_seed=q_r.tolist(), stroke=m["stroke_length"], t_sep=float(plan.t_release),
                 apex_z=float(p[2] + max(v[2], 0) ** 2 / (2 * V.G)))
        return r
    r["why"] = last
    return r


def run_targets(c, quick=False, square=False):
    tc = c["throws"]
    targets = [(x, y, z) for x in tc["target_x"] for y in tc["target_y"] for z in tc["target_z"]]
    jobs = [(t, d, square) for t in targets for d in tc["launch_angle_deg"]]
    if quick:
        jobs = jobs[::17]
    with mp.Pool(min(22, os.cpu_count()), initializer=_init_a) as pool:
        res = pool.map(eval_throw, jobs, chunksize=4)
    from collections import Counter
    ok = [r for r in res if r["ok"]]
    print(f"A: {len(res)} (target, angle) pairs, {len(ok)} feasible; rejections "
          f"{Counter(r['why'].split(':')[0] for r in res if not r['ok']).most_common()}", flush=True)
    # B side
    RE._init()
    ready = _ready()
    q_ready = np.array(ready["q"])
    z_ready = V.ready_rotation(ready["elev"])[:, 2]
    fam = [(np.array(r["p_release"]), np.array(r["v_release"])) for r in ok]
    t_decs = [0.10, 0.15, 0.20, 0.25, 0.30]
    with mp.Pool(min(22, os.cpu_count()), initializer=RE._init) as pool:
        modes = {}
        for name, fn in (("ready", lambda p, v: V.ready_rotation(ready["elev"])),
                         ("cup30", lambda p, v: V.catch_rotation(z_ready, v, np.radians(30), "cup")[0])):
            f, slack, by_t = RE.catchable(pool, q_ready, fam, fn, c, t_decs)
            modes[name] = dict(catchable=f, by_t_decision=by_t)
            for r, s in zip(ok, slack):
                r[f"slack_{name}"] = float(s) if np.isfinite(s) else None
            print(f"B ({name}): {f:.2f} of A's feasible throws catchable at t_decision {c['envelope']['t_decision_s']} s; "
                  + ", ".join(f"{k:.2f}s: {v:.2f}" for k, v in by_t.items()), flush=True)
    both = [r for r in ok if r.get("slack_ready") is not None and r["slack_ready"] >= 0
            and r.get("slack_cup30") is not None and r["slack_cup30"] >= 0]
    print(f"distribution (A feasible and B catchable in both modes): {len(both)} throws; flight to target "
          f"{min(r['t_target'] for r in both):.3f}-{max(r['t_target'] for r in both):.3f} s, speed "
          f"{min(r['speed'] for r in both):.2f}-{max(r['speed'] for r in both):.2f} m/s, angles "
          f"{sorted(set(r['deg'] for r in both))}")
    out = dict(V.flags(c), ready_pose=ready, n_pairs=len(res), n_a_feasible=len(ok), modes=modes, n_distribution=len(both),
               rejections=Counter(r["why"].split(":")[0] for r in res if not r["ok"]), a_feasible=ok,
               distribution=both, note="distribution = A feasible AND B catchable (ready orientation and cup-30) at "
               "t_decision; release (-0.05, y_target, 1.25)")
    out["square"] = square
    tilt = np.array([abs(r["arrival_tilt_deg"]) for r in both])
    print(f"cube arrival tilt vs B's cup palm over the distribution: median {np.median(tilt):.1f} deg, "
          f"<= 5 deg for {np.mean(tilt <= 5):.2f}")
    sfx = "_square" if square else ""
    V.save_json(f"throw_distribution{sfx}.json", out)
    plot_targets(out, res, sfx)


def plot_targets(out, res, sfx=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 3, figsize=(17, 5))
    okA = [r for r in res if r["ok"]]
    both = out["distribution"]
    key = lambda r: (tuple(r["target"]), r["deg"])
    bset = {key(r) for r in both}
    for ax, (i, j, xl, yl) in zip(axs[:2], ((0, 2, "x [m]", "z [m]"), (0, 1, "x [m]", "y [m]"))):
        P = lambda rs: np.array([r["target"] for r in rs]) if rs else np.zeros((0, 3))
        nA = P([r for r in res if not r["ok"]])
        ax.scatter(nA[:, i], nA[:, j], s=60, c="#dddddd", marker="s", label="A infeasible")
        a_only = P([r for r in okA if key(r) not in bset])
        ax.scatter(a_only[:, i] + 0.004, a_only[:, j], s=25, c="#d95f02", label="A feasible, B not")
        bb = P(both)
        ax.scatter(bb[:, i] - 0.004, bb[:, j], s=25, c="#1b9e77", label="A feasible and B catchable")
        ax.set_xlabel(xl)
        ax.set_ylabel(yl)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title("throw targets (all angles overlaid)")
    degs = sorted({r["deg"] for r in res})
    na = [sum(1 for r in okA if r["deg"] == d) for d in degs]
    nb = [sum(1 for r in both if r["deg"] == d) for d in degs]
    nt = [sum(1 for r in res if r["deg"] == d) for d in degs]
    x = np.arange(len(degs))
    axs[2].bar(x - 0.25, nt, 0.25, color="#dddddd", label="pairs")
    axs[2].bar(x, na, 0.25, color="#d95f02", label="A feasible")
    axs[2].bar(x + 0.25, nb, 0.25, color="#1b9e77", label="A and B")
    axs[2].set_xticks(x, [f"{d} deg" for d in degs])
    axs[2].legend()
    axs[2].set_title("by launch angle")
    fig.suptitle(("Square-arrival throws (palm pitch chosen so the cube lands square on B's palm): " if out.get("square") else "Random throws: ") + f"A feasible ({out['n_a_feasible']} of {out['n_pairs']}) and B catchable from the ready pose "
                 f"({out['n_distribution']}); kinematics/dynamics only, gripper_profile n/a")
    fig.tight_layout()
    fig.savefig(os.path.join(V.OUT, f"throw_distribution{sfx}.png"), dpi=105)


# ------------------------------------------------------------------- inverse dynamics: online vs library ------
def ros_env():
    env = dict(os.environ)
    env["PYTHONPATH"] = ":".join(glob.glob("/opt/ros/jazzy/lib/python3*/site-packages"))
    return env


def dyn_mjcf_xml(scene, phys, path):
    """The tools/feasibility.py Dynamics model (one UR5e + 2F-85 + 60 g cube, gravcomp off) as an MJCF file."""
    import mujoco
    phys = dict(phys, arm={"gravcomp": False})
    arm = V.build_mjcf.make_arm(scene, phys)
    g = [b for b in arm.bodies if b.name == "robotiq_base"][0]
    e = scene["cube"]["edge"]
    g.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[e / 2] * 3, pos=[0, 0, scene["gripper"]["palm_insert"]["top_z"] + e / 2],
               mass=scene["cube"]["mass"], contype=0, conaffinity=0)
    for b in arm.bodies:
        b.gravcomp = 0.0
    spec = mujoco.MjSpec()
    r = scene["robots"]["a"]
    fr = spec.worldbody.add_frame(pos=V.build_mjcf.robot_base_xyz(scene, "a"),
                                  quat=[np.cos((r["base_yaw"] + np.pi) / 2), 0, 0, np.sin((r["base_yaw"] + np.pi) / 2)])
    spec.attach(arm, prefix="", frame=fr)
    assets = os.path.join(V.REPO, "src/catch_description/mjcf/assets")
    for m in spec.meshes:
        m.file = os.path.join(assets, m.file)
    spec.compile()
    open(path, "w").write(spec.to_xml())
    return path


def pin_from_mujoco(dyn):
    """Pinocchio model with exactly the MuJoCo Dynamics parameters: arm joint placements / axes / inertias read
    from the compiled model, every body below wrist_3 (gripper at knuckle 0, 60 g cube) lumped onto wrist_3."""
    import mujoco
    import pinocchio as pin
    m, d = dyn.m, mujoco.MjData(dyn.m)
    mujoco.mj_forward(m, d)
    model = pin.Model()
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    links = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link"]
    se3 = lambda pos, quat: pin.SE3(pin.Quaternion(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix(), np.array(pos))
    w_T = {}
    parent, prev_T = 0, se3(d.xpos[bid("base")], d.xquat[bid("base")])
    for jn, ln in zip(V.JN, links):
        b = bid(ln)
        T = se3(d.xpos[b], d.xquat[b])
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
        ax = m.jnt_axis[j]
        assert np.allclose(m.jnt_pos[j], 0)
        jid = model.addJoint(parent, pin.JointModelRevoluteUnaligned(ax[0], ax[1], ax[2]), prev_T.inverse() * T, jn)
        I = pin.Inertia(m.body_mass[b], m.body_ipos[b], se3([0, 0, 0], m.body_iquat[b]).rotation @ np.diag(m.body_inertia[b])
                        @ se3([0, 0, 0], m.body_iquat[b]).rotation.T)
        if ln == "wrist_3_link":
            for c in range(m.nbody):
                a, inside = c, False
                while a > 0:
                    a = m.body_parentid[a]
                    if a == b:
                        inside = True
                        break
                if not inside or m.body_mass[c] == 0:
                    continue
                Ic = pin.Inertia(m.body_mass[c], np.zeros(3), np.diag(m.body_inertia[c]))
                X = T.inverse() * se3(d.xipos[c], [1, 0, 0, 0])
                X.rotation = T.rotation.T @ d.ximat[c].reshape(3, 3)
                I = I + X.act(Ic)
        model.appendBodyToJoint(jid, I, pin.SE3.Identity())
        parent, prev_T, w_T[jn] = jid, T, T
    return model


def run_id(c):
    import mujoco
    import pinocchio as pin
    from catch_ballistics import velocity_for_launch_angle  # noqa: F401
    import json
    scene, phys = V.build_mjcf.load_cfg()
    _init_a()
    dist = json.load(open(os.path.join(V.OUT, "throw_distribution_square.json" if os.path.exists(
        os.path.join(V.OUT, "throw_distribution_square.json")) else "throw_distribution.json")))["distribution"]
    rng = np.random.default_rng(0)
    sample = [dist[i] for i in rng.choice(len(dist), size=min(20, len(dist)), replace=False)]
    from catch_throw.generator import generate_throw
    from catch_throw.profile import ProfileParams
    # 1. cost of generating one throw online (numpy) + MuJoCo inverse dynamics
    t_gen, t_branch, t_id, plans = [], [], [], []
    for r in sample:
        p, v = np.array(r["p_release"]), np.array(r["v_release"])
        a = time.perf_counter()
        r0, kappa, q_r = branch_for_kappas(A["robots"]["a"], p, v, [r["kappa"]])
        b = time.perf_counter()
        plan = generate_throw(A["robots"]["a"], p, v, q_r, ProfileParams(accel=r["accel"], decel=r["decel"]) if r["accel"] != 12.0
                              else A["profiles"][0], pitch_rate=kappa, dt=0.004)
        cc = time.perf_counter()
        tau = A["dyn"].torques(plan.q, plan.qd, plan.qdd)
        d = time.perf_counter()
        t_branch.append(b - a)
        t_gen.append(cc - b)
        t_id.append(d - cc)
        plans.append((plan, tau))
    # 2. pinocchio: (a) the same MJCF model, (b) the URDF the ROS nodes load (robot_description)
    tmp = os.path.join(V.REPO, "log", "v2")
    os.makedirs(tmp, exist_ok=True)
    xml = dyn_mjcf_xml(scene, phys, os.path.join(tmp, "dyn_arm.xml"))
    res = {}
    try:
        pm = pin.buildModelFromMJCF(xml)
        res["mjcf_parse"] = f"ok: nq={pm.nq}"
    except Exception as e:      # noqa: BLE001
        pm = None
        res["mjcf_parse"] = f"failed: {e}"
    urdf = subprocess.check_output(["xacro", os.path.join(V.REPO, "src/catch_description/urdf/scene.urdf.xacro"),
                                    "ros2_control:=false"], env=ros_env()).decode()
    pu = pin.buildModelFromXML(urdf)
    # 60 g cube at a_tcp (same as the MuJoCo model)
    fid = pu.getFrameId("a_tcp")
    f = pu.frames[fid]
    pu.appendBodyToJoint(f.parentJoint, pin.Inertia.FromBox(scene["cube"]["mass"], *[scene["cube"]["edge"]] * 3), f.placement)
    ju = [pu.joints[pu.getJointId(f"a_{j}")].idx_q for j in V.JN]
    jv = [pu.joints[pu.getJointId(f"a_{j}")].idx_v for j in V.JN]
    du = pu.createData()
    if pm is not None:
        dm = pm.createData()
        jm = [pm.joints[pm.getJointId(j)].idx_q for j in V.JN]
        jmv = [pm.joints[pm.getJointId(j)].idx_v for j in V.JN]
    # MuJoCo armature (rotor inertia) of the Menagerie arm joints, not in the URDF
    arm = A["dyn"].m.dof_armature[A["dyn"].dofs]
    pp = pin_from_mujoco(A["dyn"])
    dp = pp.createData()
    rows = {"urdf": [], "urdf+armature": [], "mjcf": [], "mujoco_params": []}
    t_pin = []
    for plan, tau_mj in plans:
        for k in range(0, len(plan.t), 2):
            q, qd, qdd = plan.q[k], plan.qd[k], plan.qdd[k]
            Q, Vv, Aa = pin.neutral(pu), np.zeros(pu.nv), np.zeros(pu.nv)
            Q[ju], Vv[jv], Aa[jv] = q, qd, qdd
            a = time.perf_counter()
            tu = pin.rnea(pu, du, Q, Vv, Aa)[jv]
            t_pin.append(time.perf_counter() - a)
            rows["urdf"].append(tu - tau_mj[k])
            rows["urdf+armature"].append(tu + arm * qdd - tau_mj[k])
            rows["mujoco_params"].append(pin.rnea(pp, dp, q, qd, qdd) + arm * qdd - tau_mj[k])
            if pm is not None:
                Q, Vv, Aa = pin.neutral(pm), np.zeros(pm.nv), np.zeros(pm.nv)
                Q[jm], Vv[jmv], Aa[jmv] = q, qd, qdd
                tm = pin.rnea(pm, dm, Q, Vv, Aa)[jmv]
                rows["mjcf"].append(tm + arm * qdd - tau_mj[k])
    peak = np.max(np.abs(np.concatenate([t for _, t in plans])), axis=0)
    out = dict(V.flags(c), n_throws=len(sample), pinocchio_version=pin.__version__,
               apt="ros-jazzy-pinocchio 4.1.0-1noble on packages.ros.org (not installed; venv uses pip pin 4.1.0)",
               time_ms=dict(branch_and_kappa=1000 * float(np.median(t_branch)), generate_throw=1000 * float(np.median(t_gen)),
                            mujoco_id_per_throw=1000 * float(np.median(t_id)), pinocchio_rnea_per_sample_us=1e6 * float(np.median(t_pin)),
                            samples_per_throw=int(np.median([len(p.t) for p, _ in plans]))),
               peak_abs_tau_mujoco=peak, mjcf_parse=res["mjcf_parse"],
               note_models="urdf: robot_description (ur_description + robotiq_description inertias); urdf+armature: plus "
                           "MuJoCo's 0.1 kg m^2 rotor inertia; mjcf: pin.buildModelFromMJCF of the MuJoCo model (its "
                           "importer gives wrist_3 + gripper ~0.85 kg more mass than MuJoCo compiles); mujoco_params: "
                           "pinocchio model built from the compiled MuJoCo body parameters (+ armature)")
    for k, v in rows.items():
        if not v:
            continue
        v = np.array(v)
        out[f"diff_{k}"] = dict(max_abs=np.max(np.abs(v), axis=0), rms=np.sqrt(np.mean(v ** 2, axis=0)),
                                max_rel_to_limit=float(np.max(np.abs(v) / V.TORQUE_LIMIT)))
        print(f"pinocchio ({k}) - MuJoCo: max |diff| {np.round(np.max(np.abs(v), axis=0), 3).tolist()} N m, "
              f"rms {np.round(np.sqrt(np.mean(v ** 2, axis=0)), 3).tolist()}")
    print("timing [ms]:", {k: round(v, 3) for k, v in out["time_ms"].items()}, "MJCF parse:", res["mjcf_parse"])
    V.save_json("inverse_dynamics.json", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--id", action="store_true")
    ap.add_argument("--square", action="store_true", help="palm pitch chosen so the cube arrives square to B's palm")
    a = ap.parse_args()
    c = V.cfg()
    if a.id:
        run_id(c)
    else:
        run_targets(c, a.quick, a.square)


if __name__ == "__main__":
    main()
