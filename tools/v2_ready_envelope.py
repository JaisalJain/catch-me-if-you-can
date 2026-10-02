#!/usr/bin/env python3
"""v2 Phase 1, items 1 + 3: B's ready pose and catch envelope (offline, venv).

    env -u PYTHONPATH .venv/bin/python tools/v2_ready_envelope.py [--quick]

Ready pose candidates: TCP on a grid near the M5 catch region, approach axis (tool z) towards A at an elevation
of 0..60 deg, closing axis (tool x) along world y. IK branch = the one closest to B's home (same shoulder /
elbow / wrist family) among the "natural" ones: elbow up, >= 30 deg from the wrist singularity, joint margins,
>= 5 cm from table / pedestals / pallets. The chosen pose is rendered (docs/v2/ready_pose.png) to check that the
wrist is not flipped.

Envelope: for a ready pose, every grid point is an intercept candidate; B's move = one synchronized quintic
rest-to-rest from the ready pose to the IK solution (branch-continuous) whose duration is the shortest that keeps
joint speed <= 85 % of 180 deg/s and the inverse-dynamics torque (gravity included) <= 80 % of the UR5e limits.
Orientation at the intercept:
  ready: the ready orientation (natural pinch; closing axis across a throw in an x-z plane)
  cup:   palm normal against the nominal arrival velocity (M1 throw), full or limited to a 30 deg cone
The time left for B is the flight time minus the perception decision time (tools/v2_perception.py).
"""
import argparse
import multiprocessing as mp
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import v2_common as V  # noqa: E402

W = {}


def _init():
    import mujoco
    c = V.cfg()
    m, scene, phys = V.build_model()
    W["c"], W["m"], W["d"] = c, m, mujoco.MjData(m)
    W["robot"] = V.F.make_robots(scene)["b"]
    W["dyn"] = V.Dyn(scene, phys, gravity=True)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    W["qadr"] = [m.jnt_qposadr[jid(f"b_{j}")] for j in V.JN]
    body = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or ""
    W["bgeoms"] = [g for g in range(m.ngeom) if body(g).startswith("b_") and body(g) != "b_shoulder_link"
                   and (m.geom_contype[g] or m.geom_conaffinity[g])]
    W["env"] = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(
        ("table_top", "pallet_", "a_pedestal", "b_pedestal"))]
    import yaml
    W["home"] = np.array(yaml.safe_load(open(os.path.join(V.REPO, "src/catch_description/config/poses.yaml")))["b"]["home"])
    W["table_z"] = scene["table"]["top_z"]


def clearance(q):
    import mujoco
    m, d = W["m"], W["d"]
    d.qpos[W["qadr"]] = q
    mujoco.mj_kinematics(m, d)
    mujoco.mj_collision(m, d)
    ft = np.zeros(6)
    return min(mujoco.mj_geomDistance(m, d, a, b, 0.3, ft) for a in W["bgeoms"] for b in W["env"])


def elbow_up(robot, q):
    fr = robot.frames(q)
    s, e, w = fr[1][:3, 3], fr[2][:3, 3], fr[4][:3, 3]
    t = np.clip(np.dot(e - s, w - s) / np.dot(w - s, w - s), 0, 1)
    return bool(e[2] > (s + t * (w - s))[2])


def wdist(a, b):
    return float(np.linalg.norm(V.F.np.asarray(a) - b - 2 * np.pi * np.round((np.asarray(a) - b) / (2 * np.pi))))


def branch_metrics(q, robot, home, c):
    margin = float(np.min(V.POS_LIMIT - np.abs(q)))
    sv = np.linalg.svd(robot.jacobian(q)[:3], compute_uv=False)
    r = dict(q=q.tolist(), margin=margin, margins=(V.POS_LIMIT - np.abs(q)).tolist(), wrist_sin=float(abs(np.sin(q[4]))),
             elbow_up=elbow_up(robot, q), dist_home=float(np.linalg.norm(q - home)), manip_min_sv=float(sv.min()))
    why = []
    if not r["elbow_up"]:
        why.append("elbow down")
    if r["wrist_sin"] < c["ready"]["min_wrist_sin"]:
        why.append("near wrist singularity")
    if margin < c["limits"]["pos_margin"]:
        why.append("joint margin")
    if not why:
        r["clearance"] = float(clearance(q))
        if r["clearance"] < c["ready"]["min_clearance"]:
            why.append("clearance")
    r["why"] = why
    return r


def eval_candidate(args):
    """All IK branches; keep the natural + feasible ones; return the one closest to B's home (joint space,
    each joint unwrapped towards home)."""
    p, elev = args
    c, robot, home = W["c"], W["robot"], W["home"]
    R = V.ready_rotation(elev)
    sols = robot.ik_all(V.make_tf(R, p))
    if not sols:
        return None
    brs = [branch_metrics(home + np.arctan2(np.sin(s - home), np.cos(s - home)), robot, home, c) for s in sols]
    good = [b for b in brs if not b["why"]]
    best = min(good or brs, key=lambda b: b["dist_home"])
    res = dict(p=list(p), elev=elev, n_branches=len(brs), n_natural=len(good), wrist_sign=int(np.sign(np.sin(best["q"][4]))))
    res.update(best)
    res["ok"] = bool(good)
    from collections import Counter
    res["why"] = "" if good else ", ".join(sorted(set(w for b in brs for w in b["why"])))
    res.setdefault("clearance", float("nan"))
    return res


def move_time(q0, q1, lim, t_lo=0.03, t_hi=1.5):
    """Shortest rest-to-rest quintic duration within the speed and torque limits (bisection on T)."""
    dyn = W["dyn"]

    def ok(T):
        s = V.Quintic(q0, 0, 0, q1, 0, 0, T)
        chk = V.check_traj(s, 0, T, dyn, n=21)
        return V.limits_ok(chk, lim)

    # quintic peak speed 1.875 dq / T -> lower bound
    t_lo = max(t_lo, 1.875 * float(np.max(np.abs(q1 - q0) / (lim["vel_ratio"] * V.VEL_LIMIT))))
    if not ok(t_hi):
        return np.nan
    if ok(t_lo):
        return t_lo
    for _ in range(12):
        tm = 0.5 * (t_lo + t_hi)
        if ok(tm):
            t_hi = tm
        else:
            t_lo = tm
        if t_hi - t_lo < 0.002:
            break
    return t_hi


def eval_point(args):
    q_ready, p, R = args
    c, robot = W["c"], W["robot"]
    q = V.ik_near(robot, R, p, np.asarray(q_ready), max_jump=2.0)
    if q is None:
        return np.nan, "ik"
    if np.min(V.POS_LIMIT - np.abs(q)) < c["limits"]["pos_margin"] or abs(np.sin(q[4])) < 0.25:
        return np.nan, "margin"
    if p[2] < W["table_z"] + c["limits"]["table_clearance"] or clearance(q) < 0.02:
        return np.nan, "clearance"
    return move_time(np.asarray(q_ready), q, c["limits"]), ""


def grid(c, step=None):
    step = step or c["envelope"]["grid_step"]
    xs = np.arange(0.15, 0.6001, step)
    ys = np.arange(-0.10, 0.4501, step)
    zs = np.arange(0.95, 1.4501, step)
    return xs, ys, zs


def nominal():
    import catch_study as C
    from catch_ballistics import Projectile
    p_r, v_r, spin, R_r, T, cpt = C.throw_state()
    return Projectile(p_r, v_r), T, cpt


def family(c):
    """Prior throw family: list of (p_release, v_release) (envelope.family in v2_study.yaml)."""
    from catch_ballistics import velocity_for_launch_angle
    f = c["envelope"]["family"]
    out = []
    for y in f["y"]:
        for ang in f["angle_deg"]:
            for xc in f["x_cross"]:
                p = np.array([-0.05, y, 1.25])
                try:
                    v, _ = velocity_for_launch_angle(p, np.array([xc, y, f["z_cross"]]), np.radians(ang))
                except ValueError:
                    continue
                out.append((p, v))
    return out


def path_points(p, v, c):
    """Points on the descending branch after the decision time, until z_min: (t, p, v)."""
    f = c["envelope"]["family"]
    pts = []
    t = 0.20
    while True:
        pos = p + v * t + 0.5 * V.GVEC * t * t
        vel = v + V.GVEC * t
        if pos[2] < f["z_min"] or pos[0] > 0.65 or t > 1.2:
            break
        if vel[2] < 0:
            pts.append((t, pos, vel))
        t += f["path_dt"]
    return pts


def catchable(pool, q_ready, fam, R_fn, c, t_decs=None):
    """Fraction of the family B can intercept: some point on the path with move time <= t - t_decision.
    With t_decs: also the fraction for each of those decision times."""
    t_dec = c["envelope"]["t_decision_s"]
    jobs, owner = [], []
    for i, (p, v) in enumerate(fam):
        for t, pos, vel in path_points(p, v, c):
            jobs.append((list(q_ready), pos, R_fn(pos, vel)))
            owner.append((i, t))
    res = pool.map(eval_point, jobs, chunksize=16)
    def frac(td):
        slack = np.full(len(fam), -np.inf)
        for (i, t), (T, _) in zip(owner, res):
            if not np.isnan(T):
                slack[i] = max(slack[i], (t - td) - T)
        return float(np.mean(slack >= 0)), slack
    f, slack = frac(t_dec)
    if t_decs is None:
        return f, slack
    return f, slack, {float(td): frac(td)[0] for td in t_decs}


def envelope(pool, q_ready, R_fn, xs, ys, zs):
    pts = [(x, y, z) for x in xs for y in ys for z in zs]
    res = pool.map(eval_point, [(list(q_ready), np.array(p), R_fn(np.array(p))) for p in pts], chunksize=16)
    T = np.array([r[0] for r in res]).reshape(len(xs), len(ys), len(zs))
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    c = V.cfg()
    rc = c["ready"]
    _init()
    pool = mp.Pool(min(22, os.cpu_count()), initializer=_init)
    cands = [((x, y, z), e) for x in rc["x"] for y in rc["y"] for z in rc["z"] for e in rc["elevation_deg"]]
    res = [r for r in pool.map(eval_candidate, cands, chunksize=8) if r is not None]
    ok = [r for r in res if r["ok"]]
    from collections import Counter
    print(f"{len(cands)} candidates, {len(res)} with IK, {len(ok)} natural + feasible; rejections:",
          Counter(w for r in res for w in r["why"].split(", ") if w).most_common())
    # pre-rank by margins (joint margin, manipulability, closeness to home), keep the best per (elevation, x) cell
    for r in ok:
        r["pre_score"] = min(r["margin"], 1.5) + 2.0 * r["manip_min_sv"] - 0.15 * r["dist_home"]
    ok.sort(key=lambda r: r["pre_score"], reverse=True)
    pick, seen = [], set()
    for r in ok:
        key = (r["elev"], round(r["p"][0], 2))
        if key not in seen:
            seen.add(key)
            pick.append(r)
    if a.quick:
        pick = pick[::6]
    flight, T_nom, cpt = nominal()
    v_nom = flight.velocity(T_nom)
    fam = family(c)
    print(f"scoring {len(pick)} ready candidates on {len(fam)} throws of the prior family", flush=True)
    for r in pick:
        z_ready = V.ready_rotation(r["elev"])[:, 2]
        r["catch_ready"] = catchable(pool, r["q"], fam, lambda p, v: V.ready_rotation(r["elev"]), c)[0]
        r["catch_cup30"] = catchable(pool, r["q"], fam, lambda p, v: V.catch_rotation(z_ready, v, np.radians(30), "cup")[0], c)[0]
        r["score"] = 0.5 * (r["catch_ready"] + r["catch_cup30"])
        print(f"p={np.round(r['p'], 2).tolist()} elev={r['elev']:2d} margin={r['margin']:.2f} sv={r['manip_min_sv']:.2f} "
              f"home_dist={r['dist_home']:.2f} clr={r['clearance'] * 1000:.0f} mm -> catchable: ready-orientation "
              f"{r['catch_ready']:.2f}, cup-30 {r['catch_cup30']:.2f}", flush=True)
    # refinement: all natural candidates within 5 cm in x of the best cell, any y / z, elevation +-15 deg
    b0 = max(pick, key=lambda r: r["score"])
    done = {(tuple(np.round(r["p"], 3)), r["elev"]) for r in pick}
    ref = [r for r in ok if abs(r["p"][0] - b0["p"][0]) <= 0.051 and abs(r["elev"] - b0["elev"]) <= 15
           and (tuple(np.round(r["p"], 3)), r["elev"]) not in done]
    if a.quick:
        ref = ref[::10]
    print(f"refining around {np.round(b0['p'], 2).tolist()} elev {b0['elev']}: {len(ref)} more candidates", flush=True)
    for r in ref:
        z_ready = V.ready_rotation(r["elev"])[:, 2]
        r["catch_ready"] = catchable(pool, r["q"], fam, lambda p, v: V.ready_rotation(r["elev"]), c)[0]
        r["catch_cup30"] = catchable(pool, r["q"], fam, lambda p, v: V.catch_rotation(z_ready, v, np.radians(30), "cup")[0], c)[0]
        r["score"] = 0.5 * (r["catch_ready"] + r["catch_cup30"])
    pick += ref
    top = sorted(pick, key=lambda r: r["score"], reverse=True)[:10]
    for r in top:
        print(f"  top: p={np.round(r['p'], 2).tolist()} elev={r['elev']:2d} margin={r['margin']:.2f} clr={r['clearance'] * 1000:.0f} mm "
              f"catchable ready {r['catch_ready']:.2f} cup30 {r['catch_cup30']:.2f}")
    # among the candidates within 0.01 of the best score, prefer the larger joint margin
    smax = top[0]["score"]
    best = max([r for r in top if r["score"] >= smax - 0.01], key=lambda r: r["margin"])
    print("chosen ready pose:", best)
    # full-resolution envelope of the chosen pose
    xs, ys, zs = grid(c)
    R_ready = V.ready_rotation(best["elev"])
    z_ready = R_ready[:, 2]
    env = {}
    for name, fn in (("ready", lambda p: R_ready),
                     ("cup30", lambda p: V.catch_rotation(z_ready, v_nom, np.radians(30), "cup")[0]),
                     ("cup_full", lambda p: V.catch_rotation(z_ready, v_nom, np.pi, "cup")[0])):
        env[name] = envelope(pool, best["q"], fn, xs, ys, zs)
        for tl in c["envelope"]["t_left_s"]:
            print(f"  {name}: {int(np.sum(env[name] <= tl))} grid points ({c['envelope']['grid_step'] * 100:.1f} cm) reachable in {tl} s")
    t_decs = [0.10, 0.15, 0.20, 0.25, 0.30]
    fam_res = {}
    for name, fn in (("ready", lambda p, v: R_ready),
                     ("cup30", lambda p, v: V.catch_rotation(z_ready, v, np.radians(30), "cup")[0]),
                     ("cup_full", lambda p, v: V.catch_rotation(z_ready, v, np.pi, "cup")[0])):
        f, slack, by_t = catchable(pool, best["q"], fam, fn, c, t_decs)
        fam_res[name] = dict(catchable=f, slack=np.where(np.isinf(slack), -9, slack), by_t_decision=by_t)
        print(f"  {name}: catchable {f:.2f} of the family at t_decision {c['envelope']['t_decision_s']} s; by decision time "
              + ", ".join(f"{k:.2f}s: {v:.2f}" for k, v in by_t.items()))
    pool.close()
    out = dict(V.flags(c), candidates=res, evaluated=pick, chosen=best, grid=dict(x=xs, y=ys, z=zs),
               family=[dict(p=p, v=v) for p, v in fam], family_result=fam_res,
               envelope_move_time={k: np.where(np.isnan(v), -1, v) for k, v in env.items()},
               nominal_throw=dict(intercept=cpt, v=v_nom, T=T_nom),
               note="move time [s] rest-to-rest from the ready pose, -1 = unreachable; orientation: ready / cup "
                    "limited to 30 deg around the ready approach axis / cup full (palm against the M1 arrival velocity)")
    V.save_json("ready_envelope.json", out)
    plot(out, c)
    import yaml
    home = yaml.safe_load(open(os.path.join(V.REPO, "src/catch_description/config/poses.yaml")))["b"]["home"]
    render_pose(best["q"], os.path.join(V.OUT, "ready_pose.png"), extra=[(home, "B home (v1)")])


def render_pose(q_b, path, extra=()):
    """Render B at q_b (A at home) from two free cameras -> png (checks that the wrist is not flipped)."""
    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
    import mujoco
    import yaml
    m, _, _ = V.build_model()
    d = mujoco.MjData(m)
    poses = yaml.safe_load(open(os.path.join(V.REPO, "src/catch_description/config/poses.yaml")))
    jid = lambda n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
    imgs = []
    for qb, label in [(q_b, "ready")] + list(extra):
        for k, q in (("a", poses["a"]["home"]), ("b", qb)):
            for j, n in enumerate(V.JN):
                d.qpos[jid(f"{k}_{n}")] = q[j]
        mujoco.mj_forward(m, d)
        ren = mujoco.Renderer(m, 400, 560)
        row = []
        for look, dist, az, el in (([0.6, 0.1, 1.2], 1.3, 90, -8), ([0.6, 0.1, 1.2], 1.1, 50, -20)):   # from -y, from the front-left
            cam = mujoco.MjvCamera()
            cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = look, dist, az, el
            ren.update_scene(d, camera=cam)
            row.append(ren.render().copy())
        ren.close()
        import cv2
        img = np.concatenate(row, axis=1)
        cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        imgs.append(img)
    import cv2
    cv2.imwrite(path, np.concatenate(imgs, axis=0)[:, :, ::-1])


def plot(out, c):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    xs, ys, zs = (np.array(out["grid"][k]) for k in "xyz")
    flight, T_nom, cpt = nominal()
    tt = np.linspace(0, T_nom + 0.08, 200)
    traj = np.array([flight.position(t) for t in tt])
    best = out["chosen"]
    p_r = np.array(best["p"])
    levels = c["envelope"]["t_left_s"]
    fig, axs = plt.subplots(3, 3, figsize=(16, 13))
    for row, name in enumerate(("ready", "cup30", "cup_full")):
        T = np.array(out["envelope_move_time"][name], float)
        T[T < 0] = np.nan
        iy = int(np.argmin(np.abs(ys - 0.25)))
        iz = int(np.argmin(np.abs(zs - cpt[2])))
        ix = int(np.argmin(np.abs(xs - cpt[0])))
        for ax, (sl, X, Y, xl, yl, tx, ty, title) in zip(axs[row], (
                (T[:, iy, :].T, xs, zs, "x [m]", "z [m]", traj[:, 0], traj[:, 2], f"x-z at y = {ys[iy]:.2f} (throw plane)"),
                (T[:, :, iz].T, xs, ys, "x [m]", "y [m]", traj[:, 0], traj[:, 1], f"x-y at z = {zs[iz]:.2f}"),
                (T[ix, :, :].T, ys, zs, "y [m]", "z [m]", traj[:, 1], traj[:, 2], f"y-z at x = {xs[ix]:.2f}"))):
            im = ax.pcolormesh(X, Y, sl, vmin=0.1, vmax=0.5, cmap="viridis_r", shading="nearest")
            cs = ax.contour(X, Y, np.nan_to_num(sl, nan=9), levels=levels, colors="w", linewidths=1)
            ax.clabel(cs, fmt=lambda v: f"{v:.2f} s", fontsize=7)
            ax.plot(tx, ty, "r-", lw=1.2, label="M1 throw")
            ax.set_xlabel(xl)
            ax.set_ylabel(yl)
            ax.set_title(f"{name}: {title}", fontsize=9)
            ax.set_aspect("equal")
            if "x-z" in title:
                ax.plot(p_r[0], p_r[2], "w*", ms=12)
            elif "x-y" in title:
                ax.plot(p_r[0], p_r[1], "w*", ms=12)
            else:
                ax.plot(p_r[1], p_r[2], "w*", ms=12)
        fig.colorbar(im, ax=axs[row].tolist(), label="B move time from the ready pose [s]", shrink=0.8)
    fig.suptitle(f"Catch envelope from the ready pose (star): rest-to-rest move time within 85 % speed / 80 % torque; "
                 f"white contours = time left {levels} s; gripper_profile n/a (kinematics only)")
    fig.savefig(os.path.join(V.OUT, "catch_envelope.png"), dpi=100)


if __name__ == "__main__":
    main()
