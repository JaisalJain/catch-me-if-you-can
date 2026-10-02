#!/usr/bin/env python3
"""v2 Phase 1, item 5: catch modes in offline MuJoCo (the generated scene the ROS sim loads), vision-only B.

    env -u PYTHONPATH .venv/bin/python tools/v2_catch_sim.py --gripper          # achieved closing speeds
    env -u PYTHONPATH .venv/bin/python tools/v2_catch_sim.py --cup [--throws square]   # natural cup vs cone angle
    (results are saved after every row; a rerun resumes and skips rows already in docs/v2/catch_*.json)
    env -u PYTHONPATH .venv/bin/python tools/v2_catch_sim.py --pinch            # pinch: closing speed x velocity matching
    env -u PYTHONPATH .venv/bin/python tools/v2_catch_sim.py --one N --mode cup --cone 30 [--gripper-profile fast_idealized]
    (--gripper-profile overrides v2_study.yaml's gripper_profile; every result row carries it)

Per trial: a throw from docs/v2/throw_distribution.json is launched from A's release state (position, velocity,
spin and orientation of A's toss; A's arm holds home). B starts at the ready pose and knows nothing of the plan:
  * perception: cube positions at camera_rate_hz from first_visible_s on, measured-detector noise (white +
    Gauss-Markov, fitted to tools/v2_perception.py --detect), available latency_s later;
  * B fits the ballistic LSQ (known g); as soon as the predicted 95 % error at the intercept is <= start_mm it
    picks the intercept time on the predicted path with the most limit margin (quintic from its current state:
    speed <= 85 %, inverse-dynamics torque <= 80 %), then re-plans the same intercept time on every new estimate
    (replan) until freeze_before_s;
  * B's arm: PD (JTC gains) + inverse-dynamics feed-forward on the desired trajectory, like the JTC effort loop;
  * cup: palm normal against the predicted velocity, limited to a cone around the ready approach axis; at rest
    arrive_early_s before arrival; fingers close (at the profile's gap speed) when the palm touch sensor fires;
  * pinch: closing axis across the velocity, natural approach axis; the arm matches a fraction `match` of the
    cube's velocity (and gravity) at the intercept, then decelerates to rest; the fingers start closing from the
    pre-opened gap so that they reach the cube width (40 mm) at the predicted arrival.
Success: 0.8 s after the arm stops, the cube is at rest between the pads (< 25 mm from the TCP, knuckle in the
holding range), off the table, AND every executed (re)plan stayed within 85 % speed / 80 % torque (otherwise the
trial counts as "over limits"; held_ignoring_limits keeps the raw physics outcome).
"""
import argparse
import json
import multiprocessing as mp
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

import v2_common as V  # noqa: E402

CHI3_95 = 2.7955


class CatchSim:
    def __init__(self, profile_name=None, gap_speed=None):
        c = self.c = V.cfg()
        self.profile_name, self.prof = V.gripper_profile(c, profile_name)
        self.gap_speed = gap_speed if gap_speed is not None else self.prof["gap_speed"]
        m, scene, phys = V.build_model(profile=self.prof)
        self.m, self.d, self.scene = m, mujoco.MjData(m), scene
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.qa = {k: np.array([m.jnt_qposadr[jid(f"{k}_{j}")] for j in V.JN]) for k in "ab"}
        self.va = {k: np.array([m.jnt_dofadr[jid(f"{k}_{j}")] for j in V.JN]) for k in "ab"}
        self.act = {k: [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{k}_{j}_motor") for j in V.JN] for k in "ab"}
        self.grip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "b_robotiq_85_left_knuckle_joint")
        self.kq = m.jnt_qposadr[jid("b_robotiq_85_left_knuckle_joint")]
        self.cube = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube_red")
        self.cgeom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "cube_red_geom")
        self.cq = m.jnt_qposadr[m.body_jntadr[self.cube]]
        self.cv = m.jnt_dofadr[m.body_jntadr[self.cube]]
        self.tcp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "b_robotiq_tcp")
        self.touch = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "b_robotiq_palm_touch")]
        self.bgeoms = {g for g in range(m.ngeom)
                       if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "").startswith("b_")}
        y = yaml.safe_load(open(os.path.join(V.REPO, "src/catch_sim/config/controllers.yaml")))
        g = y["arm_b_controller"]["ros__parameters"]["gains"]
        self.kp = np.array([g[f"b_{j}"]["p"] for j in V.JN])
        self.kd = np.array([g[f"b_{j}"]["d"] for j in V.JN])
        self.robot = V.F.make_robots(scene)["b"]
        self.dyn_ff = V.Dyn(scene, *V.build_mjcf.load_cfg()[1:], gravity=False)
        self.dyn_lim = V.Dyn(scene, *V.build_mjcf.load_cfg()[1:], gravity=True)
        self.poses = yaml.safe_load(open(os.path.join(V.REPO, "src/catch_description/config/poses.yaml")))
        ready = json.load(open(os.path.join(V.OUT, "ready_envelope.json")))["chosen"]
        self.q_ready = np.array(ready["q"])
        self.z_ready = V.ready_rotation(ready["elev"])[:, 2]
        self.table_z = scene["table"]["top_z"]

    # ------------------------------------------------------------------ helpers -------------------------------
    def jdot_qd(self, q, qd):
        h = 1e-6
        return (self.robot.jacobian(q + h * qd) - self.robot.jacobian(q)) / h @ qd

    def joint_target(self, p, R, v_tcp, a_tcp, q_ref):
        q = V.ik_near(self.robot, R, p, q_ref, max_jump=2.5)
        if q is None:
            return None
        J = self.robot.jacobian(q)
        if abs(np.linalg.det(J)) < 1e-5:
            return None
        Ji = np.linalg.inv(J)
        qd = Ji @ np.r_[v_tcp, 0, 0, 0]
        qdd = Ji @ (np.r_[a_tcp, 0, 0, 0] - self.jdot_qd(q, qd))
        return q, qd, qdd

    def build(self, now, state, t_c, p_c, v_c, mode, cone, match):
        """Trajectory from `state` at `now` to the intercept (and, for the pinch, the follow-through to rest).
        Returns (Piecewise, limit ratio, info) or None."""
        R, _ = V.catch_rotation(self.z_ready, v_c, cone, mode)
        lim = self.c["limits"]
        q0, qd0, qdd0 = state
        if mode == "cup":
            t_arr = t_c - self.c["catch_sim"]["arrive_early_s"]
            if t_arr - now < 0.03:
                return None
            tgt = self.joint_target(p_c, R, np.zeros(3), np.zeros(3), self.q_ready)
            if tgt is None:
                return None
            seg = V.Quintic(q0, qd0, qdd0, tgt[0], 0, 0, t_arr - now)
            traj = V.Piecewise([(now, seg)])
            chk = V.check_traj(traj, now, t_arr, self.dyn_lim, n=12)
            return traj, V.limit_ratio(chk, lim), dict(match=0.0)
        if t_c - now < 0.03:
            return None
        tgt = self.joint_target(p_c, R, match * v_c, match * V.GVEC, self.q_ready)
        if tgt is None:
            return None
        seg = V.Quintic(q0, qd0, qdd0, *tgt, t_c - now)
        best = None
        for T_stop in (0.15, 0.2, 0.3, 0.4):
            p_end = p_c + match * v_c * T_stop / 2
            p_end[2] = max(p_end[2], self.table_z + lim["table_clearance"])
            q_end = V.ik_near(self.robot, R, p_end, tgt[0], max_jump=1.5)
            if q_end is None:
                continue
            fol = V.Quintic(*tgt, q_end, 0, 0, T_stop)
            traj = V.Piecewise([(now, seg), (t_c, fol)])
            chk = V.check_traj(traj, now, t_c + T_stop, self.dyn_lim, n=16)
            r = V.limit_ratio(chk, lim)
            if best is None or r < best[1]:
                best = (traj, r, dict(match=match, T_stop=T_stop))
        return best

    # --------------------------------------------------------------------- trial ------------------------------
    def run(self, throw, mode="cup", cone_deg=30.0, match=0.0, rng=None, perception="measured", replan=None,
            gap_speed=None, cup_pre_gap=None):
        c, cs = self.c, self.c["catch_sim"]
        rng = rng or np.random.default_rng(0)
        replan = cs["replan"] if replan is None else replan
        gap_speed = gap_speed or self.gap_speed
        cone = np.radians(min(cone_deg, 180.0))
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        d.qpos[self.qa["a"]] = self.poses["a"]["home"]
        d.qpos[self.qa["b"]] = self.q_ready
        pinch = mode.startswith("pinch")
        pre_gap = c["catch_sim"]["pinch_pre_gap_mm"] if pinch else (cup_pre_gap or 85.2)
        k_open = V.knuckle_for_gap(pre_gap)
        park = {"red": [0.0, -0.25, 0.8], "green": [0.0, -0.3, 0.8], "blue": [0.0, -0.35, 0.8]}
        for col, pos in park.items():
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"cube_{col}")
            a = m.jnt_qposadr[m.body_jntadr[b]]
            d.qpos[a:a + 3] = pos
        mujoco.mj_forward(m, d)
        traj = V.Piecewise([(0.0, V.Quintic(self.q_ready, 0, 0, self.q_ready, 0, 0, 1.0))])
        # settle (gripper pre-open, arms holding) before the release
        while d.time < 0.3:
            self._control(traj, k_open)
            mujoco.mj_step(m, d)
        t0 = d.time
        p_r, v_r = np.array(throw["p_release"]), np.array(throw["v_release"])
        n = v_r / np.linalg.norm(v_r)
        from catch_throw.generator import pitch_axis, toss_orientation
        R_r = toss_orientation(n, [0, 1.0, 0])
        spin = throw["spin"] * pitch_axis(n)
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, R_r.flatten())
        d.qpos[self.cq:self.cq + 3], d.qpos[self.cq + 3:self.cq + 7] = p_r, quat
        d.qvel[self.cv:self.cv + 3], d.qvel[self.cv + 3:self.cv + 6] = v_r, R_r.T @ spin
        mujoco.mj_forward(m, d)
        # perception
        pc = c["perception"]
        rate, lat = cs["camera_rate_hz"], cs["latency_s"]
        nm = cs["noise_model"]
        if perception == "perfect":
            rate, lat, nm = 1000.0, 0.0, dict(white_mm=0.0, corr_mm=0.0, corr_tau_s=1.0)
        a_gm = np.exp(-(1.0 / rate) / nm["corr_tau_s"])
        gm = rng.normal(0, nm["corr_mm"] / 1000, 3)
        next_frame = t0 + pc["first_visible_s"]
        frames, pending = [], []          # (t_frame, measured position); (t_available, index)
        planned = False
        t_c = None
        info = dict(mode=mode, cone_deg=cone_deg, match_cmd=match, gripper_profile=self.profile_name, pre_gap_mm=pre_gap,
                    gap_speed=gap_speed, perception=perception, replan=replan)
        k_ref_fn = lambda t: k_open
        t_hit = t_cmd = None
        touched_b = False
        start_mm = cs["start_mm"]
        sigma_eff = np.hypot(nm["white_mm"], nm["corr_mm"]) / 1000
        ratios = []
        t_end = t0 + 3.0
        while d.time < t_end:
            now = d.time
            if now + 1e-9 >= next_frame:
                gt = d.xpos[self.cube].copy()
                gm = a_gm * gm + np.sqrt(1 - a_gm ** 2) * rng.normal(0, nm["corr_mm"] / 1000, 3)
                frames.append((now - t0, gt + gm + rng.normal(0, nm["white_mm"] / 1000, 3)))
                pending.append(now + lat)
                next_frame += 1.0 / rate
            new_est = False
            while pending and pending[0] <= now + 1e-9:
                pending.pop(0)
                new_est = True
            n_avail = len(frames) - len(pending)
            if new_est and n_avail >= 4 and (not planned or (replan and now < t_c - cs["freeze_before_s"])):
                tk = np.array([f[0] for f in frames[:n_avail]])
                pk = np.array([f[1] for f in frames[:n_avail]])
                y = pk - 0.5 * np.outer(tk ** 2, V.GVEC)
                Am = np.column_stack([np.ones_like(tk), tk])
                sol, *_ = np.linalg.lstsq(Am, y, rcond=None)
                p0, v0 = sol[0], sol[1]
                pred = lambda t: (p0 + v0 * t + 0.5 * V.GVEC * t * t, v0 + V.GVEC * t)
                state = traj(now)
                if not planned:
                    # predicted 95 % error at a nominal intercept 0.44 s after release
                    Ci = np.linalg.inv(Am.T @ Am)
                    h = np.array([1.0, 0.44])
                    e95 = CHI3_95 * sigma_eff * np.sqrt(h @ Ci @ h) * 1000
                    if e95 <= start_mm:
                        def search(mt):
                            best = None
                            for tc in np.arange(now - t0 + 0.08, 1.0, 0.02):
                                pp, vv = pred(tc)
                                if vv[2] >= 0:
                                    continue
                                if pp[2] < self.table_z + 0.25 or pp[0] > 0.65:
                                    break
                                r = self.build(now, state, t0 + tc, pp, vv, mode, cone, mt)
                                if r is not None and (best is None or r[1] < best[1][1] - 1e-9):
                                    best = (tc, r)
                            return best
                        best = search(match)
                        if best is not None and mode != "cup" and best[1][1] > 1.0:
                            # deliverable velocity matching: reduce the matched fraction until within limits
                            for mt in np.arange(match - 0.1, -1e-9, -0.1):
                                b2 = search(max(mt, 0.0))
                                if b2 is not None and (b2[1][1] <= 1.0 or b2[1][1] < best[1][1]):
                                    best = b2
                                    if b2[1][1] <= 1.0:
                                        break
                        if best is not None:
                            planned = True
                            t_c = t0 + best[0]
                            traj, ratio, extra = best[1]
                            info.update(t_plan=now - t0, t_intercept=best[0], plan_ratio=ratio, e95_at_plan_mm=e95, **extra)
                            ratios.append(ratio)
                else:
                    pp, vv = pred(t_c - t0)
                    r = self.build(now, state, t_c, pp, vv, mode, cone, info.get("match", 0.0))
                    if r is not None:
                        traj = r[0]
                        ratios.append(r[1])
                if planned:
                    pp, vv = pred(t_c - t0)
                    info["pred_intercept"] = pp.tolist()
                    if pinch:
                        # fingers reach the cube width at the predicted arrival
                        k_rate = gap_speed * 1000.0
                        t_close = (pre_gap - 40.0) / k_rate
                        t_cmd = t_c - t_close
                        k_ref_fn = (lambda tc0: (lambda t: k_open if t < tc0 else
                                                 V.knuckle_for_gap(max(pre_gap - k_rate * (t - tc0), 0.0))))(t_cmd)
            # cup trigger: palm touch sensor (contact only)
            if mode == "cup" and t_hit is None and d.sensordata[self.touch] > cs["touch_threshold_n"]:
                t_hit = now
                t_cmd = now + cs["trigger_latency_s"]
                k_rate = gap_speed * 1000.0
                k_ref_fn = (lambda tc0: (lambda t: k_open if t < tc0 else
                                         V.knuckle_for_gap(max(pre_gap - k_rate * (t - tc0), 0.0))))(t_cmd)
            k_ref = min(k_ref_fn(now), cs["close_to"])
            self._control(traj, k_ref)
            mujoco.mj_step(m, d)
            if not touched_b:
                for i in range(d.ncon):
                    g1, g2 = d.contact[i].geom1, d.contact[i].geom2
                    if (g1 == self.cgeom and g2 in self.bgeoms) or (g2 == self.cgeom and g1 in self.bgeoms):
                        touched_b = True
                        break
            if planned and t_end > t0 + 2.9:
                t_end = max(traj.t_end, t_c) + cs["settle_s"]
            if not planned and now - t0 > 1.0:
                break
        R_t = d.site_xmat[self.tcp].reshape(3, 3)
        rel = R_t.T @ (d.xpos[self.cube] - d.site_xpos[self.tcp])
        k = d.qpos[self.kq]
        speed = np.linalg.norm(d.qvel[self.cv:self.cv + 3])
        held_raw = bool(planned and np.linalg.norm(rel) < 0.025 and 0.30 <= k <= 0.60 and speed < 0.05
                        and d.xpos[self.cube][2] > self.table_z + 0.05)
        within = bool(ratios) and max(ratios) <= 1.02        # every executed (re)plan within 85 % speed / 80 % torque
        held = held_raw and within
        info.update(held=held, held_ignoring_limits=held_raw, within_limits=within, planned=planned, touched=touched_b, rel_mm=(1000 * rel).round(1).tolist(), knuckle=float(k),
                    cube_z=float(d.xpos[self.cube][2]), t_hit=None if t_hit is None else t_hit - t0,
                    t_cmd=None if t_cmd is None else t_cmd - t0, max_plan_ratio=max(ratios) if ratios else None,
                    outcome="held" if held else ("no plan" if not planned else ("over limits" if not within else
                                                                             ("missed" if not touched_b else "dropped"))))
        return info

    def _control(self, traj, k_ref):
        d = self.d
        q_d, qd_d, qdd_d = traj(d.time)
        tau_ff = self.dyn_ff.tau1(q_d, qd_d, qdd_d)
        u = self.kp * (q_d - d.qpos[self.qa["b"]]) + self.kd * (qd_d - d.qvel[self.va["b"]]) + tau_ff
        d.ctrl[self.act["b"]] = u
        qa = np.array(self.poses["a"]["home"])
        d.ctrl[self.act["a"]] = 3000 * (qa - d.qpos[self.qa["a"]]) - 100 * d.qvel[self.va["a"]]
        d.ctrl[self.grip] = k_ref

    def gripper_speed(self, gap_speed, start_gap=85.2):
        """Achieved closing (no cube): time from start_gap to 40 mm."""
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        d.qpos[self.qa["b"]] = self.q_ready
        traj = V.Piecewise([(0.0, V.Quintic(self.q_ready, 0, 0, self.q_ready, 0, 0, 1.0))])
        k0 = V.knuckle_for_gap(start_gap)
        while d.time < 0.4:
            self._control(traj, k0)
            mujoco.mj_step(m, d)
        t0 = d.time
        rate = gap_speed * 1000
        while d.time < t0 + 1.0:
            self._control(traj, min(V.knuckle_for_gap(max(start_gap - rate * (d.time - t0), 0)), 0.7))
            mujoco.mj_step(m, d)
            if V.gap_for_knuckle(d.qpos[self.kq]) <= 40.0:
                return d.time - t0
        return None


# ----------------------------------------------------------------------------- batch ---------------------------
S = {}


def _init(profile, gap_speed):
    S["sim"] = CatchSim(profile, gap_speed)


def _job(args):
    throw, kw, seed = args
    sim = S["sim"]
    if kw.get("gap_speed") and abs(kw["gap_speed"] - sim.gap_speed) > 1e-9:
        pass
    return sim.run(throw, rng=np.random.default_rng(seed), **kw)


def throws(c, n=None, kind="random"):
    sfx = "_square" if kind == "square" else ""
    dist = json.load(open(os.path.join(V.OUT, f"throw_distribution{sfx}.json")))["distribution"]
    rng = np.random.default_rng(c["catch_sim"]["seed"])
    n = n or c["catch_sim"]["n_throws"]
    idx = rng.choice(len(dist), size=n, replace=len(dist) < n)
    return [dist[i] for i in idx]


def batch(profile, kw, ths, seed0=0, gap_speed=None):
    with mp.Pool(min(22, os.cpu_count()), initializer=_init, initargs=(profile, gap_speed)) as pool:
        return pool.map(_job, [(t, kw, seed0 + i) for i, t in enumerate(ths)], chunksize=2)


def summarize(rows):
    from collections import Counter
    n = len(rows)
    held = sum(r["held"] for r in rows)
    k = held / n
    lo, hi = wilson(held, n)
    return dict(n=n, held=held, rate=k, ci95=[lo, hi], outcomes=dict(Counter(r["outcome"] for r in rows)),
                held_ignoring_limits=sum(r["held_ignoring_limits"] for r in rows),
                match_used_mean=float(np.mean([r.get("match", 0.0) for r in rows if r["planned"]] or [0])),
                plan_ratio_max=float(max([r["max_plan_ratio"] or 0 for r in rows] or [0])))


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 1.0
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return float(c - h), float(c + h)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gripper", action="store_true")
    ap.add_argument("--cup", action="store_true")
    ap.add_argument("--pinch", action="store_true")
    ap.add_argument("--one", type=int)
    ap.add_argument("--mode", default="cup")
    ap.add_argument("--cone", type=float, default=30)
    ap.add_argument("--match", type=float, default=0.0)
    ap.add_argument("--gripper-profile", default=None)
    ap.add_argument("--perception", default="measured")
    ap.add_argument("--n", type=int)
    ap.add_argument("--throws", default="random", choices=["random", "square"],
                    help="throw distribution: random palm pitch (min joint speed) or square arrival on B's palm")
    a = ap.parse_args()
    c = V.cfg()
    prof_name = V.gripper_profile(c, a.gripper_profile)[0]
    print(f"gripper_profile: {prof_name}", flush=True)
    if a.gripper:
        rows = []
        for name in c["gripper_profiles"]:
            sim = CatchSim(name)
            for gs in sorted(set(c["catch_sim"]["pinch_gap_speeds_mm_s"]) | {int(sim.prof["gap_speed"] * 1000)}):
                for g0 in (85.2, c["catch_sim"]["pinch_pre_gap_mm"]):
                    t = sim.gripper_speed(gs / 1000, g0)
                    rows.append(dict(gripper_profile=name, gap_speed_cmd_mm_s=gs, start_gap_mm=g0, t_to_40mm=t,
                                     achieved_mm_s=None if t is None else (g0 - 40) / t))
                    print(f"{name:15s} cmd {gs:5d} mm/s from {g0:.0f} mm: 40 mm after "
                          f"{'-' if t is None else f'{1000 * t:.0f} ms ({(g0 - 40) / t:.0f} mm/s)'}", flush=True)
        V.save_json("gripper_speed.json", dict(V.flags(c, prof_name), rows=rows))
    if a.one is not None:
        sim = CatchSim(prof_name)
        th = throws(c, kind=a.throws)[a.one]
        print(json.dumps(sim.run(th, mode=a.mode, cone_deg=a.cone, match=a.match, perception=a.perception), indent=1))
    if not (a.cup or a.pinch):
        return
    ths = throws(c, a.n, a.throws)
    sfx = "_square" if a.throws == "square" else ""
    cs = c["catch_sim"]
    other = "fast_idealized" if prof_name == "realistic" else "realistic"
    if a.cup:
        specs = []
        for prof in (prof_name, other):
            for cone in cs["cup_cone_deg"]:
                specs.append((f"{prof}|{cone}|measured", prof, dict(mode="cup", cone_deg=cone), None))
            for cone in (0, 30, 180):
                specs.append((f"{prof}|{cone}|perfect", prof, dict(mode="cup", cone_deg=cone, perception="perfect"), None))
            for cone in (10, 30, 180):       # pre-closed cup: fingers pre-opened to the pinch gap, same contact trigger
                specs.append((f"{prof}|{cone}|measured|pregap", prof,
                              dict(mode="cup", cone_deg=cone, cup_pre_gap=cs["pinch_pre_gap_mm"]), None))
        run_rows(f"catch_cup{sfx}.json", specs, ths, c, prof_name, a.throws)
    if a.pinch:
        cone = cs["pinch_cone_deg"]
        specs = []
        for prof in ("realistic", "fast_idealized"):
            for mt in cs["pinch_match"]:
                specs.append((f"profile|{prof}|pinch|{mt}", prof, dict(mode="pinch", cone_deg=cone, match=mt), None))
        for gs in cs["pinch_gap_speeds_mm_s"]:
            for mt in (0.0, 1.0):
                specs.append((f"speed|{gs}|pinch|{mt}", "fast_idealized", dict(mode="pinch", cone_deg=cone, match=mt), gs / 1000))
        # side chopsticks: over the arm limits in every trial of the first run (all matches, all closing speeds);
        # kept at two points per profile
        for prof in ("realistic", "fast_idealized"):
            for mt in (0.0, 1.0):
                specs.append((f"profile|{prof}|pinch_side|{mt}", prof, dict(mode="pinch_side", cone_deg=cone, match=mt), None))
        run_rows(f"catch_pinch{sfx}.json", specs, ths, c, prof_name, a.throws,
                 note="keys profile|<gripper_profile>|<mode>|<match commanded>, speed|<gap mm/s>|<mode>|<match> "
                      "(speed sweep: fast_idealized actuator, the ramp speed is the variable)")


def run_rows(name, specs, ths, c, prof_name, kind, note=""):
    """Run (key, gripper_profile, run kwargs, gap_speed) rows; the json is saved after every row and rows already
    in it are skipped (resumable)."""
    path = os.path.join(V.OUT, name)
    out = json.load(open(path)) if os.path.exists(path) else dict(V.flags(c, prof_name), perception=c["catch_sim"], rows={})
    out.update(throws=kind, n_throws=len(ths))
    if note:
        out["note"] = note
    for key, prof, kw, gs in specs:
        if key in out["rows"]:
            continue
        rows = batch(prof, kw, ths, gap_speed=gs)
        s = summarize(rows)
        out["rows"][key] = dict(summary=s, trials=rows, gripper_profile=prof, gap_speed=gs)
        V.save_json(name, out)
        print(f"[{kind}] {key}: {s['held']}/{s['n']} held within limits ({100 * s['rate']:.0f} %, CI "
              f"{100 * s['ci95'][0]:.0f}-{100 * s['ci95'][1]:.0f}; {s['held_ignoring_limits']} ignoring limits), "
              f"matched {s['match_used_mean']:.2f}, {s['outcomes']}", flush=True)

if __name__ == "__main__":
    main()
