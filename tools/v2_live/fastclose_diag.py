#!/usr/bin/env python3
"""fast_close live (90 %) vs offline (69 %): run the offline catch sim (tools/v2_catch_sim.py, live-match settings)
on the SAME throws as the live benchmark cells (pinch library entries drawn like reactive_eval --seed 1, without
release noise) and compare per entry. venv + MuJoCo; --procs to parallelise (default 1).

    env -u PYTHONPATH .venv/bin/python tools/v2_live/fastclose_diag.py [--profiles fast_close realistic] [--n 60]
Output: docs/v2/phase2/fastclose_diag.json
"""
import argparse
import json
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import v2_catch_sim as CS  # noqa: E402
import v2_common as V  # noqa: E402

REPO = V.REPO


def live_throws(n, seed=1, noise=None):
    lib = json.load(open(os.path.join(REPO, "src/catch_throw/config/library_pinch.json")))["entries"]
    rng = np.random.default_rng(seed)                 # reactive_eval: lib_rng["pinch"] = default_rng(args.seed)
    out = []
    if noise:                                         # the release velocity throw_node commands (same noise seeds)
        import yaml
        from catch_throw.library import sample_noise
        lv = yaml.safe_load(open(os.path.join(REPO, "src/catch_throw/config/v2.yaml")))["noise"][noise]
    for i in range(n):
        e = lib[int(rng.integers(len(lib)))]
        v = e["params"]["v_release"]
        if noise:
            v = sample_noise(lv, v, np.random.default_rng(seed * 1000 + i))[0].tolist()
        out.append(dict(entry=e["id"] if "id" in e else None, p_release=e["params"]["p_release"],
                        v_release=v, spin=e["spin"], t_target=e["t_target"], target=e["target"]))
    return out


def live_kw(c, extra=None):
    cs = c["catch_sim"]
    kw = dict(mode="pinch", cone_deg=cs["pinch_cone_deg"], close_lead=-0.010, camera_rate=128.7, det_latency=0.005,
              first_visible=0.047, noise_model=dict(white_mm=1.0, corr_mm=0.0, corr_tau_s=0.2))
    kw.update(extra or {})
    return kw


S = {}


def _init(prof):
    sim = S["sim"] = CS.CatchSim(prof)
    orig = sim._control

    def traced(traj, k_ref):                     # per-step trace: time, finger gap, cube and TCP position
        d = sim.d
        S["trace"].append((d.time, V.gap_for_knuckle(d.qpos[sim.kq]), *d.xpos[sim.cube], *d.site_xpos[sim.tcp],
                           *d.xmat[sim.cube], *d.site_xmat[sim.tcp]))
        orig(traj, k_ref)
    sim._control = traced


def timing(th, tr, t_rel):
    """Same metrics as tools/v2_live/close_timing.py: gap 40 mm vs the free-flight arrival at the TCP (ballistic path
    from the release state, closest approach to the TCP where the cube came closest), gap speed 54 -> 42 mm."""
    tr = np.array(tr)
    t, gap, pc, pt = tr[:, 0], tr[:, 1], tr[:, 2:5], tr[:, 5:8]
    after = t > t_rel
    dist = np.linalg.norm(pc - pt, axis=1)
    i_min = int(np.argmin(np.where(after, dist, np.inf)))
    p_tcp = pt[i_min]
    ts = np.arange(0.0, 0.8, 0.0002)
    path = np.array(th["p_release"]) + np.outer(ts, th["v_release"]) + 0.5 * np.outer(ts ** 2, V.GVEC)
    t_arr = t_rel + ts[int(np.argmin(np.linalg.norm(path - p_tcp, axis=1)))]
    i40 = np.where(after & (gap < 40.0))[0]
    a_, b_ = np.where(after & (gap < 54.0))[0], np.where(after & (gap < 42.0))[0]
    sp = 12.0 / (t[b_[0]] - t[a_[0]]) if len(a_) and len(b_) and t[b_[0]] > t[a_[0]] else np.nan
    # cube misalignment in the hand at the closest approach: smallest rotation between the cube's and the TCP's
    # axes over the cube's 24 symmetries (0 = faces square to the pads)
    Rc, Rt = tr[i_min, 8:17].reshape(3, 3), tr[i_min, 17:26].reshape(3, 3)
    return (1000 * (t[i40[0]] - t_arr) if len(i40) else np.nan), sp, misalign_deg(Rt, Rc)


def _syms():
    import itertools
    out = []
    for perm in itertools.permutations(range(3)):
        for sg in itertools.product((1, -1), repeat=3):
            M = np.zeros((3, 3))
            for i, j in enumerate(perm):
                M[i, j] = sg[i]
            if np.linalg.det(M) > 0:
                out.append(M)
    return out


SYMS = _syms()


def misalign_deg(R_tool, R_cube):
    return float(min(np.degrees(np.arccos(np.clip((np.trace(R_tool.T @ R_cube @ Sm) - 1) / 2, -1, 1))) for Sm in SYMS))


def _job(a):
    i, th, kw = a
    S["trace"] = []
    r = S["sim"].run(dict(th), rng=np.random.default_rng(i), **kw)
    dt40, sp, mis = timing(th, S["trace"], 0.3)     # run() releases the cube at t = 0.3 s (after settling)
    return dict(i=i, entry=th["entry"], held=bool(r["held"]), outcome=r["outcome"], fingertip=bool(r.get("fingertip_grasp")),
                rel_mm=r.get("rel_mm"), t_cmd=r.get("t_cmd"), t_intercept=r.get("t_intercept"),
                gap40_minus_arrival_ms=round(float(dt40), 1), gap_speed_54_42=round(float(sp), 0),
                misalign_deg=round(mis, 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profiles", nargs="+", default=["fast_close", "realistic"])
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--procs", type=int, default=1)
    ap.add_argument("--tag", default="")
    ap.add_argument("--noise", default=None, help="apply the live release-noise draws of this level (reactive_eval seeds)")
    ap.add_argument("--kw", default="{}", help="extra run() kwargs as JSON (e.g. a close_lead to test)")
    a = ap.parse_args()
    c = V.cfg()
    ths = live_throws(a.n, noise=a.noise)
    out_p = os.path.join(REPO, "docs/v2/phase2/fastclose_diag.json")
    out = json.load(open(out_p)) if os.path.exists(out_p) else {}
    for prof in a.profiles:
        kw = live_kw(c, json.loads(a.kw))
        with mp.Pool(a.procs, initializer=_init, initargs=(prof,)) as pool:
            rows = pool.map(_job, [(i, th, kw) for i, th in enumerate(ths)])
        k = sum(r["held"] for r in rows)
        key = prof + (f"|{a.tag}" if a.tag else "")
        out[key] = dict(kw=kw, held=k, n=len(rows), rate=k / len(rows), ci95=CS.wilson(k, len(rows)), rows=rows,
                        fingertip=sum(r["fingertip"] for r in rows))
        g = np.array([r["gap40_minus_arrival_ms"] for r in rows], dtype=float)
        sp = np.array([r["gap_speed_54_42"] for r in rows], dtype=float)
        out[key].update(gap40_minus_arrival_ms_median=round(float(np.nanmedian(g)), 1),
                        gap_speed_median=round(float(np.nanmedian(sp)), 0))
        print(f"{key}: {k}/{len(rows)} held ({100 * k / len(rows):.0f} %), fingertip {out[key]['fingertip']}; gap 40 mm "
              f"at arrival {np.nanmedian(g):+.1f} ms, gap speed {np.nanmedian(sp):.0f} mm/s (medians)", flush=True)
        json.dump(out, open(out_p, "w"), indent=1)


if __name__ == "__main__":
    main()
