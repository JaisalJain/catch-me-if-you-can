#!/usr/bin/env python3
"""v2 Phase 1, item 2: perception timing (offline, venv, NVIDIA EGL).

    env -u PYTHONPATH .venv/bin/python tools/v2_perception.py --lsq      # prediction error vs time since release
    env -u PYTHONPATH .venv/bin/python tools/v2_perception.py --detect   # measured per-frame detection noise
    env -u PYTHONPATH .venv/bin/python tools/v2_perception.py --bench    # render cost + effect on the real-time factor

--lsq: B fits p(t) = p0 + v0 t + g t^2 / 2 (gravity known) by least squares to the cube positions seen so far
(rate r, per-axis noise sigma, first frame first_visible_s after release, estimate available latency L after the
last frame) and predicts the cube position at the intercept time. The model is linear, so the prediction
covariance is exact: per axis sigma^2 [1 T] (A^T A)^-1 [1 T]^T; 3D 95 % = 2.80 sigma_axis (chi, 3 dof). A Monte
Carlo run checks it.
--detect: renders throws from the two study cameras (side_front, top) in the real scene, segments the red cube
(HSV, gated around the predicted pixel like a tracker would), triangulates the two silhouette centroids and
compares with ground truth.
--bench: low-level render path of the camera plugin (mjv_updateScene, mjr_render, mjr_readPixels RGB + depth)
on its own process/GL context, alone and next to a free-running physics process of the full scene.
"""
import argparse
import multiprocessing as mp
import os
import sys
import time

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import v2_common as V  # noqa: E402

CHI3_95 = 2.7955


def nominal_throw():
    import catch_study as C
    p_r, v_r, spin, R_r, T, c = C.throw_state()
    return p_r, v_r, T


# --------------------------------------------------------------------------------------------- LSQ --------
def pred_sigma(t_now, T, rate, latency, t_first, sigma):
    """Per-axis 1-sigma of the predicted position at T from frames in [t_first, t_now - latency]."""
    tk = np.arange(t_first, t_now - latency + 1e-9, 1.0 / rate)
    if len(tk) < 3:
        return np.inf
    A = np.column_stack([np.ones_like(tk), tk])
    C = np.linalg.inv(A.T @ A)
    h = np.array([1.0, T])
    return sigma * float(np.sqrt(h @ C @ h))


def lsq_fit(tk, pk):
    """Ballistic least squares with known gravity: returns p0, v0 (at t = 0)."""
    y = pk - 0.5 * np.outer(tk ** 2, V.GVEC)
    A = np.column_stack([np.ones_like(tk), tk])
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    return sol[0], sol[1]


def run_lsq(c):
    pc = c["perception"]
    p_r, v_r, T = nominal_throw()
    ts = np.arange(0.04, T, 0.0025)
    curves = {}
    for rate in pc["rates_hz"]:
        for s in pc["noise_mm"]:
            for L in pc["latency_s"]:
                e95 = [CHI3_95 * pred_sigma(t, T, rate, L, pc["first_visible_s"], s) for t in ts]
                curves[f"{rate}|{s}|{L}"] = e95
    # Monte Carlo check (true cube = ballistic, noisy observations)
    rng = np.random.default_rng(1)
    mc = []
    for rate, s, L, t in [(60, 4, 0.025, 0.2), (120, 2, 0.025, 0.15), (240, 1, 0.0, 0.1), (120, 2, 0.025, 0.25)]:
        tk = np.arange(pc["first_visible_s"], t - L + 1e-9, 1.0 / rate)
        truth = p_r + np.outer(tk, v_r) + 0.5 * np.outer(tk ** 2, V.GVEC)
        pT = p_r + v_r * T + 0.5 * V.GVEC * T * T
        errs = []
        for _ in range(4000):
            p0, v0 = lsq_fit(tk, truth + rng.normal(0, s / 1000, truth.shape))
            errs.append(np.linalg.norm(p0 + v0 * T + 0.5 * V.GVEC * T * T - pT) * 1000)
        mc.append(dict(rate=rate, noise_mm=s, latency=L, t=t, mc_e95_mm=float(np.percentile(errs, 95)),
                       analytic_e95_mm=CHI3_95 * pred_sigma(t, T, rate, L, pc["first_visible_s"], s)))
    # time at which the 95 % error first drops below thresholds
    tgood = []
    for key, e in curves.items():
        rate, s, L = key.split("|")
        row = dict(rate=int(rate), noise_mm=float(s), latency=float(L))
        for th in (20.0, 10.0, 5.0):
            i = np.argmax(np.array(e) <= th)
            row[f"t_{int(th)}mm"] = float(ts[i]) if e[i] <= th else None
        tgood.append(row)
    out = dict(flight_time=T, intercept_time=T, release_speed=float(np.linalg.norm(v_r)), t=ts, curves_e95_mm=curves,
               monte_carlo=mc, t_good=tgood, first_visible_s=pc["first_visible_s"],
               note="95 % 3D error of the predicted cube position at the intercept time (M1 throw, 0.442 s flight)")
    V.save_json("perception_lsq.json", out)
    for r in mc:
        print(f"MC check rate {r['rate']} sigma {r['noise_mm']} L {r['latency']} t {r['t']}: MC {r['mc_e95_mm']:.2f} mm, "
              f"analytic {r['analytic_e95_mm']:.2f} mm")
    print("time after release when the 95 % intercept error <= 10 mm (latency 25 ms):")
    for r in tgood:
        if abs(r["latency"] - 0.025) < 1e-9:
            print(f"  {r['rate']:3d} Hz sigma {r['noise_mm']:.0f} mm: t10 = {r['t_10mm']}, t5 = {r['t_5mm']}, t20 = {r['t_20mm']}")
    plot_lsq(out, c)


def plot_lsq(out, c):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pc = c["perception"]
    ts = np.array(out["t"]) * 1000
    fig, axs = plt.subplots(1, 4, figsize=(17, 4.2), sharey=True)
    colors = {60: "#d95f02", 120: "#1b9e77", 240: "#7570b3"}
    for ax, s in zip(axs[:3], pc["noise_mm"]):
        for rate in pc["rates_hz"]:
            ax.plot(ts, out["curves_e95_mm"][f"{rate}|{s}|0.025"], color=colors[rate], label=f"{rate} Hz")
        ax.set_title(f"per-frame noise {s} mm, latency 25 ms")
    for L, ls in zip(pc["latency_s"], ["-", "--", "-.", ":"]):
        axs[3].plot(ts, out["curves_e95_mm"][f"120|2|{L}"], "k", ls=ls, label=f"latency {int(L * 1000)} ms")
    axs[3].set_title("120 Hz, 2 mm: latency")
    for ax in axs:
        ax.set_yscale("log")
        ax.set_ylim(0.5, 200)
        ax.axhline(10, color="grey", lw=0.8)
        ax.axvline(out["intercept_time"] * 1000, color="grey", lw=0.8, ls=":")
        ax.set_xlabel("time since release [ms]")
        ax.grid(alpha=0.3, which="both")
        ax.legend(fontsize=8)
    axs[0].set_ylabel("95 % error of predicted intercept [mm]")
    fig.suptitle("Ballistic LSQ (known g): predicted-intercept error vs time since release (M1 throw, intercept at 442 ms)")
    fig.tight_layout()
    fig.savefig(os.path.join(V.OUT, "perception_error_vs_time.png"), dpi=110)


# ------------------------------------------------------------------------------------------ detection -------
def cam_ray(m, d, cam, u, v, W, H):
    f = (H / 2) / np.tan(np.radians(m.cam_fovy[cam]) / 2)
    dc = np.array([(u + 0.5 - W / 2) / f, -(v + 0.5 - H / 2) / f, -1.0])
    R = d.cam_xmat[cam].reshape(3, 3)
    dw = R @ dc
    return d.cam_xpos[cam].copy(), dw / np.linalg.norm(dw)


def project(m, d, cam, p, W, H):
    f = (H / 2) / np.tan(np.radians(m.cam_fovy[cam]) / 2)
    R = d.cam_xmat[cam].reshape(3, 3)
    pc = R.T @ (p - d.cam_xpos[cam])
    return W / 2 + f * pc[0] / -pc[2] - 0.5, H / 2 - f * pc[1] / -pc[2] - 0.5


def triangulate(o1, d1, o2, d2):
    w = o1 - o2
    a, b, cc, dd, e = d1 @ d1, d1 @ d2, d2 @ d2, d1 @ w, d2 @ w
    den = a * cc - b * b
    s = (b * e - cc * dd) / den
    t = (a * e - b * dd) / den
    return 0.5 * (o1 + s * d1 + o2 + t * d2)


def detect(img, uv_pred, gate):
    import cv2
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = (((hsv[..., 0] < 8) | (hsv[..., 0] > 172)) & (hsv[..., 1] > 140) & (hsv[..., 2] > 50)).astype(np.uint8)
    H, W = mask.shape
    u0, v0 = int(round(uv_pred[0])), int(round(uv_pred[1]))
    x0, x1, y0, y1 = max(0, u0 - gate), min(W, u0 + gate + 1), max(0, v0 - gate), min(H, v0 + gate + 1)
    sub = mask[y0:y1, x0:x1]
    n, lab, stats, cent = cv2.connectedComponentsWithStats(sub, connectivity=8)
    if n < 2:
        return None, 0
    k = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    return (cent[k][0] + x0, cent[k][1] + y0), int(stats[k, cv2.CC_STAT_AREA])


def run_detect(c):
    import mujoco
    import catch_study as C
    pc = c["perception"]
    cams = pc["cameras"]
    m, scene, phys = V.build_model(cams)
    d = mujoco.MjData(m)
    poses = C.yaml.safe_load(open(os.path.join(V.REPO, "src/catch_description/config/poses.yaml")))
    cube = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube_red")
    ja, da = m.jnt_qposadr[m.body_jntadr[cube]], m.jnt_dofadr[m.body_jntadr[cube]]
    cam_ids = {n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, n) for n in cams}
    p_r, v_r, T = nominal_throw()
    rng = np.random.default_rng(3)
    res = {}
    for W, H in pc["render_sizes"]:
        m.vis.global_.offwidth, m.vis.global_.offheight = max(W, 640), max(H, 480)
        ren = mujoco.Renderer(m, H, W)
        errs, single, miss, areas, e2e = [], [], 0, [], []
        for run in range(pc["detection_runs"]):
            seq = []
            mujoco.mj_resetData(m, d)
            for k in "ab":
                for j, n in enumerate(V.JN):
                    d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{k}_{n}")]] = poses[k]["home"][j]
            dv = rng.normal(0, 0.05, 3)
            d.qpos[ja:ja + 3] = p_r + rng.normal(0, 0.01, 3)
            q = rng.normal(size=4)
            d.qpos[ja + 3:ja + 7] = q / np.linalg.norm(q)
            d.qvel[da:da + 3] = v_r + dv
            d.qvel[da + 3:da + 6] = rng.normal(0, 3.0, 3)       # tumbling cube: worst case for the centroid
            mujoco.mj_forward(m, d)
            nxt = pc["first_visible_s"]
            while d.time < T - 0.03:
                mujoco.mj_step(m, d)
                if d.time + 1e-9 < nxt:
                    continue
                nxt += 1 / 120
                gt = d.xpos[cube].copy()
                rays = []
                for name, cid in cam_ids.items():
                    ren.update_scene(d, camera=name)
                    img = ren.render()
                    uvp = project(m, d, cid, gt + rng.normal(0, 0.01, 3), W, H)
                    uv, area = detect(img, uvp, gate=int(0.06 * W))
                    if uv is None:
                        break
                    areas.append(area)
                    rays.append(cam_ray(m, d, cid, uv[0], uv[1], W, H))
                    if name == "side_front":
                        # single camera, depth from the apparent size (known 40 mm cube)
                        f = (H / 2) / np.tan(np.radians(m.cam_fovy[cid]) / 2)
                        single.append((rays[-1], np.sqrt(area), np.linalg.norm(gt - d.cam_xpos[cid]), gt, f))
                if len(rays) < 2:
                    miss += 1
                    continue
                p = triangulate(*rays[0], *rays[1])
                errs.append(p - gt)
                seq.append((d.time, p))
            # end to end: LSQ on these detections (120 Hz, latency 25 ms) vs the true position at the intercept time
            pT = d.qpos[ja:ja + 3].copy()
            while d.time < T:
                mujoco.mj_step(m, d)
            pT = d.xpos[cube].copy()
            tk = np.array([s_[0] for s_ in seq])
            pk = np.array([s_[1] for s_ in seq])
            row = []
            for t_now in (0.15, 0.20, 0.25, 0.30):
                sel = tk <= t_now - 0.025
                p0, v0 = lsq_fit(tk[sel], pk[sel])
                row.append(1000 * np.linalg.norm(p0 + v0 * T + 0.5 * V.GVEC * T * T - pT))
            e2e.append(row)
        ren.close()
        e = np.array(errs) * 1000
        # single camera: depth = k f / sqrt(area), k fitted (median) on the same data -> best case
        ks = [dist * sa / f for (_, sa, dist, _, f) in single]
        kk = float(np.median(ks))
        es = []
        for (o, dvec), sa, dist, gt, f in single:
            es.append((o + dvec * kk * f / sa - gt) * 1000)
        es = np.array(es)
        res[f"{W}x{H}"] = dict(n=len(e), missed=miss, bias_mm=e.mean(0), std_mm=e.std(0),
                               e95_3d_mm=float(np.percentile(np.linalg.norm(e - e.mean(0), axis=1), 95)),
                               blob_area_px=[float(np.min(areas)), float(np.median(areas)), float(np.max(areas))],
                               single_cam_std_mm=es.std(0), end_to_end_t=[0.15, 0.20, 0.25, 0.30],
                               end_to_end_e95_mm=np.percentile(np.array(e2e), 95, axis=0),
                               end_to_end_median_mm=np.median(np.array(e2e), axis=0), single_cam_e95_3d_mm=float(np.percentile(np.linalg.norm(es, axis=1), 95)))
        r = res[f"{W}x{H}"]
        print(f"{W}x{H}: {r['n']} stereo fixes ({miss} missed), bias {np.round(r['bias_mm'], 2)} mm, std {np.round(r['std_mm'], 2)} mm, "
              f"95% 3D (bias removed) {r['e95_3d_mm']:.2f} mm; single camera + size std {np.round(r['single_cam_std_mm'], 1)} mm; "
              f"end-to-end LSQ 95% at t=0.15/0.20/0.25/0.30 s: {np.round(r['end_to_end_e95_mm'], 1)} mm")
    V.save_json("perception_detection.json", dict(cameras=cams, results=res, rate_hz=120, latency_s=0.025, note="tumbling red cube, HSV segmentation, "
                 "silhouette centroids of side_front + top triangulated; single camera: depth from apparent size"))


# ---------------------------------------------------------------------------------------------- bench -------
def _physics_proc(seconds, q):
    import mujoco
    m, _, _ = V.build_model()
    d = mujoco.MjData(m)
    for _ in range(200):
        mujoco.mj_step(m, d)
    n, t0 = 0, time.perf_counter()
    while time.perf_counter() - t0 < seconds:
        for _ in range(100):
            mujoco.mj_step(m, d)
        n += 100
    wall = time.perf_counter() - t0
    q.put(dict(steps=n, wall=wall, step_us=wall / n * 1e6, max_rtf=n * m.opt.timestep / wall))


def _render_proc(seconds, size, cams, rate, q, ready):
    import mujoco
    W, H = size
    allc = dict(V.cfg()["perception"]["cameras"])
    m, _, _ = V.build_model(allc, size=size)
    m.vis.global_.offwidth, m.vis.global_.offheight = max(W, 640), max(H, 480)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    ctx = mujoco.GLContext(m.vis.global_.offwidth, m.vis.global_.offheight)
    ctx.make_current()
    con = mujoco.MjrContext(m, mujoco.mjtFontScale.mjFONTSCALE_150)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, con)
    scn = mujoco.MjvScene(m, maxgeom=10000)
    opt = mujoco.MjvOption()
    vp = mujoco.MjrRect(0, 0, W, H)
    rgb = np.zeros((H, W, 3), np.uint8)
    dep = np.zeros((H, W), np.float32)
    camobjs = []
    for name in cams:
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        cam.fixedcamid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, name)
        camobjs.append(cam)

    def frame():
        for cam in camobjs:
            mujoco.mjv_updateScene(m, d, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
            mujoco.mjr_render(vp, scn, con)
            mujoco.mjr_readPixels(rgb, dep, vp, con)
            _ = 0.01 / (1.0 - dep * 0.999)            # linearise depth like the plugin (CPU)

    for _ in range(20):
        frame()
    ready.set()
    times, t0 = [], time.perf_counter()
    nxt = t0
    while time.perf_counter() - t0 < seconds:
        if rate:
            now = time.perf_counter()
            if now < nxt:
                time.sleep(nxt - now)
            nxt += 1.0 / rate
        a = time.perf_counter()
        frame()
        times.append(time.perf_counter() - a)
    wall = time.perf_counter() - t0
    con.free()
    ctx.free()
    q.put(dict(fps=len(times) / wall, render_ms=1000 * float(np.mean(times)), render_ms_p95=1000 * float(np.percentile(times, 95))))


def run_bench(c, seconds=6.0):
    ctx = mp.get_context("spawn")
    q = ctx.Queue()
    out = dict(gpu="see docs/m0_toolchain.md (RTX 5060 Laptop, EGL)", rows=[])
    p = ctx.Process(target=_physics_proc, args=(seconds, q))
    p.start()
    p.join()
    base = q.get()
    out["physics_alone"] = base
    print(f"physics alone: {base['step_us']:.1f} us/step, max RTF {base['max_rtf']:.1f}")
    allcams = list(c["perception"]["cameras"])
    for W, H in c["perception"]["render_sizes"]:
        for ncam in (1, 2):
            cams = allcams[:ncam]
            ready = ctx.Event()
            r = ctx.Process(target=_render_proc, args=(3.0, (W, H), cams, 0, q, ready))
            r.start()
            r.join()
            alone = q.get()
            for rate in c["perception"]["rates_hz"]:
                ready = ctx.Event()
                r = ctx.Process(target=_render_proc, args=(seconds, (W, H), cams, rate, q, ready))
                r.start()
                ready.wait()
                p = ctx.Process(target=_physics_proc, args=(seconds - 1.0, q))
                p.start()
                p.join()
                r.join()
                a, b = q.get(), q.get()
                phys, rend = (a, b) if "steps" in a else (b, a)
                row = dict(size=f"{W}x{H}", ncam=ncam, rate=rate, render_ms_alone=alone["render_ms"],
                           max_fps_alone=1000 / alone["render_ms"], fps=rend["fps"], render_ms=rend["render_ms"],
                           render_ms_p95=rend["render_ms_p95"], physics_step_us=phys["step_us"],
                           max_rtf_concurrent=phys["max_rtf"],
                           serial_rtf=1.0 / (base["step_us"] * 1e-6 * 1000 + rate * rend["render_ms"] / 1000))
                out["rows"].append(row)
                print(f"{W}x{H} x{ncam} @ {rate:3d} Hz: render {alone['render_ms']:.2f} ms alone ({1000 / alone['render_ms']:.0f} fps max), "
                      f"achieved {rend['fps']:.0f} fps, physics max RTF {phys['max_rtf']:.1f} (alone {base['max_rtf']:.1f}), "
                      f"same-thread RTF bound {row['serial_rtf']:.2f}")
    V.save_json("perception_render_bench.json", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lsq", action="store_true")
    ap.add_argument("--detect", action="store_true")
    ap.add_argument("--bench", action="store_true")
    a = ap.parse_args()
    c = V.cfg()
    if a.lsq:
        run_lsq(c)
    if a.detect:
        run_detect(c)
    if a.bench:
        run_bench(c)


if __name__ == "__main__":
    main()
