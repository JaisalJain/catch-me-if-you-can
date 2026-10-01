#!/usr/bin/env python3
"""Demo video of a full run (M6): replay of the recorded sim state (tools/render_replay.py) with the state
machine's log as captions. Overview at `--fast` speed between throws and real time around each throw, each catch
followed by a 0.25x close-up.

    env -u PYTHONPATH .venv/bin/python tools/render_demo.py log/m6/run_seed1.npz log/m6/run_seed1.json \
        --out docs/media/demo_seed1.mp4 [--fast 2.0] [--size 1280x720]
"""
import argparse
import json
import os
import sys

import cv2
import imageio.v2 as imageio
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_replay import Replay  # noqa: E402


def caption(img, lines, y0=10):
    for k, txt in enumerate(lines):
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        y = y0 + k * (th + 12)
        img[y:y + th + 10, 8:20 + tw] = (img[y:y + th + 10, 8:20 + tw] * 0.3).astype(np.uint8)
        cv2.putText(img, txt, (14, y + th + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)


def contact_time(rec, t0, t1):
    """(time, True) of the first palm-force spike (cube hits B's palm) in [t0, t1]; else (t1 - 0.8 s, False)."""
    ft = rec["ft"]
    sel = (ft[:, 0] >= t0) & (ft[:, 0] <= t1)
    if sel.sum() > 60:
        f = np.linalg.norm(ft[sel, 1:], axis=1)
        hit = np.where(np.abs(f - np.median(f[:50])) > 8.0)[0]
        if len(hit):
            return float(ft[sel, 0][hit[0]]), True
    return t1 - 0.8, False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("events")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fast", type=float, default=2.0)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--size", default="1280x720")
    a = ap.parse_args()
    w, h = [int(v) for v in a.size.split("x")]
    rec = dict(np.load(a.rec, allow_pickle=True))
    run = json.load(open(a.events))
    ev = run["events"]
    seed = run["summary"]["seed"]
    t_first = max(float(rec["t_js"][0]), ev[0]["t_sim"] - 1.0)          # the run, not the whole recording
    t_last = min(float(rec["t_js"][-1]), ev[-1]["t_sim"] + 2.0)
    # throw windows: from "B in its cup pose" (preposition done) to 1.2 s after the catch result
    throws = [e for e in ev if e["state"] == "throw" and "caught" in e]
    pre = [e for e in ev if e["state"] == "preposition" and "cup pose" in e["msg"]]
    windows = []
    for e in throws:
        p = max([x for x in pre if x["t_sim"] <= e["t_sim"]], key=lambda x: x["t_sim"])
        windows.append((p["t_sim"] - 0.2, e["t_sim"] + 0.3, e))
    main_rp = Replay(rec, w, h)
    wr = imageio.get_writer(a.out, fps=a.fps, codec="libx264", quality=8, macro_block_size=16, pixelformat="yuv420p")
    sorted_ = []

    def state_text(t):
        past = [e for e in ev if e["t_sim"] <= t]
        return past[-1] if past else None

    def frame(rp, t, view, speed, extra="", note=None):
        img = np.ascontiguousarray(rp.render(view))
        e = state_text(t)
        for x in ev:
            if x["t_sim"] <= t and "sorted" in x and x["sorted"] not in sorted_:
                sorted_.append(x["sorted"])
        head = f"catch demo  seed {seed}  t={t - t_first:6.1f} s  ({'real time' if speed == 1 else f'{speed:g}x'})"
        lines = [head + extra, f"sorted on B: {', '.join(sorted_) or '-'}"]
        if note:
            lines.append(note)
        elif e:
            lines.append(f"[{e['state']}] {e['msg'][:110]}")
        caption(img, lines)
        return img

    t, t_prev = t_first, t_first
    k = 0
    n = 0
    while t < t_last:
        inwin = any(w0 <= t <= w1 for w0, w1, _ in windows)
        speed = 1.0 if inwin else a.fast
        main_rp.advance(t_prev, t)
        wr.append_data(frame(main_rp, t, "overview", speed))
        n += 1
        t_prev = t
        t += speed / a.fps
        # after a throw window: slow-motion close-up of that catch
        if k < len(windows) and t > windows[k][1]:
            w0, w1, e = windows[k]
            tc, contact = contact_time(rec, e["t_sim"] - 3.0, e["t_sim"])
            cr = Replay(rec, w, h)
            ts = np.arange(tc - 0.45, tc + 0.6, 0.25 / a.fps)
            tp = ts[0]
            for tt in ts:
                cr.advance(tp, tt)
                tp = tt
                if not contact:
                    note = "[catch] no palm contact: the cube misses B's cup (fingers stay open)"
                else:
                    note = ("[catch] palm contact -> fingers close" if tt >= tc else "[catch] cube in flight, B's cup open")
                wr.append_data(frame(cr, tt, "catch", 0.25, "  close-up", note))
                n += 1
            k += 1
    wr.close()
    print(f"wrote {a.out}: {n} frames ({n / a.fps:.1f} s)")


if __name__ == "__main__":
    main()
