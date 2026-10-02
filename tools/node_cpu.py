#!/usr/bin/env python3
"""CPU use per running catch Python node (and the sim / move_group for reference), from /proc deltas.

    python3 tools/node_cpu.py [--seconds 30] [--out log/cpu.json]
Reports % of one core (user + system, all threads) over the window. Processes are matched by their command line
(the installed node scripts under install/catch_*), never by this tool's own command line.
"""
import argparse
import json
import os
import time

TICK = os.sysconf("SC_CLK_TCK")


def procs():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        try:
            cmd = open(f"/proc/{pid}/cmdline", "rb").read().split(b"\0")
        except OSError:
            continue
        cmd = [c.decode(errors="replace") for c in cmd if c]
        if not cmd:
            continue
        name = None
        for c in cmd[:2]:
            if "/install/catch_" in c and "/lib/catch_" in c:
                name = os.path.basename(c)
            elif c.endswith("mujoco_ros2_control/ros2_control_node"):
                name = "ros2_control_node (sim, C++)"
            elif c.endswith("moveit_ros_move_group/move_group"):
                name = "move_group (C++)"
        if name:
            out[int(pid)] = name
    return out


def ticks(pid):
    f = open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()
    return int(f[11]) + int(f[12])          # utime + stime (fields 14, 15)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--out")
    a = ap.parse_args()
    p = procs()
    t0, c0 = time.monotonic(), {pid: ticks(pid) for pid in p}
    time.sleep(a.seconds)
    t1 = time.monotonic()
    res = {}
    for pid, name in sorted(p.items(), key=lambda x: x[1]):
        try:
            res[f"{name} [{pid}]"] = round(100.0 * (ticks(pid) - c0[pid]) / TICK / (t1 - t0), 1)
        except OSError:
            pass
    for k, v in res.items():
        print(f"{k:45s} {v:6.1f} %")
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
