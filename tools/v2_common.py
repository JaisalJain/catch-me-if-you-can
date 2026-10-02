#!/usr/bin/env python3
"""Shared helpers for the v2 Phase 1 offline study (tools/v2_*.py; config tools/v2_study.yaml).

Reuses the M1/M5 tooling: catch_kinematics (UR5e FK/IK), catch_ballistics, catch_throw, tools/feasibility.py
(MuJoCo inverse dynamics of one UR5e + 2F-85 + 60 g cube) and the generated scene (build_mjcf).
"""
import json
import os
import sys

os.environ.setdefault("MUJOCO_GL", "egl")       # offscreen on the NVIDIA GPU (docs/m0_toolchain.md)
os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
for _p in ("src/catch_kinematics", "src/catch_ballistics", "src/catch_throw", "src/catch_description/scripts", "tools"):
    sys.path.insert(0, os.path.join(REPO, _p))
import build_mjcf  # noqa: E402
import feasibility as F  # noqa: E402
from catch_kinematics import POS_LIMIT, TORQUE_LIMIT, VEL_LIMIT  # noqa: E402
from catch_kinematics.transforms import make_tf  # noqa: E402
from catch_throw.generator import toss_orientation  # noqa: E402

OUT = os.path.join(REPO, "docs", "v2")
JN = F.JN
G = 9.81
GVEC = np.array([0.0, 0.0, -G])
# measured in the generated scene (tools/v2_perception.py --gripper): pad gap [mm] vs knuckle angle [rad]
GAP_TABLE = np.array([[0.0, 85.2], [0.1, 75.7], [0.2, 65.5], [0.3, 54.8], [0.4, 43.7], [0.5, 32.2], [0.6, 20.5],
                      [0.7, 8.6], [0.782, 0.0]])


def cfg():
    return yaml.safe_load(open(os.path.join(REPO, "tools", "v2_study.yaml")))


def gripper_profile(c=None, name=None):
    c = c or cfg()
    name = name or c["gripper_profile"]
    if name not in c["gripper_profiles"]:
        raise SystemExit(f"gripper_profile must be one of {list(c['gripper_profiles'])}, got {name}")
    return name, dict(c["gripper_profiles"][name])


def knuckle_for_gap(gap_mm):
    return float(np.interp(-gap_mm, -GAP_TABLE[:, 1], GAP_TABLE[:, 0]))


def gap_for_knuckle(k):
    return float(np.interp(k, GAP_TABLE[:, 0], GAP_TABLE[:, 1]))


def save_json(name, obj):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    with open(path, "w") as f:
        json.dump(obj, f, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else float(o))
    return path


# ---------------------------------------------------------------- model --------------------------------------
def look_at_quat(pos, target):
    """MuJoCo camera quaternion: camera looks along -z, y up."""
    fwd = np.subtract(target, pos)
    fwd = fwd / np.linalg.norm(fwd)
    up = [0, 0, 1.0] if abs(fwd[2]) < 0.99 else [0, 1.0, 0]     # straight down: image up = world +y
    x = np.cross(fwd, up)
    x /= np.linalg.norm(x)
    y = np.cross(x, fwd)
    R = np.column_stack([x, y, -fwd])
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.flatten())
    return q


def build_model(cameras=None, profile=None, size=(640, 480)):
    """The scene the ROS sim loads (compiled in memory) + optional study cameras {name: {pos, target, fovy}},
    and B's gripper actuator scaled for a gripper profile dict (kp_scale, force_scale)."""
    scene, phys = build_mjcf.load_cfg()
    spec = build_mjcf.build(scene, phys)
    for name, c in (cameras or {}).items():
        spec.worldbody.add_camera(name=name, pos=c["pos"], quat=look_at_quat(c["pos"], c["target"]), fovy=c["fovy"],
                                  resolution=list(size))
    assets = os.path.join(REPO, "src/catch_description/mjcf/assets")
    for m in spec.meshes:
        m.file = os.path.join(assets, m.file)
    m = spec.compile()
    if profile:
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "b_robotiq_85_left_knuckle_joint")
        # absolute values relative to the v1 actuator (kp 100, kv 10, force +-5), independent of physics.yaml's
        # gripper section (the ROS sim's actuator, v2: kp 1000 = the realistic profile)
        m.actuator_gainprm[a, 0] = 100.0 * profile["kp_scale"]
        m.actuator_biasprm[a, 1] = -100.0 * profile["kp_scale"]
        m.actuator_biasprm[a, 2] = -10.0 * profile.get("kv_scale", profile["kp_scale"])
        m.actuator_forcerange[a] = [-5.0 * profile["force_scale"], 5.0 * profile["force_scale"]]
    return m, scene, phys


class Dyn:
    """Inverse dynamics of one UR5e + 2F-85 + 60 g cube at the TCP (tools/feasibility.py). gravity=True: total
    joint torque (limit check; the sim's gravity compensation runs through the actuators and counts against the
    limits); gravity=False: feed-forward on top of the sim's gravity compensation (like throw_ff.yaml)."""

    def __init__(self, scene, phys, gravity=True):
        self.d = F.Dynamics(scene, phys)
        if not gravity:
            self.d.m.opt.gravity[:] = 0.0

    def tau(self, q, qd, qdd):
        return self.d.torques(np.atleast_2d(q), np.atleast_2d(qd), np.atleast_2d(qdd))

    def tau1(self, q, qd, qdd):
        dd = self.d
        dd.d.qpos[:] = 0
        dd.d.qvel[:] = 0
        dd.d.qacc[:] = 0
        dd.d.qpos[dd.qadr] = q
        dd.d.qvel[dd.dofs] = qd
        dd.d.qacc[dd.dofs] = qdd
        mujoco.mj_inverse(dd.m, dd.d)
        return dd.d.qfrc_inverse[dd.dofs].copy()


# ------------------------------------------------------------ trajectories -----------------------------------
class Quintic:
    """Per-joint quintic from (q0, v0, a0) at t=0 to (q1, v1, a1) at t=T."""

    def __init__(self, q0, v0, a0, q1, v1, a1, T):
        q0 = np.asarray(q0, float)
        v0, a0, q1, v1, a1 = (np.broadcast_to(np.asarray(x, float), q0.shape) for x in (v0, a0, q1, v1, a1))
        self.T = float(T)
        T = self.T
        h = q1 - q0 - v0 * T - 0.5 * a0 * T * T
        dv = v1 - v0 - a0 * T
        da = a1 - a0
        c3 = (10 * h - 4 * dv * T + 0.5 * da * T * T) / T ** 3
        c4 = (-15 * h + 7 * dv * T - da * T * T) / T ** 4
        c5 = (6 * h - 3 * dv * T + 0.5 * da * T * T) / T ** 5
        self.c = np.array([q0, v0, 0.5 * a0, c3, c4, c5])

    def __call__(self, t):
        t = float(np.clip(t, 0.0, self.T))
        c = self.c
        q = c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))))
        qd = c[1] + t * (2 * c[2] + t * (3 * c[3] + t * (4 * c[4] + t * 5 * c[5])))
        qdd = 2 * c[2] + t * (6 * c[3] + t * (12 * c[4] + t * 20 * c[5]))
        return q, qd, qdd


class Piecewise:
    """Sequence of (t_start, Quintic); holds the last state afterwards."""

    def __init__(self, segs):
        self.segs = segs

    def __call__(self, t):
        for t0, s in reversed(self.segs):
            if t >= t0:
                if t - t0 > s.T:
                    q, _, _ = s(s.T)
                    return q, np.zeros_like(q), np.zeros_like(q)
                return s(t - t0)
        return self.segs[0][1](0.0)

    @property
    def t_end(self):
        return self.segs[-1][0] + self.segs[-1][1].T


def check_traj(traj, t0, t1, dyn, robot=None, n=30, table_z=None):
    """Ratios against the limits along a trajectory: vel (of 180 deg/s), torque (of the UR5e torques, gravity
    included), min position margin, min TCP height."""
    ts = np.linspace(t0, t1, n)
    S = [traj(t) for t in ts]
    q = np.array([s[0] for s in S])
    qd = np.array([s[1] for s in S])
    qdd = np.array([s[2] for s in S])
    tau = dyn.tau(q, qd, qdd)
    out = dict(vel=float(np.max(np.abs(qd) / VEL_LIMIT)), torque=float(np.max(np.abs(tau) / TORQUE_LIMIT)),
               pos_margin=float(np.min(POS_LIMIT - np.abs(q))))
    if robot is not None:
        out["tcp_z_min"] = float(min(robot.fk(qq)[2, 3] for qq in q[::3]))
    return out


def limits_ok(chk, lim):
    return (chk["vel"] <= lim["vel_ratio"] and chk["torque"] <= lim["torque_ratio"]
            and chk["pos_margin"] >= lim["pos_margin"])


def limit_ratio(chk, lim):
    """>1 means a limit (with margin) is exceeded."""
    r = max(chk["vel"] / lim["vel_ratio"], chk["torque"] / lim["torque_ratio"])
    if chk["pos_margin"] < lim["pos_margin"]:
        r = max(r, 1.0 + (lim["pos_margin"] - chk["pos_margin"]))
    return r


# ------------------------------------------------------------- orientation -----------------------------------
def ready_rotation(elev_deg):
    """Approach axis (tool z) towards A (-x), elev_deg above horizontal; closing axis (tool x) along world y."""
    e = np.radians(elev_deg)
    return toss_orientation([-np.cos(e), 0.0, np.sin(e)], [0.0, 1.0, 0.0])


def closing_axis_for(v):
    """Horizontal axis across the cube's velocity (= normal of the throw plane): the pads squeeze the cube faces
    that A's fingers held (spin from the throw is about this axis)."""
    a = np.cross([0, 0, 1.0], v)
    if np.linalg.norm(a) < 1e-6:
        return np.array([0.0, 1.0, 0.0])
    a /= np.linalg.norm(a)
    return a if a[1] >= 0 else -a


def rotate_towards(z0, z1, max_angle):
    """Unit vector from z0 towards z1 by at most max_angle (rad)."""
    z0, z1 = np.asarray(z0, float) / np.linalg.norm(z0), np.asarray(z1, float) / np.linalg.norm(z1)
    ang = np.arccos(np.clip(z0 @ z1, -1, 1))
    if ang <= max_angle or ang < 1e-9:
        return z1, float(ang)
    ax = np.cross(z0, z1)
    ax /= np.linalg.norm(ax)
    a = max_angle
    z = z0 * np.cos(a) + np.cross(ax, z0) * np.sin(a) + ax * (ax @ z0) * (1 - np.cos(a))
    return z / np.linalg.norm(z), float(ang)


def catch_rotation(z_ready, v, cone_rad, mode):
    """cup: palm normal (tool z) = -v limited to a cone around the ready approach axis.
    pinch: approach axis = the ready approach axis made orthogonal to the closing axis (natural pose; with an
    elevated ready pose the cube then flies towards the palm). pinch_side: approach axis also orthogonal to v.
    The closing axis is across v in all modes."""
    xa = closing_axis_for(v)
    if mode == "cup":
        z, ang = rotate_towards(z_ready, -np.asarray(v) / np.linalg.norm(v), cone_rad)
    elif mode == "pinch_side":
        # chopsticks from the side: approach axis across the velocity too (the cube passes between the pads
        # across the finger width, the palm is not in its way)
        vn = np.asarray(v, float) / np.linalg.norm(v)
        z = np.asarray(z_ready, float) - (np.asarray(z_ready) @ xa) * xa
        z = z - (z @ vn) * vn
        z, ang = z / np.linalg.norm(z), 0.0
    else:
        z = np.asarray(z_ready, float) - (np.asarray(z_ready) @ xa) * xa
        z, ang = z / np.linalg.norm(z), 0.0
    return toss_orientation(z, xa), ang


def ik_near(robot, R, p, q_ref, max_jump=None):
    q = robot.ik_closest(make_tf(R, p), q_ref)
    if q is None:
        return None
    if max_jump is not None and np.max(np.abs(q - q_ref)) > max_jump:
        return None
    return q


def flags(c=None, profile=None):
    """Header stored in every output file."""
    c = c or cfg()
    name, prof = gripper_profile(c, profile)
    return dict(gripper_profile=name, gripper=prof, limits=c["limits"],
                note="offline v2 Phase 1 study (tools/v2_*.py); fast_idealized is NOT a real 2F-85")
