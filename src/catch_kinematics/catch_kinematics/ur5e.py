"""UR5e kinematics: standard DH forward kinematics, geometric Jacobian, closed-form IK (8 branches).

Frames
  world --(mount)--> <p>base_link --Rz(pi)--> DH base ("base" in ur_description) --DH--> <p>tool0 --> TCP
The DH values are the nominal ones used by ur_description (config/ur5e/default_kinematics.yaml).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .transforms import inv_tf, make_tf, rot_z

D = np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996])
A = np.array([0.0, -0.425, -0.3922, 0.0, 0.0, 0.0])
ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])

JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
               "wrist_3_joint"]
# Published UR5e limits (UR5e user manual / ur_description joint_limits.yaml)
POS_LIMIT = np.array([2 * np.pi, 2 * np.pi, np.pi, 2 * np.pi, 2 * np.pi, 2 * np.pi])
VEL_LIMIT = np.full(6, np.pi)                       # 180 deg/s
TORQUE_LIMIT = np.array([150.0, 150.0, 150.0, 28.0, 28.0, 28.0])

BASELINK_T_DHBASE = make_tf(rot_z(np.pi))


def _dh(theta, d, a, alpha):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def wrap(q):
    return (np.asarray(q) + np.pi) % (2 * np.pi) - np.pi


@dataclass
class UR5e:
    """world_T_base: pose of <p>base_link in the world. tcp: pose of the TCP in tool0."""

    world_T_base: np.ndarray = field(default_factory=lambda: np.eye(4))
    tool0_T_tcp: np.ndarray = field(default_factory=lambda: np.eye(4))

    def __post_init__(self):
        self.world_T_dh = self.world_T_base @ BASELINK_T_DHBASE
        self.dh_T_world = inv_tf(self.world_T_dh)
        self.tcp_T_tool0 = inv_tf(self.tool0_T_tcp)

    # --- forward kinematics -------------------------------------------------------------------
    def frames(self, q):
        """World poses of DH frames 0..6 (frame 6 = tool0)."""
        T = self.world_T_dh.copy()
        out = [T]
        for i in range(6):
            T = T @ _dh(q[i], D[i], A[i], ALPHA[i])
            out.append(T)
        return out

    def fk_tool0(self, q):
        return self.frames(q)[-1]

    def fk(self, q):
        """World pose of the TCP."""
        return self.fk_tool0(q) @ self.tool0_T_tcp

    def jacobian(self, q):
        """6x6 geometric Jacobian of the TCP in the world frame, rows [v; w]."""
        fr = self.frames(q)
        p_e = (fr[-1] @ self.tool0_T_tcp)[:3, 3]
        J = np.zeros((6, 6))
        for i in range(6):
            z, p = fr[i][:3, 2], fr[i][:3, 3]
            J[:3, i] = np.cross(z, p_e - p)
            J[3:, i] = z
        return J

    # --- inverse kinematics -------------------------------------------------------------------
    def ik_all(self, world_T_tcp):
        """All (up to 8) closed-form solutions for a TCP pose, each wrapped to [-pi, pi)."""
        T = self.dh_T_world @ world_T_tcp @ self.tcp_T_tool0
        d1, d4, d5, d6 = D[0], D[3], D[4], D[5]
        a2, a3 = A[1], A[2]
        sols = []
        p05 = T @ np.array([0, 0, -d6, 1.0])
        r = np.hypot(p05[0], p05[1])
        if r < abs(d4):
            return []
        phi1 = np.arctan2(p05[1], p05[0])
        phi2 = np.arccos(np.clip(d4 / r, -1, 1))
        for s1 in (1, -1):
            t1 = phi1 + s1 * phi2 + np.pi / 2
            c1, sn1 = np.cos(t1), np.sin(t1)
            c5 = (T[0, 3] * sn1 - T[1, 3] * c1 - d4) / d6
            if abs(c5) > 1 + 1e-9:
                continue
            for s5 in (1, -1):
                t5 = s5 * np.arccos(np.clip(c5, -1, 1))
                sn5 = np.sin(t5)
                if abs(sn5) < 1e-9:
                    t6 = 0.0  # wrist singularity: t6 arbitrary
                else:
                    Ti = inv_tf(T)
                    t6 = np.arctan2((-Ti[1, 0] * sn1 + Ti[1, 1] * c1) / sn5, (Ti[0, 0] * sn1 - Ti[0, 1] * c1) / sn5)
                T01 = _dh(t1, d1, 0, ALPHA[0])
                T45 = _dh(t5, d5, 0, ALPHA[4])
                T56 = _dh(t6, d6, 0, ALPHA[5])
                T14 = inv_tf(T01) @ T @ inv_tf(T45 @ T56)
                p13 = T14 @ np.array([0, -d4, 0, 1.0])
                px, py = p13[0], p13[1]
                n2 = px * px + py * py
                c3 = (n2 - a2 * a2 - a3 * a3) / (2 * a2 * a3)
                if abs(c3) > 1 + 1e-9:
                    continue
                for s3 in (1, -1):
                    t3 = s3 * np.arccos(np.clip(c3, -1, 1))
                    t2 = -np.arctan2(py, -px) + np.arcsin(np.clip(a3 * np.sin(t3) / np.sqrt(n2), -1, 1))
                    T13 = _dh(t2, 0, a2, 0) @ _dh(t3, 0, a3, 0)
                    T34 = inv_tf(T13) @ T14
                    t4 = np.arctan2(T34[1, 0], T34[0, 0])
                    sols.append(wrap(np.array([t1, t2, t3, t4, t5, t6])))
        return sols

    def ik_closest(self, world_T_tcp, q_ref, check=True):
        """Solution closest to q_ref (each joint unwrapped to the 2*pi copy nearest q_ref)."""
        best, best_d = None, np.inf
        q_ref = np.asarray(q_ref, float)
        for s in self.ik_all(world_T_tcp):
            s = q_ref + wrap(s - q_ref)
            if check and (np.any(np.abs(s) > POS_LIMIT)):
                continue
            dist = np.linalg.norm(s - q_ref)
            if dist < best_d:
                best, best_d = s, dist
        return best
