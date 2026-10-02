"""UR5e (+ lumped 2F-85) inverse dynamics in numpy: recursive Newton-Euler on the MuJoCo-derived parameters
(config/ur5e_dynamics.yaml, tools/export_dynamics.py). Equals MuJoCo's mj_inverse for the same model (test).

Used by the ROS nodes (no MuJoCo in system python): torque-limit checks of replanned trajectories and
feed-forward torques (gravity=False: the sim compensates gravity through the actuators; the limit check uses
gravity=True because that compensation counts against the motor limits).
"""
from __future__ import annotations

import os

import numpy as np
import yaml


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def _axis_rot(axis, a):
    k = np.asarray(axis, float)
    K = _skew(k)
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)


class ArmDynamics:
    def __init__(self, path=None):
        if path is None:
            path = os.path.join(os.path.dirname(__file__), "..", "config", "ur5e_dynamics.yaml")
            if not os.path.exists(path):
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(get_package_share_directory("catch_kinematics"), "config", "ur5e_dynamics.yaml")
        d = yaml.safe_load(open(path))
        self.g = np.array(d["gravity"], float)
        J = d["joints"]
        self.Rp = [np.array(j["R_parent"]) for j in J]
        self.pp = [np.array(j["p_parent"]) for j in J]
        self.ax = [np.array(j["axis"], float) for j in J]
        self.m = np.array([j["mass"] for j in J])
        self.c = [np.array(j["com"]) for j in J]
        self.I = [np.array(j["inertia"]) for j in J]          # about the COM, link frame
        self.arm = np.array([j["armature"] for j in J])

    def rnea(self, q, qd, qdd, gravity=True):
        """Joint torques for one state (arrays of 6)."""
        n = 6
        R = [None] * n          # R[i]: parent -> child rotation (child frame expressed in parent)
        w, wd, a = np.zeros(3), np.zeros(3), (-self.g if gravity else np.zeros(3))   # base: fictitious up accel
        ws, wds, acs, Rs = [], [], [], []
        for i in range(n):
            Ri = self.Rp[i] @ _axis_rot(self.ax[i], q[i])     # child in parent
            RT = Ri.T
            p = self.pp[i]
            # linear acceleration of the child origin (in the parent frame), then to the child frame
            a_o = a + np.cross(wd, p) + np.cross(w, np.cross(w, p))
            z = self.ax[i]
            w_c = RT @ w + z * qd[i]
            wd_c = RT @ wd + z * qdd[i] + np.cross(RT @ w, z * qd[i])
            a_c = RT @ a_o
            w, wd, a = w_c, wd_c, a_c
            ws.append(w)
            wds.append(wd)
            acs.append(a)
            Rs.append(Ri)
        tau = np.zeros(n)
        f_next, m_next = np.zeros(3), np.zeros(3)
        for i in reversed(range(n)):
            c, I, m = self.c[i], self.I[i], self.m[i]
            w, wd, a = ws[i], wds[i], acs[i]
            a_c = a + np.cross(wd, c) + np.cross(w, np.cross(w, c))
            F = m * a_c
            N = I @ wd + np.cross(w, I @ w)
            if i + 1 < n:
                R1 = Rs[i + 1]
                f_ch = R1 @ f_next
                m_ch = R1 @ m_next + np.cross(self.pp[i + 1], f_ch)
            else:
                f_ch, m_ch = np.zeros(3), np.zeros(3)
            f = F + f_ch
            mo = N + np.cross(c, F) + m_ch
            tau[i] = mo @ self.ax[i] + self.arm[i] * qdd[i]
            f_next, m_next = f, mo
        return tau

    def torques(self, Q, QD, QDD, gravity=True):
        return np.array([self.rnea(q, qd, qdd, gravity) for q, qd, qdd in zip(Q, QD, QDD)])
