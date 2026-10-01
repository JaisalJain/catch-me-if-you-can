"""Minimal URDF forward kinematics (fixed / revolute / continuous / prismatic, incl. mimic), numpy only.

Used to cross-check our DH model and the MJCF against the exact URDF that MoveIt loads."""
from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np

from .transforms import axis_angle, make_tf, rpy_to_matrix


class UrdfChain:
    def __init__(self, urdf_xml: str):
        root = ET.fromstring(urdf_xml)
        self.joints = {}
        self.child_to_joint = {}
        for j in root.findall("joint"):
            origin = j.find("origin")
            xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
            axis_el = j.find("axis")
            axis = [float(v) for v in axis_el.get("xyz").split()] if axis_el is not None else [1.0, 0.0, 0.0]
            mimic = j.find("mimic")
            info = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                        child=j.find("child").get("link"), origin=make_tf(rpy_to_matrix(*rpy), xyz),
                        axis=np.array(axis),
                        mimic=None if mimic is None else (mimic.get("joint"), float(mimic.get("multiplier", 1)),
                                                          float(mimic.get("offset", 0))))
            self.joints[info["name"]] = info
            self.child_to_joint[info["child"]] = info
        self.links = [l.get("name") for l in root.findall("link")]

    def _q(self, joint, q: dict) -> float:
        if joint["mimic"] is not None:
            src, mul, off = joint["mimic"]
            return mul * q.get(src, 0.0) + off
        return q.get(joint["name"], 0.0)

    def link_pose(self, link: str, q: dict, root: str | None = None) -> np.ndarray:
        """4x4 pose of `link` in `root` (default: the tree root)."""
        chain = []
        cur = link
        while cur in self.child_to_joint and cur != root:
            j = self.child_to_joint[cur]
            chain.append(j)
            cur = j["parent"]
        if root is not None and cur != root:
            raise ValueError(f"{root} is not an ancestor of {link}")
        T = np.eye(4)
        for j in reversed(chain):
            T = T @ j["origin"]
            if j["type"] in ("revolute", "continuous"):
                T = T @ make_tf(axis_angle(j["axis"], self._q(j, q)))
            elif j["type"] == "prismatic":
                T = T @ make_tf(p=j["axis"] * self._q(j, q))
        return T
