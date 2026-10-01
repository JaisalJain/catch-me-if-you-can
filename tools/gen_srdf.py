#!/usr/bin/env python3
"""Generate catch_moveit_config/config/catch_scene.srdf: groups, named states, end effectors and the
disable-collisions matrix (the MoveIt Setup Assistant algorithm, run headless):

  * Adjacent : links whose nearest geometric ancestors are joined by one joint
  * Default  : in collision at the home configuration
  * Always   : in collision in every random sample
  * Never    : never in collision over N random samples (both arms random at once)

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    env -u PYTHONPATH .venv/bin/python tools/gen_srdf.py [--samples 20000]
"""
import argparse
import glob
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from itertools import combinations

import fcl
import numpy as np
import trimesh
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO, "src", "catch_kinematics"))
from catch_kinematics import UrdfChain  # noqa: E402
from catch_kinematics.transforms import make_tf, rpy_to_matrix  # noqa: E402

GRIPPER_MAX = 0.70  # never command the 2F-85 beyond this (palm insert clearance, see scene.yaml)
JN = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def ros_env():
    env = dict(os.environ)
    env["PYTHONPATH"] = ":".join(glob.glob(f"/opt/ros/{env.get('ROS_DISTRO', 'jazzy')}/lib/python3*/site-packages"))
    return env


def resolve(uri):
    if uri.startswith("package://"):
        pkg, rel = uri[len("package://"):].split("/", 1)
        share = subprocess.check_output(["ros2", "pkg", "prefix", "--share", pkg], env=ros_env()).decode().strip()
        return os.path.join(share, rel)
    return uri.replace("file://", "")


def collision_objects(urdf_xml):
    """{link: [(fcl geometry, local 4x4)]} from <collision> elements."""
    root = ET.fromstring(urdf_xml)
    out = {}
    for link in root.findall("link"):
        geoms = []
        for col in link.findall("collision"):
            o = col.find("origin")
            T = make_tf(rpy_to_matrix(*[float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]),
                        [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
            g = col.find("geometry")[0]
            if g.tag == "box":
                geoms.append((fcl.Box(*[float(v) for v in g.get("size").split()]), T))
            elif g.tag == "cylinder":
                geoms.append((fcl.Cylinder(float(g.get("radius")), float(g.get("length"))), T))
            elif g.tag == "sphere":
                geoms.append((fcl.Sphere(float(g.get("radius"))), T))
            elif g.tag == "mesh":
                mesh = trimesh.load(resolve(g.get("filename")), force="mesh")
                s = [float(v) for v in g.get("scale", "1 1 1").split()]
                mesh.apply_scale(s)
                m = fcl.BVHModel()
                m.beginModel(len(mesh.vertices), len(mesh.faces))
                m.addSubModel(mesh.vertices, mesh.faces)
                m.endModel()
                geoms.append((m, T))
        if geoms:
            out[link.get("name")] = geoms
    return out


def geometric_adjacency(chain, links_with_geom):
    def geo_parent(link):
        cur = link
        while cur in chain.child_to_joint:
            cur = chain.child_to_joint[cur]["parent"]
            if cur in links_with_geom:
                return cur
        return None
    pairs = set()
    for l in links_with_geom:
        p = geo_parent(l)
        if p is not None:
            pairs.add(tuple(sorted((l, p))))
    return pairs


def static_links(chain, links):
    """Links rigidly attached to the world (no movable joint on their path to the root)."""
    out = set()
    for l in links:
        cur, ok = l, True
        while cur in chain.child_to_joint:
            j = chain.child_to_joint[cur]
            if j["type"] != "fixed":
                ok = False
                break
            cur = j["parent"]
        if ok:
            out.add(l)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=20000)
    args = ap.parse_args()
    xacro = os.path.join(REPO, "src/catch_description/urdf/scene.urdf.xacro")
    urdf = subprocess.check_output(["xacro", xacro, "ros2_control:=false"], env=ros_env()).decode()
    chain = UrdfChain(urdf)
    cols = collision_objects(urdf)
    links = sorted(cols)
    poses = yaml.safe_load(open(os.path.join(REPO, "src/catch_description/config/poses.yaml")))
    objs = {l: [fcl.CollisionObject(g, fcl.Transform()) for g, _ in cols[l]] for l in links}

    def collide_pairs(q):
        cache = {}
        hits = set()
        for l in links:
            T = chain.link_pose(l, q)
            cache[l] = T
            for o, (_, Tl) in zip(objs[l], cols[l]):
                Tw = T @ Tl
                o.setTransform(fcl.Transform(Tw[:3, :3], Tw[:3, 3]))
        for a, b in combinations(links, 2):
            for oa in objs[a]:
                for ob in objs[b]:
                    if fcl.collide(oa, ob, fcl.CollisionRequest(), fcl.CollisionResult()):
                        hits.add((a, b))
                        break
                else:
                    continue
                break
        return hits

    all_pairs = set(combinations(links, 2))
    adjacent = geometric_adjacency(chain, set(links))
    static = static_links(chain, links)
    static_pairs = {tuple(sorted(p)) for p in combinations(sorted(static), 2)}
    home = {f"{k}_{j}": poses[k]["home"][i] for k in ("a", "b") for i, j in enumerate(JN)}
    default = collide_pairs(home)
    rng = np.random.default_rng(0)
    counts = {p: 0 for p in all_pairs}
    gj = ["a_robotiq_85_left_knuckle_joint", "b_robotiq_85_left_knuckle_joint"]
    for s in range(args.samples):
        q = {f"{k}_{j}": rng.uniform(-np.pi, np.pi) for k in ("a", "b") for j in JN}
        q.update({g: rng.uniform(0, GRIPPER_MAX) for g in gj})
        for p in collide_pairs(q):
            counts[p] += 1
        if s % 2000 == 0:
            print(f"sample {s}/{args.samples}", flush=True)
    reasons = {}
    for p in sorted(all_pairs):
        if p in adjacent:
            reasons[p] = "Adjacent"
        elif p in static_pairs:
            reasons[p] = "Adjacent" if p in default else "Never"
        elif p in default:
            reasons[p] = "Default"
        elif counts[p] == args.samples:
            reasons[p] = "Always"
        elif counts[p] == 0:
            reasons[p] = "Never"
    print(f"{len(links)} collision links, {len(all_pairs)} pairs, disabled {len(reasons)} "
          f"(adjacent {sum(r == 'Adjacent' for r in reasons.values())}, default {sum(r == 'Default' for r in reasons.values())}, "
          f"always {sum(r == 'Always' for r in reasons.values())}, never {sum(r == 'Never' for r in reasons.values())}); "
          f"checked {len(all_pairs) - len(reasons)}")
    write_srdf(reasons, poses)


def write_srdf(reasons, poses):
    L = ['<?xml version="1.0" encoding="UTF-8"?>',
         "<!-- GENERATED by tools/gen_srdf.py (groups + collision matrix). Do not edit by hand. -->",
         '<robot name="catch_scene">']
    for k in ("a", "b"):
        L += [f'  <group name="arm_{k}">', f'    <chain base_link="{k}_base_link" tip_link="{k}_tcp"/>', "  </group>",
              f'  <group name="gripper_{k}">', f'    <link name="{k}_robotiq_85_base_link"/>']
        for l in ("left_knuckle", "right_knuckle", "left_finger", "right_finger", "left_inner_knuckle",
                  "right_inner_knuckle", "left_finger_tip", "right_finger_tip"):
            L.append(f'    <link name="{k}_robotiq_85_{l}_link"/>')
        L += [f'    <link name="{k}_palm_insert"/>', f'    <joint name="{k}_robotiq_85_left_knuckle_joint"/>', "  </group>"]
        for name, q in poses[k].items():
            L.append(f'  <group_state name="{name}" group="arm_{k}">')
            L += [f'    <joint name="{k}_{j}" value="{v}"/>' for j, v in zip(JN, q)]
            L.append("  </group_state>")
        for name, v in (("open", 0.0), ("closed", GRIPPER_MAX)):
            L += [f'  <group_state name="{name}" group="gripper_{k}">',
                  f'    <joint name="{k}_robotiq_85_left_knuckle_joint" value="{v}"/>', "  </group_state>"]
        L.append(f'  <end_effector name="hand_{k}" parent_link="{k}_tcp" group="gripper_{k}" parent_group="arm_{k}"/>')
    for (a, b), r in sorted(reasons.items()):
        L.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="{r}"/>')
    L.append("</robot>")
    out = os.path.join(REPO, "src/catch_moveit_config/config/catch_scene.srdf")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    open(out, "w").write("\n".join(L) + "\n")
    print("wrote", out)


if __name__ == "__main__":
    main()
