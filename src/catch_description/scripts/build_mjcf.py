#!/usr/bin/env python3
"""Generate the MuJoCo scene (mjcf/scene.xml + mjcf/assets/) from pinned Menagerie models and config YAML.

Runs offline in the venv (needs the `mujoco` Python package, pinned to the mujoco_vendor version):
    env -u PYTHONPATH .venv/bin/python src/catch_description/scripts/build_mjcf.py
The output is committed; test/test_mjcf_up_to_date.py checks it matches a fresh generation.

Changes applied to the Menagerie models (see docs/m0_toolchain.md for why):
  * UR5e body offsets patched to the exact ur_description DH values (FK agreement with the URDF).
  * UR5e position servos -> torque motors clamped to the UR5e joint torques; gravity compensation on all
    robot bodies, routed through the actuators (actuatorgravcomp) so it counts against the torque limits.
  * 2F-85: driver joint renamed to the URDF name `robotiq_85_left_knuckle_joint`; its actuator gets the same
    name and is re-expressed as an equivalent position servo in radians (physics.yaml gripper: kp/kv, ctrl 0..0.8;
    v1 used kp=100, kv=10).
  * 2F-85: a primitive palm insert (box) that only collides with cubes, a palm touch sensor and a TCP site.
"""
import argparse
import os
import shutil
import sys

import mujoco
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
REPO = os.path.abspath(os.path.join(PKG, "..", ".."))
MENAGERIE = os.path.join(REPO, "third_party", "mujoco_menagerie")

DH_PATCH = {  # body (or site) -> new pos, Menagerie values in comments
    "shoulder_link": [0, 0, 0.1625],      # 0.163
    "wrist_1_link": [0, 0, 0.3922],       # 0.392
    "wrist_2_link": [0, 0.1263, 0],       # 0.127 (0.138 - 0.131 + 0.1263 = d4 = 0.1333)
    "wrist_3_link": [0, 0, 0.0997],       # 0.1
}
SITE_PATCH = {"attachment_site": [0, 0.0996, 0]}  # 0.1

# collision bit masks: robot/table use bit 0; cubes use bits 0+1; palm insert accepts only bit 1
CUBE_CONTYPE, CUBE_CONAFF = 3, 1
INSERT_CONTYPE, INSERT_CONAFF = 0, 2


def robot_keys(scene):
    return [k for k in scene["robots"] if isinstance(scene["robots"][k], dict)]


def robot_base_xyz(scene, key):
    r = scene["robots"][key]
    return [r["base_xy"][0], r["base_xy"][1], scene["table"]["top_z"] + r.get("pedestal_height", 0.0)]


def load_cfg():
    with open(os.path.join(PKG, "config", "scene.yaml")) as f:
        scene = yaml.safe_load(f)
    with open(os.path.join(PKG, "config", "physics.yaml")) as f:
        phys = yaml.safe_load(f)
    return scene, phys


def _load(rel, asset_subdir, renames=None):
    """Load a Menagerie MJCF with attribute-aware renames; mesh files become '<asset_subdir>/<file>'."""
    import re
    path = os.path.join(MENAGERIE, rel)
    txt = open(path).read()
    for (attrs, old), new in (renames or {}).items():
        txt = re.sub(r'(\b(?:%s)=)"%s"' % (attrs, re.escape(old)), r'\1"%s"' % new, txt)
    spec = mujoco.MjSpec.from_string(txt)
    spec.meshdir = ""
    for m in spec.meshes:
        m.file = os.path.join(asset_subdir, m.file)
    return spec


def make_arm(scene, phys, with_camera=False):
    """UR5e + 2F-85 (+ palm insert, TCP site, touch sensor), unprefixed. Returns an MjSpec."""
    ur = _load("universal_robots_ur5e/ur5e.xml", "ur5e")
    for name, pos in DH_PATCH.items():
        ur.body(name).pos = pos
    for name, pos in SITE_PATCH.items():
        ur.site(name).pos = pos
    for a in ur.actuators:
        lim = float(a.forcerange[1])
        a.name = a.target + "_motor"
        a.set_to_motor()
        a.biasprm = np.zeros(10)
        a.ctrlrange = [-lim, lim]
        a.forcerange = [-lim, lim]
    if phys["arm"]["gravcomp"]:
        for b in ur.bodies:
            if b.name != "world":
                b.gravcomp = 1.0
        for j in ur.joints:
            j.actgravcomp = True

    g = _load("robotiq_2f85/2f85.xml", "2f85",
              {("name|joint|joint1|joint2", "right_driver_joint"): "85_left_knuckle_joint",
               ("name", "fingers_actuator"): "85_left_knuckle_joint"})
    act = g.actuator("85_left_knuckle_joint")
    gp = phys.get("gripper", {"kp": 100.0, "kv": 10.0, "force_scale": 1.0})
    act.set_to_position(kp=gp["kp"], kv=gp["kv"])
    act.ctrlrange = [0, 0.8]
    act.forcerange = np.asarray(act.forcerange) * gp.get("force_scale", 1.0)
    for geom in g.geoms:
        if geom.classname.name in ("pad_box1", "pad_box2"):
            geom.friction = phys["pads"]["friction"]
    for b in g.bodies:
        if b.name != "world":
            b.gravcomp = 1.0
    base = g.body("base")
    ins = scene["gripper"]["palm_insert"]
    hz = ins["half_size"][2]
    # the palm insert is its own (welded) body so a force/torque sensor measures exactly the contact force the
    # cube applies to the palm; it is exported to ROS as an FTS sensor (the MuJoCo touch sensor is kept for checks)
    palm = base.add_body(name="palm", pos=[0, 0, ins["top_z"] - hz], gravcomp=1.0)
    palm.add_geom(name="palm_insert", type=mujoco.mjtGeom.mjGEOM_BOX, size=ins["half_size"],
                  contype=INSERT_CONTYPE, conaffinity=INSERT_CONAFF,
                  friction=phys["palm_insert"]["friction"], solref=phys["palm_insert"]["solref"],
                  solimp=phys["palm_insert"]["solimp"], rgba=[0.25, 0.25, 0.28, 1], mass=0.005, group=1)
    palm.add_site(name="palm_ft", pos=[0, 0, 0], size=[0.003] * 3, rgba=[1, 0, 0, 0.0], group=4)
    g.add_sensor(name="palm_ft_force", type=mujoco.mjtSensor.mjSENS_FORCE, objtype=mujoco.mjtObj.mjOBJ_SITE,
                 objname="palm_ft")
    g.add_sensor(name="palm_ft_torque", type=mujoco.mjtSensor.mjSENS_TORQUE, objtype=mujoco.mjtObj.mjOBJ_SITE,
                 objname="palm_ft")
    # touch sensor zone: a thin site just covering the insert top face
    palm.add_site(name="palm_touch", type=mujoco.mjtGeom.mjGEOM_BOX,
                  size=[ins["half_size"][0], ins["half_size"][1], 0.004], pos=[0, 0, hz],
                  rgba=[0, 1, 0, 0.0], group=4)
    edge = scene["cube"]["edge"]
    # TCP: centre of a cube resting on the insert; oriented like tool0 (x = finger opening axis), i.e. the
    # Menagerie base frame rotated +90 deg about z
    base.add_site(name="tcp", pos=[0, 0, ins["top_z"] + edge / 2], quat=_quat_z(np.pi / 2), size=[0.004] * 3,
                  rgba=[1, 1, 0, 0.0], group=4)
    g.add_sensor(name="palm_touch", type=mujoco.mjtSensor.mjSENS_TOUCH, objtype=mujoco.mjtObj.mjOBJ_SITE,
                 objname="palm_touch")
    if with_camera:
        cam = scene["cameras"]["b_wrist"]
        p = np.array(cam["pos_in_gripper_base"], float)
        tgt = np.array([0, 0, ins["top_z"] + edge / 2])
        fwd = (tgt - p) / np.linalg.norm(tgt - p)
        up = np.array([0, 0, 1.0])
        xax = np.cross(fwd, up)
        xax /= np.linalg.norm(xax)
        yax = np.cross(xax, fwd)
        base.add_camera(name="wrist_cam", pos=p.tolist(), quat=_axes_quat(xax, yax), fovy=cam["fovy"],
                        resolution=cam["resolution"])
        # camera hardware (visual only in the physics, like v1's housing; scene.yaml b_wrist_camera_parts)
        R_opt = np.column_stack([xax, -yax, fwd])          # optical frame: x right, y down, z = view direction
        for part in scene["cameras"]["b_wrist_camera_parts"]:
            pos, R = camera_part_pose(part, p, R_opt)
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, R.flatten())
            sz = part["size"]
            if part["type"] == "box":
                kw = dict(type=mujoco.mjtGeom.mjGEOM_BOX, size=[x / 2 for x in sz])
            else:
                kw = dict(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[sz[0], sz[1] / 2, 0])
            base.add_geom(name=f"wrist_cam_{part['name']}", pos=pos.tolist(), quat=q.tolist(), contype=0,
                          conaffinity=0, group=2, rgba=part["rgba"], **kw)
    ur.attach(g, prefix="robotiq_", site=ur.site("attachment_site"))
    return ur


AXIS_R = {"z": np.eye(3), "x": np.array([[0, 0, 1.0], [0, 1, 0], [-1, 0, 0]]),   # R_y(pi/2): z -> x
          "y": np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]])}                    # R_x(-pi/2): z -> y


def camera_part_pose(part, p_cam, R_opt):
    """Position + rotation in the gripper base frame of a b_wrist_camera_parts entry (cylinder axis along `axis`)."""
    Ra = AXIS_R[part.get("axis", "z")] if part["type"] == "cylinder" else np.eye(3)
    if part["frame"] == "camera":
        return p_cam + R_opt @ np.asarray(part["pos"], float), R_opt @ Ra
    return np.asarray(part["pos"], float), Ra


def _axes_quat(xax, yax):
    """Quaternion (w,x,y,z) of a frame with the given x and y axes (MuJoCo camera looks along -z)."""
    xax = np.asarray(xax, float) / np.linalg.norm(xax)
    yax = np.asarray(yax, float) / np.linalg.norm(yax)
    R = np.column_stack([xax, yax, np.cross(xax, yax)])
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.flatten())
    return q.tolist()


def _quat_z(yaw):
    return [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]


def build(scene, phys):
    spec = mujoco.MjSpec()
    spec.modelname = "catch_scene"
    spec.compiler.degree = False
    o = spec.option
    o.timestep = phys["timestep"]
    o.integrator = {"implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
                    "rk4": mujoco.mjtIntegrator.mjINT_RK4,
                    "euler": mujoco.mjtIntegrator.mjINT_EULER}[phys["integrator"]]
    o.cone = mujoco.mjtCone.mjCONE_ELLIPTIC if phys["cone"] == "elliptic" else mujoco.mjtCone.mjCONE_PYRAMIDAL
    o.impratio = phys["impratio"]
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 720
    light = scene["lighting"]
    spec.visual.headlight.ambient = [light["headlight_ambient"]] * 3
    spec.visual.headlight.diffuse = [light["headlight_diffuse"]] * 3
    spec.visual.headlight.specular = [0.0] * 3
    spec.visual.quality.shadowsize = 4096
    spec.stat.center = [0, 0, 0.9]
    spec.stat.extent = 1.6

    # --- appearance (primitives + builtin textures only) ---
    spec.add_texture(name="skybox", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, rgb1=[0.55, 0.65, 0.78], rgb2=[0.12, 0.14, 0.18],
                     width=512, height=3072)
    spec.add_texture(name="floor_tex", type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, rgb1=[0.42, 0.42, 0.44],
                     rgb2=[0.36, 0.36, 0.38], width=512, height=512, mark=mujoco.mjtMark.mjMARK_EDGE,
                     markrgb=[0.3, 0.3, 0.32])
    spec.add_texture(name="wood_tex", type=mujoco.mjtTexture.mjTEXTURE_2D,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_FLAT, rgb1=[0.62, 0.48, 0.33], rgb2=[0.55, 0.42, 0.28],
                     width=256, height=256, random=0.15)
    floor_mat = spec.add_material(name="floor_mat", reflectance=0.08, texrepeat=[6, 6])
    floor_mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "floor_tex"
    wood = spec.add_material(name="wood", reflectance=0.05, texrepeat=[2, 1])
    wood.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "wood_tex"
    spec.add_material(name="steel", rgba=[0.55, 0.56, 0.58, 1], specular=0.6, shininess=0.6)
    spec.add_material(name="pallet_mat", rgba=[0.80, 0.80, 0.78, 1])

    w = spec.worldbody
    w.add_light(name="key", pos=[0.6, -1.2, 3.0], dir=[-0.2, 0.4, -1], diffuse=[light["key"]] * 3,
                specular=[0.1] * 3, castshadow=True)
    w.add_light(name="fill", pos=[-1.5, 1.5, 2.5], dir=[0.5, -0.5, -1], diffuse=[light["fill"]] * 3,
                specular=[0.0] * 3, castshadow=False)
    w.add_light(name="top", pos=[0, 0, 3.0], dir=[0, 0, -1], diffuse=[light["top"]] * 3, specular=[0.0] * 3,
                castshadow=False)
    w.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[4, 4, 0.05], material="floor_mat")

    # --- table: top + legs (boxes) ---
    t = scene["table"]
    sx, sy = t["size"]
    th = t["top_thickness"]
    w.add_geom(name="table_top", type=mujoco.mjtGeom.mjGEOM_BOX, size=[sx / 2, sy / 2, th / 2],
               pos=[0, 0, t["top_z"] - th / 2], material="wood", friction=phys["table"]["friction"])
    ls = t["leg_size"] / 2
    for i, (lx, ly) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
        w.add_geom(name=f"table_leg{i}", type=mujoco.mjtGeom.mjGEOM_BOX,
                   size=[ls, ls, (t["top_z"] - th) / 2],
                   pos=[lx * (sx / 2 - 0.08), ly * (sy / 2 - 0.08), (t["top_z"] - th) / 2], material="steel")

    # --- robots ---
    for key in robot_keys(scene):
        r = scene["robots"][key]
        base = robot_base_xyz(scene, key)
        arm = make_arm(scene, phys, with_camera=(key == "b"))
        ph = r.get("pedestal_height", 0.0)
        if ph > 0:
            ps = scene["robots"]["pedestal_size"] / 2
            w.add_geom(name=f"{key}_pedestal", type=mujoco.mjtGeom.mjGEOM_BOX, size=[ps, ps, ph / 2],
                       pos=[base[0], base[1], base[2] - ph / 2], material="steel")
        # robot mounting plate (visual only)
        w.add_geom(name=f"{key}_mount_plate", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.09, 0.004],
                   pos=[base[0], base[1], base[2] + 0.004], material="steel", contype=0, conaffinity=0)
        # Menagerie's UR5e model frame is rotated by pi about z w.r.t. ur_description's base_link
        # (verified by the FK agreement test), so the MJCF mount frame = base_link * Rz(pi).
        fr = w.add_frame(pos=base, quat=_quat_z(r["base_yaw"] + np.pi))
        spec.attach(arm, prefix=f"{key}_", frame=fr)

    # --- pallets (boxes) ---
    p = scene["pallets"]
    colors = scene["cube"]["colors"]
    top_z = scene["table"]["top_z"]
    bx, by, bz = p["base_size"]
    for key in ("a", "b"):
        cx, cy = p[key]["center"]
        pal = w.add_body(name=f"pallet_{key}", pos=[cx, cy, top_z])
        pal.add_geom(name=f"pallet_{key}_base", type=mujoco.mjtGeom.mjGEOM_BOX, size=[bx / 2, by / 2, bz / 2],
                     pos=[0, 0, bz / 2], material="pallet_mat")
        bw, bh, inner = p["border_width"], p["border_height"], p["slot_inner"]
        for i, color in enumerate(p["slot_order"]):
            sxc = (i - 1) * p["slot_pitch"]
            half = inner / 2 + bw / 2
            for side, dx, dy, lx_, ly_ in (("n", 0, half, inner / 2 + bw, bw / 2),
                                           ("s", 0, -half, inner / 2 + bw, bw / 2),
                                           ("e", half, 0, bw / 2, inner / 2),
                                           ("w", -half, 0, bw / 2, inner / 2)):
                pal.add_geom(name=f"pallet_{key}_{color}_border_{side}", type=mujoco.mjtGeom.mjGEOM_BOX,
                             size=[lx_, ly_, bh / 2], pos=[sxc + dx, dy, bz + bh / 2], rgba=colors[color])
            pal.add_site(name=f"pallet_{key}_slot_{color}", pos=[sxc, 0, bz], size=[0.005] * 3, group=4)

    # --- cubes: start in A's slots ---
    edge = scene["cube"]["edge"]
    cphys = phys["cube"]
    ca = p["a"]["center"]
    for i, color in enumerate(p["slot_order"]):
        c = w.add_body(name=f"cube_{color}", pos=[ca[0] + (i - 1) * p["slot_pitch"], ca[1], top_z + bz + edge / 2])
        c.add_freejoint(name=f"cube_{color}_joint")
        c.add_geom(name=f"cube_{color}_geom", type=mujoco.mjtGeom.mjGEOM_BOX, size=[edge / 2] * 3,
                   mass=scene["cube"]["mass"], rgba=colors[color], friction=cphys["friction"], condim=cphys["condim"],
                   solref=cphys["solref"], solimp=cphys["solimp"], contype=CUBE_CONTYPE, conaffinity=CUBE_CONAFF)

    # --- fixed cameras ---
    cams = scene["cameras"]
    for name in ("overhead_a", "overhead_b"):
        c = cams[name]
        w.add_camera(name=name, pos=c["pos"], quat=[1, 0, 0, 0], fovy=c["fovy"], resolution=c["resolution"])
    c = cams["side"]
    fwd = np.subtract(c["target"], c["pos"])
    fwd = fwd / np.linalg.norm(fwd)
    xax = np.cross(fwd, [0, 0, 1.0])
    xax /= np.linalg.norm(xax)
    w.add_camera(name="side", pos=c["pos"], quat=_axes_quat(xax, np.cross(xax, fwd)), fovy=c["fovy"],
                 resolution=c["resolution"])
    w.add_camera(name="demo", pos=[0.0, -2.3, 1.85], quat=_axes_quat([1, 0, 0], [0, 0.45, 0.89]), fovy=45)
    for name, c in cams.get("tracking", {}).items():       # v2 flight tracking cameras
        xax, yax = look_at_axes(c["pos"], c["target"], c.get("up", [0, 0, 1.0]))
        w.add_camera(name=name, pos=c["pos"], quat=_axes_quat(xax, yax), fovy=c["fovy"], resolution=c["resolution"])
    return spec


def look_at_axes(pos, target, up=(0, 0, 1.0)):
    """Camera x (image right) and y (image up) axes in the world for a MuJoCo camera at pos looking at target
    (MuJoCo cameras look along -z). Same function as catch_tracker's extrinsics (tested)."""
    fwd = np.subtract(target, pos).astype(float)
    fwd /= np.linalg.norm(fwd)
    xax = np.cross(fwd, up)
    xax /= np.linalg.norm(xax)
    return xax, np.cross(xax, fwd)


def compile_model(scene=None, phys=None):
    """Compile in memory (for offline parameter studies); meshes are read from the committed mjcf/assets."""
    if scene is None or phys is None:
        scene, phys = load_cfg()
    spec = build(scene, phys)
    assets = os.path.join(PKG, "mjcf", "assets")
    for m in spec.meshes:
        m.file = os.path.join(assets, m.file)
    return spec.compile()


def generate(out_dir=None, verbose=True, scene=None, phys=None):
    if scene is None or phys is None:
        scene, phys = load_cfg()
    out_dir = out_dir or os.path.join(PKG, "mjcf")
    spec = build(scene, phys)
    assets = os.path.join(out_dir, "assets")
    os.makedirs(assets, exist_ok=True)
    for sub, src in (("ur5e", "universal_robots_ur5e/assets"), ("2f85", "robotiq_2f85/assets")):
        dst = os.path.join(assets, sub)
        os.makedirs(dst, exist_ok=True)
        for f in os.listdir(os.path.join(MENAGERIE, src)):
            shutil.copy2(os.path.join(MENAGERIE, src, f), os.path.join(dst, f))
        lic_src = os.path.join(MENAGERIE, os.path.dirname(src), "LICENSE")
        shutil.copy2(lic_src, os.path.join(dst, "LICENSE"))
    rel = {}
    for m in spec.meshes:
        rel[m.name] = m.file
        m.file = os.path.join(assets, m.file)
    model = spec.compile()
    xml = spec.to_xml()
    for name, f in rel.items():  # make mesh paths relative to scene.xml (portable install)
        xml = xml.replace(f'file="{os.path.join(assets, f)}"', f'file="assets/{f}"')
    header = ("<!-- GENERATED by src/catch_description/scripts/build_mjcf.py from config/scene.yaml, "
              "config/physics.yaml and MuJoCo Menagerie. Do not edit by hand. -->\n")
    with open(os.path.join(out_dir, "scene.xml"), "w") as f:
        f.write(header + xml)
    if verbose:
        print(f"wrote {out_dir}/scene.xml: nq={model.nq} nv={model.nv} nu={model.nu} nbody={model.nbody} "
              f"ngeom={model.ngeom} ncam={model.ncam} nsensor={model.nsensor}")
    return model


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    generate(ap.parse_args().out)
