"""M0 spike: compose two prefixed UR5e + 2F-85 from Menagerie into one MJCF with MjSpec."""
import os, shutil, sys
import mujoco

MEN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "third_party", "mujoco_menagerie")
OUT = os.path.dirname(os.path.abspath(__file__))


def load(rel, renames=None):
    path = os.path.join(MEN, rel)
    txt = open(path).read()
    import re
    for (attrs, old), new in (renames or {}).items():
        txt = re.sub(r'(\b(?:%s)=)"%s"' % (attrs, re.escape(old)), r'\1"%s"' % new, txt)
    spec = mujoco.MjSpec.from_string(txt)
    spec.meshdir = ""
    d = os.path.join(os.path.dirname(path), "assets")
    for m in spec.meshes:
        m.file = os.path.join(d, m.file)
    return spec


def make_arm():
    ur = load("universal_robots_ur5e/ur5e.xml")
    g = load("robotiq_2f85/2f85.xml", {("name|joint|joint1|joint2", "right_driver_joint"): "85_left_knuckle_joint",
                                       ("name", "fingers_actuator"): "85_left_knuckle_joint"})
    # gripper: replace actuator by an equivalent position servo in radians
    act = g.actuator("85_left_knuckle_joint")
    act.set_to_position(kp=100, kv=10)
    act.ctrlrange = [0, 0.8]
    # arm: effort actuators
    for a in ur.actuators:
        lim = a.forcerange[1]
        a.name = a.target + "_motor"
        a.set_to_motor()
        a.ctrlrange = [-lim, lim]
        a.forcerange = [-lim, lim]
    site = ur.site("attachment_site")
    ur.attach(g, prefix="robotiq_", site=site)
    return ur


spec = mujoco.MjSpec()
spec.modelname = "spike"
spec.option.timestep = 0.001
spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
spec.option.impratio = 10
w = spec.worldbody
w.add_light(pos=[0, 0, 3], dir=[0, 0, -1])
w.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1], rgba=[.3, .3, .3, 1])
w.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX, size=[1.0, 0.4, 0.375], pos=[0, 0, 0.375], rgba=[.6, .5, .4, 1])
for prefix, x, yaw in (("a_", -0.8, 0), ("b_", 0.8, 180)):
    arm = make_arm()
    fr = w.add_frame(pos=[x, 0, 0.75], euler=[0, 0, yaw * 3.14159265 / 180])
    spec.attach(arm, prefix=prefix, frame=fr)
cube = w.add_body(name="cube_red", pos=[0, 0, 0.8])
cube.add_freejoint()
cube.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.02] * 3, mass=0.06, rgba=[1, 0, 0, 1])
w.add_camera(name="overhead", pos=[0, 0, 2.2], resolution=[640, 480], fovy=60)
m = spec.compile()
print("nq", m.nq, "nu", m.nu, "njnt", m.njnt)
for i in range(m.nu):
    print("act", mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i))
xml = spec.to_xml()
open(os.path.join(OUT, "spike.xml"), "w").write(xml)
