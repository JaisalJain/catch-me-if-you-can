"""Synthetic-image tests for the vision functions (no ROS, no sim)."""
import numpy as np
import pytest
import yaml
import os

from catch_perception.vision import cube_blobs, dominant_color, pixel_ray_plane

CFG = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "perception.yaml")))
COL = {"red": (220, 25, 25), "green": (30, 200, 45), "blue": (50, 120, 255)}


def canvas():
    return np.full((480, 640, 3), (105, 105, 110), np.uint8)


def draw_square(img, cu, cv, half, rgb, ring=False, width=10):
    import cv2
    if ring:
        cv2.rectangle(img, (cu - half, cv - half), (cu + half, cv + half), rgb, width)
    else:
        cv2.rectangle(img, (cu - half, cv - half), (cu + half, cv + half), rgb, -1)


def test_cube_inside_same_colored_ring_is_found_once():
    img = canvas()
    for i, (name, rgb) in enumerate(COL.items()):
        cu = 200 + 120 * i
        draw_square(img, cu, 240, 42, rgb, ring=True, width=10)    # slot border
        draw_square(img, cu + 3, 238, 24, rgb)                       # cube top, slightly off-centre
    blobs = cube_blobs(img, CFG["colors"], 7, 150)
    assert sorted(b.color for b in blobs) == ["blue", "green", "red"]
    for b in blobs:
        i = list(COL).index(b.color)
        assert b.u == pytest.approx(200 + 120 * i + 3, abs=1.0)
        assert b.v == pytest.approx(238, abs=1.0)


def test_rotated_cube_yaw():
    import cv2
    img = canvas()
    box = cv2.boxPoints(((320, 240), (50, 50), 20.0)).astype(np.int32)
    cv2.fillPoly(img, [box], COL["green"])
    b = [b for b in cube_blobs(img, CFG["colors"], 7, 150) if b.color == "green"][0]
    assert abs(np.degrees(b.yaw)) == pytest.approx(20.0, abs=3.0)


def test_ray_plane_roundtrip():
    K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])
    R = np.diag([1.0, -1.0, -1.0])        # looking straight down
    t = np.array([-0.55, -0.28, 1.45])
    P = np.array([-0.50, -0.30, 0.802])
    pc = R.T @ (P - t)
    uv = K @ pc
    u, v = uv[:2] / uv[2]
    np.testing.assert_allclose(pixel_ray_plane(u, v, K, R, t, 0.802), P, atol=1e-9)


def test_dominant_color():
    img = canvas()
    draw_square(img, 320, 240, 60, COL["blue"])
    best, frac, fr = dominant_color(img, CFG["colors"], (0.25, 0.15, 0.75, 0.85))
    assert best == "blue" and frac == pytest.approx(121 * 121 / (320 * 336), rel=0.05) and fr["red"] == 0.0


# ------------------------------------------------------------------------------------------ in-hand pose
def _wrist_camera(p_cam=(0.12, 0.0, 0.02)):
    """Pinhole camera in the TCP frame looking at the TCP (like B's wrist camera), ROS optical convention."""
    p = np.array(p_cam)
    z = -p / np.linalg.norm(p)
    x = np.cross(z, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = np.column_stack([x, y, z]), p
    K = np.array([[208.0, 0, 160], [0, 208.0, 120], [0, 0, 1]])
    return K, T


def _render_face(K, T, cy, cz, th, edge=0.04, occluder_px=18):
    """Red cube face (plane x = edge/2, square rotated by th about x) on grey, with a dark vertical 'finger' band
    occluding the middle of the face."""
    h, w = 240, 320
    v, u = np.mgrid[0:h, 0:w]
    d = T[:3, :3] @ (np.linalg.inv(K) @ np.vstack([u.ravel(), v.ravel(), np.ones(u.size)]))
    s = (edge / 2 - T[0, 3]) / d[0]
    y, z = T[1, 3] + s * d[1] - cy, T[2, 3] + s * d[2] - cz
    c, sn = np.cos(th), np.sin(th)
    a, b = c * y + sn * z, -sn * y + c * z
    inside = ((np.abs(a) <= edge / 2) & (np.abs(b) <= edge / 2)).reshape(h, w)
    img = np.full((h, w, 3), (90, 95, 100), np.uint8)
    img[inside] = (200, 30, 30)
    uc = K[0, 2]
    img[:, int(uc - occluder_px):int(uc + occluder_px)] = (30, 30, 32)
    # knuckle: a wider dark block covering the palm-side part of the face
    vk = int(K[1, 2] + 0.55 * K[1, 1] * edge / T[0, 3])
    img[vk:, int(uc - 2.2 * occluder_px):int(uc + 2.2 * occluder_px)] = (30, 30, 32)
    # a same-colored slot border in the background, just below the hand (like B's red slot border in the real view)
    img[195:203, 205:240] = (200, 30, 30)
    img[203:212, 205:211] = (200, 30, 30)
    return img


@pytest.mark.parametrize("th_deg,cy,cz", [(0, 0, 0), (3, 0.002, 0.004), (-12.6, 0.018, 0.005), (25, -0.005, 0.01),
                                          (-40, 0.0, 0.0)])
def test_in_hand_pose_from_occluded_face(th_deg, cy, cz):
    from catch_perception.vision import in_hand_pose
    K, T = _wrist_camera()
    img = _render_face(K, T, cy, cz, np.radians(th_deg))
    r = in_hand_pose(img, CFG["colors"]["red"], K, T, 0.04)
    assert r is not None
    err = (np.degrees(r["angle"]) - th_deg + 45) % 90 - 45
    assert abs(err) < (0.5 if abs(th_deg) <= 25 else 1.5), r    # near 45 deg the edge/side assignment is ambiguous
    assert abs(r["p"][1] - cy) < 0.001 and abs(r["p"][2] - cz) < 0.001, r
