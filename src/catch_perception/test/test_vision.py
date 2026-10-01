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
