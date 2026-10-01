"""Pure image-processing functions (numpy + OpenCV, no ROS) — unit-testable on synthetic images."""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


def color_mask(rgb: np.ndarray, spec: dict) -> np.ndarray:
    """Binary mask (uint8 0/255) of pixels matching an HSV spec {'h': [[lo, hi], ...], 's_min', 'v_min'}."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    hm = np.zeros(h.shape, bool)
    for lo, hi in spec["h"]:
        hm |= (h >= lo) & (h <= hi)
    return ((hm & (s >= spec["s_min"]) & (v >= spec["v_min"])) * 255).astype(np.uint8)


@dataclass
class Blob:
    color: str
    u: float          # pixel centroid of the cube core
    v: float
    yaw: float        # rad, in image coordinates, folded to [-pi/4, pi/4)
    area: float       # core area in px


def cube_blobs(rgb: np.ndarray, colors: dict, core_px: float = 7.0, min_core_area: float = 150.0) -> list[Blob]:
    """Cube tops of each color. The distance transform keeps only pixels deeper than core_px inside the color
    mask, which removes thin same-colored structures (slot borders) and side-face slivers."""
    out = []
    for name, spec in colors.items():
        mask = color_mask(rgb, spec)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
        core = (dist > core_px).astype(np.uint8)
        n, labels, stats, cents = cv2.connectedComponentsWithStats(core)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < min_core_area:
                continue
            pts = np.column_stack(np.nonzero(labels == i))[:, ::-1].astype(np.float32)   # (u, v)
            (cu, cv_), (w, h), ang = cv2.minAreaRect(pts)
            yaw = np.radians(ang)
            yaw = (yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4
            out.append(Blob(name, float(cents[i][0]), float(cents[i][1]), float(yaw), float(stats[i, cv2.CC_STAT_AREA])))
    return out


def pixel_ray_plane(u: float, v: float, K: np.ndarray, R_wc: np.ndarray, t_wc: np.ndarray, z: float) -> np.ndarray:
    """Intersect the camera ray through pixel (u, v) with the world plane Z = z.
    R_wc, t_wc: pose of the ROS optical frame in the world (x right, y down, z forward)."""
    d_c = np.linalg.solve(K, np.array([u, v, 1.0]))
    d_w = R_wc @ d_c
    if abs(d_w[2]) < 1e-9:
        raise ValueError("ray parallel to plane")
    s = (z - t_wc[2]) / d_w[2]
    return t_wc + s * d_w


def dominant_color(rgb: np.ndarray, colors: dict, roi=(0.0, 0.0, 1.0, 1.0)) -> tuple[str, float, dict]:
    """Fraction of ROI pixels per color; returns (best color, its fraction, all fractions)."""
    hgt, wid = rgb.shape[:2]
    u0, v0, u1, v1 = roi
    crop = rgb[int(v0 * hgt):int(v1 * hgt), int(u0 * wid):int(u1 * wid)]
    fr = {name: float(np.count_nonzero(color_mask(crop, spec))) / crop[..., 0].size for name, spec in colors.items()}
    best = max(fr, key=fr.get)
    return best, fr[best], fr
