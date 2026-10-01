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


# ----------------------------------------------------------------------------------------------- in-hand pose
def square_perimeter_residuals(params, pts, half):
    """Signed distance of 2D points to the perimeter of a square (centre cy, cz, rotation th, half edge `half`):
    > 0 outside, < 0 inside."""
    cy, cz, th = params
    c, s = np.cos(th), np.sin(th)
    d = pts - [cy, cz]
    u = c * d[:, 0] + s * d[:, 1]           # point in the square frame (rotate by -th)
    v = -s * d[:, 0] + c * d[:, 1]
    au, av = np.abs(u) - half, np.abs(v) - half
    outside = np.hypot(np.maximum(au, 0), np.maximum(av, 0))
    inside = np.minimum(np.maximum(au, av), 0)
    return outside + inside


def fit_square(face_pts: np.ndarray, edge_pts: np.ndarray, edge: float, f_scale: float = 0.001,
               reliable_sides=("+u", "-u", "+v")):
    """Fit a square of known edge to the visible part of a cube face.

    face_pts: all face pixels mapped to the face plane (N, 2) [m]; edge_pts: the mask boundary pixels (M, 2).
    The visible face is cut by occluders (the finger in front of the face, the knuckle at its palm side); their
    boundary points lie inside the square. Stage 1: robust fit (soft-L1) of all boundary points to the perimeter,
    initialised with the min-area rectangle of the face pixels. Stage 2: least squares on the inliers that belong
    to the `reliable_sides` of the square only (square frame: u ~ first plane axis, v ~ second; for the wrist
    camera v = tool z, so "-v" is the palm side, hidden by the knuckle). Returns (cy, cz, th, rms, inlier
    fraction); th folded to [-pi/4, pi/4)."""
    from scipy.optimize import least_squares
    (cy, cz), _, ang = cv2.minAreaRect((face_pts * 1e4).astype(np.float32))
    x0 = np.array([cy * 1e-4, cz * 1e-4, (np.radians(ang) + np.pi / 4) % (np.pi / 2) - np.pi / 4])
    best = None
    for dth in (0.0, np.radians(-15), np.radians(15)):       # a few starts: the loss is non-convex in th
        r = least_squares(square_perimeter_residuals, x0 + [0, 0, dth], args=(edge_pts, edge / 2),
                          loss="soft_l1", f_scale=f_scale)
        if best is None or r.cost < best.cost:
            best = r
    x = best.x.copy()
    x[2] = (x[2] + np.pi / 4) % (np.pi / 2) - np.pi / 4
    for _ in range(2):
        res = square_perimeter_residuals(x, edge_pts, edge / 2)
        c, s = np.cos(x[2]), np.sin(x[2])
        d = edge_pts - x[:2]
        u, v = c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]
        side = np.where(np.abs(u) >= np.abs(v), np.where(u > 0, "+u", "-u"), np.where(v > 0, "+v", "-v"))
        sel = (np.abs(res) < 2 * f_scale) & np.isin(side, reliable_sides)
        if sel.sum() < 10:
            break
        x = least_squares(square_perimeter_residuals, x, args=(edge_pts[sel], edge / 2)).x
    res = square_perimeter_residuals(x, edge_pts, edge / 2)
    inl = np.abs(res) < 2 * f_scale
    rms = float(np.sqrt(np.mean(res[sel] ** 2))) if sel.any() else np.inf
    cy, cz, th = x
    th = (th + np.pi / 4) % (np.pi / 2) - np.pi / 4
    return float(cy), float(cz), float(th), rms, float(inl.mean())


def in_hand_pose(rgb: np.ndarray, spec: dict, K: np.ndarray, T_tcp_cam: np.ndarray, edge: float, min_px: int = 200,
                 max_offset: float = 0.045):
    """Cube pose in the TCP frame from B's wrist camera, which looks along the closing axis (tool x).

    The near cube face lies in the plane x_tcp = +edge/2 (the pads centre the cube on the closing axis). Its
    pixels are back-projected onto that plane; pixels farther than max_offset from the TCP (in y or z) are dropped
    (e.g. a slot border of the same color in the background); a square of known edge is fitted in (y, z) using the
    two lateral edges and the fingertip-side edge (the palm-side edge is hidden by the knuckle). Returns
    dict(p=[x, y, z] cube centre, angle=rotation about tool x [rad], rms, inlier_frac, n_px) or None."""
    mask = color_mask(rgb, spec)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    keep = np.zeros_like(mask)
    for i in range(1, n):                       # drop specks (other cubes far away in the background are small)
        if stats[i, cv2.CC_STAT_AREA] >= 50:
            keep[labels == i] = 255
    if np.count_nonzero(keep) < min_px:
        return None
    boundary = keep & ~cv2.erode(keep, np.ones((3, 3), np.uint8))
    R, t = T_tcp_cam[:3, :3], T_tcp_cam[:3, 3]
    Kinv = np.linalg.inv(K)

    def to_plane(mask_):
        v, u = np.nonzero(mask_)
        d = R @ (Kinv @ np.vstack([u, v, np.ones_like(u)]).astype(float))    # rays in the TCP frame
        s = (edge / 2 - t[0]) / d[0]
        return (t[:, None] + s * d)[1:].T                                         # (y, z) on the face plane

    face, bnd = to_plane(keep), to_plane(boundary)
    near = lambda q: np.all(np.abs(q) < max_offset, axis=1)      # drop same-colored things away from the hand
    face, bnd = face[near(face)], bnd[near(bnd)]
    if len(face) < min_px:
        return None
    cy, cz, th, rms, inl = fit_square(face, bnd, edge)
    return dict(p=[0.0, cy, cz], angle=th, rms=rms, inlier_frac=inl, n_px=int(np.count_nonzero(keep)))
