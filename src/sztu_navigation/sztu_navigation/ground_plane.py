"""Bounded per-cloud local floor estimation for the X2 sensor adapter.

All inputs are in the same leveled pelvis frame at the cloud timestamp. No
previous plane is cached: missing floor/ankle evidence invalidates this frame.
These geometric guards are commissioning heuristics, not contact sensing or
proof that stairs, drop-offs or every tabletop can be distinguished from floor.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class GroundPlane:
    normal: np.ndarray
    offset: float
    inliers: int
    residual_p95: float
    xy_span: tuple


def estimate_ground(points, ankles):
    """Fit floor with spatial support and independent ankle plausibility guards.

    Fit ROI: 0.6--3 m, z -1.5--0 m; RANSAC cap 2048 points/64 hypotheses.
    Require >=200 inliers, >=25% of ROI, >=0.7m forward and >=0.5m lateral
    support, tilt <=15 degrees, p95 residual <=2cm, <=5% returns >5cm below
    the plane. Both ankle origins must lie above the plane, with the lower
    ankle 2--20cm above it and the other <=40cm above. These broad limits
    exclude common walls/tables/degenerate fits; they do not establish a sole
    contact offset. Rejection returns None and must suppress the safety scan.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    ankles = np.asarray(ankles, dtype=float)
    if ankles.shape != (2, 3) or not np.all(np.isfinite(ankles)):
        return None
    distance = np.hypot(points[:, 0], points[:, 1])
    roi = points[np.all(np.isfinite(points), axis=1)
                 & (distance >= 0.6) & (distance <= 3.0)
                 & (points[:, 2] >= -1.5) & (points[:, 2] <= 0.0)]
    if len(roi) < 200:
        return None
    # Seeded subsampling bounds cost without depending on a cloud's ordering.
    rng = np.random.default_rng(0)
    sample = roi[rng.choice(len(roi), min(len(roi), 2048), replace=False)]
    triples = sample[rng.integers(0, len(sample), size=(64, 3))]
    normals = np.cross(triples[:, 1] - triples[:, 0], triples[:, 2] - triples[:, 0])
    norms = np.linalg.norm(normals, axis=1)
    valid = norms > 1e-8
    normals[valid] /= norms[valid, None]
    normals[normals[:, 2] < 0] *= -1
    offsets = np.einsum('ij,ij->i', normals, triples[:, 0])
    foot_heights = ankles @ normals.T - offsets
    valid &= ((normals[:, 2] >= math.cos(math.radians(15)))
              & (foot_heights.min(axis=0) >= 0.02)
              & (foot_heights.min(axis=0) <= 0.20)
              & (foot_heights.max(axis=0) <= 0.40))
    if not valid.any():
        return None
    normals, offsets = normals[valid], offsets[valid]
    residuals = np.abs(sample @ normals.T - offsets)
    scores = (residuals <= 0.02).sum(axis=0)
    best = int(np.argmax(scores))
    if scores[best] < max(200, len(sample) * 0.25):
        return None
    normal, offset = normals[best], offsets[best]
    for _ in range(2):
        selected = sample[np.abs(sample @ normal - offset) <= 0.02]
        if len(selected) < 200:
            return None
        center = selected.mean(axis=0)
        _, _, vectors = np.linalg.svd(selected - center, full_matrices=False)
        normal = vectors[-1]
        if normal[2] < 0:
            normal = -normal
        offset = float(center @ normal)
    residuals = roi @ normal - offset
    inlier = np.abs(residuals) <= 0.02
    support = roi[inlier]
    if len(support) < max(200, len(roi) * 0.25):
        return None
    # Percentile spans reject a tiny cluster with a few distant stray points.
    span = np.diff(np.percentile(support[:, :2], [5, 95], axis=0), axis=0)[0]
    p95 = float(np.percentile(np.abs(residuals[inlier]), 95))
    foot_heights = ankles @ normal - offset
    if (normal[2] < math.cos(math.radians(15)) or span[0] < 0.7 or span[1] < 0.5
            or p95 > 0.02 or np.mean(residuals < -0.05) > 0.05
            or foot_heights.min() < 0.02 or foot_heights.min() > 0.20
            or foot_heights.max() > 0.40):
        return None
    return GroundPlane(normal, offset, int(inlier.sum()), p95, tuple(span))
