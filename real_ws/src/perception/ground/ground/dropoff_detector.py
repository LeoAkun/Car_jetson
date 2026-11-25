"""Conservative image-space evidence for a forward ground discontinuity.

The output is a *suspected* drop-off, not a proof of a traversable or unsafe
surface. Missing depth can also be caused by reflective material or IR shadows.
This detector deliberately keeps that case separate from a measured low surface.
Its short temporal confirmation is expressed in base coordinates and is intended
for stationary-camera indoor evaluation. It is not odometry-based hazard memory.
"""

import numpy as np


class GroundRayDropoffDetector:
    def __init__(
        self, min_drop=0.04, min_width=0.12, support_pixels=3,
        gap_pixels=2, confirmation_frames=3, max_range=2.5,
        border_pixels=8, min_valid_ratio=0.1, max_depth=5.0,
        plane_tolerance=0.025, x_min=0.5, x_max=4.0,
        y_min=-1.5, y_max=1.5,
    ):
        self.min_drop = float(min_drop)
        self.min_width = float(min_width)
        self.support_pixels = max(int(support_pixels), 1)
        self.gap_pixels = max(int(gap_pixels), 1)
        self.confirmation_frames = max(int(confirmation_frames), 1)
        self.max_range = float(max_range)
        self.border_pixels = max(int(border_pixels), 0)
        self.min_valid_ratio = float(min_valid_ratio)
        self.max_depth = float(max_depth)
        self.plane_tolerance = float(plane_tolerance)
        self.x_min, self.x_max = float(x_min), float(x_max)
        self.y_min, self.y_max = float(y_min), float(y_max)
        self.reset()

    def reset(self):
        self._tracks = []
        self._last_stamp = None
        self._ray_cache = None
        self.last_status = "unavailable"
        self.last_debug = {"reason": "not_processed"}

    def _unavailable(self, reason):
        self._tracks = []
        self.last_status = "unavailable"
        self.last_debug["reason"] = reason
        return []

    def _base_rays(self, shape, K, R):
        # Static mounting and CameraInfo make this the same on nearly every
        # frame. Cache only geometry, never observed depth or hazard evidence.
        key = (shape, tuple(K.ravel()), tuple(R.ravel()))
        if self._ray_cache is not None and self._ray_cache[0] == key:
            return self._ray_cache[1]
        height, width = shape
        optical_x = (np.arange(width, dtype=np.float32)[None, :] - K[0, 2]) / K[0, 0]
        optical_y = (np.arange(height, dtype=np.float32)[:, None] - K[1, 2]) / K[1, 1]
        rays = tuple(
            (R[axis, 0] * optical_x + R[axis, 1] * optical_y + R[axis, 2]).astype(np.float32)
            for axis in range(3)
        )
        self._ray_cache = (key, rays)
        return rays

    def process(self, depth_img, K, R, t, ground_plane, stamp_seconds=None):
        """Return temporally confirmed upper-edge positions in ``base_link``.

        ``ground_plane=(a,b,c)`` describes z=a*x+b*y+c. ``R,t`` transform
        optical-camera points into base coordinates; optical depth is in meters.
        ``drop`` is None if only missing-depth evidence is available. Empty
        output never establishes that the ground is safe.
        """
        self.last_debug = {}
        if ground_plane is None:
            return self._unavailable("ground_plane_missing")
        depth = np.asarray(depth_img)
        K, R = np.asarray(K, dtype=np.float32), np.asarray(R, dtype=np.float32)
        t = np.asarray(t, dtype=np.float32).reshape(-1)
        plane = np.asarray(ground_plane, dtype=np.float32)
        if (
            depth.ndim != 2 or K.shape != (3, 3) or R.shape != (3, 3)
            or t.shape != (3,) or plane.shape != (3,)
            or not np.all(np.isfinite(K)) or not np.all(np.isfinite(R))
            or not np.all(np.isfinite(t)) or not np.all(np.isfinite(plane))
            or K[0, 0] <= 0 or K[1, 1] <= 0
        ):
            return self._unavailable("invalid_geometry")
        height, width = depth.shape
        border = self.border_pixels
        if height <= 2 * border + self.support_pixels + self.gap_pixels or width <= 2 * border:
            return self._unavailable("image_too_small")

        if stamp_seconds is not None:
            stamp = float(stamp_seconds)
            if not np.isfinite(stamp):
                return self._unavailable("invalid_stamp")
            if self._last_stamp is not None and not 0.0 < stamp - self._last_stamp <= 0.3:
                self._tracks = []
            self._last_stamp = stamp

        valid_depth = np.isfinite(depth) & (depth > 0.1) & (depth < self.max_depth)
        valid_ratio = float(np.mean(valid_depth))
        self.last_debug["valid_depth_ratio"] = valid_ratio
        if valid_ratio < self.min_valid_ratio:
            return self._unavailable("insufficient_depth")

        # A camera on/below the alleged ground cannot supply this visibility test.
        a, b, c = plane
        if t[2] - a * t[0] - b * t[1] - c <= self.plane_tolerance:
            return self._unavailable("camera_not_above_ground")
        ray_x, ray_y, ray_z = self._base_rays(depth.shape, K, R)
        denominator = ray_z - a * ray_x - b * ray_y
        expected = np.full(depth.shape, np.nan, dtype=np.float32)
        downward = denominator < -1e-4
        np.divide(c + a * t[0] + b * t[1] - t[2], denominator,
                  out=expected, where=downward)
        px, py = t[0] + expected * ray_x, t[1] + expected * ray_y
        visible_ground = (
            downward & np.isfinite(expected) & (expected > 0.1)
            & (expected < self.max_depth)
            & (np.hypot(px - t[0], py - t[1]) <= self.max_range)
            & (px >= self.x_min) & (px <= self.x_max)
            & (py >= self.y_min) & (py <= self.y_max)
        )
        self.last_debug["expected_ground_pixels"] = int(np.count_nonzero(visible_ground))
        if not np.any(visible_ground):
            return self._unavailable("ground_outside_view")

        # This is a vertical plane residual, not an optical-depth difference.
        residual = denominator * (np.where(valid_depth, depth, expected) - expected)
        support = visible_ground & valid_depth & (np.abs(residual) <= self.plane_tolerance)
        low = visible_ground & valid_depth & (residual <= -self.min_drop)
        unknown = visible_ground & ~valid_depth
        unsafe = low | unknown
        # A nearby positive return is an occluder rather than drop-off evidence.
        positive = visible_ground & valid_depth & (residual > self.plane_tolerance)

        near_support = np.ones(depth.shape, dtype=bool)
        far_gap = np.ones(depth.shape, dtype=bool)
        for offset in range(1, self.support_pixels + 1):
            shifted = np.zeros(depth.shape, dtype=bool)
            shifted[:-offset] = support[offset:]
            near_support &= shifted
        for offset in range(self.gap_pixels):
            shifted = np.zeros(depth.shape, dtype=bool)
            if offset == 0:
                shifted = unsafe
            else:
                shifted[offset:] = unsafe[:-offset]
            far_gap &= shifted
        candidate = near_support & far_gap
        candidate[:border + self.gap_pixels - 1] = False
        candidate[height - border - self.support_pixels:] = False
        if border:
            candidate[:, :border] = False
            candidate[:, width - border:] = False

        columns = []
        for u in np.flatnonzero(np.any(candidate, axis=0)):
            # The nearest supported transition is the relevant upper edge.
            rows = np.flatnonzero(candidate[:, u])
            for v in rows[::-1]:
                region = positive[max(0, v - self.gap_pixels - 1):v + self.support_pixels + 2,
                                  max(0, u - 2):min(width, u + 3)]
                if np.any(region):
                    continue
                edge_x = float((px[v, u] + px[v + 1, u]) * 0.5)
                edge_y = float((py[v, u] + py[v + 1, u]) * 0.5)
                hazard_rows = slice(v - self.gap_pixels + 1, v + 1)
                drops = -residual[hazard_rows, u][low[hazard_rows, u]]
                columns.append((int(u), edge_x, edge_y, drops))
                break
        groups = []
        for entry in columns:
            if groups:
                previous = groups[-1][-1]
                connected = (
                    entry[0] - previous[0] <= 2
                    and abs(entry[1] - previous[1]) <= 0.08
                    and np.hypot(entry[1] - previous[1], entry[2] - previous[2]) <= 0.12
                )
            else:
                connected = False
            if not connected:
                groups.append([])
            groups[-1].append(entry)

        detections = []
        for group in groups:
            if len(group) < 3:
                continue
            xs, ys = np.array([(entry[1], entry[2]) for entry in group]).T
            adjacent_width = float(np.median(np.hypot(np.diff(xs), np.diff(ys))))
            edge_width = float(np.hypot(xs[-1] - xs[0], ys[-1] - ys[0]) + adjacent_width)
            # Both boundaries may fall between pixel centers. Permit one pixel
            # of threshold uncertainty; retain the measured width below.
            if edge_width + adjacent_width < self.min_width:
                continue
            drops = np.concatenate([entry[3] for entry in group])
            known_ratio = len(drops) / float(len(group) * self.gap_pixels)
            depth_known = known_ratio >= 0.5
            detections.append({
                "x": float(np.median(xs)), "y": float(np.median(ys)),
                "width": edge_width,
                "confidence": float(0.75 if depth_known else 0.45),
                "depth_known": bool(depth_known),
                "drop": float(np.median(drops)) if depth_known else None,
            })

        confirmed, tracks = [], []
        unmatched = list(self._tracks)
        for detection in detections:
            matching = [
                track for track in unmatched
                if np.hypot(detection["x"] - track["x"], detection["y"] - track["y"]) <= 0.12
                and abs(detection["y"] - track["y"]) <= (detection["width"] + track["width"]) * 0.5
                and detection["depth_known"] == track["depth_known"]
            ]
            match = min(matching, key=lambda track: np.hypot(
                detection["x"] - track["x"], detection["y"] - track["y"]
            )) if matching else None
            hits = match["hits"] + 1 if match is not None else 1
            if match is not None:
                unmatched.remove(match)
            tracks.append(dict(detection, hits=hits))
            if hits >= self.confirmation_frames:
                confirmed.append(detection)
        self._tracks = tracks
        self.last_status = "suspected_dropoff" if confirmed else "observing"
        self.last_debug.update({
            "reason": "suspected_dropoff" if confirmed else "no_confirmed_edge",
            "candidate_columns": len(columns), "candidate_edges": len(detections),
            "confirmed_edges": len(confirmed),
            "unknown_edges": sum(not detection["depth_known"] for detection in confirmed),
        })
        return confirmed
