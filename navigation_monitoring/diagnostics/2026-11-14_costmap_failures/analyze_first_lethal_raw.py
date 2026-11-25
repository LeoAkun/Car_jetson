#!/usr/bin/env python3
"""Offline raw-costmap audit of the 2026-11-14 11:13 lethal-start.

This script opens rosbag2 sqlite files read-only. It does not create ROS nodes,
publish topics, play bags, or access hardware.
"""
from __future__ import annotations

import collections
import datetime as dt
import json
import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.path import Path as MplPath
import numpy as np
from PIL import Image
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path("/home/akun/workspace/Car_jetson")
OUT = ROOT / "navigation_monitoring/diagnostics/2026-11-14_costmap_failures"
MAP_YAML = ROOT / "multi_map_navigation/src/map/3b2mus/3b2mus_clean.yaml"
MAP_PGM = ROOT / "multi_map_navigation/src/map/3b2mus/3b2mus_clean.pgm"
TZ = dt.timezone(dt.timedelta(hours=8))
FOOTPRINT = np.array([[-0.45, -0.30], [-0.45, 0.30], [0.45, 0.30], [0.45, -0.30]], float)
ANGLE_BINS = 64
ANGLE_STEP = 2 * math.pi / ANGLE_BINS
POSSIBLE_INSCRIBED_RAW = int(252 * math.exp(-5.0 * (math.hypot(0.45, 0.30) - 0.30)))
POSSIBLE_INSCRIBED_DISPLAY = 1 + (97 * (POSSIBLE_INSCRIBED_RAW - 1)) // 251

RUNS = [
    {
        "label": "11:14:58",
        "run": ROOT / "navigation_monitoring/runs/2026-11-14_11-12-06_manual_publish_monitor",
        "event_ns": 1794626098812463091,
        "nav_log": "nav2_2026-11-14_11-14-11.log",
    },
]


def ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def iso(t_ns):
    sec, nano = divmod(int(t_ns), 1_000_000_000)
    return dt.datetime.fromtimestamp(sec, TZ).strftime("%Y-%m-%dT%H:%M:%S") + f".{nano:09d}+08:00"


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    """Compose SE2 transforms a(parent->mid) and b(mid->child)."""
    x = a[0] + math.cos(a[2]) * b[0] - math.sin(a[2]) * b[1]
    y = a[1] + math.sin(a[2]) * b[0] + math.cos(a[2]) * b[1]
    return (x, y, wrap(a[2] + b[2]))


def transform_polygon(poly, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    r = np.array([[c, -s], [s, c]])
    return poly @ r.T + np.array(pose[:2])


def bresenham(x0, y0, x1, y1):
    """Match nav2_util::LineIterator exactly, including integer tie handling."""
    cells = []
    x, y = x0, y0
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    xinc1 = xinc2 = 1 if x1 >= x0 else -1
    yinc1 = yinc2 = 1 if y1 >= y0 else -1
    if dx >= dy:
        xinc1 = 0
        yinc2 = 0
        den = dx
        num = dx // 2
        numadd = dy
        npixels = dx
    else:
        xinc2 = 0
        yinc1 = 0
        den = dy
        num = dy // 2
        numadd = dx
        npixels = dy
    for _ in range(npixels + 1):
        cells.append((x, y))
        num += numadd
        if num >= den:
            num -= den
            x += xinc1
            y += yinc1
        x += xinc2
        y += yinc2
    return cells


def display_to_raw(value):
    # Raw nav2_msgs/Costmap values already use the internal 0..255 cost scale.
    if value > 100:
        return value
    if value == -1:
        return 255
    if value == 0:
        return 0
    if value == 99:
        return 253
    if value == 100:
        return 254
    candidates = [i for i in range(1, 253) if 1 + (97 * (i - 1)) // 251 == value]
    return max(candidates) if candidates else None


@dataclass
class Grid:
    data: np.ndarray
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0
    frame: str = "map"
    bag_ns: int | None = None
    header_ns: int | None = None

    @classmethod
    def from_msg(cls, msg, bag_ns):
        arr = np.frombuffer(memoryview(msg.data), dtype=np.uint8).reshape(
            msg.metadata.size_y, msg.metadata.size_x)
        return cls(
            arr, float(msg.metadata.resolution), int(msg.metadata.size_x), int(msg.metadata.size_y),
            float(msg.metadata.origin.position.x), float(msg.metadata.origin.position.y),
            yaw(msg.metadata.origin.orientation), msg.header.frame_id, bag_ns, ns(msg.header.stamp))

    def world_to_map(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        mx, my = math.floor(lx / self.resolution), math.floor(ly / self.resolution)
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return None
        return int(mx), int(my)

    def map_to_world(self, mx, my):
        lx, ly = (mx + 0.5) * self.resolution, (my + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return self.origin_x + c * lx - s * ly, self.origin_y + s * lx + c * ly

    def cost(self, x, y):
        cell = self.world_to_map(x, y)
        return None if cell is None else int(self.data[cell[1], cell[0]])

    def polygon_boundary(self, polygon):
        vertices = [self.world_to_map(*p) for p in polygon]
        if any(v is None for v in vertices):
            return [], True
        cells = []
        for a, b in zip(vertices, vertices[1:] + vertices[:1]):
            cells.extend(bresenham(a[0], a[1], b[0], b[1]))
        return list(dict.fromkeys(cells)), False

    def footprint_stats(self, pose, polygon=None):
        polygon = transform_polygon(FOOTPRINT, pose) if polygon is None else polygon
        center_cell = self.world_to_map(pose[0], pose[1])
        center = None if center_cell is None else int(self.data[center_cell[1], center_cell[0]])
        cells, outside = self.polygon_boundary(polygon)
        vals = [int(self.data[y, x]) for x, y in cells]
        counts = dict(sorted(collections.Counter(vals).items()))
        raw_max = max((display_to_raw(v) for v in vals), default=None)
        # Match Smac's optimized polygon check. Raw Costmap uses 254 lethal,
        # 253 inscribed and 255 unknown; OccupancyGrid uses 100/99/-1.
        raw_grid = self.data.dtype == np.uint8
        lethal_value = 254 if raw_grid else 100
        inscribed_value = 253 if raw_grid else 99
        unknown_value = 255 if raw_grid else -1
        if center is None:
            collision = True
            mechanism = "center_outside"
        elif center == unknown_value:
            collision = True
            mechanism = "center_unknown"
        elif center >= inscribed_value and center != unknown_value:
            collision = True
            mechanism = "center_inscribed_or_lethal"
        elif display_to_raw(center) >= POSSIBLE_INSCRIBED_RAW:
            collision = outside or any(v in (unknown_value, lethal_value) for v in vals)
            mechanism = "footprint_boundary" if collision else "checked_boundary_clear"
        else:
            collision = False
            mechanism = "center_below_full_check_threshold"
        return {
            "pose": {"x": pose[0], "y": pose[1], "yaw_rad": pose[2]},
            "polygon": polygon.tolist(),
            "center_cell": center_cell,
            "center_cost_display": center,
            "center_cost_raw_upper": display_to_raw(center) if center is not None else None,
            "boundary_cell_count": len(cells),
            "boundary_cost_counts_display": {str(k): v for k, v in counts.items()},
            "boundary_raw_max": raw_max,
            "boundary_outside": outside,
            "smac_collision_reconstructed": collision,
            "smac_collision_mechanism": mechanism,
        }

    def nearest_values(self, pose, values, radius=2.0):
        center = self.world_to_map(pose[0], pose[1])
        if center is None:
            return None
        n = math.ceil(radius / self.resolution)
        x0, x1 = max(0, center[0] - n), min(self.width, center[0] + n + 1)
        y0, y1 = max(0, center[1] - n), min(self.height, center[1] + n + 1)
        sub = self.data[y0:y1, x0:x1]
        mask = np.isin(sub, list(values))
        ys, xs = np.nonzero(mask)
        if not len(xs):
            return None
        wx = self.origin_x + (x0 + xs + 0.5) * self.resolution
        wy = self.origin_y + (y0 + ys + 0.5) * self.resolution
        d = np.hypot(wx - pose[0], wy - pose[1])
        i = int(np.argmin(d))
        dx, dy = float(wx[i] - pose[0]), float(wy[i] - pose[1])
        forward = math.cos(pose[2]) * dx + math.sin(pose[2]) * dy
        left = -math.sin(pose[2]) * dx + math.cos(pose[2]) * dy
        return {
            "distance_m": float(d[i]), "x": float(wx[i]), "y": float(wy[i]),
            "relative_forward_m": forward, "relative_left_m": left,
            "cost_display": int(sub[ys[i], xs[i]]),
        }

    def crop(self, pose, radius=2.0):
        cell = self.world_to_map(pose[0], pose[1])
        n = math.ceil(radius / self.resolution)
        x0, x1 = max(0, cell[0] - n), min(self.width, cell[0] + n + 1)
        y0, y1 = max(0, cell[1] - n), min(self.height, cell[1] + n + 1)
        extent = [
            self.origin_x + x0 * self.resolution,
            self.origin_x + x1 * self.resolution,
            self.origin_y + y0 * self.resolution,
            self.origin_y + y1 * self.resolution,
        ]
        return self.data[y0:y1, x0:x1].copy(), extent


class StaticGrid(Grid):
    pass


def load_static_grid():
    meta = yaml.safe_load(MAP_YAML.read_text())
    gray = np.asarray(Image.open(MAP_PGM), dtype=np.uint8)
    occ = np.full(gray.shape, -1, dtype=np.int8)
    probability = (255.0 - gray.astype(float)) / 255.0 if not meta["negate"] else gray.astype(float) / 255.0
    occ[probability > meta["occupied_thresh"]] = 100
    occ[probability < meta["free_thresh"]] = 0
    # ROS map y=0 is the bottom image row.
    occ = np.flipud(occ).copy()
    return StaticGrid(
        occ, float(meta["resolution"]), int(gray.shape[1]), int(gray.shape[0]),
        float(meta["origin"][0]), float(meta["origin"][1]), float(meta["origin"][2]), "map")


MSG_CLASSES = {}


def topic_rows(bag_dir, topic, lo, hi):
    rows = []
    typ = None
    for db in sorted(bag_dir.glob("*.db3")):
        con = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
        con.execute("PRAGMA query_only=ON")
        info = con.execute("SELECT id,type FROM topics WHERE name=?", (topic,)).fetchone()
        if info:
            topic_id, typ = info
            rows.extend(con.execute(
                "SELECT timestamp,data FROM messages WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp,id",
                (topic_id, lo, hi)).fetchall())
        con.close()
    rows.sort(key=lambda r: r[0])
    return typ, rows


def decode(typ, blob):
    if typ not in MSG_CLASSES:
        MSG_CLASSES[typ] = get_message(typ)
    return deserialize_message(blob, MSG_CLASSES[typ])


def nearest_tf(bag_dir, event_ns):
    typ, rows = topic_rows(bag_dir, "/tf", event_ns - 1_000_000_000, event_ns + 1_000_000_000)
    found = collections.defaultdict(list)
    for bag_ns, blob in rows:
        msg = decode(typ, blob)
        for tr in msg.transforms:
            key = (tr.header.frame_id, tr.child_frame_id)
            if key in [("map", "odom"), ("odom", "base_link")]:
                p, q = tr.transform.translation, tr.transform.rotation
                found[key].append({
                    "bag_ns": bag_ns, "header_ns": ns(tr.header.stamp),
                    "pose": (float(p.x), float(p.y), yaw(q)),
                })
    result = {}
    for key in [("map", "odom"), ("odom", "base_link")]:
        result[key] = min(found[key], key=lambda z: abs(z["bag_ns"] - event_ns))
    map_base = compose(result[("map", "odom")]["pose"], result[("odom", "base_link")]["pose"])
    return result, map_base


def quantized_smac_pose(grid, map_pose):
    cell = grid.world_to_map(map_pose[0], map_pose[1])
    x, y = grid.map_to_world(*cell)
    angle_bin = round(map_pose[2] / ANGLE_STEP) % ANGLE_BINS
    return (x, y, angle_bin * ANGLE_STEP), angle_bin, cell


def plan_analysis(bag_dir, event_ns, actual_pose, smac_pose):
    typ, rows = topic_rows(bag_dir, "/plan", event_ns - 30_000_000_000, event_ns)
    if not rows:
        return None, None
    bag_ns, blob = rows[-1]
    msg = decode(typ, blob)
    pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses], float)
    ds = np.hypot(pts[:, 0] - actual_pose[0], pts[:, 1] - actual_pose[1])
    i = int(np.argmin(ds))
    start_d = math.hypot(pts[0, 0] - actual_pose[0], pts[0, 1] - actual_pose[1])
    smac_d = np.hypot(pts[:, 0] - smac_pose[0], pts[:, 1] - smac_pose[1])
    out = {
        "bag_time": iso(bag_ns), "event_offset_s": (bag_ns - event_ns) / 1e9,
        "header_time": iso(ns(msg.header.stamp)), "frame": msg.header.frame_id,
        "pose_count": len(msg.poses),
        "start": pts[0].tolist(), "end": pts[-1].tolist(),
        "actual_pose_to_plan_start_m": start_d,
        "actual_pose_nearest_index": i,
        "actual_pose_cross_track_m": float(ds[i]),
        "nearest_point": pts[i].tolist(),
        "smac_pose_cross_track_m": float(smac_d.min()),
    }
    return out, pts


def nearest_scan_analysis(bag_dir, event_ns, map_pose, target_lethal):
    typ, rows = topic_rows(bag_dir, "/laser/scan_merge_laser", event_ns - 50_000_000, event_ns + 50_000_000)
    if not rows:
        return None
    bag_ns, blob = min(rows, key=lambda z: abs(z[0] - event_ns))
    msg = decode(typ, blob)
    finite_count = 0
    close = []
    for i, value in enumerate(msg.ranges):
        r = float(value)
        if not math.isfinite(r):
            continue
        finite_count += 1
        if r >= 1.0:
            continue
        angle = float(msg.angle_min + i * msg.angle_increment)
        x, y = r * math.cos(angle), r * math.sin(angle)
        outside_dx = max(abs(x) - 0.45, 0.0)
        outside_dy = max(abs(y) - 0.30, 0.0)
        outside_gap = math.hypot(outside_dx, outside_dy)
        inside = outside_gap == 0.0
        wx = map_pose[0] + math.cos(map_pose[2]) * x - math.sin(map_pose[2]) * y
        wy = map_pose[1] + math.sin(map_pose[2]) * x + math.cos(map_pose[2]) * y
        target_distance = None
        if target_lethal:
            target_distance = math.hypot(wx - target_lethal["x"], wy - target_lethal["y"])
        close.append({
            "index": i, "range_m": r, "angle_rad": angle,
            "base_x_forward_m": x, "base_y_left_m": y,
            "inside_effective_footprint": inside,
            "outside_footprint_gap_m": outside_gap,
            "map_x": wx, "map_y": wy,
            "distance_to_primary_global_nearest_lethal_m": target_distance,
        })
    outside = [p for p in close if not p["inside_effective_footprint"]]
    near_target = [p for p in close if p["distance_to_primary_global_nearest_lethal_m"] is not None]
    return {
        "bag_time": iso(bag_ns), "header_time": iso(ns(msg.header.stamp)),
        "bag_event_offset_s": (bag_ns - event_ns) / 1e9,
        "header_event_offset_s": (ns(msg.header.stamp) - event_ns) / 1e9,
        "frame": msg.header.frame_id, "beam_count": len(msg.ranges),
        "finite_count": finite_count, "range_lt_1m_count": len(close),
        "range_lt_1m_inside_effective_footprint_count": sum(p["inside_effective_footprint"] for p in close),
        "range_lt_1m_outside_effective_footprint_count": len(outside),
        "minimum_finite_range_m": min((float(v) for v in msg.ranges if math.isfinite(v)), default=None),
        "closest_outside_return_to_footprint": min(outside, key=lambda p: p["outside_footprint_gap_m"]) if outside else None,
        "closest_lt_1m_return_to_primary_global_nearest_lethal": min(near_target, key=lambda p: p["distance_to_primary_global_nearest_lethal_m"]) if near_target else None,
        "range_lt_1m_returns": close,
    }


def static_footprint_stats(grid, pose):
    polygon = transform_polygon(FOOTPRINT, pose)
    cell = grid.world_to_map(pose[0], pose[1])
    center = grid.cost(pose[0], pose[1])
    cells, outside = grid.polygon_boundary(polygon)
    vals = [int(grid.data[y, x]) for x, y in cells]
    counts = dict(sorted(collections.Counter(vals).items()))
    colliding = center in (-1, 100) or outside or any(v in (-1, 100) for v in vals)
    return {
        "pose": {"x": pose[0], "y": pose[1], "yaw_rad": pose[2]},
        "polygon": polygon.tolist(), "center_cell": cell, "center_occupancy": center,
        "boundary_cell_count": len(cells),
        "boundary_occupancy_counts": {str(k): v for k, v in counts.items()},
        "geometric_collision_if_unknown_disallowed": colliding,
        "nearest_static_occupied": grid.nearest_values(pose, {100}),
        "nearest_static_unknown": grid.nearest_values(pose, {-1}),
    }


def load_costmap_frames(bag_dir, topic, event_ns, window_ns):
    typ, rows = topic_rows(bag_dir, topic, event_ns - window_ns, event_ns + window_ns)
    frames = []
    for bag_ns, blob in rows:
        msg = decode(typ, blob)
        frames.append(Grid.from_msg(msg, bag_ns))
    return frames


def frame_summary(grid, event_ns, pose):
    f = grid.footprint_stats(pose)
    f.update({
        "bag_time": iso(grid.bag_ns), "header_time": iso(grid.header_ns),
        "bag_event_offset_s": (grid.bag_ns - event_ns) / 1e9,
        "header_event_offset_s": (grid.header_ns - event_ns) / 1e9,
        "resolution": grid.resolution, "width": grid.width, "height": grid.height,
        "origin": [grid.origin_x, grid.origin_y, grid.origin_yaw], "frame": grid.frame,
        "nearest_lethal": grid.nearest_values(pose, {254, 100}),
        "nearest_inscribed_or_lethal": grid.nearest_values(pose, {253, 254, 99, 100}),
        "nearest_unknown": grid.nearest_values(pose, {255, -1}),
    })
    return f


def polygon_static_overlap(global_grid, static_grid, smac_pose):
    polygon = transform_polygon(FOOTPRINT, smac_pose)
    cells, _ = global_grid.polygon_boundary(polygon)
    results = collections.Counter()
    detailed = []
    for mx, my in cells:
        gv = int(global_grid.data[my, mx])
        wx, wy = global_grid.map_to_world(mx, my)
        sv = static_grid.cost(wx, wy)
        results[(gv, sv)] += 1
        if gv in (-1, 100, 254, 255):
            detailed.append({"x": wx, "y": wy, "global": gv, "static": sv})
    return {
        "boundary_global_static_pairs": {f"global={g},static={s}": n for (g, s), n in sorted(results.items())},
        "global_lethal_or_unknown_boundary_cells": detailed,
    }


def plot_run(axs, label, static_grid, global_grid, local_grid, map_pose, odom_pose, smac_pose, plan_pts):
    cmap = ListedColormap(["#888888", "#ffffff", "#fff0a8", "#f39c5a", "#d7191c", "#5b1010"])
    norm = BoundaryNorm([-1.5, -0.5, 0.5, 49.5, 98.5, 99.5, 100.5], cmap.N)
    entries = [(static_grid, smac_pose, "static map"), (global_grid, smac_pose, "global costmap"), (local_grid, odom_pose, "local costmap")]
    for ax, (grid, pose, title) in zip(axs, entries):
        arr, extent = grid.crop(pose, 2.0)
        ax.imshow(arr, origin="lower", extent=extent, cmap=cmap, norm=norm, interpolation="nearest")
        poly = transform_polygon(FOOTPRINT, pose)
        poly = np.vstack([poly, poly[0]])
        ax.plot(poly[:, 0], poly[:, 1], color="#00bcd4", lw=2.0, label="padded footprint")
        ax.plot(pose[0], pose[1], "o", color="#1565c0", ms=4)
        if title != "local costmap" and plan_pts is not None:
            use = (np.abs(plan_pts[:, 0] - pose[0]) < 2.2) & (np.abs(plan_pts[:, 1] - pose[1]) < 2.2)
            ax.plot(plan_pts[use, 0], plan_pts[use, 1], color="#7b1fa2", lw=1.5, label="last successful plan")
        ax.set_title(f"{label} {title}")
        ax.set_aspect("equal")
        ax.set_xlim(pose[0] - 2, pose[0] + 2)
        ax.set_ylim(pose[1] - 2, pose[1] + 2)
        ax.grid(alpha=0.2)
        ax.set_xlabel(f"{grid.frame} x (m)")
        ax.set_ylabel(f"{grid.frame} y (m)")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    static_grid = load_static_grid()
    report = {
        "created_at": dt.datetime.now(TZ).isoformat(),
        "method": "read-only sqlite rosbag2 deserialization; no playback or ROS graph",
        "static_map": {
            "yaml": str(MAP_YAML), "pgm": str(MAP_PGM),
            "resolution": static_grid.resolution, "width": static_grid.width,
            "height": static_grid.height, "origin": [static_grid.origin_x, static_grid.origin_y, static_grid.origin_yaw],
        },
        "footprint": {
            "configured_unpadded_m": [[-0.43, -0.28], [-0.43, 0.28], [0.43, 0.28], [0.43, -0.28]],
            "padding_m": 0.02, "effective_padded_m": FOOTPRINT.tolist(),
            "smac_angle_bins": ANGLE_BINS,
            "circumscribed_radius_m": math.hypot(0.45, 0.30),
            "inscribed_radius_m": 0.30,
            "possible_inscribed_raw_threshold": POSSIBLE_INSCRIBED_RAW,
            "possible_inscribed_display_threshold": POSSIBLE_INSCRIBED_DISPLAY,
        },
        "runs": [],
        "limitations": [
            "Full costmap topics are snapshots (global 1 Hz, local 20 Hz); the planner checks the in-memory global map at 5 Hz.",
            "The exact in-memory grid at the log nanosecond is not recorded. Bracketing global snapshots are retained.",
            "TF is reconstructed from nearest bag-arrival samples. odom->base_link header stamps are about 0.3 s ahead of bag arrival in these runs.",
            "The static-map comparison uses the unchanged map file whose mtime predates all three tests.",
        ],
    }
    fig, axes = plt.subplots(1, 3, figsize=(15, 5), constrained_layout=True)
    axes = np.array([axes])
    for row_i, run in enumerate(RUNS):
        event_ns = run["event_ns"]
        bag_dir = run["run"] / "navigation_core_bag"
        tf, map_pose = nearest_tf(bag_dir, event_ns)
        odom_pose = tf[("odom", "base_link")]["pose"]
        global_frames = load_costmap_frames(bag_dir, "/global_costmap/costmap_raw", event_ns, 2_000_000_000)
        local_frames = load_costmap_frames(bag_dir, "/local_costmap/costmap_raw", event_ns, 400_000_000)
        primary_global = min(global_frames, key=lambda g: abs(g.header_ns - event_ns))
        primary_local = min(local_frames, key=lambda g: abs(g.header_ns - event_ns))
        smac_pose, angle_bin, smac_cell = quantized_smac_pose(primary_global, map_pose)
        plan_info, plan_pts = plan_analysis(bag_dir, event_ns, map_pose, smac_pose)
        g_summaries = [frame_summary(g, event_ns, smac_pose) for g in global_frames]
        l_summaries = [frame_summary(g, event_ns, odom_pose) for g in local_frames]
        static_stats = static_footprint_stats(static_grid, smac_pose)
        overlap = polygon_static_overlap(primary_global, static_grid, smac_pose)
        primary_global_index = next(i for i, g in enumerate(global_frames) if g is primary_global)
        primary_local_index = next(i for i, g in enumerate(local_frames) if g is primary_local)
        primary_global_stats = g_summaries[primary_global_index]
        scan_info = nearest_scan_analysis(
            bag_dir, event_ns, map_pose, primary_global_stats["nearest_lethal"])
        run_out = {
            "label": run["label"], "run_dir": str(run["run"]),
            "nav_log": str(run["run"] / run["nav_log"]),
            "first_lethal_event_ns": event_ns, "first_lethal_event": iso(event_ns),
            "tf": {
                "map_to_odom": {**tf[("map", "odom")], "pose": list(tf[("map", "odom")]["pose"]), "bag_time": iso(tf[("map", "odom")]["bag_ns"]), "header_time": iso(tf[("map", "odom")]["header_ns"])},
                "odom_to_base_link": {**tf[("odom", "base_link")], "pose": list(tf[("odom", "base_link")]["pose"]), "bag_time": iso(tf[("odom", "base_link")]["bag_ns"]), "header_time": iso(tf[("odom", "base_link")]["header_ns"])},
            },
            "actual_tf_pose_map": {"x": map_pose[0], "y": map_pose[1], "yaw_rad": map_pose[2]},
            "actual_tf_pose_odom": {"x": odom_pose[0], "y": odom_pose[1], "yaw_rad": odom_pose[2]},
            "smac_discretized_start": {
                "x": smac_pose[0], "y": smac_pose[1], "yaw_rad": smac_pose[2],
                "map_cell": smac_cell, "angle_bin": angle_bin,
                "translation_from_actual_m": math.hypot(smac_pose[0] - map_pose[0], smac_pose[1] - map_pose[1]),
                "yaw_from_actual_rad": wrap(smac_pose[2] - map_pose[2]),
            },
            "static_map_at_smac_start": static_stats,
            "global_costmap_primary_index": primary_global_index,
            "global_costmap_frames": g_summaries,
            "local_costmap_primary_index": primary_local_index,
            "local_costmap_frames": l_summaries,
            "primary_global_static_overlap": overlap,
            "last_successful_plan_before_failure": plan_info,
            "nearest_merged_scan": scan_info,
        }
        report["runs"].append(run_out)
        plot_run(axes[row_i], run["label"], static_grid, primary_global, primary_local, map_pose, odom_pose, smac_pose, plan_pts)
        # Release large frames once crops and summaries are retained.
        del global_frames, local_frames
    fig.suptitle("2026-11-14 11:13 first lethal-start raw costmap audit", fontsize=16)
    fig.savefig(OUT / "first_lethal_raw_costmap.png", dpi=180)
    plt.close(fig)
    (OUT / "first_lethal_raw_analysis.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
