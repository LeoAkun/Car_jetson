#!/usr/bin/env python3
"""Plot the recorded 11:16:38 odom/TF discontinuity against the static map.

The bag is opened read-only.  The script does not create ROS nodes, publish,
play a bag, or access vehicle hardware.
"""
from __future__ import annotations

import datetime as dt
import glob
import json
import math
import sqlite3
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from matplotlib.collections import LineCollection
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ROOT = Path("/home/akun/workspace/Car_jetson")
OUT = ROOT / "navigation_monitoring/diagnostics/2026-11-14_costmap_failures"
BAG = (
    ROOT
    / "navigation_monitoring/runs/2026-11-14_11-12-06_manual_publish_monitor/navigation_core_bag"
)
MAP_PGM = ROOT / "multi_map_navigation/src/map/3b2mus/3b2mus_clean.pgm"
MAP_ORIGIN = (-92.9, -532.0)
MAP_RESOLUTION = 0.1
TZ = dt.timezone(dt.timedelta(hours=8))

WINDOW_START = dt.datetime(2026, 11, 14, 11, 16, 37, tzinfo=TZ).timestamp()
WINDOW_END = dt.datetime(2026, 11, 14, 11, 16, 42, tzinfo=TZ).timestamp()
FIRST_DENSE_LETHAL = 1794626199.073431296
FOOTPRINT = np.array(
    [[-0.45, -0.30], [-0.45, 0.30], [0.45, 0.30], [0.45, -0.30], [-0.45, -0.30]],
    dtype=float,
)


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def compose(a, b):
    return (
        a[0] + math.cos(a[2]) * b[0] - math.sin(a[2]) * b[1],
        a[1] + math.sin(a[2]) * b[0] + math.cos(a[2]) * b[1],
        math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])),
    )


def transform_polygon(poly, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    rotation = np.array([[c, -s], [s, c]])
    return poly @ rotation.T + np.array(pose[:2])


def topic_rows(topic: str):
    lo = int(WINDOW_START * 1e9)
    hi = int(WINDOW_END * 1e9)
    result = []
    message_type = None
    for db in sorted(glob.glob(str(BAG / "*.db3"))):
        connection = sqlite3.connect(f"file:{db}?mode=ro&immutable=1", uri=True)
        connection.execute("PRAGMA query_only=ON")
        metadata = connection.execute(
            "SELECT id, type FROM topics WHERE name=?", (topic,)
        ).fetchone()
        if metadata:
            topic_id, message_type = metadata
            result.extend(
                connection.execute(
                    "SELECT timestamp, data FROM messages "
                    "WHERE topic_id=? AND timestamp BETWEEN ? AND ? ORDER BY timestamp, id",
                    (topic_id, lo, hi),
                ).fetchall()
            )
        connection.close()
    result.sort(key=lambda item: item[0])
    return message_type, result


def load_messages(topic: str):
    message_type, rows = topic_rows(topic)
    cls = get_message(message_type)
    return [(bag_ns / 1e9, deserialize_message(blob, cls)) for bag_ns, blob in rows]


def load_tf():
    records = []
    map_odom = None
    for bag_time, message in load_messages("/tf"):
        for transform in message.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            p, q = transform.transform.translation, transform.transform.rotation
            pose = (float(p.x), float(p.y), yaw(q))
            if key == ("map", "odom"):
                map_odom = pose
            elif key == ("odom", "base_link"):
                records.append(
                    {
                        "bag_time": bag_time,
                        "header_time": stamp_seconds(transform.header.stamp),
                        "odom_pose": pose,
                    }
                )
    if map_odom is None:
        raise RuntimeError("map->odom was not recorded in the selected window")
    for record in records:
        record["map_pose"] = compose(map_odom, record["odom_pose"])
    return map_odom, records


def load_mapping_odom():
    records = []
    for bag_time, message in load_messages("/lio_sam/mapping/odometry"):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        records.append(
            {
                "bag_time": bag_time,
                "header_time": stamp_seconds(message.header.stamp),
                "pose": (float(p.x), float(p.y), yaw(q)),
            }
        )
    return records


def static_map_crop(xlim, ylim):
    image = np.asarray(Image.open(MAP_PGM), dtype=np.uint8)
    mx0 = max(0, int(math.floor((xlim[0] - MAP_ORIGIN[0]) / MAP_RESOLUTION)))
    mx1 = min(image.shape[1], int(math.ceil((xlim[1] - MAP_ORIGIN[0]) / MAP_RESOLUTION)))
    my0 = max(0, int(math.floor((ylim[0] - MAP_ORIGIN[1]) / MAP_RESOLUTION)))
    my1 = min(image.shape[0], int(math.ceil((ylim[1] - MAP_ORIGIN[1]) / MAP_RESOLUTION)))
    # PGM row 0 is the top of the image; ROS map cell y=0 is the bottom row.
    ros_order = np.flipud(image)
    crop = ros_order[my0:my1, mx0:mx1]
    extent = (
        MAP_ORIGIN[0] + mx0 * MAP_RESOLUTION,
        MAP_ORIGIN[0] + mx1 * MAP_RESOLUTION,
        MAP_ORIGIN[1] + my0 * MAP_RESOLUTION,
        MAP_ORIGIN[1] + my1 * MAP_RESOLUTION,
    )
    return crop, extent


def nearest_before(records, target):
    candidates = [record for record in records if record["bag_time"] <= target]
    return max(candidates, key=lambda record: record["bag_time"])


def main():
    map_odom, tf_records = load_tf()
    mapping = load_mapping_odom()

    jumps = []
    for previous, current in zip(tf_records, tf_records[1:]):
        dt_s = current["header_time"] - previous["header_time"]
        distance = math.hypot(
            current["odom_pose"][0] - previous["odom_pose"][0],
            current["odom_pose"][1] - previous["odom_pose"][1],
        )
        current["step_distance_m"] = distance
        current["step_dt_s"] = dt_s
        if dt_s > 0 and distance >= 0.20:
            jumps.append(
                {
                    "bag_time": current["bag_time"],
                    "header_time": current["header_time"],
                    "dt_s": dt_s,
                    "distance_m": distance,
                    "apparent_speed_mps": distance / dt_s,
                    "before_odom_pose": previous["odom_pose"],
                    "after_odom_pose": current["odom_pose"],
                    "before_map_pose": previous["map_pose"],
                    "after_map_pose": current["map_pose"],
                }
            )

    major = max(jumps, key=lambda item: item["distance_m"])
    before = nearest_before(tf_records, major["bag_time"] - 1e-6)
    after = nearest_before(tf_records, major["bag_time"] + 1e-6)
    at_lethal = nearest_before(tf_records, FIRST_DENSE_LETHAL)

    relative = lambda value: value - WINDOW_START
    odom_time = np.array([relative(record["bag_time"]) for record in tf_records])
    odom_x = np.array([record["odom_pose"][0] for record in tf_records])
    odom_y = np.array([record["odom_pose"][1] for record in tf_records])
    map_xy = np.array([record["map_pose"][:2] for record in tf_records])
    mapping_time = np.array([relative(record["bag_time"]) for record in mapping])
    mapping_x = np.array([record["pose"][0] for record in mapping])
    mapping_y = np.array([record["pose"][1] for record in mapping])

    step_time, step_distance = [], []
    for record in tf_records[1:]:
        step_time.append(relative(record["bag_time"]))
        step_distance.append(record.get("step_distance_m", np.nan))

    figure, axes = plt.subplots(1, 3, figsize=(18, 6), constrained_layout=True)

    # Static-map view.
    axis = axes[0]
    xlim, ylim = (149.9, 152.2), (-146.8, -144.2)
    crop, extent = static_map_crop(xlim, ylim)
    axis.imshow(
        crop,
        origin="lower",
        extent=extent,
        cmap="gray",
        vmin=0,
        vmax=255,
        interpolation="nearest",
    )
    points = map_xy.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    collection = LineCollection(
        segments,
        cmap="viridis",
        norm=plt.Normalize(odom_time.min(), odom_time.max()),
        linewidth=2.2,
    )
    collection.set_array(odom_time[:-1])
    axis.add_collection(collection)
    for record, color, label in [
        (before, "#1f77b4", "before 1.19 m jump"),
        (after, "#d62728", "after 10 ms"),
        (at_lethal, "#ff7f0e", "first dense lethal"),
    ]:
        polygon = transform_polygon(FOOTPRINT, record["map_pose"])
        axis.plot(polygon[:, 0], polygon[:, 1], color=color, lw=2.1, label=label)
        axis.scatter(record["map_pose"][0], record["map_pose"][1], color=color, s=35, zorder=4)
    axis.set(xlim=xlim, ylim=ylim, xlabel="map x (m)", ylabel="map y (m)")
    axis.set_title("Recorded TF over the prebuilt static map")
    axis.legend(loc="lower left", fontsize=8)
    axis.grid(alpha=0.2)

    # Fused /odom versus the smooth mapping corrections.
    axis = axes[1]
    axis.plot(odom_time, odom_x, label="/odom x (100 Hz)", lw=1.5, color="#1f77b4")
    axis.plot(odom_time, odom_y, label="/odom y (100 Hz)", lw=1.5, color="#ff7f0e")
    axis.scatter(mapping_time, mapping_x, label="mapping odom x", s=24, marker="x", color="#2ca02c")
    axis.scatter(mapping_time, mapping_y, label="mapping odom y", s=24, marker="x", color="#9467bd")
    axis.axvline(relative(major["bag_time"]), color="#d62728", ls="--", lw=1.4, label="1.19 m jump")
    axis.axvline(relative(FIRST_DENSE_LETHAL), color="black", ls=":", lw=1.4, label="first dense lethal")
    axis.set(xlabel="seconds after 11:16:37", ylabel="odom coordinate (m)")
    axis.set_title("Fused odometry discontinuities")
    axis.legend(fontsize=8, ncol=2)
    axis.grid(alpha=0.25)

    # Per-message displacement makes the impossible instantaneous jumps explicit.
    axis = axes[2]
    axis.plot(step_time, step_distance, color="#4c78a8", lw=1.1)
    axis.scatter(
        [relative(item["bag_time"]) for item in jumps],
        [item["distance_m"] for item in jumps],
        color="#d62728",
        s=30,
        label="step >= 0.20 m",
        zorder=3,
    )
    axis.axvline(relative(FIRST_DENSE_LETHAL), color="black", ls=":", lw=1.4, label="first dense lethal")
    for record in mapping:
        axis.axvline(relative(record["bag_time"]), color="#2ca02c", alpha=0.13, lw=1)
    axis.set(xlabel="seconds after 11:16:37", ylabel="TF translation per message (m)")
    axis.set_title("10 ms TF steps; green lines are mapping updates")
    axis.set_ylim(bottom=0)
    axis.legend(fontsize=8)
    axis.grid(alpha=0.25)

    figure.suptitle(
        "2026-11-14 11:13 run: runtime /odom jump precedes persistent lethal-start",
        fontsize=16,
    )
    figure.savefig(OUT / "odom_jump_1113.png", dpi=180)
    plt.close(figure)

    evidence = {
        "method": "read-only rosbag2 deserialization; times use bag arrival unless explicitly named header_time",
        "bag": str(BAG),
        "window_local": ["2026-11-14T11:16:37+08:00", "2026-11-14T11:16:42+08:00"],
        "map_to_odom_constant_pose": map_odom,
        "first_dense_lethal_bag_time": FIRST_DENSE_LETHAL,
        "major_jump": major,
        "jump_count_ge_0_20m": len(jumps),
        "jumps_ge_0_20m": jumps,
        "nearest_tf_before_first_dense_lethal": at_lethal,
        "note": (
            "The 1.19 m / 10 ms displacement cannot be physical vehicle motion. "
            "It is a discontinuity in the published odom->base_link estimate."
        ),
    }
    (OUT / "odom_jump_1113.json").write_text(
        json.dumps(evidence, ensure_ascii=False, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
