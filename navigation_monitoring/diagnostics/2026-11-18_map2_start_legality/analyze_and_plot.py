#!/usr/bin/env python3
"""Reproduce the map2 grid and the 11:53 task start/first-lethal geometry."""

import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from PIL import Image
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.ndimage import distance_transform_edt


ROOT = Path(__file__).resolve().parents[3]
BAG = ROOT / "navigation_monitoring/runs/2026-11-18_11-25-25_manual_publish_monitor/navigation_core_bag_segment_3"
MAP = ROOT / "multi_map_navigation/src/map/map2/map2_clean.pgm"
OUT = Path(__file__).resolve().parent
ORIGIN = (-395.0, -148.0)
RESOLUTION = 0.1
GOAL_ACCEPTED = 1794974050.011
FIRST_LETHAL = 1794974081.670
TASK_WAYPOINTS = [
    (15, "Library junction 3", -113.0, 189.0),
    (16, "Library junction 2", -114.0, 297.0),
    (17, "Library junction", -187.0, 366.0),
    (18, "Building 8 junction", -223.0, 418.0),
    (19, "Building 8 boundary 3", -293.0, 493.0),
]


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def extract_bag_geometry():
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(BAG), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    topics = ["/plan", "/navigate_to_pose/_action/feedback"]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    classes = {topic: get_message(types[topic]) for topic in topics}

    first_plan = None
    feedback = []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        stamp = stamp_ns / 1e9
        if stamp < GOAL_ACCEPTED - 0.1:
            continue
        if stamp > FIRST_LETHAL + 0.1:
            break
        message = deserialize_message(data, classes[topic])
        if topic == "/plan" and first_plan is None:
            first_plan = np.asarray(
                [
                    [pose.pose.position.x, pose.pose.position.y]
                    for pose in message.poses
                ]
            )
        elif topic.endswith("/feedback"):
            pose = message.feedback.current_pose.pose
            feedback.append(
                [stamp, pose.position.x, pose.position.y, yaw(pose.orientation)]
            )

    feedback = np.asarray(feedback)
    if first_plan is None or feedback.size == 0:
        raise RuntimeError("11:53 plan or feedback is missing")
    start = feedback[np.argmin(np.abs(feedback[:, 0] - GOAL_ACCEPTED))]
    lethal = feedback[np.argmin(np.abs(feedback[:, 0] - FIRST_LETHAL))]
    return first_plan, feedback, start, lethal


def map_clearance(image, x, y):
    free = image != 0
    clearance = distance_transform_edt(free) * RESOLUTION
    column = int(math.floor((x - ORIGIN[0]) / RESOLUTION))
    map_row = int(math.floor((y - ORIGIN[1]) / RESOLUTION))
    image_row = image.shape[0] - 1 - map_row
    return float(clearance[image_row, column])


def add_vehicle_arrow(axis, pose, color, label):
    x, y, heading = pose[1], pose[2], pose[3]
    axis.scatter([x], [y], s=55, color=color, zorder=8, label=label)
    axis.arrow(
        x,
        y,
        3.0 * math.cos(heading),
        3.0 * math.sin(heading),
        width=0.18,
        head_width=1.2,
        color=color,
        length_includes_head=True,
        zorder=8,
    )


def draw_map(axis, image):
    height, width = image.shape
    axis.imshow(
        image,
        cmap="gray",
        vmin=0,
        vmax=254,
        origin="upper",
        extent=[
            ORIGIN[0],
            ORIGIN[0] + width * RESOLUTION,
            ORIGIN[1],
            ORIGIN[1] + height * RESOLUTION,
        ],
        interpolation="nearest",
        rasterized=True,
    )
    axis.set_aspect("equal")
    axis.set_xlabel("map x (m)")
    axis.set_ylabel("map y (m)")
    axis.grid(color="#777777", alpha=0.20, linewidth=0.5)


def main():
    image = np.asarray(Image.open(MAP))
    first_plan, feedback, start, lethal = extract_bag_geometry()
    trajectory = feedback[:, 1:3]
    route = np.asarray([[point[2], point[3]] for point in TASK_WAYPOINTS])

    target_direction = math.atan2(route[0, 1] - start[2], route[0, 0] - start[1])
    relative_target = math.atan2(
        math.sin(target_direction - start[3]),
        math.cos(target_direction - start[3]),
    )
    summary = {
        "map": {
            "width_cells": int(image.shape[1]),
            "height_cells": int(image.shape[0]),
            "resolution_m": RESOLUTION,
            "origin": list(ORIGIN),
            "extent_m": [
                ORIGIN[0],
                ORIGIN[0] + image.shape[1] * RESOLUTION,
                ORIGIN[1],
                ORIGIN[1] + image.shape[0] * RESOLUTION,
            ],
        },
        "task": {
            "received_at": "2026-11-18T11:53:23.300+08:00",
            "goal_accepted_at": "2026-11-18T11:54:10.011+08:00",
            "first_lethal_at": "2026-11-18T11:54:41.670+08:00",
            "waypoint_ids": [point[0] for point in TASK_WAYPOINTS],
        },
        "start_pose": {
            "x": float(start[1]),
            "y": float(start[2]),
            "yaw_deg": math.degrees(float(start[3])),
            "static_map_center_clearance_m": map_clearance(
                image, float(start[1]), float(start[2])
            ),
        },
        "first_goal": {
            "x": float(route[0, 0]),
            "y": float(route[0, 1]),
            "bearing_from_start_deg": math.degrees(target_direction),
            "bearing_relative_to_vehicle_deg": math.degrees(relative_target),
        },
        "first_lethal_pose": {
            "x": float(lethal[1]),
            "y": float(lethal[2]),
            "yaw_deg": math.degrees(float(lethal[3])),
            "straight_line_displacement_from_start_m": float(
                np.linalg.norm(lethal[1:3] - start[1:3])
            ),
            "static_map_center_clearance_m": map_clearance(
                image, float(lethal[1]), float(lethal[2])
            ),
        },
        "first_plan_points": int(first_plan.shape[0]),
    }
    (OUT / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    fig, axes = plt.subplots(1, 2, figsize=(17, 10), constrained_layout=True)
    draw_map(axes[0], image)
    axes[0].plot(route[:, 0], route[:, 1], "o--", color="#ff8c00", lw=2, ms=5)
    add_vehicle_arrow(axes[0], start, "#00a6a6", "vehicle at goal acceptance")
    axes[0].set_title("map2_clean: complete 2D occupancy grid\nwhite=free, black=occupied")
    axes[0].legend(loc="lower right", fontsize=8)

    draw_map(axes[1], image)
    axes[1].plot(route[:, 0], route[:, 1], "o--", color="#ff8c00", lw=2, ms=6, label="task route 15-19")
    axes[1].plot(first_plan[:, 0], first_plan[:, 1], color="#1686c2", lw=2.2, label="first Nav2 plan to waypoint 15")
    axes[1].plot(trajectory[:, 0], trajectory[:, 1], color="#d62728", lw=2.0, label="reported vehicle pose")
    add_vehicle_arrow(axes[1], start, "#00a6a6", "vehicle start pose")
    axes[1].scatter([lethal[1]], [lethal[2]], marker="X", s=90, color="#d000d0", zorder=9, label="first lethal report")
    for point in TASK_WAYPOINTS:
        axes[1].annotate(str(point[0]), (point[2], point[3]), xytext=(5, 4), textcoords="offset points", fontsize=8)
    axes[1].set_xlim(-315, -90)
    axes[1].set_ylim(65, 510)
    axes[1].set_title("11:53 task on the carved/free corridor")
    axes[1].legend(loc="lower right", fontsize=8)

    inset = axes[1].inset_axes([0.05, 0.48, 0.42, 0.46])
    draw_map(inset, image)
    inset.plot(first_plan[:, 0], first_plan[:, 1], color="#1686c2", lw=1.6)
    inset.plot(trajectory[:, 0], trajectory[:, 1], color="#d62728", lw=1.6)
    add_vehicle_arrow(inset, start, "#00a6a6", "start")
    inset.scatter([lethal[1]], [lethal[2]], marker="X", s=55, color="#d000d0", zorder=9)
    inset.set_xlim(-210, -202)
    inset.set_ylim(77, 95)
    inset.set_title("start and first lethal", fontsize=8)
    inset.tick_params(labelsize=6)
    inset.set_xlabel("")
    inset.set_ylabel("")

    fig.suptitle("map2 grid and 2026-11-18 11:53 navigation task", fontsize=16)
    fig.savefig(OUT / "map2_grid_and_1153_route.png", dpi=180)
    plt.close(fig)

    # A clean map-only PNG is also provided for direct inspection.
    full = Image.fromarray(image)
    lanczos = getattr(getattr(Image, "Resampling", Image), "LANCZOS")
    full.thumbnail((1800, 1800), lanczos)
    full.save(OUT / "map2_clean_grid_full.png")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
