#!/usr/bin/env python3

# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Measure the Stage 3 camera and AprilTag ROS topics."""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import statistics
import subprocess
import time
from pathlib import Path

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from charging_dock_msgs.msg import DockTagObservationArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


TOPICS = {
    "image_raw": ("/rear_camera/image_raw", Image),
    "image_rect": ("/rear_camera/image_rect", Image),
    "camera_info": ("/rear_camera/camera_info", CameraInfo),
    "detections": ("/apriltag/detections", AprilTagDetectionArray),
    "observations": (
        "/charging_dock/tag_observations",
        DockTagObservationArray,
    ),
}


def stamp_seconds(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def package_version(package):
    try:
        result = subprocess.run(
            ["dpkg-query", "-W", "-f=${Version}", package],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return None
    return result.stdout.strip() if result.returncode == 0 else None


class Probe(Node):
    """Collect topic samples without altering the running graph."""

    def __init__(self):
        super().__init__("stage3_camera_check")
        self.samples = {
            name: {"ages": [], "periods": [], "non_empty": 0, "count": 0}
            for name in TOPICS
        }
        self._last_receive = {}
        for name, (topic, message_type) in TOPICS.items():
            self.create_subscription(
                message_type,
                topic,
                lambda message, key=name: self._on_message(key, message),
                qos_profile_sensor_data,
            )

    def _on_message(self, name, message):
        now = self.get_clock().now().nanoseconds / 1e9
        received = self.samples[name]
        received["count"] += 1
        age = now - stamp_seconds(message)
        received["ages"].append(age)
        previous = self._last_receive.get(name)
        if previous is not None:
            received["periods"].append(now - previous)
        self._last_receive[name] = now
        if hasattr(message, "detections") and message.detections:
            received["non_empty"] += 1
        if hasattr(message, "observations") and message.observations:
            received["non_empty"] += 1


def summarize(sample, duration, minimum_hz=None, maximum_p95_age=None):
    count = sample["count"]
    periods = sample["periods"]
    ages = sample["ages"]
    hz = (count - 1) / sum(periods) if periods and sum(periods) > 0 else None
    p95_age = percentile(ages, 0.95)
    result = {
        "count": count,
        "observed_hz": hz,
        "age_mean_sec": statistics.mean(ages) if ages else None,
        "age_p95_sec": p95_age,
        "age_max_sec": max(ages) if ages else None,
        "non_empty_messages": sample["non_empty"],
    }
    checks = []
    if minimum_hz is not None:
        checks.append(hz is not None and hz >= minimum_hz)
    if maximum_p95_age is not None and p95_age is not None:
        checks.append(p95_age <= maximum_p95_age)
    result["pass"] = bool(checks) and all(checks)
    result["duration_sec"] = duration
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--min-image-hz", type=float, default=20.0)
    parser.add_argument("--min-observation-hz", type=float, default=15.0)
    parser.add_argument("--max-p95-age-ms", type=float, default=150.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.duration <= 0 or args.max_p95_age_ms <= 0:
        parser.error("duration and max-p95-age-ms must be positive")

    rclpy.init()
    probe = Probe()
    start = time.monotonic()
    end = start + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)
    duration = time.monotonic() - start
    report = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "host": platform.node(),
        "kernel": platform.release(),
        "python": platform.python_version(),
        "device": os.path.realpath("/dev/rear_camera"),
        "packages": {
            package: package_version(package)
            for package in (
                "ros-humble-apriltag-ros",
                "ros-humble-apriltag-msgs",
                "ros-humble-usb-cam",
            )
        },
        "thresholds": {
            "image_min_hz": args.min_image_hz,
            "observation_min_hz": args.min_observation_hz,
            "p95_age_max_sec": args.max_p95_age_ms / 1000.0,
        },
        "topics": {
            "image_raw": summarize(
                probe.samples["image_raw"],
                duration,
                args.min_image_hz,
                args.max_p95_age_ms / 1000.0,
            ),
            "image_rect": summarize(
                probe.samples["image_rect"],
                duration,
                args.min_image_hz,
                args.max_p95_age_ms / 1000.0,
            ),
            "camera_info": summarize(
                probe.samples["camera_info"],
                duration,
                maximum_p95_age=args.max_p95_age_ms / 1000.0,
            ),
            "detections": summarize(
                probe.samples["detections"],
                duration,
                maximum_p95_age=args.max_p95_age_ms / 1000.0,
            ),
            "observations": summarize(
                probe.samples["observations"],
                duration,
                args.min_observation_hz,
                args.max_p95_age_ms / 1000.0,
            ),
        },
    }
    report["camera_pass"] = all(
        report["topics"][key]["pass"] for key in ("image_raw", "image_rect")
    )
    report["stage3_pass"] = report["camera_pass"] and report["topics"][
        "observations"
    ]["pass"]
    print(json.dumps(report, ensure_ascii=True, indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=True, indent=2) + "\n"
        )
    probe.destroy_node()
    rclpy.shutdown()
    return 0 if report["stage3_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
