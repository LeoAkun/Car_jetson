#!/usr/bin/env python3
"""Extract task_1794627698 plan and /odom records with JSONL line provenance."""

import csv
import json
import math
from pathlib import Path


OBSERVER = Path(
    "/home/akun/workspace/Car_jetson/navigation_monitoring/runs/"
    "2026-11-14_11-02-27_manual_publish_monitor/detail_observer.jsonl"
)
OUTPUT_DIR = Path(__file__).resolve().parent
WINDOW_START = 1794627735.0
WINDOW_END = 1794627855.0
FIRST_LETHAL = 1794627805.581230176


def main() -> None:
    plans = []
    poses = []
    with OBSERVER.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            stamp = event.get("stamp", 0.0)
            if not WINDOW_START <= stamp <= WINDOW_END:
                continue
            if event.get("event") == "plan":
                plans.append(
                    {
                        "line": line_number,
                        "host_time": event["time"],
                        "stamp": stamp,
                        "count": event.get("count"),
                        "first_x": event["first"][0],
                        "first_y": event["first"][1],
                        "last_x": event["last"][0],
                        "last_y": event["last"][1],
                    }
                )
            elif event.get("event") == "pose" and event.get("topic") == "/odom":
                qx, qy, qz, qw = event["q"]
                yaw = math.atan2(
                    2.0 * (qw * qz + qx * qy),
                    1.0 - 2.0 * (qy * qy + qz * qz),
                )
                poses.append(
                    {
                        "line": line_number,
                        "host_time": event["time"],
                        "stamp": stamp,
                        "x": event["x"],
                        "y": event["y"],
                        "yaw_odom": yaw,
                    }
                )

    for filename, rows in (("plan_records.csv", plans), ("odom_records.csv", poses)):
        with (OUTPUT_DIR / filename).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

    manifest = "\n".join(
        [
            f"observer {OBSERVER}",
            f"window_ros_epoch {WINDOW_START:.0f} {WINDOW_END:.0f}",
            f"plan_count {len(plans)} first_line {plans[0]['line']} last_line {plans[-1]['line']}",
            f"plans_before_first_lethal {sum(p['stamp'] < FIRST_LETHAL for p in plans)}",
            f"plans_after_first_lethal {sum(p['stamp'] >= FIRST_LETHAL for p in plans)}",
            f"odom_count {len(poses)} first_line {poses[0]['line']} last_line {poses[-1]['line']}",
            f"plan_csv {OUTPUT_DIR / 'plan_records.csv'}",
            f"odom_csv {OUTPUT_DIR / 'odom_records.csv'}",
            "",
        ]
    )
    (OUTPUT_DIR / "extract_manifest.txt").write_text(manifest, encoding="utf-8")
    print(manifest, end="")


if __name__ == "__main__":
    main()
