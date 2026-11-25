"""Offline analysis of the Nov 2 run; never imports ROS or controls the vehicle."""
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


RUN = Path(__file__).resolve().parent
MANAGER = Path("/home/akun/.ros/log/python3_10311_1793611823820.log")
TZ = ZoneInfo("Asia/Shanghai")
GOALS = {
    113: (109.31, -46.35, 1793612231.665068626, 1793612340.055391255),
    106: (115.12, -143.71, 1793612352.129863701, 1793612784.706013161),
}


def local_time(t):
    return datetime.fromtimestamp(t, TZ).isoformat(timespec="milliseconds")


def fresh_pose(row):
    pose = row.get("pose", {})
    return "x" in pose and 0 <= row["time"] - pose["stamp"] <= 0.3


def fresh_topic(row, topic):
    age = row.get("ages", {}).get(topic)
    return row.get(topic) is not None and age is not None and 0 <= age <= 0.2


def signed_ecu(row):
    ecu = row["ecu"]
    return ecu["motor"] * {1: 1, 3: -1}.get(ecu["shift"], 0)


def direction_changes(values):
    signs = [1 if value > 0 else -1 for value in values if abs(value) > 1e-6]
    return sum(a != b for a, b in zip(signs, signs[1:]))


rows = []
for line, raw in enumerate((RUN / "pose_observer.jsonl").read_text().splitlines(), 1):
    row = json.loads(raw)
    row["source_line"] = line
    rows.append(row)

summary = {
    "timezone": "Asia/Shanghai",
    "run": RUN.name,
    "pose_filter": "0 <= observer_time - TF_stamp <= 0.3 seconds",
    "topic_filter": "topic age <= 0.2 seconds, vehicle mode == 1",
    "distance_definition": "map TF base_link XY to original requested waypoint, NOT saved path.back()",
    "limitations": [
        "Observer samples approximately every 2 seconds; not full odometry.",
        "ECU shift is a request, not measured gearbox feedback.",
        "Original global/local paths, per-candidate costs and progress baselines were not recorded.",
        "WARN logging cannot exclude successful BackUp actions without explicit messages.",
    ],
    "source_sha256": {},
    "goals": {},
}
for path in [RUN / "pose_observer.jsonl", RUN / "nav2_2026-11-02_17-36-55.log", MANAGER]:
    summary["source_sha256"][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()

goal_rows = {}
for goal_id, (gx, gy, accepted, succeeded) in GOALS.items():
    selected = []
    for row in rows:
        if accepted <= row["time"] <= succeeded and fresh_pose(row):
            row = dict(row)
            row["distance"] = math.hypot(row["pose"]["x"] - gx, row["pose"]["y"] - gy)
            selected.append(row)
    goal_rows[goal_id] = selected
    result = {
        "requested_xy": [gx, gy],
        "accepted": local_time(accepted),
        "succeeded": local_time(succeeded),
        "succeeded_epoch": succeeded,
        "fresh_pose_samples": len(selected),
        "first_distance_crossings": {},
    }
    for distance in (5, 2, 1, 0.5):
        first = next((row for row in selected if row["distance"] <= distance), None)
        result["first_distance_crossings"][str(distance)] = None if first is None else {
            "time": local_time(first["time"]),
            "epoch": first["time"],
            "distance_m": first["distance"],
            "seconds_until_success": succeeded - first["time"],
            "observer_line": first["source_line"],
        }
    result["last_pre_success_sample"] = {
        "time": local_time(selected[-1]["time"]),
        "distance_m": selected[-1]["distance"],
        "seconds_before_success": succeeded - selected[-1]["time"],
    }
    summary["goals"][str(goal_id)] = result

stall = [row for row in goal_rows[106] if 1793612658 <= row["time"] <= 1793612770.3]
valid = [row for row in stall if all(fresh_topic(row, topic) for topic in ("cmd", "ecu", "vehicle"))
         and row["vehicle"]["mode"] == 1]
ecu_values = [signed_ecu(row) for row in valid]
summary["goal106_stall"] = {
    "start": local_time(stall[0]["time"]),
    "end": local_time(stall[-1]["time"]),
    "duration_s": stall[-1]["time"] - stall[0]["time"],
    "fresh_pose_samples": len(stall),
    "fresh_cmd_ecu_vehicle_samples": len(valid),
    "distance_min_m": min(row["distance"] for row in stall),
    "distance_max_m": max(row["distance"] for row in stall),
    "cmd_positive_samples": sum(row["cmd"]["linear_x"] > 0 for row in valid),
    "cmd_negative_samples": sum(row["cmd"]["linear_x"] < 0 for row in valid),
    "ecu_forward_request_samples": sum(v > 0 for v in ecu_values),
    "ecu_reverse_request_samples": sum(v < 0 for v in ecu_values),
    "ecu_zero_request_samples": sum(v == 0 for v in ecu_values),
    "ecu_nonzero_request_direction_changes": direction_changes(ecu_values),
    "ecu_at_0_8_kmh_samples": sum(abs(row["ecu"]["motor"] - 0.8) < 1e-6 for row in valid),
    "samples": [dict(observer_line=row["source_line"], time=local_time(row["time"]),
                     distance_m=row["distance"], cmd_mps=row["cmd"]["linear_x"],
                     ecu_signed_kmh=signed_ecu(row)) for row in valid],
}

events = []
for line_number, line in enumerate((RUN / "nav2_2026-11-02_17-36-55.log").read_text().splitlines(), 1):
    match = re.search(r"\[(\d{10}\.\d+)\]", line)
    if match and any(text in line for text in ("Failed to make progress", "Optimizer fail to compute path", "backup failed")):
        t = float(match.group(1))
        events.append({"epoch": t, "time": local_time(t), "source_line": line_number, "message": line})
summary["events"] = events
summary["optimizer_failures_during_stall"] = sum(
    stall[0]["time"] <= event["epoch"] <= stall[-1]["time"] and "Optimizer fail" in event["message"]
    for event in events)
(RUN / "goal_oscillation_metrics.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")

# Plot distances to the original goal only. Do not overlay MPPI thresholds:
# those refer to a different, historically unrecorded local-path endpoint.
view = [row for row in goal_rows[106] if row["time"] >= 1793612648]
origin = view[0]["time"]
x = [row["time"] - origin for row in view]
fig, axes = plt.subplots(2, 1, figsize=(11, 6.2), sharex=True, constrained_layout=True)
axes[0].plot(x, [row["distance"] for row in view], "o-", color="#176cab", markersize=3,
             label="Distance to requested waypoint 106")
axes[0].axhline(0.5, color="#667788", linestyle="--", label="0.5 m reference (actual path endpoint unrecorded)")
axes[0].set_ylabel("Distance (m)")
axes[0].set_ylim(bottom=0)
axes[0].legend(loc="upper right", fontsize=8)
cmd = [row["cmd"]["linear_x"] if fresh_topic(row, "cmd") else math.nan for row in view]
ecu = [signed_ecu(row) / 3.6 if fresh_topic(row, "ecu") else math.nan for row in view]
axes[1].plot(x, cmd, ".-", color="#176cab", label="Navigation command (m/s)")
axes[1].plot(x, ecu, "s-", color="#bd6025", markersize=3, label="Signed ECU request (km/h / 3.6)")
axes[1].axhline(0, color="#667788", linewidth=0.8)
axes[1].set_ylabel("Requested speed (m/s)")
axes[1].set_xlabel(f"Seconds since {local_time(origin)} (Asia/Shanghai)")
axes[1].legend(loc="upper right", fontsize=8)
for ax in axes:
    ax.grid(alpha=0.2)
    ax.axvline(1793612770.349471213 - origin, color="#b02c35", linestyle=":")
    ax.axvline(GOALS[106][3] - origin, color="#25824b", linestyle=":")
    ax.set_xlim(0, GOALS[106][3] - origin + 2)
axes[0].text(1793612770.349471213 - origin - 1, 3.4, "Progress failure", ha="right", color="#b02c35", fontsize=9)
axes[0].text(GOALS[106][3] - origin - 1, 2.5, "Goal succeeded", ha="right", color="#25824b", fontsize=9)
fig.suptitle("Historical waypoint 106: repeated direction requests near the goal", fontsize=13)
fig.savefig(RUN / "goal106_distance_and_commands.png", dpi=160)
plt.close(fig)
print(json.dumps({"goals": summary["goals"], "stall": {k: v for k, v in summary["goal106_stall"].items() if k != "samples"},
                  "optimizer_failures_during_stall": summary["optimizer_failures_during_stall"]}, ensure_ascii=False, indent=2))
