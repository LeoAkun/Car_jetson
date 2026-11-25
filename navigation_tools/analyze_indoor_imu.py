#!/usr/bin/env python3
"""Analyze a completed indoor startup run without connecting to ROS or hardware.

Usage: python3 analyze_indoor_imu.py RUN_DIR --compare PREVIOUS_RUN_DIR
Both input directories must contain result.json and a test_finished event.
Only the primary run receives imu_timing_analysis.json/.md and imu_timing.png/.svg.
"""

import argparse
from collections import defaultdict
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
from zoneinfo import ZoneInfo


LOCAL_ZONE = ZoneInfo("Asia/Shanghai")
NANOSECOND = 1_000_000_000
GAP_SEC = 0.2
QUATERNION_TOLERANCE = 0.1
PHASE_COLORS = {"sensor_baseline": "#93c5fd", "lio_start": "#fcd34d",
                "nav2_start": "#c4b5fd", "hold": "#86efac"}
LOG_TIMESTAMP = re.compile(r"\[(\d+)\.(\d{9})\]")


def local_time(ns):
    seconds, nanoseconds = divmod(int(ns), NANOSECOND)
    return datetime.fromtimestamp(seconds, LOCAL_ZONE).strftime("%Y-%m-%d %H:%M:%S") + f".{nanoseconds:09d}+08:00"


def iso_time_ns(value):
    value = datetime.fromisoformat(value)
    return int(value.timestamp()) * NANOSECOND + value.microsecond * 1000


def finite(value):
    return float(value) if math.isfinite(value) else None


def json_safe(value):
    if isinstance(value, float):
        return finite(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def describe(values):
    values = sorted(float(value) for value in values if math.isfinite(value))
    if not values:
        return None
    def percentile(fraction):
        return values[round((len(values) - 1) * fraction)]
    return {"count": len(values), "min": values[0], "p01": percentile(.01),
            "p05": percentile(.05), "median": percentile(.5), "p95": percentile(.95),
            "p99": percentile(.99), "max": values[-1], "mean": statistics.fmean(values),
            "stddev": statistics.pstdev(values)}


def read_jsonl(path):
    records = []
    with path.open() as stream:
        for line_number, line in enumerate(stream, 1):
            if line.strip():
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{path}:{line_number}: invalid/incomplete JSON; do not analyze an active run") from exc
    return records


def require_complete(directory):
    result_path = directory / "result.json"
    events_path = directory / "events.jsonl"
    if not result_path.is_file() or not events_path.is_file():
        raise ValueError(f"Run is not closed: {directory} lacks result.json/events.jsonl")
    events = read_jsonl(events_path)
    if not any(event.get("event") == "test_finished" for event in events):
        raise ValueError(f"Run is not closed: no test_finished event in {directory}")
    return json.loads(result_path.read_text()), events


def log_timestamp_ns(line):
    match = LOG_TIMESTAMP.search(line)
    return None if match is None else int(match[1]) * NANOSECOND + int(match[2])


def vector_statistics(rows, field):
    vectors = [row[field] for row in rows]
    valid = [vector for vector in vectors if all(math.isfinite(value) for value in vector)]
    return {"nonfinite_count": len(vectors) - len(valid),
            "norm": describe(math.hypot(*vector) for vector in valid),
            "axes": {axis: describe(vector[index] for vector in valid)
                     for index, axis in enumerate(("x", "y", "z"))}}


def orientation_statistics(rows):
    """Normalize valid xyzw quaternions before comparing their rotations."""
    valid = []
    for row in rows:
        norm = math.hypot(*row["q"])
        if not math.isfinite(norm) or norm < 1e-6 or abs(norm - 1) > QUATERNION_TOLERANCE:
            continue
        quaternion = [value / norm for value in row["q"]]
        x, y, z, w = quaternion
        angles = [math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
                  math.asin(max(-1.0, min(1.0, 2 * (w*y - z*x)))),
                  math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))]
        if valid:
            previous_yaw = valid[-1][2][2]
            angles[2] = previous_yaw + math.atan2(math.sin(angles[2] - previous_yaw),
                                               math.cos(angles[2] - previous_yaw))
        valid.append((row, quaternion, angles))
    if not valid:
        return {"valid_count": 0, "excluded_count": len(rows)}
    first = valid[0][1]
    def relative_angle(quaternion):
        dot = abs(sum(left * right for left, right in zip(first, quaternion)))
        return math.degrees(2 * math.acos(max(0.0, min(1.0, dot))))
    relative = [relative_angle(item[1]) for item in valid]
    peak = valid[max(range(len(relative)), key=relative.__getitem__)][0]
    def endpoint(item):
        row, quaternion, angles = item
        return {"received_ns": row["received_ns"], "received_local": local_time(row["received_ns"]),
                "header_ns": row["header_ns"], "phase": row["phase"],
                "unit_q_xyzw": quaternion,
                "roll_pitch_yaw_deg": dict(zip(("roll", "pitch", "yaw"), map(math.degrees, angles)))}
    return {"valid_count": len(valid), "excluded_count": len(rows) - len(valid),
            "definition": "Euler XYZ roll/pitch/yaw from normalized xyzw; yaw unwrapped; quaternion relative angle uses abs(dot) to ignore q/-q sign flips",
            "first": endpoint(valid[0]), "last": endpoint(valid[-1]),
            "roll_pitch_yaw_deg": {axis: describe(math.degrees(item[2][index]) for item in valid)
                                   for index, axis in enumerate(("roll", "pitch", "yaw"))},
            "first_to_last_euler_delta_deg": {axis: math.degrees(valid[-1][2][index] - valid[0][2][index])
                                              for index, axis in enumerate(("roll", "pitch", "yaw"))},
            "first_to_last_rotation_deg": relative[-1],
            "relative_to_first_rotation_deg": describe(relative),
            "max_relative_rotation_received_local": local_time(peak["received_ns"]),
            "max_relative_rotation_received_ns": peak["received_ns"]}


def analyze_run(directory):
    result, events = require_complete(directory)
    rows = read_jsonl(directory / "imu_samples.jsonl")
    if len(rows) < 2:
        raise ValueError(f"At least two IMU samples are required: {directory}")
    for line, row in enumerate(rows, 1):
        for key in ("header_ns", "received_ns", "monotonic_ns", "phase", "q", "angular_velocity", "linear_acceleration"):
            if key not in row:
                raise ValueError(f"imu_samples.jsonl:{line} lacks {key}")
    norms = [math.hypot(*row["q"]) for row in rows]
    arrival = [None] + [(row["monotonic_ns"] - rows[index - 1]["monotonic_ns"]) / NANOSECOND
                        for index, row in enumerate(rows) if index]
    headers = [None] + [(row["header_ns"] - rows[index - 1]["header_ns"]) / NANOSECOND
                        for index, row in enumerate(rows) if index]
    offsets = [(row["header_ns"] - row["received_ns"]) / NANOSECOND for row in rows]
    elapsed = [(row["monotonic_ns"] - rows[0]["monotonic_ns"]) / NANOSECOND for row in rows]
    bad_norms = [index for index, norm in enumerate(norms)
                 if not math.isfinite(norm) or abs(norm - 1.0) > QUATERNION_TOLERANCE]
    jumps = [index for index in range(1, len(rows)) if headers[index] > GAP_SEC or headers[index] <= 0]
    arrival_gaps = [index for index in range(1, len(rows)) if arrival[index] > GAP_SEC]

    def sample(index):
        row = rows[index]
        return json_safe({"line": index + 1, "phase": row["phase"], "elapsed_sec": elapsed[index],
                          "received_ns": row["received_ns"], "received_local": local_time(row["received_ns"]),
                          "monotonic_ns": row["monotonic_ns"], "header_ns": row["header_ns"],
                          "header_local": local_time(row["header_ns"]), "header_minus_received_sec": offsets[index],
                          "arrival_interval_sec": arrival[index], "header_interval_sec": headers[index],
                          "quaternion_norm": norms[index], "q": row["q"],
                          "angular_velocity": row["angular_velocity"], "linear_acceleration": row["linear_acceleration"]})

    def contexts(indices):
        return [{"event_line": index + 1, "context": [sample(position) for position in
                 range(max(0, index - 5), min(len(rows), index + 4))]} for index in indices]

    def statistics_for(indices):
        subset = [rows[index] for index in indices]
        positive_arrival = [arrival[index] for index in indices if index and arrival[index] is not None]
        source_intervals = [headers[index] for index in indices if index]
        span = (subset[-1]["monotonic_ns"] - subset[0]["monotonic_ns"]) / NANOSECOND
        selected_norms = [norms[index] for index in indices]
        return {"count": len(indices), "first": sample(indices[0]), "last": sample(indices[-1]),
                "received_span_sec": span, "received_hz": (len(indices) - 1) / span if span > 0 else None,
                "header_minus_received_sec": describe(offsets[index] for index in indices),
                "future_header_gt_0_2_count": sum(offsets[index] > GAP_SEC for index in indices),
                "arrival_interval_sec": describe(positive_arrival), "header_interval_sec": describe(source_intervals),
                "arrival_gap_gt_0_2_count": sum(value > GAP_SEC for value in positive_arrival),
                "header_gap_gt_0_2_count": sum(value > GAP_SEC for value in source_intervals),
                "header_nonpositive_count": sum(value <= 0 for value in source_intervals),
                "header_backward_count": sum(value < 0 for value in source_intervals),
                "header_duplicate_count": sum(value == 0 for value in source_intervals),
                "quaternion_norm": describe(selected_norms),
                "quaternion_nonfinite_count": sum(not all(math.isfinite(value) for value in row["q"]) for row in subset),
                "quaternion_zero_norm_count": sum(math.isfinite(norm) and norm < 1e-6 for norm in selected_norms),
                "bad_quaternion_norm_count": sum(not math.isfinite(norm) or abs(norm - 1.0) > QUATERNION_TOLERANCE for norm in selected_norms),
                "orientation": orientation_statistics(subset),
                "angular_velocity": vector_statistics(subset, "angular_velocity"),
                "linear_acceleration": vector_statistics(subset, "linear_acceleration")}

    indices_by_phase = defaultdict(list)
    for index, row in enumerate(rows):
        indices_by_phase[row["phase"]].append(index)
    phases = {phase: statistics_for(indices) for phase, indices in indices_by_phase.items()}
    total = statistics_for(list(range(len(rows))))
    early = [offsets[index] for index, value in enumerate(elapsed) if 2 <= value < 7]
    late = [offsets[index] for index, value in enumerate(elapsed) if elapsed[-1] - value < 5]
    drift = {"early_window_elapsed_sec": [2, 7], "late_window_elapsed_sec": [max(0, elapsed[-1] - 5), elapsed[-1]],
             "early_offset_median_sec": statistics.median(early) if early else None,
             "late_offset_median_sec": statistics.median(late) if late else None,
             "difference_sec": statistics.median(late) - statistics.median(early) if early and late else None}
    imu_lines = (directory / "imu.log").read_text(errors="replace").splitlines()
    shutdown = next((log_timestamp_ns(line) for line in imu_lines if "signal_handler(SIGINT/SIGTERM)" in line), None)
    crc_events = []
    for line in imu_lines:
        if "crc" not in line.lower():
            continue
        timestamp = log_timestamp_ns(line)
        item = {"line": line, "time_ns": timestamp, "time_local": local_time(timestamp) if timestamp else None}
        count = re.search(r"total_crc_errors=(\d+)", line)
        item["reported_total_crc_errors"] = int(count[1]) if count else None
        if timestamp and jumps:
            nearest = min(jumps, key=lambda index: abs(rows[index]["received_ns"] - timestamp))
            item["nearest_header_jump_line"] = nearest + 1
            item["jump_receipt_minus_crc_sec"] = (rows[nearest]["received_ns"] - timestamp) / NANOSECOND
        crc_events.append(item)
    lio_logs = {}
    for path in sorted(directory.glob("lio_*.log")):
        lines = path.read_text(errors="replace").splitlines()
        lio_logs[path.name] = {"line_count": len(lines),
                               "gap_or_reset_lines": [line for line in lines if re.search(r"\b(?:gap|reset|resetting|recover\w*)\b|timeline broke", line, re.I)],
                               "fatal_lines": [line for line in lines if re.search(r"fatal|indeterminan|optimization failure|non-finite", line, re.I)],
                               "warnings_or_errors": [line for line in lines if re.search(r"\[(WARN|ERROR|FATAL)\]", line)]}
    first_lio = next((event for event in events if event.get("event") == "process_started"
                      and event.get("label", "").startswith("lio_")), None)
    lio_ns = iso_time_ns(first_lio["time"]) if first_lio else None
    short_reads = [{"line": line, "after_shutdown_signal": bool(shutdown is not None and log_timestamp_ns(line) is not None and log_timestamp_ns(line) >= shutdown)}
                   for line in imu_lines if "Short IMU serial read" in line]
    observations = read_jsonl(directory / "observations.jsonl") if (directory / "observations.jsonl").exists() else []
    ready_changes = []
    previous_ready = None
    for observation in observations:
        ready = observation.get("imu_preintegration", {}).get("ready")
        if ready is not None and ready != previous_ready:
            ready_changes.append({"timestamp_utc": observation.get("timestamp_utc"),
                                  "phase": observation.get("phase"), "ready": ready})
            previous_ready = ready
    sources = [directory / name for name in ("imu_samples.jsonl", "imu.log", "events.jsonl", "result.json")]
    sources.extend(sorted(directory.glob("lio_*.log")))
    report = {"schema_version": 1, "run": directory.name, "run_directory": str(directory.resolve()),
              "signed_offset_definition": "header_ns - received_ns; positive means future-dated at subscriber arrival",
              "thresholds": {"gap_sec": GAP_SEC, "quaternion_norm_tolerance": QUATERNION_TOLERANCE,
                             "quaternion_zero_norm_definition": "norm < 1e-6, includes near-zero values; not only literal zero"},
              "source_sha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
              "total": total, "phases": phases, "offset_window_drift": drift,
              "wall_interval_minus_monotonic_interval_sec": describe(((rows[index]["received_ns"] - rows[index-1]["received_ns"]) -
                         (rows[index]["monotonic_ns"] - rows[index-1]["monotonic_ns"])) / NANOSECOND for index in range(1, len(rows))),
              "jump_contexts": contexts(jumps), "arrival_gap_contexts": contexts(arrival_gaps),
              "bad_quaternion_contexts": contexts(bad_norms), "first_15_samples": [sample(index) for index in range(min(15, len(rows)))],
              "crc_events": crc_events, "short_read_events": short_reads,
              "device_timestamp_diagnostics": [line for line in imu_lines if re.search(r"timestamp.*reset|re-anchoring|non-monotonic", line, re.I)],
              "lio_logs": lio_logs, "first_lio_process_start": first_lio,
              "header_jump_before_first_lio_count": sum(rows[index]["received_ns"] < lio_ns for index in jumps) if lio_ns else None,
              "ready_summary": result.get("observations", {}).get("imu_preintegration"), "observed_ready_changes": ready_changes,
              "startup_result": {key: result.get(key) for key in ("passed", "pass_scope", "started", "ended", "duration_sec", "error")},
              "limitations": ["ROS messages do not include the original serial frames or independent device timestamp values.",
                              "Header-minus-receipt is not an independently calibrated cross-sensor acquisition-time offset.",
                              "Raw recording and aggregate observer have separate callbacks; their arrival jitter and phase boundaries can differ slightly.",
                              "IMU orientation/gyro/acceleration alone cannot attribute a mapping position change to physical motion or localization error.",
                              "Startup/freshness success does not establish correct timestamp synchronization or motion accuracy."]}
    return report, rows


def compare_runs(current, previous):
    keys = ("count", "received_span_sec", "received_hz", "arrival_gap_gt_0_2_count",
            "header_gap_gt_0_2_count", "header_nonpositive_count", "bad_quaternion_norm_count")
    comparison = {"previous_run": previous["run"], "previous_directory": previous["run_directory"],
                  "totals": {key: {"previous": previous["total"][key], "current": current["total"][key]} for key in keys},
                  "phase_offset_median_sec": {}, "crc_event_count": {"previous": len(previous["crc_events"]), "current": len(current["crc_events"])}}
    comparison["orientation_and_motion"] = {
        name: {"previous": previous["total"][name], "current": current["total"][name]}
        for name in ("orientation", "angular_velocity", "linear_acceleration")}
    for phase in set(previous["phases"]) | set(current["phases"]):
        before = previous["phases"].get(phase, {}).get("header_minus_received_sec")
        after = current["phases"].get(phase, {}).get("header_minus_received_sec")
        comparison["phase_offset_median_sec"][phase] = {"previous": before["median"] if before else None,
                                                        "current": after["median"] if after else None}
    return comparison


def milliseconds(value, digits=3):
    return "—" if value is None else f"{value * 1000:.{digits}f}"


def write_markdown(directory, report):
    total = report["total"]
    offset = total["header_minus_received_sec"]
    lines = [f"# 真实 IMU 连续性与时间分析：{report['run']}", "",
             "本报告仅离线读取已结束运行的原始消息和日志。所有精确样本时间均为北京时间 UTC+8；`header − received` 为正表示消息抵达观察节点时 header 已指向未来。", "",
             f"共 {total['count']} 条 IMU，实际接收跨度 {total['received_span_sec']:.3f} 秒，{total['received_hz']:.3f} Hz。",
             f"接收间隔中位 / P99 / 最大为 {milliseconds(total['arrival_interval_sec']['median'])} / {milliseconds(total['arrival_interval_sec']['p99'])} / {milliseconds(total['arrival_interval_sec']['max'])} ms。",
             f"接收 >0.2 秒空洞 {total['arrival_gap_gt_0_2_count']}；header >0.2 秒间隔 {total['header_gap_gt_0_2_count']}；header 回退 / 重复 {total['header_backward_count']} / {total['header_duplicate_count']}。",
             f"header − received 中位 / P99 / 最大为 {milliseconds(offset['median'])} / {milliseconds(offset['p99'])} / {milliseconds(offset['max'])} ms。", "",
             "| 阶段 | IMU 条数 | 接收 Hz | 接收最大 ms | header 最大 ms | offset 中位 ms | offset 最大 ms | 异常四元数范数 |", "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for name, phase in report["phases"].items():
        lines.append(f"| {name} | {phase['count']} | {phase['received_hz']:.3f} | {milliseconds(phase['arrival_interval_sec']['max'])} | {milliseconds(phase['header_interval_sec']['max'])} | {milliseconds(phase['header_minus_received_sec']['median'])} | {milliseconds(phase['header_minus_received_sec']['max'])} | {phase['bad_quaternion_norm_count']} |")
    drift = report["offset_window_drift"]
    lines += ["", f"第 2–7 秒 offset 中位 {milliseconds(drift['early_offset_median_sec'])} ms，最后 5 秒 {milliseconds(drift['late_offset_median_sec'])} ms，差 {milliseconds(drift['difference_sec'])} ms。时间变化不单独证明设备振荡器、host 时钟或消息延迟中哪个是原因。", "",
              f"角速度非有限 {total['angular_velocity']['nonfinite_count']}；加速度非有限 {total['linear_acceleration']['nonfinite_count']}；四元数非有限 {total['quaternion_nonfinite_count']}、近零范数（<1e-6，非仅字面零）{total['quaternion_zero_norm_count']}、范数偏离 1 超过 0.1 共 {total['bad_quaternion_norm_count']} 条。完整轴向及范数分布见 JSON。", "", "## 姿态、角速度与加速度", ""]
    orientation = total["orientation"]
    if orientation["valid_count"]:
        lines += [f"有效四元数 {orientation['valid_count']} 条、排除 {orientation['excluded_count']} 条；先归一化再计算姿态，yaw 连续展开，q 与 -q 视作同一旋转。", "",
                  "| 角度（度） | 首条有效值 | 末条有效值 | 最小 | 最大 | 首尾差 |", "|---|---:|---:|---:|---:|---:|"]
        for axis in ("roll", "pitch", "yaw"):
            angles = orientation["roll_pitch_yaw_deg"][axis]
            lines.append(f"| {axis} | {orientation['first']['roll_pitch_yaw_deg'][axis]:.6f} | {orientation['last']['roll_pitch_yaw_deg'][axis]:.6f} | {angles['min']:.6f} | {angles['max']:.6f} | {orientation['first_to_last_euler_delta_deg'][axis]:.6f} |")
        lines += ["", f"首尾旋转夹角 {orientation['first_to_last_rotation_deg']:.6f}°；相对首条最大旋转 {orientation['relative_to_first_rotation_deg']['max']:.6f}°，发生在 `{orientation['max_relative_rotation_received_local']}`。",
                  "", "| 阶段 | 有效姿态条数 | 首尾旋转度 | 相对阶段首条最大旋转度 |", "|---|---:|---:|---:|"]
        for phase, values in report["phases"].items():
            value = values["orientation"]
            if value["valid_count"]:
                lines.append(f"| {phase} | {value['valid_count']} | {value['first_to_last_rotation_deg']:.6f} | {value['relative_to_first_rotation_deg']['max']:.6f} |")
    gyro = total["angular_velocity"]["norm"]
    accel = total["linear_acceleration"]["norm"]
    lines += ["", f"角速度向量范数中位 / P99 / 最大：{gyro['median']:.9f} / {gyro['p99']:.9f} / {gyro['max']:.9f} rad/s。",
              f"加速度向量范数最小 / 中位 / P99 / 最大：{accel['min']:.9f} / {accel['median']:.9f} / {accel['p99']:.9f} / {accel['max']:.9f} m/s²。",
              "这些测量描述 IMU 自身记录的姿态和运动量，不能单独判断 mapping 位置变化来自人工移动还是定位误差；加速度包含重力，不能据此排除匀速平移。", "", "## 时间戳和四元数异常样本", ""]
    if not report["jump_contexts"] and not report["bad_quaternion_contexts"]:
        lines.append("未发现 header 超过 0.2 秒、回退/重复或异常四元数范数。")
    emitted = set()
    for kind, contexts in (("header", report["jump_contexts"]), ("quaternion", report["bad_quaternion_contexts"])):
        for context in contexts:
            index = context["event_line"]
            item = next(sample for sample in context["context"] if sample["line"] == index)
            lines.append(f"- {kind} 异常第 {index} 行：received `{item['received_local']}`，header `{item['header_local']}`；接收间隔 {milliseconds(item['arrival_interval_sec'], 6)} ms，header 间隔 {milliseconds(item['header_interval_sec'], 6)} ms，offset {milliseconds(item['header_minus_received_sec'], 6)} ms，q 范数 `{item['quaternion_norm']}`。")
            emitted.add(index)
    if emitted:
        lines += ["", "每个异常前 5 条、后 3 条完整 q/gyro/accel 与整数纳秒时间戳已保存在 JSON；启动前 15 条也单独保留。"]
    lines += ["", "## CRC、设备时间与 LIO 日志", ""]
    if report["crc_events"]:
        for event in report["crc_events"]:
            correlation = ""
            if "nearest_header_jump_line" in event:
                correlation = f"；最近 header 异常行 {event['nearest_header_jump_line']} 的接收时间比 CRC 日志晚 {milliseconds(event['jump_receipt_minus_crc_sec'], 6)} ms"
            lines.append(f"- `{event['time_local']}`：`{event['line']}`{correlation}。")
    else:
        lines.append("本轮 imu.log 没有 CRC 警告。")
    lines.append("CRC 与样本异常时间相近只能证明时序关联，不能仅凭 ROS 消息证明底层因果。")
    for event in report["short_read_events"]:
        lines.append(f"- 短读：`{event['line']}`；是否在关闭信号后：{event['after_shutdown_signal']}。")
    lines.append(f"- 设备时间 reset/re-anchor/non-monotonic 诊断：{len(report['device_timestamp_diagnostics'])} 条。")
    for name, data in report["lio_logs"].items():
        lines.append(f"- `{name}`：gap/reset/recovery {len(data['gap_or_reset_lines'])} 条；fatal/优化异常 {len(data['fatal_lines'])} 条；WARN/ERROR/FATAL {len(data['warnings_or_errors'])} 条。")
    lines += ["", f"最终 ready 汇总：`{json.dumps(report['ready_summary'], ensure_ascii=False)}`。"]
    if report["first_lio_process_start"]:
        lines.append(f"首个 LIO 进程启动于 `{report['first_lio_process_start']['time']}`；发生在此前的 header 异常共 {report['header_jump_before_first_lio_count']} 次。这些启动前样本不代表 LIO 已运行时经历了同一跳变。")
    comparison = report.get("comparison")
    if comparison:
        lines += ["", f"## 对照 {comparison['previous_run']}", "", "| 指标 | 前轮 | 本轮 |", "|---|---:|---:|"]
        for name, values in comparison["totals"].items():
            lines.append(f"| {name} | {values['previous']} | {values['current']} |")
        for phase, values in sorted(comparison["phase_offset_median_sec"].items()):
            lines.append(f"| {phase} offset 中位 ms | {milliseconds(values['previous'])} | {milliseconds(values['current'])} |")
        motion = comparison["orientation_and_motion"]
        for name, values in (("首尾姿态旋转度", {run: motion['orientation'][run]['first_to_last_rotation_deg'] for run in ('previous', 'current')}),
                             ("最大相对姿态旋转度", {run: motion['orientation'][run]['relative_to_first_rotation_deg']['max'] for run in ('previous', 'current')}),
                             ("gyro 范数中位 rad/s", {run: motion['angular_velocity'][run]['norm']['median'] for run in ('previous', 'current')}),
                             ("gyro 范数最大 rad/s", {run: motion['angular_velocity'][run]['norm']['max'] for run in ('previous', 'current')}),
                             ("accel 范数中位 m/s²", {run: motion['linear_acceleration'][run]['norm']['median'] for run in ('previous', 'current')})):
            lines.append(f"| {name} | {values['previous']:.9f} | {values['current']:.9f} |")
    lines += ["", "## 结论范围", ""] + [f"- {item}" for item in report["limitations"]]
    lines += ["", f"室内启动程序结果：`passed={report['startup_result']['passed']}`；IMU 时间质量按上述指标独立判断。", "", f"![IMU timing]({directory.resolve() / 'imu_timing.png'})", ""]
    (directory / "imu_timing_analysis.md").write_text("\n".join(lines))


def write_plot(directory, report, rows, previous_data=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    times = np.array([(row["monotonic_ns"] - rows[0]["monotonic_ns"]) / NANOSECOND for row in rows])
    offsets = np.array([(row["header_ns"] - row["received_ns"]) / 1e6 for row in rows])
    arrival = np.diff(np.array([row["monotonic_ns"] for row in rows], dtype=np.int64)) / 1e6
    headers = np.diff(np.array([row["header_ns"] for row in rows], dtype=np.int64)) / 1e6
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for axis in axes.flat:
        axis.grid(alpha=.22)
        axis.spines[["top", "right"]].set_visible(False)
    for axis in (axes[0, 0], axes[0, 1], axes[1, 0]):
        for phase, values in report["phases"].items():
            axis.axvspan(values["first"]["elapsed_sec"], values["last"]["elapsed_sec"], alpha=.15,
                         color=PHASE_COLORS.get(phase, "#cbd5e1"), label=phase)
        axis.set_xlabel("Seconds since first received IMU")
    axes[0, 0].plot(times, offsets, color="#1d4ed8", lw=.7)
    axes[0, 0].set(ylabel="Header minus receipt (ms)", title="Full capture: IMU header vs host receipt")
    axes[0, 0].legend(loc="best", fontsize=7, ncol=2)
    axes[0, 1].plot(times[1:], arrival, color="#2563eb", lw=.5, label="Arrival interval")
    axes[0, 1].plot(times[1:], headers, color="#ea580c", lw=.6, alpha=.8, label="Header interval")
    axes[0, 1].set(ylabel="Adjacent interval (ms)", title="Arrival and source/header continuity")
    axes[0, 1].legend(loc="best", fontsize=7)
    selected = times >= 1
    axes[1, 0].plot(times[selected], offsets[selected], color="#1d4ed8", lw=.5, label=report["run"])
    if previous_data:
        previous, previous_rows = previous_data
        prior_times = np.array([(row["monotonic_ns"] - previous_rows[0]["monotonic_ns"]) / NANOSECOND for row in previous_rows])
        prior_offsets = np.array([(row["header_ns"] - row["received_ns"]) / 1e6 for row in previous_rows])
        selected_prior = prior_times >= 1
        axes[1, 0].plot(prior_times[selected_prior], prior_offsets[selected_prior], color="#a16207", lw=.45, alpha=.55,
                        label=previous["run"])
    axes[1, 0].set(ylabel="Header minus receipt (ms)", title="Offset after the first second")
    axes[1, 0].legend(loc="best", fontsize=7)
    startup = times < .1
    header_elapsed = np.array([(row["header_ns"] - rows[0]["header_ns"]) / 1e6 for row in rows])
    axes[1, 1].plot(times[startup] * 1000, header_elapsed[startup], "-o", ms=3, color="#ea580c", label="Header elapsed")
    axes[1, 1].plot(times[startup] * 1000, times[startup] * 1000, color="#2563eb", label="Arrival elapsed")
    axes[1, 1].set(xlabel="Received elapsed at startup (ms)", ylabel="Elapsed time (ms)", title="First 100 ms")
    axes[1, 1].legend(loc="best", fontsize=7)
    figure.suptitle(f"Live IMU / live LiDAR indoor startup | {report['run']} UTC+8 | no navigation goals", fontsize=12)
    figure.savefig(directory / "imu_timing.png", dpi=180)
    figure.savefig(directory / "imu_timing.svg")
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path)
    parser.add_argument("--compare", type=Path, help="Completed previous run; read only")
    parser.add_argument("--skip-plot", action="store_true", help="Write JSON/Markdown only")
    arguments = parser.parse_args()
    report, rows = analyze_run(arguments.run_directory)
    previous_data = analyze_run(arguments.compare) if arguments.compare else None
    if previous_data:
        report["comparison"] = compare_runs(report, previous_data[0])
    report = json_safe(report)
    if not arguments.skip_plot:
        write_plot(arguments.run_directory, report, rows, previous_data)
    (arguments.run_directory / "imu_timing_analysis.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    write_markdown(arguments.run_directory, report)
    total = report["total"]
    print(json.dumps({"run": report["run"], "samples": total["count"], "received_hz": total["received_hz"],
                      "arrival_gap_gt_0_2_count": total["arrival_gap_gt_0_2_count"],
                      "header_gap_gt_0_2_count": total["header_gap_gt_0_2_count"],
                      "header_nonpositive_count": total["header_nonpositive_count"],
                      "bad_quaternion_norm_count": total["bad_quaternion_norm_count"],
                      "offset_median_ms": total["header_minus_received_sec"]["median"] * 1000,
                      "crc_events": len(report["crc_events"]), "report": str(arguments.run_directory / "imu_timing_analysis.md")}, ensure_ascii=False))


if __name__ == "__main__":
    main()
