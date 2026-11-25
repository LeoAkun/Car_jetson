#!/usr/bin/env python3
"""Run a guarded, ROS-independent CPU thermal test.

The test deliberately does not start or stop any ROS process.  It records the
CPU package temperature and a few useful system counters while applying a
small-to-large sequence of CPU loads.  A temperature limit, missing-sensor
fail-safe, and parent-death signal protect the machine when the test is
interrupted unexpectedly.

Examples (the first command only prints the plan):

    python3 thermal_cpu_test.py --profile conservative
    python3 thermal_cpu_test.py --profile conservative --execute \
        --stop-after half_cores
    python3 thermal_cpu_test.py --monitor-only --duration 300 --execute

Use ``--logical-all`` only after the physical-core run remains below the
warning limit.  The default limit is 78 C, below the CPU's 100 C critical
trip point.  ``--hardware-protection-only`` is reserved for an attended
diagnostic run: it records temperature thresholds but leaves throttling and
thermal shutdown to the CPU/firmware.
"""

from __future__ import annotations

import argparse
import csv
import ctypes
import glob
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import platform
import signal
import sys
import time
import traceback
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Iterable


DEFAULT_OUTPUT_ROOT = Path(__file__).resolve().parent / "thermal_tests"
DEFAULT_MAX_TEMP_C = 78.0
DEFAULT_WARN_TEMP_C = 70.0
DEFAULT_START_TEMP_C = 55.0
DEFAULT_INTERVAL_SEC = 0.5
DEFAULT_FSYNC_INTERVAL_SEC = 5.0

# These are intentionally specific.  A broad ``nav2`` substring would match
# unrelated shell commands and make a harmless thermal test fail its preflight.
NAVIGATION_EXECUTABLES = (
    "component_container_isolated",
    "lio_sam_imuPreintegration",
    "lio_sam_imageProjection",
    "lio_sam_featureExtraction",
    "lio_sam_mapOptimization",
    "pcl_localization_node",
    "nav2_init_pose",
    "navigation_manager",
    "process_manager",
)


@dataclass(frozen=True)
class Phase:
    name: str
    duration_sec: float
    workers: int


def iso_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def boot_id() -> str | None:
    value = read_text("/proc/sys/kernel/random/boot_id")
    return value or None


def read_text(path: str | Path, default: str = "") -> str:
    try:
        return Path(path).read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        return default


def read_number(path: str | Path) -> float | None:
    text = read_text(path)
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        return None


def natural_key(path: str | Path) -> tuple[Any, ...]:
    """Sort paths containing numbers in numeric rather than lexicographic order."""

    parts: list[Any] = []
    for part in str(path).split("/"):
        try:
            parts.append(int(part))
        except ValueError:
            parts.append(part)
    return tuple(parts)


def cpu_model() -> str | None:
    for line in read_text("/proc/cpuinfo").splitlines():
        if line.lower().startswith("model name") and ":" in line:
            return line.split(":", 1)[1].strip()
    return None


def logical_cpu_ids() -> list[int]:
    try:
        available = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        available = list(range(os.cpu_count() or 1))
    return [int(item) for item in available]


def physical_cpu_count() -> int:
    """Return the number of physical cores visible to sysfs when possible."""

    pairs: set[tuple[str, str]] = set()
    for cpu_path in sorted(glob.glob("/sys/devices/system/cpu/cpu[0-9]*"), key=natural_key):
        package = read_text(Path(cpu_path) / "topology/physical_package_id")
        core = read_text(Path(cpu_path) / "topology/core_id")
        if package and core:
            pairs.add((package, core))
    if pairs:
        return len(pairs)
    return max(1, len(logical_cpu_ids()) // 2)


def read_temperatures() -> dict[str, float]:
    values: dict[str, float] = {}
    for zone in sorted(glob.glob("/sys/class/thermal/thermal_zone*"), key=natural_key):
        name = read_text(Path(zone) / "type", Path(zone).name)
        raw = read_number(Path(zone) / "temp")
        if raw is None:
            continue
        # Linux thermal-zone temperatures are normally millidegrees C.  The
        # fallback also handles a few firmware drivers that expose degrees C.
        value = raw / 1000.0 if abs(raw) > 200.0 else raw
        if -50.0 <= value <= 200.0:
            key = name
            if key in values:
                key = f"{name}@{Path(zone).name}"
            values[key] = round(value, 1)
    # coretemp exposes Package id and per-core sensors through hwmon.  On this
    # host the thermal-zone package value can lag one of the individual cores,
    # so include every coretemp input and let the safety selector use the
    # hottest CPU reading.
    for hwmon in sorted(glob.glob("/sys/class/hwmon/hwmon*"), key=natural_key):
        hwmon_name = read_text(Path(hwmon) / "name", Path(hwmon).name)
        if hwmon_name.lower() not in {"coretemp", "k10temp", "zenpower"}:
            continue
        for input_path in sorted(glob.glob(f"{hwmon}/temp*_input"), key=natural_key):
            raw = read_number(input_path)
            if raw is None:
                continue
            value = raw / 1000.0 if abs(raw) > 200.0 else raw
            if not -50.0 <= value <= 200.0:
                continue
            stem = Path(input_path).name.removesuffix("_input")
            label = read_text(Path(hwmon) / f"{stem}_label", stem)
            key = f"{hwmon_name}:{label}"
            if key in values:
                key = f"{key}@{Path(hwmon).name}"
            values[key] = round(value, 1)
    return values


def select_cpu_temperature(temperatures: dict[str, float]) -> tuple[str | None, float | None]:
    """Select a CPU/package sensor, avoiding the unrelated ACPI zone."""

    candidates: list[tuple[str, float]] = []
    for name, value in temperatures.items():
        lower = name.lower()
        is_cpu = (
            lower.startswith("x86_pkg_temp")
            or lower.startswith("coretemp")
            or lower.startswith("k10temp")
            or lower.startswith("zenpower")
            or lower.startswith("cpu_thermal")
            or "package id" in lower
            or lower.startswith("cpu:")
            or lower == "cpu"
            or lower.startswith("cpu@")
        )
        if is_cpu:
            candidates.append((name, value))
    if not candidates:
        return None, None
    # The hottest package/core value is the conservative safety reading.
    return max(candidates, key=lambda item: item[1])


def cpu_temperature_details(temperatures: dict[str, float]) -> dict[str, Any]:
    sensor, hottest = select_cpu_temperature(temperatures)
    packages = [
        value
        for name, value in temperatures.items()
        if name.lower().startswith("x86_pkg_temp") or "package id" in name.lower()
    ]
    cores = [
        value
        for name, value in temperatures.items()
        if name.lower().startswith(("coretemp:core", "k10temp:tdie", "zenpower:tdie"))
    ]
    return {
        "hottest_sensor": sensor,
        "hottest_c": hottest,
        "package_c": max(packages) if packages else None,
        "core_max_c": max(cores) if cores else None,
    }


def read_trip_points() -> dict[str, list[dict[str, Any]]]:
    result: dict[str, list[dict[str, Any]]] = {}
    for zone in sorted(glob.glob("/sys/class/thermal/thermal_zone*"), key=natural_key):
        zone_name = read_text(Path(zone) / "type", Path(zone).name)
        points: list[dict[str, Any]] = []
        for temp_path in sorted(glob.glob(f"{zone}/trip_point_*_temp"), key=natural_key):
            prefix = str(temp_path)[: -len("_temp")]
            raw = read_number(temp_path)
            if raw is None:
                continue
            value = raw / 1000.0 if abs(raw) > 200.0 else raw
            points.append(
                {
                    "type": read_text(f"{prefix}_type"),
                    "temp_c": round(value, 1),
                    "hysteresis_c": round(
                        (read_number(f"{prefix}_hyst") or 0.0)
                        / (1000.0 if abs(read_number(f"{prefix}_hyst") or 0.0) > 200.0 else 1.0),
                        1,
                    ),
                }
            )
        if points:
            result[zone_name] = points
    return result


def read_fans() -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for device in sorted(glob.glob("/sys/class/thermal/cooling_device*"), key=natural_key):
        kind = read_text(Path(device) / "type")
        state = read_text(Path(device) / "cur_state")
        maximum = read_text(Path(device) / "max_state")
        if not kind and not state and not maximum:
            continue
        result[Path(device).name] = {
            "type": kind or None,
            "cur_state": int(state) if state.isdigit() else state or None,
            "max_state": int(maximum) if maximum.isdigit() else maximum or None,
        }
    return result


def read_temperature_alarms() -> dict[str, int]:
    result: dict[str, int] = {}
    for hwmon in sorted(glob.glob("/sys/class/hwmon/hwmon*"), key=natural_key):
        hwmon_name = read_text(Path(hwmon) / "name", Path(hwmon).name)
        if hwmon_name.lower() not in {"coretemp", "k10temp", "zenpower"}:
            continue
        for alarm_path in sorted(glob.glob(f"{hwmon}/temp*_alarm"), key=natural_key):
            value = read_text(alarm_path)
            if value.lstrip("-").isdigit():
                result[f"{hwmon_name}:{Path(alarm_path).name}"] = int(value)
    return result


def read_frequencies() -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    policy_paths = sorted(glob.glob("/sys/devices/system/cpu/cpufreq/policy*"), key=natural_key)
    if not policy_paths:
        policy_paths = sorted(glob.glob("/sys/devices/system/cpu/cpu*/cpufreq"), key=natural_key)
    for policy in policy_paths:
        name = Path(policy).name
        current = read_number(Path(policy) / "scaling_cur_freq")
        if current is None:
            current = read_number(Path(policy) / "cpuinfo_cur_freq")
        result[name] = {
            "current_mhz": round(current / 1000.0, 1) if current is not None else None,
            "governor": read_text(Path(policy) / "scaling_governor") or None,
            "min_mhz": _khz_to_mhz(read_number(Path(policy) / "scaling_min_freq")),
            "max_mhz": _khz_to_mhz(read_number(Path(policy) / "scaling_max_freq")),
            "driver": read_text(Path(policy) / "scaling_driver") or None,
        }
    return result


def _khz_to_mhz(value: float | None) -> float | None:
    return round(value / 1000.0, 1) if value is not None else None


def read_cpu_ticks() -> tuple[int, int] | None:
    first = read_text("/proc/stat").splitlines()
    if not first or not first[0].startswith("cpu "):
        return None
    try:
        values = [int(value) for value in first[0].split()[1:]]
    except ValueError:
        return None
    if len(values) < 4:
        return None
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def cpu_percent(previous: tuple[int, int] | None) -> tuple[float | None, tuple[int, int] | None]:
    current = read_cpu_ticks()
    if previous is None or current is None:
        return None, current
    total_delta = current[0] - previous[0]
    idle_delta = current[1] - previous[1]
    if total_delta <= 0:
        return None, current
    value = 100.0 * (total_delta - idle_delta) / total_delta
    return round(max(0.0, min(100.0, value)), 1), current


def memory_available_mb() -> float | None:
    for line in read_text("/proc/meminfo").splitlines():
        if line.startswith("MemAvailable:"):
            try:
                return round(int(line.split()[1]) / 1024.0, 1)
            except (IndexError, ValueError):
                return None
    return None


def read_load() -> list[float] | None:
    fields = read_text("/proc/loadavg").split()
    try:
        return [round(float(value), 2) for value in fields[:3]]
    except ValueError:
        return None


def read_throttle() -> dict[str, Any]:
    """Read thermal throttling without counting shared sysfs views repeatedly."""

    packages: dict[str, dict[str, int]] = {}
    cores: dict[str, dict[str, int]] = {}
    for cpu_path in sorted(glob.glob("/sys/devices/system/cpu/cpu[0-9]*"), key=natural_key):
        package_id = read_text(Path(cpu_path) / "topology/physical_package_id", "unknown")
        core_id = read_text(Path(cpu_path) / "topology/core_id", Path(cpu_path).name)
        throttle_dir = Path(cpu_path) / "thermal_throttle"
        for kind, target, key in (
            ("package", packages, f"package{package_id}"),
            ("core", cores, f"package{package_id}:core{core_id}"),
        ):
            count = read_number(throttle_dir / f"{kind}_throttle_count")
            time_ms = read_number(throttle_dir / f"{kind}_throttle_total_time_ms")
            if count is None and time_ms is None:
                continue
            current = target.setdefault(key, {"count": 0, "total_time_ms": 0})
            # Sibling logical CPUs expose the same underlying package/core
            # counters.  Keep the maximum view instead of summing duplicates.
            if count is not None:
                current["count"] = max(current["count"], int(count))
            if time_ms is not None:
                current["total_time_ms"] = max(current["total_time_ms"], int(time_ms))

    primary = packages if packages else cores
    return {
        "count": sum(item["count"] for item in primary.values()),
        "total_time_ms": sum(item["total_time_ms"] for item in primary.values()),
        "packages": packages,
        "cores": cores,
    }


def read_rapl_energy_uj() -> int | None:
    values: list[int] = []
    for path in glob.glob("/sys/class/powercap/intel-rapl*/energy_uj"):
        value = read_text(path)
        try:
            values.append(int(value))
        except ValueError:
            continue
    return sum(values) if values else None


def read_navigation_processes() -> list[dict[str, Any]]:
    found: list[dict[str, Any]] = []
    own_pid = os.getpid()
    for path in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            pid = int(path.split("/")[2])
        except (IndexError, ValueError):
            continue
        if pid == own_pid:
            continue
        raw = read_text(path)
        argv = [item for item in raw.split("\x00") if item]
        if not argv:
            continue
        exe = ""
        try:
            exe = os.path.basename(os.readlink(f"/proc/{pid}/exe"))
        except OSError:
            pass
        argv0 = os.path.basename(argv[0])
        matched: list[str] = []
        if exe in NAVIGATION_EXECUTABLES or argv0 in NAVIGATION_EXECUTABLES:
            matched.append(exe or argv0)
        # ROS Python console scripts usually have /usr/bin/python3 as their
        # executable, so match only the exact script basename instead of the
        # entire command text (which would also match diagnostic shells).
        if argv0.startswith("python") and len(argv) > 1:
            script = os.path.basename(argv[1])
            if script in {"navigation_manager.py", "process_manager.py"}:
                matched.append(script)
        # Detect the specific launch commands used by this repository.  Require
        # argv token positions, not a substring in an arbitrary shell command.
        if argv0 == "ros2" and len(argv) >= 4:
            if argv[1:4] == ["launch", "multi_map_navigation", "multi_map_navigation.launch.py"]:
                matched.append("multi_map_navigation_launch")
            elif argv[1:4] in (
                ["launch", "re_localization", "run.real_launch.py"],
                ["launch", "lio_sam", "run.real_launch.py"],
            ):
                matched.append(" ".join(argv[1:4]))
            elif argv[1:4] == ["run", "nav2_init_pose", "nav2_init_pose"]:
                matched.append("nav2_init_pose_run")
        if matched:
            found.append({"pid": pid, "exe": exe or None, "argv": argv, "markers": matched})
    return sorted(found, key=lambda item: int(item["pid"]))


def set_parent_death_signal() -> None:
    """Ask Linux to terminate this worker if the test parent disappears."""

    try:
        libc = ctypes.CDLL(None, use_errno=True)
        prctl = libc.prctl
        prctl.argtypes = [
            ctypes.c_int,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_ulong,
            ctypes.c_ulong,
        ]
        prctl.restype = ctypes.c_int
        # PR_SET_PDEATHSIG = 1; SIGTERM lets the worker leave cleanly.
        prctl(1, signal.SIGTERM, 0, 0, 0)
    except (AttributeError, OSError):
        # The parent-PID polling in ``cpu_worker`` remains as a fallback.
        pass


def cpu_worker(
    phase_event: Any, abort_event: Any, cpu_id: int, deadline_monotonic: float
) -> None:
    """Consume one logical CPU until the phase or test asks us to stop."""

    parent_pid = os.getppid()
    set_parent_death_signal()
    if os.getppid() != parent_pid or parent_pid == 1:
        return
    try:
        os.sched_setaffinity(0, {cpu_id})
    except (AttributeError, OSError):
        pass
    try:
        os.nice(10)
    except OSError:
        pass

    # A pure Python integer loop is intentionally simple and does not allocate
    # memory.  Each process has its own interpreter, so one worker occupies at
    # most one logical CPU without creating a thread storm.
    value = (cpu_id + 1) * 0x9E3779B1
    while (
        time.monotonic() < deadline_monotonic
        and not phase_event.is_set()
        and not abort_event.is_set()
    ):
        if os.getppid() != parent_pid:
            return
        for _ in range(250_000):
            value = (value * 1_664_525 + 1_013_904_223) & 0xFFFFFFFF


def choose_run_dir(output_root: Path, output_dir: str | None) -> Path:
    if output_dir:
        run_dir = Path(output_dir).expanduser()
        run_dir.mkdir(parents=True, exist_ok=True)
        return run_dir
    output_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y-%m-%d_%H-%M-%S")
    run_dir = output_root / stamp
    suffix = 1
    while run_dir.exists():
        run_dir = output_root / f"{stamp}-{suffix:02d}"
        suffix += 1
    run_dir.mkdir()
    return run_dir


def _positive(value: str) -> float:
    number = float(value)
    if number <= 0 or not math.isfinite(number):
        raise argparse.ArgumentTypeError("必须是正数")
    return number


def _nonnegative(value: str) -> float:
    number = float(value)
    if number < 0 or not math.isfinite(number):
        raise argparse.ArgumentTypeError("必须是非负数")
    return number


def _finite(value: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise argparse.ArgumentTypeError("必须是有限数值")
    return number


def durable_write_json(path: Path, value: dict[str, Any]) -> None:
    """Atomically persist small state files used to diagnose an abrupt reboot."""

    temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)
    try:
        directory_fd = os.open(path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    except OSError:
        return
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


class ThermalRunner:
    CSV_FIELDS = (
        "time",
        "boot_id",
        "elapsed_sec",
        "phase",
        "workers",
        "cpu_sensor",
        "cpu_temp_c",
        "max_temp_c",
        "cpu_temperature_details_json",
        "temperatures_json",
        "cpu_percent",
        "load1",
        "load5",
        "load15",
        "mem_available_mb",
        "frequencies_json",
        "fans_json",
        "temperature_alarms_json",
        "throttle_count",
        "throttle_time_ms",
        "package_power_w",
    )

    def __init__(self, args: argparse.Namespace, run_dir: Path, phases: list[Phase]):
        self.args = args
        self.run_dir = run_dir
        self.phases = phases
        self.raw_handle = (run_dir / "samples.jsonl").open("w", encoding="utf-8", buffering=1)
        self.csv_handle = (run_dir / "samples.csv").open(
            "w", encoding="utf-8", newline="", buffering=1
        )
        self.csv_writer = csv.DictWriter(self.csv_handle, fieldnames=self.CSV_FIELDS)
        self.csv_writer.writeheader()
        self.last_fsync = time.monotonic()
        self.previous_ticks = read_cpu_ticks()
        self.previous_energy = read_rapl_energy_uj()
        self.previous_energy_time = time.monotonic()
        self.abort_event = mp.Event()
        self.phase_event: Any | None = None
        self.workers: list[mp.Process] = []
        self.started_monotonic = time.monotonic()
        self.started_at = iso_now()
        self.start_boot_id = boot_id()
        self.reason: str | None = None
        self.status = "planned" if not args.execute else "running"
        self.warning_active = False
        self.max_line_active = False
        # Once a phase reaches the warning line, do not escalate to a hotter
        # phase even if the temperature falls during the subsequent cooldown.
        self.warning_reached = False
        self.warning_phase: str | None = None
        self.missing_sensor_samples = 0
        self.samples_by_phase: dict[str, list[dict[str, Any]]] = {}
        self.phase_stats: list[dict[str, Any]] = []
        self.throttle_start = read_throttle()
        self.throttle_end = self.throttle_start
        self.max_temp_c: float | None = None
        self.write_metadata()
        self.write_run_state("running")

    def write_metadata(self) -> None:
        metadata = {
            "created_at": self.started_at,
            "boot_id": boot_id(),
            "command": sys.argv,
            "profile": self.args.profile,
            "execute": bool(self.args.execute),
            "monitor_only": bool(self.args.monitor_only),
            "ab_label": self.args.ab_label,
            "ambient_c": self.args.ambient_c,
            "interval_sec": self.args.interval,
            "max_temp_c": self.args.max_temp,
            "warn_temp_c": self.args.warn_temp,
            "start_temp_limit_c": self.args.start_temp_limit,
            "software_temperature_stop_enabled": not self.args.hardware_protection_only,
            "fsync_interval_sec": self.args.fsync_interval,
            "cpu_model": cpu_model(),
            "kernel": platform.release(),
            "logical_cpus": logical_cpu_ids(),
            "physical_core_count": physical_cpu_count(),
            "temperatures_initial": read_temperatures(),
            "cpu_temperature_initial": select_cpu_temperature(read_temperatures()),
            "cpu_temperature_details_initial": cpu_temperature_details(read_temperatures()),
            "trip_points": read_trip_points(),
            "fans_initial": read_fans(),
            "temperature_alarms_initial": read_temperature_alarms(),
            "frequencies_initial": read_frequencies(),
            "throttle_initial": self.throttle_start,
            "navigation_processes": read_navigation_processes(),
            "phases": [phase.__dict__ for phase in self.phases],
        }
        durable_write_json(self.run_dir / "metadata.json", metadata)
        self.emit({"event": "test_created", "metadata": metadata})

    def write_run_state(self, status: str) -> None:
        durable_write_json(
            self.run_dir / "run_state.json",
            {
                "status": status,
                "reason": self.reason,
                "pid": os.getpid(),
                "started_at": self.started_at,
                "updated_at": iso_now(),
                "boot_id": boot_id(),
                "software_temperature_stop_enabled": not self.args.hardware_protection_only,
            },
        )

    def emit(self, record: dict[str, Any]) -> None:
        record = {"time": iso_now(), **record}
        self.raw_handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
        if record.get("event") == "sample":
            sample = record["sample"]
            loads = sample.get("load") or [None, None, None]
            power = sample.get("package_power_w")
            row = {
                "time": record["time"],
                "boot_id": sample.get("boot_id"),
                "elapsed_sec": sample.get("elapsed_sec"),
                "phase": sample.get("phase"),
                "workers": sample.get("workers"),
                "cpu_sensor": sample.get("cpu_sensor"),
                "cpu_temp_c": sample.get("cpu_temp_c"),
                "max_temp_c": sample.get("max_temp_c"),
                "cpu_temperature_details_json": json.dumps(
                    sample.get("cpu_temperature_details", {}), ensure_ascii=False
                ),
                "temperatures_json": json.dumps(sample.get("temperatures_c", {}), ensure_ascii=False),
                "cpu_percent": sample.get("cpu_percent"),
                "load1": loads[0],
                "load5": loads[1],
                "load15": loads[2],
                "mem_available_mb": sample.get("mem_available_mb"),
                "frequencies_json": json.dumps(sample.get("frequencies", {}), ensure_ascii=False),
                "fans_json": json.dumps(sample.get("fans", {}), ensure_ascii=False),
                "temperature_alarms_json": json.dumps(
                    sample.get("temperature_alarms", {}), ensure_ascii=False
                ),
                "throttle_count": (sample.get("throttle") or {}).get("count"),
                "throttle_time_ms": (sample.get("throttle") or {}).get("total_time_ms"),
                "package_power_w": power,
            }
            self.csv_writer.writerow(row)
        now = time.monotonic()
        if record.get("event") != "sample" or now - self.last_fsync >= self.args.fsync_interval:
            self.raw_handle.flush()
            self.csv_handle.flush()
            try:
                os.fsync(self.raw_handle.fileno())
                os.fsync(self.csv_handle.fileno())
            except OSError:
                pass
            self.last_fsync = now

    def snapshot(self, phase: Phase, phase_started: float) -> dict[str, Any]:
        temperatures = read_temperatures()
        cpu_sensor, cpu_temp = select_cpu_temperature(temperatures)
        cpu_details = cpu_temperature_details(temperatures)
        max_temp = max(temperatures.values()) if temperatures else None
        percent, self.previous_ticks = cpu_percent(self.previous_ticks)
        now = time.monotonic()
        energy = read_rapl_energy_uj()
        power = None
        if energy is not None and self.previous_energy is not None:
            elapsed = now - self.previous_energy_time
            if elapsed > 0 and energy >= self.previous_energy:
                power = round((energy - self.previous_energy) / elapsed / 1_000_000.0, 2)
        if energy is not None:
            self.previous_energy = energy
            self.previous_energy_time = now
        throttle = read_throttle()
        self.throttle_end = throttle
        if cpu_temp is not None:
            self.max_temp_c = cpu_temp if self.max_temp_c is None else max(self.max_temp_c, cpu_temp)
            self.missing_sensor_samples = 0
        else:
            self.missing_sensor_samples += 1
        sample = {
            "boot_id": boot_id(),
            "elapsed_sec": round(now - phase_started, 3),
            "test_elapsed_sec": round(now - self.started_monotonic, 3),
            "phase": phase.name,
            "workers": phase.workers,
            "cpu_sensor": cpu_sensor,
            "cpu_temp_c": cpu_temp,
            "cpu_temperature_details": cpu_details,
            "max_temp_c": round(max_temp, 1) if max_temp is not None else None,
            "temperatures_c": temperatures,
            "cpu_percent": percent,
            "load": read_load(),
            "mem_available_mb": memory_available_mb(),
            "frequencies": read_frequencies(),
            "fans": read_fans(),
            "temperature_alarms": read_temperature_alarms(),
            "throttle": throttle,
            "package_power_w": power,
        }
        self.samples_by_phase.setdefault(phase.name, []).append(sample)
        self.emit({"event": "sample", "sample": sample})
        temp_text = "n/a" if cpu_temp is None else f"{cpu_temp:.1f}C"
        print(
            f"[{phase.name:>14}] temp={temp_text:>6} cpu={percent if percent is not None else 'n/a':>5} "
            f"workers={phase.workers:>2}",
            flush=True,
        )
        return sample

    def check_safety(self, sample: dict[str, Any]) -> bool:
        cpu_temp = sample.get("cpu_temp_c")
        if cpu_temp is None:
            # The sample reader already counted this missing reading. Never
            # continue a loaded phase without a trustworthy CPU sensor.
            self.emit(
                {
                    "event": "thermal_abort",
                    "reason": "cpu_temperature_sensor_unavailable",
                    "missing_samples": self.missing_sensor_samples,
                }
            )
            self.abort("cpu_temperature_sensor_unavailable")
            return False
        if cpu_temp >= self.args.max_temp and not self.args.hardware_protection_only:
            self.emit(
                {
                    "event": "thermal_abort",
                    "reason": "temperature_limit",
                    "cpu_temp_c": cpu_temp,
                    "limit_c": self.args.max_temp,
                }
            )
            self.abort(f"temperature_limit_{cpu_temp:.1f}C")
            return False
        if cpu_temp >= self.args.max_temp and not self.max_line_active:
            self.max_line_active = True
            self.emit(
                {
                    "event": "temperature_line_observed",
                    "cpu_temp_c": cpu_temp,
                    "line_c": self.args.max_temp,
                    "action": "record_only_hardware_protection_mode",
                }
            )
        elif cpu_temp < self.args.max_temp - 2.0:
            self.max_line_active = False
        if cpu_temp >= self.args.warn_temp and not self.warning_active:
            self.warning_active = True
            self.warning_reached = True
            self.warning_phase = self.warning_phase or str(sample.get("phase"))
            self.emit(
                {
                    "event": "thermal_warning",
                    "cpu_temp_c": cpu_temp,
                    "warning_c": self.args.warn_temp,
                }
            )
            if sample.get("workers", 0) and not self.args.hardware_protection_only:
                # Stop the active load at the warning line.  Thermal inertia
                # can carry the package several degrees higher after workers
                # are removed, so waiting for the phase deadline is unsafe.
                self.abort(f"warning_limit_{sample.get('phase')}")
                return False
        elif cpu_temp < self.args.warn_temp - 2.0:
            self.warning_active = False
        return True

    def abort(self, reason: str) -> None:
        if self.reason is None:
            self.reason = reason
        self.abort_event.set()

    def start_workers(self, phase: Phase, deadline_monotonic: float) -> None:
        self.phase_event = mp.Event()
        cpu_ids = logical_cpu_ids()
        count = min(phase.workers, len(cpu_ids))
        self.workers = []
        for index in range(count):
            worker = mp.Process(
                target=cpu_worker,
                args=(
                    self.phase_event,
                    self.abort_event,
                    cpu_ids[index],
                    deadline_monotonic,
                ),
                name=f"thermal-cpu-{index}",
                daemon=True,
            )
            worker.start()
            self.workers.append(worker)
            # Stagger process creation slightly to avoid a single launch spike.
            if index + 1 < count:
                time.sleep(0.1)
        self.emit(
            {
                "event": "workers_started",
                "phase": phase.name,
                "requested": phase.workers,
                "started": len(self.workers),
                "pids": [worker.pid for worker in self.workers],
            }
        )

    def stop_workers(self) -> None:
        if self.phase_event is not None:
            self.phase_event.set()
        for worker in self.workers:
            worker.join(timeout=2.0)
        for worker in self.workers:
            if worker.is_alive():
                worker.terminate()
        for worker in self.workers:
            worker.join(timeout=1.0)
        if self.workers:
            self.emit(
                {
                    "event": "workers_stopped",
                    "pids": [worker.pid for worker in self.workers],
                    "alive_after_stop": [worker.pid for worker in self.workers if worker.is_alive()],
                }
            )
        self.workers = []
        self.phase_event = None

    def run_phase(self, phase: Phase, *, enforce_safety: bool = True) -> None:
        phase_started = time.monotonic()
        self.samples_by_phase.setdefault(phase.name, [])
        self.emit(
            {
                "event": "phase_started",
                "phase": phase.name,
                "duration_sec": phase.duration_sec,
                "workers": phase.workers,
            }
        )
        deadline = phase_started + phase.duration_sec
        if phase.workers:
            self.start_workers(phase, deadline)
        while time.monotonic() < deadline and (not self.abort_event.is_set() or not enforce_safety):
            sample = self.snapshot(phase, phase_started)
            if enforce_safety and not self.check_safety(sample):
                break
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(min(self.args.interval, remaining))
        self.stop_workers()
        stats = self.calculate_phase_stats(phase, phase_started)
        self.phase_stats.append(stats)
        self.emit({"event": "phase_finished", "phase": phase.name, "stats": stats})

    def calculate_phase_stats(self, phase: Phase, phase_started: float) -> dict[str, Any]:
        samples = self.samples_by_phase.get(phase.name, [])
        temps = [float(item["cpu_temp_c"]) for item in samples if item.get("cpu_temp_c") is not None]
        cpu_values = [float(item["cpu_percent"]) for item in samples if item.get("cpu_percent") is not None]
        return {
            "phase": phase.name,
            "workers": phase.workers,
            "planned_sec": phase.duration_sec,
            "observed_sec": round(time.monotonic() - phase_started, 3),
            "samples": len(samples),
            "start_temp_c": temps[0] if temps else None,
            "peak_temp_c": max(temps) if temps else None,
            "end_temp_c": temps[-1] if temps else None,
            "average_temp_c": round(sum(temps) / len(temps), 2) if temps else None,
            "peak_cpu_percent": max(cpu_values) if cpu_values else None,
            "warning_reached": any(value >= self.args.warn_temp for value in temps),
        }

    def run_cooldown(self, duration_sec: float, name: str) -> None:
        """Record a no-load cooldown even after an abort event is set."""

        if duration_sec <= 0:
            return
        was_aborted = self.abort_event.is_set()
        if was_aborted:
            self.abort_event.clear()
        try:
            self.run_phase(Phase(name, duration_sec, 0), enforce_safety=False)
        finally:
            if was_aborted:
                self.abort_event.set()

    def run(self) -> int:
        if not self.args.execute:
            self.status = "planned"
            print("未执行高负载；以上计划仅供检查。需要实际测试时请加 --execute。")
            self.reason = "dry_run"
            return 0

        navigation = read_navigation_processes()
        if navigation and not self.args.allow_navigation:
            self.status = "aborted"
            self.reason = "navigation_processes_detected"
            self.emit({"event": "preflight_abort", "reason": self.reason, "processes": navigation})
            print("检测到导航进程，未启动 CPU 负载；如确认已驻车可用 --allow-navigation。", file=sys.stderr)
            return 3

        initial_temps = read_temperatures()
        _, initial_cpu_temp = select_cpu_temperature(initial_temps)
        if initial_cpu_temp is None:
            self.status = "aborted"
            self.reason = "cpu_temperature_sensor_unavailable"
            self.emit({"event": "preflight_abort", "reason": self.reason})
            print("找不到 CPU 温度传感器，出于安全原因不启动负载。", file=sys.stderr)
            return 3
        if initial_cpu_temp >= self.args.max_temp and not self.args.hardware_protection_only:
            self.status = "aborted"
            self.reason = f"initial_temperature_limit_{initial_cpu_temp:.1f}C"
            self.emit(
                {
                    "event": "preflight_abort",
                    "reason": "initial_temperature_limit",
                    "cpu_temp_c": initial_cpu_temp,
                    "limit_c": self.args.max_temp,
                }
            )
            print(f"初始 CPU 温度 {initial_cpu_temp:.1f}C 已达到上限，未启动负载。", file=sys.stderr)
            return 3
        self.emit({"event": "test_started", "cpu_temp_c": initial_cpu_temp})
        try:
            for phase in self.phases:
                self.run_phase(phase)
                if self.abort_event.is_set():
                    break
                phase_stats = self.phase_stats[-1] if self.phase_stats else None
                # Python/process initialization can briefly heat one core.  Use
                # the end of the no-load baseline for the cold-start gate so a
                # transient cannot hide a genuinely warm, steady-state CPU.
                if (
                    phase.name == "baseline"
                    and not self.args.hardware_protection_only
                ):
                    recent_temps = [
                        float(item["cpu_temp_c"])
                        for item in self.samples_by_phase.get(phase.name, [])[-5:]
                        if item.get("cpu_temp_c") is not None
                    ]
                    stabilized_temp = max(recent_temps) if recent_temps else None
                    if stabilized_temp is None or stabilized_temp > self.args.start_temp_limit:
                        self.reason = (
                            "baseline_temperature_unavailable"
                            if stabilized_temp is None
                            else f"baseline_temperature_above_start_limit_{stabilized_temp:.1f}C"
                        )
                        self.emit(
                            {
                                "event": "escalation_blocked",
                                "phase": phase.name,
                                "reason": self.reason,
                                "stabilized_cpu_temp_c": stabilized_temp,
                                "start_limit_c": self.args.start_temp_limit,
                            }
                        )
                        print(
                            "零负载基线结束温度不可用，未启动 CPU 负载。"
                            if stabilized_temp is None
                            else (
                                f"零负载基线结束温度 {stabilized_temp:.1f}C 高于启动门槛 "
                                f"{self.args.start_temp_limit:.1f}C，未启动 CPU 负载。"
                            ),
                            file=sys.stderr,
                        )
                        break
                if (
                    not self.args.monitor_only
                    and not self.args.hardware_protection_only
                    and phase_stats
                    and phase_stats.get("warning_reached")
                ):
                    # A warning is a hard gate for escalation.  The operator
                    # can inspect the phase log and choose a lower-power run;
                    # the script must not proceed automatically to a hotter
                    # phase.
                    self.reason = f"warning_limit_reached_{phase.name}"
                    self.emit(
                        {
                            "event": "escalation_blocked",
                            "phase": phase.name,
                            "peak_temp_c": phase_stats.get("peak_temp_c"),
                            "warning_c": self.args.warn_temp,
                        }
                    )
                    break
                if phase.workers and self.args.settle_sec > 0:
                    settle = Phase(f"settle_after_{phase.name}", self.args.settle_sec, 0)
                    self.run_phase(settle)
                    if self.abort_event.is_set():
                        break
            if self.reason is None:
                self.status = "completed"
                self.reason = "all_phases_completed"
            else:
                self.status = "aborted"
                # Always observe cooldown after a safety stop; no workers are
                # started in this phase.
                self.run_cooldown(self.args.cooldown_sec or 0.0, "emergency_cooldown")
        except KeyboardInterrupt:
            self.abort("keyboard_interrupt")
            self.status = "aborted"
        except Exception as exc:  # pragma: no cover - defensive finalization
            self.abort(f"exception_{type(exc).__name__}")
            self.status = "aborted"
            (self.run_dir / "error.log").write_text(traceback.format_exc(), encoding="utf-8")
            raise
        finally:
            self.stop_workers()
            self.throttle_end = read_throttle()
        return 0 if self.status == "completed" else 3

    def write_summary(self) -> None:
        ended_at = iso_now()
        end_boot_id = boot_id()
        summary = {
            "status": self.status,
            "reason": self.reason,
            "ab_label": self.args.ab_label,
            "ambient_c": self.args.ambient_c,
            "started_at": self.started_at,
            "ended_at": ended_at,
            "boot_id_at_start": self.start_boot_id,
            "boot_id_at_end": end_boot_id,
            "boot_id_changed": self.start_boot_id != end_boot_id,
            "duration_sec": round(time.monotonic() - self.started_monotonic, 3),
            "max_cpu_temp_c": self.max_temp_c,
            "limits": {
                "warning_c": self.args.warn_temp,
                "abort_c": self.args.max_temp,
                "start_c": self.args.start_temp_limit,
                "software_temperature_stop_enabled": not self.args.hardware_protection_only,
            },
            "throttle_start": self.throttle_start,
            "throttle_end": self.throttle_end,
            "throttle_delta": {
                "count": self.throttle_end["count"] - self.throttle_start["count"],
                "total_time_ms": self.throttle_end["total_time_ms"]
                - self.throttle_start["total_time_ms"],
            },
            "fans_final": read_fans(),
            "temperature_alarms_final": read_temperature_alarms(),
            "frequencies_final": read_frequencies(),
            "phases": self.phase_stats,
        }
        durable_write_json(self.run_dir / "summary.json", summary)
        lines = [
            "# CPU Thermal Test Summary",
            "",
            f"- 状态: **{summary['status']}**",
            f"- 原因: `{summary['reason']}`",
            f"- 开始: {summary['started_at']}",
            f"- 结束: {summary['ended_at']}",
            f"- boot_id 变化: `{summary['boot_id_changed']}`",
            f"- A/B 标签: `{self.args.ab_label}`",
            f"- 室温: {self.args.ambient_c if self.args.ambient_c is not None else '未填写'} C",
            f"- CPU 最高温度: {summary['max_cpu_temp_c']} C",
            (
                "- 软件温度停止: **禁用**（温度线仅记录，CPU/固件保护保留）"
                if self.args.hardware_protection_only
                else f"- 保护上限: {self.args.max_temp:.1f} C（警告 {self.args.warn_temp:.1f} C）"
            ),
            f"- 节流计数增量: {summary['throttle_delta']['count']}",
            "",
            "| 阶段 | worker 数 | 样本数 | 起始 C | 峰值 C | 结束 C | 平均 C | 峰值 CPU |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
        for phase in self.phase_stats:
            lines.append(
                "| {phase} | {workers} | {samples} | {start} | {peak} | {end} | {avg} | {cpu} |".format(
                    phase=phase["phase"],
                    workers=phase["workers"],
                    samples=phase["samples"],
                    start=phase["start_temp_c"],
                    peak=phase["peak_temp_c"],
                    end=phase["end_temp_c"],
                    avg=phase["average_temp_c"],
                    cpu=phase["peak_cpu_percent"],
                )
            )
        lines += [
            "",
            "原始数据: `samples.jsonl`；表格: `samples.csv`；硬件/传感器快照: `metadata.json`。",
            "风扇 cooling_device 状态仅表示内核请求，不等同于物理 RPM；请结合现场声音或 BIOS RPM 判断。",
        ]
        (self.run_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def close(self) -> None:
        try:
            self.write_summary()
            self.write_run_state(self.status)
        finally:
            self.raw_handle.close()
            self.csv_handle.close()


def make_phases(args: argparse.Namespace) -> list[Phase]:
    logical = len(logical_cpu_ids())
    physical = min(physical_cpu_count(), logical)
    default_max_workers = logical if args.logical_all else physical
    max_workers = args.max_workers if args.max_workers is not None else default_max_workers
    max_workers = max(1, min(max_workers, logical))
    full_workers = logical if args.logical_all else physical
    full_workers = min(full_workers, max_workers)
    half_workers = min(max_workers, max(1, physical // 2))
    if args.profile == "quick":
        baseline, one, half, full, cooldown = 15.0, 30.0, 45.0, 60.0, 60.0
    else:
        baseline, one, half, full, cooldown = 30.0, 90.0, 90.0, 120.0, 120.0
    phases = [
        Phase("baseline", baseline, 0),
        Phase("one_core", one, min(1, max_workers)),
        Phase("half_cores", half, half_workers),
        Phase("physical_cores", full, min(physical, max_workers)),
    ]
    if args.logical_all:
        phases.append(Phase("logical_cores", full, full_workers))
    if args.stop_after:
        names = [phase.name for phase in phases]
        if args.stop_after not in names:
            raise ValueError(f"未知 --stop-after 阶段: {args.stop_after}")
        phases = phases[: names.index(args.stop_after) + 1]
    if args.final_duration is not None:
        phases[-1] = Phase(phases[-1].name, args.final_duration, phases[-1].workers)
    if args.cooldown_sec is None:
        args.cooldown_sec = cooldown
    return phases


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="安全、独立于 ROS 的 CPU 温度分阶段测试")
    parser.add_argument(
        "--profile", choices=("conservative", "quick"), default="conservative", help="测试时长配置"
    )
    parser.add_argument("--execute", action="store_true", help="真正启动 CPU 负载；不加则只打印计划")
    parser.add_argument("--monitor-only", action="store_true", help="只采温度，不启动任何 CPU worker")
    parser.add_argument("--duration", type=_positive, default=300.0, help="monitor-only 持续秒数")
    parser.add_argument("--interval", type=_positive, default=DEFAULT_INTERVAL_SEC, help="采样间隔（秒）")
    parser.add_argument("--max-temp", type=_finite, default=DEFAULT_MAX_TEMP_C, help="CPU 温度保护上限（C）")
    parser.add_argument("--warn-temp", type=_finite, default=DEFAULT_WARN_TEMP_C, help="CPU 温度警告线（C）")
    parser.add_argument(
        "--start-temp-limit",
        type=_finite,
        default=DEFAULT_START_TEMP_C,
        help="开始负载前的最高允许温度（C）",
    )
    parser.add_argument("--max-workers", type=int, default=None, help="最多启动的 worker 数")
    parser.add_argument("--logical-all", action="store_true", help="最后阶段使用所有逻辑 CPU")
    parser.add_argument("--final-duration", type=_positive, default=None, help="覆盖最后负载阶段的持续秒数")
    parser.add_argument(
        "--stop-after",
        choices=("baseline", "one_core", "half_cores", "physical_cores", "logical_cores"),
        help="在指定阶段结束后进入冷却并停止",
    )
    parser.add_argument("--settle-sec", type=_nonnegative, default=15.0, help="阶段间无负载观察秒数")
    parser.add_argument("--cooldown-sec", type=_nonnegative, default=None, help="安全停止后的冷却秒数")
    parser.add_argument(
        "--fsync-interval",
        type=_positive,
        default=DEFAULT_FSYNC_INTERVAL_SEC,
        help="原始样本强制落盘间隔（秒）",
    )
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT, help="测试目录根路径")
    parser.add_argument("--output-dir", help="指定本次测试的完整输出目录")
    parser.add_argument(
        "--ab-label",
        default="unspecified",
        help="散热条件标签，例如 external_fan_on 或 external_fan_off",
    )
    parser.add_argument("--ambient-c", type=_finite, default=None, help="室温（C，可选）")
    parser.add_argument("--allow-navigation", action="store_true", help="允许检测到导航进程时继续（不推荐）")
    parser.add_argument(
        "--hardware-protection-only",
        "--no-software-temp-stop",
        dest="hardware_protection_only",
        action="store_true",
        help="温度线只记录、不停止负载；保留 CPU/固件热保护",
    )
    return parser


def print_plan(args: argparse.Namespace, phases: Iterable[Phase], run_dir: Path) -> None:
    print(f"输出目录: {run_dir}")
    print(f"CPU: {cpu_model() or 'unknown'}; 逻辑 CPU: {len(logical_cpu_ids())}; 物理核心: {physical_cpu_count()}")
    if args.hardware_protection_only:
        print(
            f"软件温度停止: 禁用; 记录线: {args.warn_temp:.1f}/{args.max_temp:.1f} C; "
            "CPU/固件热保护: 保留"
        )
    else:
        print(
            f"保护: {args.max_temp:.1f} C; 警告: {args.warn_temp:.1f} C; "
            f"启动门槛: {args.start_temp_limit:.1f} C"
        )
    for phase in phases:
        print(f"  {phase.name:>14}: {phase.duration_sec:.0f}s, workers={phase.workers}")
    if args.settle_sec:
        print(f"  阶段间冷却观察: {args.settle_sec:.0f}s")
    if args.monitor_only:
        print(f"  monitor-only: {args.duration:.0f}s")


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.interval < 0.5 or args.interval > 2.0:
        parser.error("--interval 建议并限制在 0.5 到 2 秒之间")
    if args.max_temp <= args.warn_temp:
        parser.error("--max-temp 必须高于 --warn-temp")
    if args.max_temp > 90.0 and not args.hardware_protection_only:
        parser.error("为避免越过 CPU 临界点，--max-temp 不得高于 90C")
    if args.start_temp_limit >= args.max_temp:
        parser.error("--start-temp-limit 必须低于 --max-temp")
    if args.max_workers is not None and args.max_workers < 1:
        parser.error("--max-workers 必须至少为 1")
    if args.monitor_only:
        phases = [Phase("monitor_only", args.duration, 0)]
        if args.cooldown_sec is None:
            args.cooldown_sec = 60.0
    else:
        try:
            phases = make_phases(args)
        except ValueError as exc:
            parser.error(str(exc))
    run_dir = choose_run_dir(args.output_root, args.output_dir)
    runner = ThermalRunner(args, run_dir, phases)
    print_plan(args, phases, run_dir)
    exit_code = 0
    try:
        if args.monitor_only and args.execute:
            exit_code = runner.run()
        elif args.monitor_only and not args.execute:
            runner.reason = "dry_run"
            print("未执行 monitor-only；需要采样时请加 --execute。")
        else:
            exit_code = runner.run()
    except KeyboardInterrupt:
        runner.abort("keyboard_interrupt")
        runner.status = "aborted"
        exit_code = 3
    finally:
        runner.close()
    print(f"结果目录: {run_dir}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
