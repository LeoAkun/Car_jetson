#!/usr/bin/env python3
"""Persist one-second system samples to investigate abrupt idle reboots."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
import signal
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RUNS = ROOT / "idle_crash_runs"


def read(path: Path, default: str = "") -> str:
    try:
        return path.read_text().strip()
    except (OSError, ValueError):
        return default


def boot_id() -> str:
    return read(Path("/proc/sys/kernel/random/boot_id"), "unknown")


def atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(".tmp")
    with temporary.open("w") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def cpu_times() -> tuple[int, int]:
    values = [int(item) for item in Path("/proc/stat").read_text().splitlines()[0].split()[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def temperatures() -> tuple[float | None, float | None, float | None]:
    package = None
    core_max = None
    pch = None
    for directory in Path("/sys/class/hwmon").glob("hwmon*"):
        chip = read(directory / "name")
        for source in directory.glob("temp*_input"):
            stem = source.name.removesuffix("_input")
            label = read(directory / f"{stem}_label", stem)
            try:
                value = int(source.read_text()) / 1000.0
            except (OSError, ValueError):
                continue
            if chip == "coretemp" and "Package id" in label:
                package = value
            elif chip == "coretemp" and label.startswith("Core"):
                core_max = value if core_max is None else max(core_max, value)
            elif chip == "pch_cannonlake":
                pch = value
    return package, core_max, pch


def avg_frequency_mhz() -> float | None:
    values = []
    for source in Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq"):
        try:
            values.append(int(source.read_text()) / 1000.0)
        except (OSError, ValueError):
            pass
    return sum(values) / len(values) if values else None


def meminfo() -> tuple[int, int]:
    values = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        key, raw = line.split(":", 1)
        values[key] = int(raw.strip().split()[0])
    return values.get("MemAvailable", 0), values.get("SwapFree", 0)


def throttle_count(kind: str) -> int:
    values = []
    for source in Path("/sys/devices/system/cpu").glob(
        f"cpu[0-9]*/thermal_throttle/{kind}_throttle_count"
    ):
        try:
            values.append(int(source.read_text()))
        except (OSError, ValueError):
            pass
    return max(values, default=0)


def cstate_time(name: str) -> int:
    total = 0
    for state in Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpuidle/state*"):
        if read(state / "name") != name:
            continue
        try:
            total += int((state / "time").read_text())
        except (OSError, ValueError):
            pass
    return total


def robot_processes() -> int:
    count = 0
    for process in Path("/proc").glob("[0-9]*"):
        command = read(process / "cmdline").replace("\x00", " ")
        if any(token in command for token in ("start_robot_service", "ros2 launch", "/real_ws/")):
            count += 1
    return count


def fmt(value: float | None) -> str:
    return "" if value is None else f"{value:.1f}"


def mark_previous_run_if_rebooted(latest: Path, new_boot_id: str) -> None:
    """Close a prior RUNNING run when this process starts after a host reboot."""
    try:
        previous_dir = latest.resolve(strict=True)
        state_path = previous_dir / "state.json"
        state = json.loads(state_path.read_text())
    except (OSError, ValueError, json.JSONDecodeError):
        return
    if state.get("status") != "RUNNING" or state.get("boot_id") in (None, new_boot_id):
        return
    state.update({
        "status": "ABNORMAL_REBOOT_DETECTED",
        "reason": "monitor restarted with a different host boot ID",
        "detected_next_boot_id": new_boot_id,
        "detected_next_boot_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
    })
    atomic_json(state_path, state)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=int, default=43200, help="monitor duration in seconds")
    parser.add_argument("--interval", type=float, default=1.0, help="sample interval in seconds")
    parser.add_argument("--label", default="idle_watch", help="run label")
    args = parser.parse_args()
    if args.duration < 1 or args.interval <= 0:
        parser.error("duration and interval must be positive")

    RUNS.mkdir(parents=True, exist_ok=True)
    current_boot = boot_id()
    latest = ROOT / "latest_idle_crash_run"
    mark_previous_run_if_rebooted(latest, current_boot)
    stamp = dt.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    run_dir = RUNS / f"{stamp}_{args.label}"
    run_dir.mkdir()
    latest.unlink(missing_ok=True)
    latest.symlink_to(run_dir.relative_to(ROOT))

    state = {
        "status": "RUNNING",
        "started_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "boot_id": current_boot,
        "duration_s": args.duration,
        "interval_s": args.interval,
        "label": args.label,
    }
    atomic_json(run_dir / "state.json", state)

    stopped = False

    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    fields = [
        "timestamp", "elapsed_s", "boot_id", "cpu_busy_pct", "load1", "load5",
        "freq_avg_mhz", "package_c", "core_max_c", "pch_c", "mem_available_kib",
        "swap_free_kib", "package_throttle_count", "core_throttle_count",
        "c6_time_us", "c8_time_us", "c10_time_us", "robot_processes",
    ]
    start = time.monotonic()
    previous_total, previous_idle = cpu_times()
    sample_count = 0
    status = "COMPLETE"
    reason = "duration elapsed"

    with (run_dir / "samples.csv").open("w", newline="") as samples:
        writer = csv.DictWriter(samples, fieldnames=fields)
        writer.writeheader()
        samples.flush()
        os.fsync(samples.fileno())
        while True:
            elapsed = time.monotonic() - start
            total, idle = cpu_times()
            delta_total = total - previous_total
            delta_idle = idle - previous_idle
            busy = 100.0 * (delta_total - delta_idle) / delta_total if delta_total else 0.0
            previous_total, previous_idle = total, idle
            package, core_max, pch = temperatures()
            memory, swap = meminfo()
            loads = os.getloadavg()
            now_boot = boot_id()
            row = {
                "timestamp": dt.datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "elapsed_s": f"{elapsed:.1f}",
                "boot_id": now_boot,
                "cpu_busy_pct": f"{busy:.1f}",
                "load1": f"{loads[0]:.2f}",
                "load5": f"{loads[1]:.2f}",
                "freq_avg_mhz": fmt(avg_frequency_mhz()),
                "package_c": fmt(package),
                "core_max_c": fmt(core_max),
                "pch_c": fmt(pch),
                "mem_available_kib": memory,
                "swap_free_kib": swap,
                "package_throttle_count": throttle_count("package"),
                "core_throttle_count": throttle_count("core"),
                "c6_time_us": cstate_time("C6"),
                "c8_time_us": cstate_time("C8"),
                "c10_time_us": cstate_time("C10"),
                "robot_processes": robot_processes(),
            }
            writer.writerow(row)
            samples.flush()
            os.fsync(samples.fileno())
            sample_count += 1
            state.update({
                "last_sample_at": row["timestamp"],
                "last_elapsed_s": round(elapsed, 1),
                "last_package_c": package,
                "last_cpu_busy_pct": round(busy, 1),
                "last_mem_available_kib": memory,
                "samples": sample_count,
            })
            atomic_json(run_dir / "state.json", state)

            if sample_count == 1 or sample_count % 10 == 0:
                print(
                    f"{row['timestamp']} elapsed={elapsed:.0f}s cpu={busy:.1f}% "
                    f"pkg={fmt(package)}C mem={memory / 1048576:.1f}GiB "
                    f"robot_pids={row['robot_processes']}",
                    flush=True,
                )
            if now_boot != current_boot:
                status = "BOOT_ID_CHANGED"
                reason = f"boot ID changed from {current_boot} to {now_boot}"
                break
            if stopped:
                status = "INTERRUPTED"
                reason = "received stop signal"
                break
            if elapsed >= args.duration:
                break
            time.sleep(min(args.interval, max(0.0, args.duration - elapsed)))

    state.update({
        "status": status,
        "reason": reason,
        "finished_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "boot_id_at_finish": boot_id(),
    })
    atomic_json(run_dir / "state.json", state)
    print(f"{status}: {reason}; log={run_dir}", flush=True)
    return 0 if status == "COMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
