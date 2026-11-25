#!/usr/bin/env python3
"""Run one temperature-test stage with persistent, reboot-survivable logging."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
import platform
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RUNS = ROOT / "runs"


def read_text(path: Path, default: str = "") -> str:
    try:
        return path.read_text().strip()
    except (OSError, ValueError):
        return default


def boot_id() -> str:
    return read_text(Path("/proc/sys/kernel/random/boot_id"), "unknown")


def temperatures() -> dict[str, float]:
    result: dict[str, float] = {}
    for hwmon in Path("/sys/class/hwmon").glob("hwmon*"):
        chip = read_text(hwmon / "name", hwmon.name)
        for source in hwmon.glob("temp*_input"):
            stem = source.name.removesuffix("_input")
            label = read_text(hwmon / f"{stem}_label", stem)
            raw = read_text(source)
            try:
                result[f"{chip}:{label}"] = int(raw) / 1000.0
            except ValueError:
                pass
    return result


def pick_temperature(values: dict[str, float], contains: str) -> float | None:
    matches = [value for key, value in values.items() if contains.lower() in key.lower()]
    return max(matches) if matches else None


def cpu_times() -> tuple[int, int]:
    fields = Path("/proc/stat").read_text().splitlines()[0].split()[1:]
    ticks = [int(value) for value in fields]
    idle = ticks[3] + (ticks[4] if len(ticks) > 4 else 0)
    return sum(ticks), idle


def cpu_mhz() -> tuple[float | None, float | None]:
    values = []
    for path in Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq"):
        try:
            values.append(int(path.read_text()) / 1000.0)
        except (OSError, ValueError):
            pass
    if not values:
        return None, None
    return sum(values) / len(values), max(values)


def throttle_count(kind: str) -> int:
    values = []
    pattern = f"cpu[0-9]*/thermal_throttle/{kind}_throttle_count"
    for path in Path("/sys/devices/system/cpu").glob(pattern):
        try:
            values.append(int(path.read_text()))
        except (OSError, ValueError):
            pass
    return max(values, default=0)


def atomic_json(path: Path, data: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)


def terminate_group(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def fmt(value: float | None) -> str:
    return "" if value is None else f"{value:.1f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True, help="stage label, e.g. idle or cpu_1")
    parser.add_argument("--duration", type=int, required=True, help="seconds to monitor")
    parser.add_argument("--workers", type=int, default=0, help="OpenSSL worker processes; 0 is idle")
    parser.add_argument("--limit", type=float, default=95.0, help="stop workload at this CPU temperature")
    parser.add_argument("--interval", type=float, default=1.0, help="sample interval in seconds")
    args = parser.parse_args()
    if args.duration < 1 or args.workers < 0 or args.interval <= 0:
        parser.error("duration/interval must be positive and workers cannot be negative")

    RUNS.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    run_dir = RUNS / f"{stamp}_{args.stage}"
    run_dir.mkdir()
    (ROOT / "latest_run").unlink(missing_ok=True)
    (ROOT / "latest_run").symlink_to(run_dir.relative_to(ROOT))

    started_wall = dt.datetime.now().astimezone().isoformat(timespec="seconds")
    current_boot = boot_id()
    metadata = {
        "stage": args.stage,
        "duration_s": args.duration,
        "workers": args.workers,
        "temperature_limit_c": args.limit,
        "sample_interval_s": args.interval,
        "started_at": started_wall,
        "boot_id": current_boot,
        "hostname": platform.node(),
        "kernel": platform.release(),
        "cpu_count": os.cpu_count(),
    }
    atomic_json(run_dir / "metadata.json", metadata)
    state = {**metadata, "status": "RUNNING", "last_elapsed_s": 0.0}
    atomic_json(run_dir / "state.json", state)

    workload_log = (run_dir / "workload.log").open("w")
    process: subprocess.Popen | None = None
    command: list[str] | None = None
    if args.workers:
        # A single buffer size makes -seconds equal the intended stage duration.
        command = [
            "openssl", "speed", "-elapsed", "-seconds", str(args.duration + 5),
            "-bytes", "1048576", "-multi", str(args.workers), "-evp", "sha256",
        ]
        workload_log.write("command=" + " ".join(command) + "\n")
        workload_log.flush()
        process = subprocess.Popen(
            command,
            stdout=workload_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    fields = [
        "timestamp", "elapsed_s", "cpu_busy_pct", "load1", "load5",
        "freq_avg_mhz", "freq_max_mhz", "package_c", "core_max_c", "pch_c",
        "acpi_c", "package_throttle_count", "core_throttle_count", "workload_alive",
    ]
    status = "COMPLETE"
    reason = "duration elapsed"
    maximum = None
    start = time.monotonic()
    previous_total, previous_idle = cpu_times()
    try:
        with (run_dir / "samples.csv").open("w", newline="") as samples:
            writer = csv.DictWriter(samples, fieldnames=fields)
            writer.writeheader()
            while True:
                now = time.monotonic()
                elapsed = now - start
                temps = temperatures()
                package = pick_temperature(temps, "Package id")
                core_max = pick_temperature(temps, "coretemp:Core")
                control_temp = max(v for v in (package, core_max) if v is not None)
                maximum = control_temp if maximum is None else max(maximum, control_temp)
                total, idle = cpu_times()
                total_delta, idle_delta = total - previous_total, idle - previous_idle
                busy = 100.0 * (total_delta - idle_delta) / total_delta if total_delta else 0.0
                previous_total, previous_idle = total, idle
                avg_mhz, max_mhz = cpu_mhz()
                loads = os.getloadavg()
                row = {
                    "timestamp": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
                    "elapsed_s": f"{elapsed:.1f}",
                    "cpu_busy_pct": f"{busy:.1f}",
                    "load1": f"{loads[0]:.2f}",
                    "load5": f"{loads[1]:.2f}",
                    "freq_avg_mhz": fmt(avg_mhz),
                    "freq_max_mhz": fmt(max_mhz),
                    "package_c": fmt(package),
                    "core_max_c": fmt(core_max),
                    "pch_c": fmt(pick_temperature(temps, "pch_cannonlake")),
                    "acpi_c": fmt(pick_temperature(temps, "acpitz")),
                    "package_throttle_count": throttle_count("package"),
                    "core_throttle_count": throttle_count("core"),
                    "workload_alive": int(process is not None and process.poll() is None),
                }
                writer.writerow(row)
                samples.flush()
                os.fsync(samples.fileno())
                state.update({
                    "last_sample_at": row["timestamp"],
                    "last_elapsed_s": round(elapsed, 1),
                    "last_package_c": package,
                    "last_core_max_c": core_max,
                    "maximum_cpu_c": maximum,
                })
                atomic_json(run_dir / "state.json", state)
                print(
                    f"\r{args.stage}: {elapsed:6.1f}/{args.duration}s  "
                    f"CPU {busy:5.1f}%  pkg {fmt(package):>5}C  core-max {fmt(core_max):>5}C  "
                    f"peak {maximum:5.1f}C",
                    end="", flush=True,
                )
                if control_temp >= args.limit:
                    status = "THERMAL_ABORT"
                    reason = f"CPU temperature reached {control_temp:.1f} C (limit {args.limit:.1f} C)"
                    break
                if elapsed >= args.duration:
                    break
                if process is not None and process.poll() is not None:
                    status = "WORKLOAD_FAILED"
                    reason = f"workload exited early with return code {process.returncode}"
                    break
                time.sleep(min(args.interval, max(0.0, args.duration - elapsed)))
    except (KeyboardInterrupt, SystemExit):
        status = "INTERRUPTED"
        reason = "operator interrupted test"
    except Exception as exc:
        status = "ERROR"
        reason = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        terminate_group(process)
        workload_log.close()
        state.update({
            "status": status,
            "reason": reason,
            "finished_at": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
            "maximum_cpu_c": maximum,
            "boot_id_at_finish": boot_id(),
        })
        atomic_json(run_dir / "state.json", state)
        print(f"\n{status}: {reason}; peak CPU temperature={maximum} C")
        print(f"log directory: {run_dir}")
    return 0 if status == "COMPLETE" else 2


if __name__ == "__main__":
    sys.exit(main())
