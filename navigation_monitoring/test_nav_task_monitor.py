import json
import math
import signal
import sys
from types import SimpleNamespace

import pytest

import navigation_monitoring.nav_task_monitor as monitor_module


def test_monitor_counts_raw_and_filtered_scans_separately(monkeypatch):
    monitor = object.__new__(monitor_module.NavigationTaskMonitor)
    monitor.counts = {"scan": 0, "filtered_scan": 0}
    monitor.last_seen = {}
    timestamps = iter((10.0, 11.0))
    monkeypatch.setattr(monitor_module.time, "monotonic", lambda: next(timestamps))

    monitor.on_scan(None)
    monitor.on_filtered_scan(None)

    assert monitor.counts == {"scan": 1, "filtered_scan": 1}
    assert monitor.last_seen == {"scan": 10.0, "filtered_scan": 11.0}


def test_monitor_retains_lio_epoch_and_correction_metadata(monkeypatch):
    monitor = object.__new__(monitor_module.NavigationTaskMonitor)
    monitor.counts = {
        "epoch_odom": 0,
        "correction_event": 0,
        "map_odom_correction": 0,
    }
    monitor.last_seen = {}
    timestamps = iter((10.0, 11.0, 12.0))
    monkeypatch.setattr(monitor_module.time, "monotonic", lambda: next(timestamps))

    monitor.on_epoch_odom(
        SimpleNamespace(instance_id="lio-a", generation=4, correction_epoch=12)
    )
    monitor.on_correction_event(
        SimpleNamespace(
            instance_id="lio-a",
            generation=4,
            correction_epoch=12,
            source=2,
            translation_residual=0.1234567,
            yaw_residual=0.0123456,
            source_score=0.1,
            arrival_delay_sec=0.45,
            degenerate=False,
            accepted=True,
            reason="loop factor accepted",
        )
    )
    monitor.on_map_odom_correction(
        SimpleNamespace(
            instance_id="lio-a",
            generation=4,
            correction_epoch=12,
            translation_residual=0.02,
            yaw_residual=0.002,
            state=1,
            accepted=True,
            reason="RELEASING_SMALL",
        )
    )

    assert monitor.last_lio_epoch == {
        "instance_id": "lio-a",
        "generation": 4,
        "correction_epoch": 12,
    }
    assert monitor.last_map_odom_correction["correction_epoch"] == 12
    assert monitor.last_map_odom_correction["accepted"] is True
    assert monitor.last_lio_correction["source"] == 2
    assert monitor.last_lio_correction["translation_residual_m"] == 0.123457
    assert monitor.last_lio_correction["accepted"] is True


def front_scan(point_angle_deg, point_range):
    angle_min = -math.pi
    angle_increment = math.radians(0.25)
    ranges = [math.inf] * 1441
    index = round((math.radians(point_angle_deg) - angle_min) / angle_increment)
    ranges[index] = point_range
    return SimpleNamespace(
        angle_min=angle_min,
        angle_increment=angle_increment,
        ranges=ranges,
    )


def test_legacy_filter_counter_matches_all_four_filter_conditions():
    assert monitor_module.legacy_front_filter_hits(front_scan(133.0, 0.70)) == 1
    assert monitor_module.legacy_front_filter_hits(front_scan(130.0, 0.70)) == 0
    assert monitor_module.legacy_front_filter_hits(front_scan(133.0, 0.50)) == 0


def test_filter_removal_reminder_requires_three_sessions_and_one_hour(tmp_path):
    tracker_path = tmp_path / "filter_retirement_tracker.json"
    for index in range(3):
        state = monitor_module.update_filter_retirement_tracker(
            tracker_path,
            {
                "ended_at": f"2026-11-18T1{index}:00:00+08:00",
                "monitor_output": f"run-{index}.jsonl",
                "task_id": f"task-{index}",
                "front_scan_frames": 36000,
                "observation_seconds": 1200.0,
                "candidate_frames": 0,
                "candidate_points": 0,
            },
        )

    assert state["consecutive_zero_hit_sessions"] == 3
    assert state["consecutive_zero_hit_observation_seconds"] == 3600.0
    assert state["removal_reminder_due"] is True
    assert json.loads(tracker_path.read_text(encoding="utf-8")) == state


def test_filter_candidate_resets_zero_hit_streak(tmp_path):
    tracker_path = tmp_path / "filter_retirement_tracker.json"
    common = {
        "ended_at": "2026-11-18T13:00:00+08:00",
        "monitor_output": "run.jsonl",
        "task_id": "task",
        "front_scan_frames": 36000,
        "observation_seconds": 1200.0,
        "candidate_frames": 0,
        "candidate_points": 0,
    }
    monitor_module.update_filter_retirement_tracker(tracker_path, common)
    state = monitor_module.update_filter_retirement_tracker(
        tracker_path,
        {**common, "candidate_frames": 1, "candidate_points": 1},
    )

    assert state["consecutive_zero_hit_sessions"] == 0
    assert state["consecutive_zero_hit_observation_seconds"] == 0.0
    assert state["last_candidate_seen_at"] == common["ended_at"]
    assert state["removal_reminder_due"] is False


def terminal_monitor(terminal_kind, stop_on_terminal):
    monitor = object.__new__(monitor_module.NavigationTaskMonitor)
    monitor.task_seen = True
    monitor.process_state = {
        "process_manager": [10],
        "navigation_manager": [11],
        "re_localization": [],
        "nav2_init_pose": [],
        "liosam": [],
        "nav2": [],
    }
    monitor.robot_state = terminal_kind
    if terminal_kind == "manager_lost":
        monitor.process_state["process_manager"] = []
    monitor.terminal_kind = None
    monitor.terminal_since = None
    monitor.terminal_reported = False
    monitor.stop_on_terminal = stop_on_terminal
    monitor.shutdown_requested = False
    monitor.task_id = "test-task"
    monitor.fault = None
    monitor.protective_abort = False
    monitor.vehicle = {"speed_kmh": 0.0}
    monitor.goal_statuses = []
    events = []
    monitor.write_event = lambda event, **fields: events.append((event, fields))
    return monitor, events


@pytest.mark.parametrize("terminal_kind", ["idle", "fault", "manager_lost"])
def test_terminal_state_records_once_and_continues_by_default(
    monkeypatch, terminal_kind
):
    monitor, events = terminal_monitor(terminal_kind, stop_on_terminal=False)

    def fail_shutdown(*_args, **_kwargs):
        pytest.fail("terminal timer callback must not shut down the ROS context")

    monkeypatch.setattr(monitor_module.rclpy, "shutdown", fail_shutdown)
    monitor_module.NavigationTaskMonitor.update_terminal_state(monitor, now=0.0)
    monitor_module.NavigationTaskMonitor.update_terminal_state(monitor, now=15.0)
    monitor_module.NavigationTaskMonitor.update_terminal_state(monitor, now=16.0)

    assert monitor.shutdown_requested is False
    assert events == [
        ("terminal_candidate", {"kind": terminal_kind}),
        (
            "monitor_terminal",
            {
                "kind": terminal_kind,
                "task_id": "test-task",
                "fault": None,
                "protective_abort": False,
                "vehicle": {"speed_kmh": 0.0},
                "goal_statuses": [],
            },
        )
    ]


@pytest.mark.parametrize("terminal_kind", ["idle", "fault", "manager_lost"])
def test_terminal_state_requests_shutdown_when_enabled(monkeypatch, terminal_kind):
    monitor, events = terminal_monitor(terminal_kind, stop_on_terminal=True)

    def fail_shutdown(*_args, **_kwargs):
        pytest.fail("terminal timer callback must not shut down the ROS context")

    monkeypatch.setattr(monitor_module.rclpy, "shutdown", fail_shutdown)
    monitor_module.NavigationTaskMonitor.update_terminal_state(monitor, now=0.0)
    monitor_module.NavigationTaskMonitor.update_terminal_state(monitor, now=15.0)

    assert monitor.shutdown_requested is True
    assert [event for event, _fields in events] == [
        "terminal_candidate",
        "monitor_terminal",
    ]


@pytest.mark.parametrize("stop_source", ["terminal", "signal"])
def test_main_shuts_down_only_after_callback_returns(monkeypatch, stop_source):
    calls = []
    created = []
    handlers = {}
    state = {"in_callback": False, "context_ok": True}
    monitor = SimpleNamespace(shutdown_requested=False)
    monitor.close = lambda reason: calls.append(("close", reason))
    monitor.destroy_node = lambda: calls.append("destroy_node")

    class FakeExecutor:
        def add_node(self, node):
            assert node is monitor

        def spin_once(self, timeout_sec):
            state["in_callback"] = True
            if stop_source == "terminal":
                monitor.shutdown_requested = True
            else:
                handlers[signal.SIGTERM](signal.SIGTERM, None)
            assert state["context_ok"]
            state["in_callback"] = False

        def remove_node(self, node):
            assert node is monitor
            assert not state["in_callback"]
            calls.append("remove_node")

        def shutdown(self):
            assert not state["in_callback"]
            calls.append("executor_shutdown")

    def shutdown_context():
        assert not state["in_callback"]
        calls.append("context_shutdown")
        state["context_ok"] = False

    argv = ["monitor", "--output", "unused.jsonl"]
    if stop_source == "terminal":
        argv.append("--stop-on-terminal")
    monkeypatch.setattr(sys, "argv", argv)

    def create_monitor(output_path, **kwargs):
        created.append((output_path, kwargs))
        return monitor

    monkeypatch.setattr(monitor_module, "NavigationTaskMonitor", create_monitor)
    monkeypatch.setattr(monitor_module, "SingleThreadedExecutor", FakeExecutor)
    monkeypatch.setattr(
        monitor_module.signal,
        "signal",
        lambda signum, handler: handlers.update({signum: handler}),
    )
    monkeypatch.setattr(monitor_module.rclpy, "init", lambda: None)
    monkeypatch.setattr(monitor_module.rclpy, "ok", lambda: state["context_ok"])
    monkeypatch.setattr(monitor_module.rclpy, "shutdown", shutdown_context)

    monitor_module.main()

    assert created == [
        (
            "unused.jsonl",
            {
                "enable_protection": False,
                "stop_on_terminal": stop_source == "terminal",
                "filter_tracker_path": monitor_module.DEFAULT_FILTER_TRACKER_PATH,
            },
        )
    ]
    reason = "rclpy_shutdown" if stop_source == "terminal" else f"signal_{signal.SIGTERM}"
    assert calls == [
        "remove_node",
        "executor_shutdown",
        "context_shutdown",
        ("close", reason),
        "destroy_node",
    ]
