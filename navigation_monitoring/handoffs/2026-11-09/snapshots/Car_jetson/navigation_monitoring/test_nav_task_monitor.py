import signal
import sys
from types import SimpleNamespace

import pytest

import navigation_monitoring.nav_task_monitor as monitor_module


@pytest.mark.parametrize("terminal_kind", ["idle", "fault", "manager_lost"])
def test_terminal_state_requests_shutdown_without_shutting_down_ros(
    monkeypatch, terminal_kind
):
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
    monitor.terminal_kind = terminal_kind
    monitor.terminal_since = 0.0
    monitor.task_id = "test-task"
    monitor.fault = None
    monitor.protective_abort = False
    monitor.vehicle = {"speed_kmh": 0.0}
    monitor.goal_statuses = []
    events = []
    monitor.write_event = lambda event, **fields: events.append((event, fields))

    def fail_shutdown(*_args, **_kwargs):
        pytest.fail("terminal timer callback must not shut down the ROS context")

    monkeypatch.setattr(monitor_module.rclpy, "shutdown", fail_shutdown)
    monitor_module.NavigationTaskMonitor.update_terminal_state(monitor, now=15.0)

    assert monitor.shutdown_requested is True
    assert events == [
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


@pytest.mark.parametrize("stop_source", ["terminal", "signal"])
def test_main_shuts_down_only_after_callback_returns(monkeypatch, stop_source):
    calls = []
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

    monkeypatch.setattr(sys, "argv", ["monitor", "--output", "unused.jsonl"])
    monkeypatch.setattr(
        monitor_module, "NavigationTaskMonitor", lambda *_args, **_kwargs: monitor
    )
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

    reason = "rclpy_shutdown" if stop_source == "terminal" else f"signal_{signal.SIGTERM}"
    assert calls == [
        "remove_node",
        "executor_shutdown",
        "context_shutdown",
        ("close", reason),
        "destroy_node",
    ]
