"""Tests for the first-goal static footprint stability gate."""

import threading
from types import SimpleNamespace

from multi_map_navigation.navigation_manager import NavigationManager


class FakeLogger:
    def debug(self, _message):
        pass

    def error(self, _message):
        pass

    def info(self, _message):
        pass


class TransformSequence:
    def __init__(self, states):
        self.states = iter(states)
        self.current_state = None
        self.stamp = 0

    def lookup_transform(self, **_kwargs):
        self.current_state = next(self.states)
        self.stamp += 1
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=0, nanosec=self.stamp)),
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=1.0, y=2.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )


class SequenceChecker:
    def __init__(self, transforms):
        self.transforms = transforms

    def check_rectangular_footprint(self, *_args, **_kwargs):
        legal = self.transforms.current_state
        return legal, 'free' if legal else 'occupied'


def make_manager(states):
    manager = NavigationManager.__new__(NavigationManager)
    manager.state_lock = threading.RLock()
    manager._active_localization_session = 'session-a'
    manager._startup_footprint_verified_session = ''
    manager._startup_tf_buffer = TransformSequence(states)
    manager._static_footprint_checker = lambda _map: SequenceChecker(
        manager._startup_tf_buffer)
    manager._stack_start_cancelled = lambda: False
    manager.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=manager._startup_tf_buffer.stamp))
    manager.get_logger = lambda: FakeLogger()
    return manager


def test_ten_distinct_legal_samples_release_first_goal_gate(monkeypatch):
    manager = make_manager([True] * 10)
    monkeypatch.setattr(
        'multi_map_navigation.navigation_manager.time.sleep', lambda _delay: None)

    ready = manager._wait_for_startup_static_footprint(
        'map2', required_samples=10, sample_period=0.0, timeout_sec=1.0)

    assert ready
    assert manager._startup_footprint_verified_session == 'session-a'


def test_illegal_sample_resets_consecutive_count(monkeypatch):
    manager = make_manager([True] * 9 + [False] + [True] * 10)
    monkeypatch.setattr(
        'multi_map_navigation.navigation_manager.time.sleep', lambda _delay: None)

    ready = manager._wait_for_startup_static_footprint(
        'map2', required_samples=10, sample_period=0.0, timeout_sec=1.0)

    assert ready
    assert manager._startup_tf_buffer.stamp == 20
