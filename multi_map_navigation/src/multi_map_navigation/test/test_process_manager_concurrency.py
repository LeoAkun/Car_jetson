import ast
import inspect
from pathlib import Path
import textwrap
import threading
import time
from types import SimpleNamespace
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from multi_map_navigation.process_manager import ProcessManagerNode, main
from multi_map_navigation_msgs.msg import LioReadiness


class FakeLogger:
    def __init__(self):
        self.messages = []

    def debug(self, message):
        self.messages.append(('debug', message))

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))

    def error(self, message):
        self.messages.append(('error', message))


class FakePublisher:
    def publish(self, _message):
        return None


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeProcess:
    def __init__(self, pid):
        self.pid = pid

    def poll(self):
        return None


def make_manager():
    manager = ProcessManagerNode.__new__(ProcessManagerNode)
    manager.processes = {}
    manager.process_pgids = {}
    manager.process_maps = {}
    manager.process_started_at = {}
    manager.process_generations = {}
    manager.process_instance_ids = {}
    manager.process_cancel_events = {}
    manager._lifecycle_generation_counter = 0
    manager.process_lifecycle_lock = threading.RLock()
    manager.process_states = {}
    manager.health_failures = {}
    manager.fault_reasons = {}
    manager.confirmed_healthy_processes = set()
    manager.active_localization_session = ''
    manager.active_map = ''
    manager.localization_session_start_ns = 0
    manager.external_process_guard_enabled = False
    manager.max_startup_time = 60.0
    manager.nav2_max_startup_time = 150.0
    manager.health_failure_threshold = 3
    manager.shutdown_timeout = 0.0
    manager.liosam_stability_ready = True
    manager.liosam_stable_correction_count = 10
    manager.liosam_stable_correction_count_required = 10
    manager.liosam_stable_translation_m = 0.05
    manager.liosam_stable_yaw_rad = 0.017453292519943295
    manager.liosam_last_correction_epoch = 0
    manager.logger = FakeLogger()
    manager.get_logger = lambda: manager.logger
    manager.fault_pub = FakePublisher()
    return manager


def wait_for_log(manager, text, timeout=0.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if any(text in message for _level, message in manager.logger.messages):
            return True
        time.sleep(0.001)
    return False


def repository_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / 'slam/src/LIO-SAM/src/imuPreintegration.cpp').is_file():
            return parent
    raise AssertionError('repository root not found')


def test_shutdown_interrupts_long_start_health_wait_promptly():
    manager = make_manager()
    process = FakeProcess(101)
    cancel_event = threading.Event()
    manager.processes['liosam'] = process
    manager.process_pgids['liosam'] = 501
    manager.process_maps['liosam'] = ''
    manager.process_started_at['liosam'] = time.monotonic()
    manager.process_generations['liosam'] = 7
    manager.process_cancel_events['liosam'] = cancel_event
    manager.process_states['liosam'] = 'STARTING'
    manager.health_failures['liosam'] = 0
    probe_started = threading.Event()
    wait_result = []

    def probe(_process_name):
        probe_started.set()
        return False, 'still starting', False

    manager.probe_process_health = probe
    manager._live_process_group_members = lambda _pgid: set()

    waiter = threading.Thread(
        target=lambda: wait_result.append(
            manager.wait_for_process_healthy('liosam', 7)),
        daemon=True,
    )
    waiter.start()
    assert probe_started.wait(0.5)

    started = time.monotonic()
    assert manager.shutdown_process('liosam')
    waiter.join(0.75)

    assert not waiter.is_alive()
    assert time.monotonic() - started < 0.75
    assert wait_result == [False]
    assert cancel_event.is_set()
    assert 'liosam' not in manager.processes
    assert manager.process_states['liosam'] == 'STOPPED'


def test_launch_callback_can_be_cancelled_by_concurrent_shutdown(monkeypatch):
    manager = make_manager()
    process = FakeProcess(151)
    probe_started = threading.Event()
    probe_calls = []
    launch_result = []

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        lambda *_args, **_kwargs: process,
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid',
        lambda _pid: 551,
    )
    manager._live_process_group_members = lambda _pgid: set()

    def probe(_process_name):
        probe_calls.append(_process_name)
        probe_started.set()
        return False, 'health check still pending', False

    manager.probe_process_health = probe
    manager.max_startup_time = 30.0

    starter = threading.Thread(
        target=lambda: launch_result.append(manager.launch_liosam()),
        daemon=True,
    )
    starter.start()
    assert probe_started.wait(0.5)

    calls_before_status = len(probe_calls)
    status_started = time.monotonic()
    status = manager.get_process_status_callback(
        SimpleNamespace(), SimpleNamespace())
    assert time.monotonic() - status_started < 0.1
    assert status.stack_state == 'STARTING'
    assert status.liosam_running
    assert len(probe_calls) == calls_before_status

    # This models a stop service arriving while the start service callback is
    # still synchronously waiting for health.  It must cancel the exact
    # generation and release the process group without waiting 30 seconds.
    assert manager.shutdown_process('liosam')
    starter.join(0.75)

    assert not starter.is_alive()
    assert launch_result == [False]
    assert manager.processes == {}
    assert manager.process_generations == {}
    assert manager.process_states['liosam'] == 'STOPPED'


def test_shutdown_before_spawn_cancels_start_admission(monkeypatch):
    manager = make_manager()
    generation = manager._begin_start_admission('navigation2')

    assert generation is not None
    assert manager.shutdown_process('navigation2')
    assert manager.process_cancel_events['navigation2'].is_set()

    def unexpected_popen(*_args, **_kwargs):
        raise AssertionError('a cancelled admission must not spawn a process')

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        unexpected_popen,
    )

    assert manager._spawn_tracked_process(
        'navigation2', 'map_114', ['ignored']) is None

    manager._finish_start_admission('navigation2', generation)
    assert 'navigation2' not in manager.processes
    assert 'navigation2' not in manager.process_generations
    assert 'navigation2' not in manager.process_cancel_events
    assert manager.process_states['navigation2'] == 'STOPPED'


def test_begin_shutdown_cancels_pending_admission_and_blocks_popen(
        monkeypatch):
    manager = make_manager()
    generation = manager._begin_start_admission('navigation2')
    cancel_event = manager.process_cancel_events['navigation2']

    def unexpected_popen(*_args, **_kwargs):
        raise AssertionError('manager shutdown must prevent pending Popen')

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        unexpected_popen,
    )

    manager.begin_shutdown()

    assert cancel_event.is_set()
    assert manager.process_states['navigation2'] == 'STOPPING'
    assert manager._spawn_tracked_process(
        'navigation2', 'map_114', ['ignored']) is None
    assert manager._begin_start_admission('liosam') is None

    manager._finish_start_admission('navigation2', generation)
    assert manager.process_states['navigation2'] == 'STOPPED'


def test_start_request_after_manager_shutdown_returns_busy_without_launch():
    manager = make_manager()
    manager.begin_shutdown()

    def unexpected_launch():
        raise AssertionError('start callback must not launch during shutdown')

    manager.launch_liosam = unexpected_launch
    response = manager.start_process_callback(
        SimpleNamespace(process_name='liosam', map_name=''),
        SimpleNamespace(),
    )

    assert not response.success
    assert response.busy
    assert '正在关闭' in response.message
    assert 'liosam' not in manager.processes


def test_status_callback_reads_snapshot_without_health_or_process_probes():
    manager = make_manager()
    manager.processes = {
        'liosam': FakeProcess(201),
        'navigation2': FakeProcess(202),
    }
    manager.process_states = {
        'liosam': 'HEALTHY',
        'navigation2': 'STARTING',
    }
    manager.process_maps = {'navigation2': 'map_114'}
    manager.active_map = 'map_114'
    manager.active_localization_session = 'session-9'
    manager.localization_session_start_ns = 123456

    def unexpected_probe(*_args, **_kwargs):
        raise AssertionError('status must only read the cached snapshot')

    manager.is_process_running = unexpected_probe
    manager.probe_process_health = unexpected_probe
    manager._find_untracked_processes = unexpected_probe

    response = manager.get_process_status_callback(
        SimpleNamespace(), SimpleNamespace())

    assert response.liosam_running
    assert response.navigation2_running
    assert not response.re_localization_running
    assert not response.nav2_init_pose_running
    assert response.stack_state == 'STARTING'
    assert response.active_map == 'map_114'
    assert response.localization_session_id == 'session-9'
    assert response.session_start_ns == 123456
    assert 'navigation2: STARTING' in response.message


def test_status_requires_every_component_healthy_for_running_stack():
    manager = make_manager()
    names = (
        're_localization', 'nav2_init_pose', 'liosam', 'navigation2')
    manager.processes = {
        name: FakeProcess(220 + index)
        for index, name in enumerate(names)
    }
    manager.process_states = {name: 'HEALTHY' for name in names}

    healthy = manager.get_process_status_callback(
        SimpleNamespace(), SimpleNamespace())
    assert healthy.stack_state == 'RUNNING'

    manager.process_states['liosam'] = 'DEGRADED'
    degraded = manager.get_process_status_callback(
        SimpleNamespace(), SimpleNamespace())

    assert degraded.stack_state == 'PARTIAL'
    assert degraded.liosam_running
    assert 'liosam: DEGRADED' in degraded.message


def test_status_reports_tracked_failed_process_without_fault_text():
    manager = make_manager()
    manager.processes = {'navigation2': FakeProcess(231)}
    manager.process_states = {'navigation2': 'FAILED'}

    response = manager.get_process_status_callback(
        SimpleNamespace(), SimpleNamespace())

    assert response.navigation2_running
    assert response.stack_state == 'FAILED'
    assert 'reason unavailable' in response.message


def test_nav2_start_fails_if_bound_liosam_stops_during_health_probe():
    manager = make_manager()
    nav2_process = FakeProcess(241)
    liosam_process = FakeProcess(242)
    manager.processes = {
        'navigation2': nav2_process,
        'liosam': liosam_process,
    }
    manager.process_pgids = {'navigation2': 741, 'liosam': 742}
    manager.process_generations = {'navigation2': 21, 'liosam': 22}
    manager.process_cancel_events = {
        'navigation2': threading.Event(),
        'liosam': threading.Event(),
    }
    manager.process_states = {
        'navigation2': 'STARTING',
        'liosam': 'HEALTHY',
    }
    manager.health_failures = {'navigation2': 0, 'liosam': 0}
    manager._live_process_group_members = lambda _pgid: set()

    def probe(process_name):
        if process_name == 'liosam':
            return True, '', False
        assert manager.shutdown_process('liosam')
        return True, '', False

    manager.probe_process_health = probe

    assert not manager.wait_for_process_healthy(
        'navigation2',
        21,
        liosam_process=liosam_process,
        liosam_generation=22,
    )
    assert 'liosam' not in manager.processes
    assert manager.process_states['navigation2'] == 'FAILED'
    assert 'stopped or was replaced' in manager.fault_reasons['navigation2']


def test_nav2_start_cannot_commit_when_current_liosam_probe_is_unhealthy():
    manager = make_manager()
    nav2_process = FakeProcess(243)
    liosam_process = FakeProcess(244)
    manager.processes = {
        'navigation2': nav2_process,
        'liosam': liosam_process,
    }
    manager.process_generations = {'navigation2': 31, 'liosam': 32}
    manager.process_cancel_events = {
        'navigation2': threading.Event(),
        'liosam': threading.Event(),
    }
    manager.process_states = {
        'navigation2': 'STARTING',
        'liosam': 'HEALTHY',
    }
    manager.health_failures = {'navigation2': 0, 'liosam': 0}

    def probe(process_name):
        if process_name == 'navigation2':
            return True, '', False
        assert process_name == 'liosam'
        return False, 'current LIO-SAM probe is unhealthy', False

    manager.probe_process_health = probe

    assert not manager.wait_for_process_healthy(
        'navigation2',
        31,
        liosam_process=liosam_process,
        liosam_generation=32,
    )
    assert manager.process_states['navigation2'] == 'FAILED'
    assert 'current LIO-SAM probe is unhealthy' in (
        manager.fault_reasons['navigation2'])


def test_nav2_runtime_health_fails_if_bound_liosam_generation_is_replaced():
    manager = make_manager()
    nav2_process = FakeProcess(246)
    old_liosam = FakeProcess(247)
    replacement_liosam = FakeProcess(248)
    manager.processes = {
        'navigation2': nav2_process,
        'liosam': replacement_liosam,
    }
    manager.process_generations = {'navigation2': 41, 'liosam': 43}
    manager.process_cancel_events = {
        'navigation2': threading.Event(),
        'liosam': threading.Event(),
    }
    manager.process_states = {
        'navigation2': 'HEALTHY',
        'liosam': 'HEALTHY',
    }
    manager.process_dependencies = {
        'navigation2': ('liosam', old_liosam, 42),
    }
    manager.required_process_patterns = {'navigation2': ()}
    manager.required_nodes = {}
    manager.required_services = {}
    manager.liosam_ready = True
    manager._lifecycle_nodes_active = lambda: (True, '')

    healthy, reason, immediate = manager.probe_process_health('navigation2')

    assert not healthy
    assert immediate
    assert 'stopped or was replaced' in reason


def test_nav2_launch_is_rejected_without_healthy_tracked_liosam(monkeypatch):
    manager = make_manager()
    manager.map_dir = '/maps'

    def unexpected_popen(*_args, **_kwargs):
        raise AssertionError('Nav2 must not spawn without LIO-SAM')

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        unexpected_popen,
    )

    assert not manager.launch_navigation2('map_114')
    assert manager.process_states['navigation2'] == 'FAILED'
    assert 'LIO-SAM' in manager.fault_reasons['navigation2']


def test_liosam_launch_passes_its_generation_to_health_wait(monkeypatch):
    manager = make_manager()
    child = FakeProcess(245)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        lambda *_args, **_kwargs: child,
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid',
        lambda _pid: 745,
    )
    waits = []
    manager.wait_for_process_healthy = (
        lambda process_name, generation:
        waits.append((process_name, generation)) or True
    )

    assert manager.launch_liosam()
    assert waits == [('liosam', manager.process_generations['liosam'])]


def test_liosam_launch_injects_exact_readiness_identity(monkeypatch):
    manager = make_manager()
    child = FakeProcess(249)
    popen_kwargs = []
    admitted_generation = manager._begin_start_admission('liosam')

    def popen(*_args, **kwargs):
        popen_kwargs.append(kwargs)
        return child

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen', popen)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid',
        lambda _pid: 749,
    )
    manager.wait_for_process_healthy = lambda *_args, **_kwargs: True

    assert manager.launch_liosam()

    generation = manager.process_generations['liosam']
    instance_id = manager.process_instance_ids['liosam']
    launch_env = popen_kwargs[0]['env']
    assert generation == admitted_generation
    assert instance_id
    assert launch_env['LIO_SAM_INSTANCE_ID'] == instance_id
    assert launch_env['LIO_SAM_GENERATION'] == str(generation)


def test_stopped_relocalization_cannot_publish_stale_active_session(
        monkeypatch):
    manager = make_manager()
    manager.map_dir = '/maps'
    manager.startup_delay = 0.0
    manager.localization_session_pub = RecordingPublisher()
    child = FakeProcess(251)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        lambda *_args, **_kwargs: child,
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid',
        lambda _pid: 751,
    )
    manager._live_process_group_members = lambda _pgid: set()
    real_spawn = manager._spawn_tracked_process

    def spawn_then_stop(*args, **kwargs):
        spawned = real_spawn(*args, **kwargs)
        assert spawned is not None
        assert manager.shutdown_process('re_localization')
        return spawned

    manager._spawn_tracked_process = spawn_then_stop

    assert not manager.launch_relocalization('map_114')
    assert manager.active_localization_session == ''
    assert all(
        not message.active
        for message in manager.localization_session_pub.messages)


def test_concurrent_start_request_returns_busy_without_waiting():
    manager = make_manager()
    manager._ensure_process_tracking()
    manager.start_operation_lock.acquire()
    try:
        started = time.monotonic()
        response = manager.start_process_callback(
            SimpleNamespace(process_name='liosam', map_name=''),
            SimpleNamespace(),
        )
    finally:
        manager.start_operation_lock.release()

    assert time.monotonic() - started < 0.1
    assert not response.success
    assert '正在处理' in response.message


def test_healthy_process_on_incompatible_map_is_rejected_not_busy():
    manager = make_manager()
    process = FakeProcess(311)
    manager.processes['re_localization'] = process
    manager.process_maps['re_localization'] = 'map_old'
    manager.process_generations['re_localization'] = 51
    manager.process_cancel_events['re_localization'] = threading.Event()
    manager.process_states['re_localization'] = 'HEALTHY'
    manager.active_localization_session = 'session-old'
    manager.active_map = 'map_old'
    manager.localization_session_start_ns = 123

    response = manager.start_process_callback(
        SimpleNamespace(
            process_name='re_localization',
            map_name='map_new',
        ),
        SimpleNamespace(),
    )

    assert not response.success
    assert not response.busy
    assert 'map_old' in response.message
    assert 'map_new' in response.message


@pytest.mark.parametrize('owner_result', [True, False])
def test_duplicate_shutdown_waits_and_inherits_same_generation_result(
        owner_result):
    manager = make_manager()
    process = FakeProcess(321)
    manager.processes['navigation2'] = process
    manager.process_generations['navigation2'] = 61
    manager.process_cancel_events['navigation2'] = threading.Event()
    manager.process_states['navigation2'] = 'HEALTHY'
    owner_entered = threading.Event()
    release_owner = threading.Event()
    calls = []
    results = {}

    def shutdown_impl(
            process_name, preserve_fault=False,
            expected_process=None, expected_generation=None):
        calls.append((
            process_name,
            preserve_fault,
            expected_process,
            expected_generation,
        ))
        owner_entered.set()
        assert release_owner.wait(1.0)
        return owner_result

    manager._shutdown_process_impl = shutdown_impl

    owner = threading.Thread(
        target=lambda: results.setdefault(
            'owner', manager.shutdown_process('navigation2')),
        daemon=True,
    )
    owner.start()
    assert owner_entered.wait(0.5)

    waiter_started = threading.Event()

    def wait_for_same_shutdown():
        waiter_started.set()
        results['waiter'] = manager.shutdown_process('navigation2')

    waiter = threading.Thread(target=wait_for_same_shutdown, daemon=True)
    waiter.start()
    assert waiter_started.wait(0.5)
    assert wait_for_log(manager, '等待现有关闭事务结果')
    assert waiter.is_alive()

    release_owner.set()
    owner.join(0.5)
    waiter.join(0.5)

    assert not owner.is_alive()
    assert not waiter.is_alive()
    assert results == {'owner': owner_result, 'waiter': owner_result}
    assert calls == [('navigation2', False, process, 61)]


def test_shutdown_registration_immediately_cancels_and_hides_running_state():
    manager = make_manager()
    process = FakeProcess(326)
    cancel_event = threading.Event()
    manager.processes = {
        're_localization': FakeProcess(323),
        'nav2_init_pose': FakeProcess(324),
        'liosam': FakeProcess(325),
        'navigation2': process,
    }
    manager.process_maps = {
        're_localization': 'map_a',
        'nav2_init_pose': 'map_a',
        'liosam': '',
        'navigation2': 'map_a',
    }
    manager.process_generations = {
        're_localization': 62,
        'nav2_init_pose': 63,
        'liosam': 64,
        'navigation2': 65,
    }
    manager.process_cancel_events = {
        're_localization': threading.Event(),
        'nav2_init_pose': threading.Event(),
        'liosam': threading.Event(),
        'navigation2': cancel_event,
    }
    manager.process_states = {
        name: 'HEALTHY' for name in manager.processes
    }
    owner_registered = threading.Event()
    release_owner = threading.Event()

    def shutdown_impl(*_args, **_kwargs):
        owner_registered.set()
        assert release_owner.wait(1.0)
        return True

    manager._shutdown_process_impl = shutdown_impl
    owner = threading.Thread(
        target=lambda: manager.shutdown_process('navigation2'),
        daemon=True,
    )
    owner.start()
    assert owner_registered.wait(0.5)

    assert cancel_event.is_set()
    assert manager.process_states['navigation2'] == 'STOPPING'

    start_response = manager.start_process_callback(
        SimpleNamespace(process_name='navigation2', map_name='map_a'),
        SimpleNamespace(),
    )
    status = manager.get_process_status_callback(
        SimpleNamespace(), SimpleNamespace())

    assert not start_response.success
    assert start_response.busy
    assert status.navigation2_running
    assert status.stack_state == 'STOPPING'
    assert status.stack_state != 'RUNNING'

    release_owner.set()
    owner.join(0.5)
    assert not owner.is_alive()


def test_old_generation_shutdown_tail_blocks_start_and_waiter_closes_replacement(
        monkeypatch):
    manager = make_manager()
    old_process = FakeProcess(331)
    replacement = FakeProcess(332)
    manager.processes['navigation2'] = old_process
    manager.process_generations['navigation2'] = 71
    manager.process_cancel_events['navigation2'] = threading.Event()
    manager.process_states['navigation2'] = 'HEALTHY'
    old_tail_open = threading.Event()
    release_old_owner = threading.Event()
    replacement_shutdown = threading.Event()
    calls = []
    results = {}

    def shutdown_impl(
            process_name, preserve_fault=False,
            expected_process=None, expected_generation=None):
        del preserve_fault
        calls.append((process_name, expected_process, expected_generation))
        if expected_process is old_process:
            with manager.process_lifecycle_lock:
                manager.processes.pop(process_name, None)
                manager.process_generations.pop(process_name, None)
                manager.process_cancel_events.pop(process_name, None)
                manager.process_states[process_name] = 'STOPPED'
            old_tail_open.set()
            assert release_old_owner.wait(1.0)
            return True

        assert expected_process is replacement
        assert expected_generation == 72
        replacement_shutdown.set()
        return False

    manager._shutdown_process_impl = shutdown_impl
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        lambda *_args, **_kwargs: pytest.fail(
            'shutdown tail must prevent a replacement Popen'),
    )

    owner = threading.Thread(
        target=lambda: results.setdefault(
            'old', manager.shutdown_process('navigation2')),
        daemon=True,
    )
    owner.start()
    assert old_tail_open.wait(0.5)

    assert manager._begin_start_admission('navigation2') is None
    assert manager._spawn_tracked_process(
        'navigation2', 'map_new', ['ignored']) is None

    # Inject the replacement that the old implementation allowed in this
    # tail window. The waiting shutdown must bind to it and run again after
    # the old generation's operation completes.
    with manager.process_lifecycle_lock:
        manager.processes['navigation2'] = replacement
        manager.process_generations['navigation2'] = 72
        manager.process_cancel_events['navigation2'] = threading.Event()
        manager.process_states['navigation2'] = 'HEALTHY'

    waiter = threading.Thread(
        target=lambda: results.setdefault(
            'replacement', manager.shutdown_process('navigation2')),
        daemon=True,
    )
    waiter.start()
    assert wait_for_log(manager, '等待现有关闭事务结果')
    assert waiter.is_alive()

    release_old_owner.set()
    owner.join(0.5)
    waiter.join(0.5)

    assert not owner.is_alive()
    assert not waiter.is_alive()
    assert replacement_shutdown.is_set()
    assert results == {'old': True, 'replacement': False}
    assert calls == [
        ('navigation2', old_process, 71),
        ('navigation2', replacement, 72),
    ]


def test_stale_generation_cannot_forget_or_cancel_replacement_instance():
    manager = make_manager()
    old_process = FakeProcess(301)
    replacement = FakeProcess(302)
    replacement_cancel = threading.Event()
    manager.processes['navigation2'] = replacement
    manager.process_pgids['navigation2'] = 702
    manager.process_maps['navigation2'] = 'new_map'
    manager.process_started_at['navigation2'] = 20.0
    manager.process_generations['navigation2'] = 12
    manager.process_cancel_events['navigation2'] = replacement_cancel
    manager.process_states['navigation2'] = 'HEALTHY'
    manager.health_failures['navigation2'] = 0

    assert not manager._forget_process(
        'navigation2',
        expected_process=old_process,
        expected_generation=11,
    )
    assert manager.shutdown_process(
        'navigation2',
        expected_process=old_process,
        expected_generation=11,
    )

    assert manager.processes['navigation2'] is replacement
    assert manager.process_pgids['navigation2'] == 702
    assert manager.process_maps['navigation2'] == 'new_map'
    assert manager.process_generations['navigation2'] == 12
    assert not replacement_cancel.is_set()
    assert manager.process_states['navigation2'] == 'HEALTHY'


def test_services_use_separate_callback_groups():
    source = textwrap.dedent(inspect.getsource(ProcessManagerNode.__init__))
    tree = ast.parse(source)
    service_groups = {}
    group_types = {}
    timer_group = None

    for assignment in (
            node for node in ast.walk(tree) if isinstance(node, ast.Assign)):
        if len(assignment.targets) != 1:
            continue
        target = assignment.targets[0]
        value = assignment.value
        if not isinstance(target, ast.Attribute):
            continue
        if not isinstance(target.value, ast.Name):
            continue
        if target.value.id != 'self' or not isinstance(value, ast.Call):
            continue
        if isinstance(value.func, ast.Name):
            group_types[target.attr] = value.func.id

    for call in (
            node for node in ast.walk(tree) if isinstance(node, ast.Call)):
        function = call.func
        if not isinstance(function, ast.Attribute):
            continue
        keyword = next(
            (item.value for item in call.keywords
             if item.arg == 'callback_group'),
            None,
        )
        if function.attr == 'create_service':
            service_name = call.args[1].value
            service_groups[service_name] = ast.unparse(keyword)
        elif function.attr == 'create_timer':
            timer_group = ast.unparse(keyword)

    assert service_groups['/process_manager/start_process'] == (
        'self.start_callback_group')
    assert service_groups['/process_manager/shutdown_process'] == (
        'self.shutdown_callback_group')
    assert service_groups['/process_manager/get_status'] == (
        'self.status_callback_group')
    assert timer_group == 'self.health_callback_group'
    assert group_types['start_callback_group'] == 'ReentrantCallbackGroup'
    assert group_types['shutdown_callback_group'] == 'ReentrantCallbackGroup'
    assert group_types['status_callback_group'] == 'ReentrantCallbackGroup'
    assert group_types['health_callback_group'] == (
        'MutuallyExclusiveCallbackGroup')
    assert len(set(service_groups.values())) == 3


def test_liosam_readiness_subscription_is_latched_and_serialized():
    source = textwrap.dedent(inspect.getsource(ProcessManagerNode.__init__))
    tree = ast.parse(source)
    assignments = {}
    subscriptions = []

    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                assignments[ast.unparse(target)] = ast.unparse(node.value)
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr == 'create_subscription':
            subscriptions.append(node)

    assert assignments['self.liosam_ready_callback_group'] == (
        'MutuallyExclusiveCallbackGroup()')
    assert assignments['ready_qos'] == 'QoSProfile(depth=1)'
    assert assignments['ready_qos.reliability'] == (
        'ReliabilityPolicy.RELIABLE')
    assert assignments['ready_qos.durability'] == (
        'DurabilityPolicy.TRANSIENT_LOCAL')

    readiness_sub = None
    for call in subscriptions:
        if len(call.args) < 2 or not isinstance(call.args[1], ast.Constant):
            continue
        if call.args[1].value == '/lio_sam/imu_preintegration_readiness':
            readiness_sub = call
            break
    assert readiness_sub is not None
    callback_group = next(
        item.value for item in readiness_sub.keywords
        if item.arg == 'callback_group')
    assert ast.unparse(readiness_sub.args[0]) == 'LioReadiness'
    assert ast.unparse(readiness_sub.args[2]) == (
        'self.liosam_ready_callback')
    assert ast.unparse(readiness_sub.args[3]) == 'ready_qos'
    assert ast.unparse(callback_group) == (
        'self.liosam_ready_callback_group')


def test_liosam_structured_readiness_is_latched_without_changing_legacy_bool():
    source_path = (
        repository_root() / 'slam/src/LIO-SAM/src/imuPreintegration.cpp')
    source = source_path.read_text(encoding='utf-8')
    legacy_start = source.index('auto legacyReadyQos')
    structured_start = source.index('auto readinessQos', legacy_start)
    publisher_end = source.index('const char * instanceId', structured_start)
    legacy_publisher = source[legacy_start:structured_start]
    structured_publisher = source[structured_start:publisher_end]

    assert 'rclcpp::QoS(rclcpp::KeepLast(1)).reliable()' in legacy_publisher
    assert '.transient_local()' not in legacy_publisher
    assert 'create_publisher<std_msgs::msg::Bool>' in legacy_publisher
    assert '"lio_sam/imu_preintegration_ready"' in legacy_publisher

    assert 'rclcpp::QoS(rclcpp::KeepLast(1))' in structured_publisher
    assert '.reliable()' in structured_publisher
    assert '.transient_local()' in structured_publisher
    assert ('create_publisher<multi_map_navigation_msgs::msg::LioReadiness>'
            in structured_publisher)
    assert '"lio_sam/imu_preintegration_readiness"' in structured_publisher

    publish_start = source.index('void publishReady(bool ready)')
    publish_end = source.index('void processingLoop()', publish_start)
    publish_body = source[publish_start:publish_end]
    assert 'std_msgs::msg::Bool message;' in publish_body
    assert 'pubReady->publish(message);' in publish_body
    assert ('multi_map_navigation_msgs::msg::LioReadiness readiness;'
            in publish_body)
    assert 'pubReadiness->publish(readiness);' in publish_body


def test_transient_local_readiness_reaches_late_subscriber_with_latest_value():
    context = Context()
    rclpy.init(context=context)
    suffix = uuid.uuid4().hex
    topic = f'/test/lio_readiness_retained_{suffix}'
    writer = Node(f'lio_readiness_writer_{suffix}', context=context)
    reader = Node(f'lio_readiness_reader_{suffix}', context=context)
    executor = SingleThreadedExecutor(context=context)
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    received = []

    try:
        publisher = writer.create_publisher(LioReadiness, topic, qos)
        old = LioReadiness()
        old.instance_id = 'lio-qos-test'
        old.generation = 42
        old.ready = False
        publisher.publish(old)
        current = LioReadiness()
        current.instance_id = 'lio-qos-test'
        current.generation = 42
        current.ready = True
        publisher.publish(current)

        reader.create_subscription(
            LioReadiness, topic, received.append, qos)
        executor.add_node(writer)
        executor.add_node(reader)
        deadline = time.monotonic() + 3.0
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)

        assert len(received) == 1
        assert received[0].instance_id == 'lio-qos-test'
        assert received[0].generation == 42
        assert received[0].ready is True
    finally:
        executor.shutdown()
        writer.destroy_node()
        reader.destroy_node()
        context.shutdown()


def test_main_uses_eight_executor_threads_for_lifecycle_callbacks():
    source = textwrap.dedent(inspect.getsource(main))
    tree = ast.parse(source)
    executor_calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name):
            continue
        if node.func.id == 'MultiThreadedExecutor':
            executor_calls.append(node)

    assert len(executor_calls) == 1
    num_threads = next(
        keyword.value for keyword in executor_calls[0].keywords
        if keyword.arg == 'num_threads')
    assert isinstance(num_threads, ast.Constant)
    assert num_threads.value == 8
