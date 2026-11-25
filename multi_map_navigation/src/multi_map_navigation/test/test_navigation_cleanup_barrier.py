from types import SimpleNamespace
import threading

import pytest

from multi_map_navigation.navigation_manager import NavigationManager


class FakeLogger:
    def __init__(self):
        self.messages = []

    def debug(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)

    def info(self, message):
        self.messages.append(message)

    def warn(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)


class FinishedFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class RecordingClient:
    def __init__(self, response_for_request):
        self.response_for_request = response_for_request
        self.requests = []

    def call_async(self, request):
        self.requests.append(request)
        return FinishedFuture(self.response_for_request(request))


class RecordingTransactionLock:
    def __init__(self, acquire_result):
        self.acquire_result = acquire_result
        self.timeouts = []
        self.releases = 0

    def acquire(self, timeout_sec=0.0):
        self.timeouts.append(timeout_sec)
        return self.acquire_result

    def release(self):
        self.releases += 1


def all_stopped():
    return {
        're_localization': False,
        'nav2_init_pose': False,
        'liosam': False,
        'navigation2': False,
    }


def active_manager():
    manager = NavigationManager.__new__(NavigationManager)
    manager.state_lock = threading.RLock()
    manager._navigation_generation = 1
    manager._navigation_cleanup_active = False
    manager._active_goal_token = None
    manager.current_goal_handle = None
    manager.current_waypoint_index = 0
    manager.waypoint_list = [SimpleNamespace(map_name='map1', type=1)]
    manager.is_navigating = True
    manager.is_map_switching = False
    manager._active_map_switch_transaction_id = ''
    manager._active_map_switch_context = None
    manager.stack_fault_active = False
    manager.docking_hold = False
    manager.pending_replan_endpoints = None
    manager._stack_recovery_token = None
    manager.get_logger = lambda: FakeLogger()
    manager.states = []
    manager.publish_robot_state = manager.states.append
    return manager


@pytest.mark.parametrize(
    'status',
    [
        None,
        {
            're_localization': True,
            'nav2_init_pose': True,
            'liosam': False,
            'navigation2': False,
        },
    ],
)
def test_unknown_or_partial_status_never_starts_a_second_stack(status):
    manager = active_manager()
    launches = []
    recoveries = []
    manager.get_process_status = lambda: status
    manager.launch_new_stack = lambda map_name: launches.append(map_name) or True
    manager.abort_navigation = lambda reason, navigation_context=None: (
        recoveries.append((reason, navigation_context)) or True)

    manager.navigate_to_next_waypoint()

    assert launches == []
    assert recoveries == [(2, (1, 0))]


def test_running_stack_on_another_map_is_never_reused():
    manager = active_manager()
    manager._last_status_from_service = True
    manager._last_status_has_identity = True
    manager._last_reported_stack_map = 'old_map'
    manager._last_reported_stack_session = 'old_session'
    manager.get_process_status = lambda: {
        're_localization': True,
        'nav2_init_pose': True,
        'liosam': True,
        'navigation2': True,
    }
    goals = []
    manager.send_nav2_goal = lambda *_args: goals.append(True)
    recoveries = []
    manager.abort_navigation = lambda reason, navigation_context=None: (
        recoveries.append((reason, navigation_context)) or True)

    manager.navigate_to_next_waypoint()

    assert goals == []
    assert recoveries == [(2, (1, 0))]


def test_status_query_timeout_is_unknown_instead_of_all_stopped():
    manager = NavigationManager.__new__(NavigationManager)
    manager.get_logger = lambda: FakeLogger()
    manager.get_process_status_client = RecordingClient(lambda _request: object())
    manager._wait_for_future = lambda _future, timeout_sec: None

    assert manager.get_process_status() is None


@pytest.mark.parametrize('stack_state', ['PARTIAL', 'FAILED', 'STOPPING'])
def test_non_running_stack_state_is_never_reused(stack_state):
    manager = active_manager()
    status = SimpleNamespace(
        re_localization_running=True,
        nav2_init_pose_running=True,
        liosam_running=True,
        navigation2_running=True,
        stack_state=stack_state,
        active_map='map1',
        localization_session_id='session-map1',
    )
    manager.get_process_status_client = RecordingClient(lambda _request: status)
    manager._wait_for_future = lambda future, timeout_sec: future.result()
    goals = []
    recoveries = []
    manager.send_nav2_goal = lambda *_args: goals.append(True)
    manager.abort_navigation = lambda reason, navigation_context=None: (
        recoveries.append((reason, navigation_context)) or True)

    manager.navigate_to_next_waypoint()

    assert goals == []
    assert recoveries == [(2, (1, 0))]


def test_running_stack_state_with_identity_can_be_reused(monkeypatch):
    manager = active_manager()
    status = SimpleNamespace(
        re_localization_running=True,
        nav2_init_pose_running=True,
        liosam_running=True,
        navigation2_running=True,
        stack_state='RUNNING',
        active_map='map1',
        localization_session_id='session-map1',
    )
    manager.get_process_status_client = RecordingClient(lambda _request: status)
    manager._wait_for_future = lambda future, timeout_sec: future.result()
    monkeypatch.setattr(
        'multi_map_navigation.navigation_manager.time.sleep',
        lambda _seconds: None,
    )
    goals = []
    manager.send_nav2_goal = lambda *_args: goals.append(True)

    manager.navigate_to_next_waypoint()

    assert goals == [True]


def test_shutdown_collects_every_response_and_requires_consecutive_clean_status():
    manager = NavigationManager.__new__(NavigationManager)
    manager.get_logger = lambda: FakeLogger()
    shutdowns = []
    manager.shutdown_process_by_name = lambda name: shutdowns.append(name) or True
    statuses = iter([
        {**all_stopped(), 'navigation2': True},
        all_stopped(),
        None,
        all_stopped(),
        all_stopped(),
    ])
    manager.get_process_status = lambda: next(statuses)
    manager.CLEAN_STATUS_TIMEOUT_SEC = 1.0
    manager.CLEAN_STATUS_POLL_SEC = 0.0

    assert manager.shutdown_all_processes_service()
    assert shutdowns == [
        'navigation2', 'liosam', 'nav2_init_pose', 're_localization']
    assert manager._last_shutdown_results == {
        name: True for name in shutdowns}


def test_shutdown_failure_does_not_skip_later_processes_or_report_clean():
    manager = NavigationManager.__new__(NavigationManager)
    manager.get_logger = lambda: FakeLogger()
    shutdowns = []

    def shutdown(name):
        shutdowns.append(name)
        return name != 'liosam'

    manager.shutdown_process_by_name = shutdown
    manager.get_process_status = all_stopped
    manager.CLEAN_STATUS_TIMEOUT_SEC = 1.0
    manager.CLEAN_STATUS_POLL_SEC = 0.0

    assert not manager.shutdown_all_processes_service()
    assert shutdowns == [
        'navigation2', 'liosam', 'nav2_init_pose', 're_localization']


def test_shutdown_client_wait_exceeds_process_manager_grace_period():
    manager = NavigationManager.__new__(NavigationManager)
    manager.get_logger = lambda: FakeLogger()
    manager.shutdown_process_client = RecordingClient(
        lambda _request: SimpleNamespace(success=True))
    waits = []
    manager._wait_for_future = lambda future, timeout_sec: (
        waits.append(timeout_sec) or future.result())

    assert manager.shutdown_process_by_name('navigation2')
    assert waits == [manager.PROCESS_SHUTDOWN_TIMEOUT_SEC]
    assert waits[0] > 10.0


def test_cleanup_returns_deferred_when_stack_transaction_lock_stays_busy():
    manager = NavigationManager.__new__(NavigationManager)
    manager.get_logger = lambda: FakeLogger()
    transaction_lock = RecordingTransactionLock(False)
    manager._stack_transaction_lock = transaction_lock
    manager._shutdown_all_processes_owned = lambda: pytest.fail(
        'cleanup ownership must not run without the transaction lock')

    result = manager.shutdown_all_processes_service()

    assert result is manager.CLEANUP_DEFERRED
    assert transaction_lock.timeouts == [manager.STACK_TRANSACTION_WAIT_SEC]
    assert transaction_lock.releases == 0


def completion_manager(cleanup_result):
    manager = active_manager()
    manager.waypoint_list = [SimpleNamespace(map_name='map1')]
    manager.current_waypoint_index = 0
    manager._reset_stack_recovery = lambda: None
    manager.shutdown_all_processes_service = lambda: cleanup_result
    return manager


def test_completed_task_only_publishes_idle_after_verified_cleanup():
    manager = completion_manager(True)

    assert manager.complete_navigation(expected_generation=1)
    assert manager.states == ['idle']
    assert not manager._navigation_cleanup_active
    assert not manager.stack_fault_active


def test_only_current_navigation_generation_publishes_map_switch_cancel():
    manager = completion_manager(True)
    manager._active_map_switch_transaction_id = 'tx-current'
    published = []
    manager.map_switch_cancel_pub = SimpleNamespace(
        publish=published.append)

    assert manager.complete_navigation(expected_generation=0) is None
    assert published == []
    assert manager._active_map_switch_transaction_id == 'tx-current'
    assert manager._navigation_generation == 1

    assert manager.complete_navigation(expected_generation=1)
    assert len(published) == 1
    assert published[0].transaction_id == 'tx-current'
    assert published[0].reason == 'navigation_complete'
    assert manager._active_map_switch_transaction_id == ''


def test_completed_task_latches_fault_and_gate_when_cleanup_is_unverified():
    manager = completion_manager(False)

    assert not manager.complete_navigation(expected_generation=1)
    assert manager.states == ['fault']
    assert manager._navigation_cleanup_active
    assert manager.stack_fault_active
    assert not manager.is_navigating


def test_completed_task_keeps_gate_and_schedules_cleanup_when_lock_is_busy():
    manager = completion_manager(NavigationManager.CLEANUP_DEFERRED)
    scheduled = []
    manager._schedule_deferred_cleanup = lambda **kwargs: scheduled.append(
        kwargs)
    manager._latch_cleanup_failure = lambda _reason: pytest.fail(
        'lock contention is not a permanent cleanup failure')

    assert manager.complete_navigation(expected_generation=1) is None

    assert manager._navigation_cleanup_active
    assert not manager.stack_fault_active
    assert manager.states == []
    assert scheduled == [{
        'mode': 'complete',
        'reason': '导航完成后的堆栈清理',
    }]


def test_fault_cleanup_failure_keeps_task_gate_closed():
    manager = active_manager()
    manager.stack_fault_active = True
    manager._navigation_cleanup_active = True
    manager.shutdown_all_processes_service = lambda: False

    manager._cleanup_failed_stack('injected fault')

    assert manager.states == ['fault']
    assert manager._navigation_cleanup_active
    assert manager.stack_fault_active


def test_fault_cleanup_keeps_gate_and_schedules_retry_when_lock_is_busy():
    manager = active_manager()
    manager.stack_fault_active = True
    manager._navigation_cleanup_active = True
    manager.shutdown_all_processes_service = (
        lambda: NavigationManager.CLEANUP_DEFERRED)
    scheduled = []
    manager._schedule_deferred_cleanup = lambda **kwargs: scheduled.append(
        kwargs)
    manager._finish_fault_cleanup = lambda *_args: pytest.fail(
        'deferred cleanup must not be finalized as a failure')
    manager._latch_cleanup_failure = lambda _reason: pytest.fail(
        'lock contention is not a permanent cleanup failure')

    manager._cleanup_failed_stack('injected lock contention')

    assert manager._navigation_cleanup_active
    assert manager.stack_fault_active
    assert not manager.is_navigating
    assert manager.states == ['fault']
    assert scheduled == [{
        'mode': 'fault',
        'reason': 'injected lock contention',
    }]


class FakeTimer:
    def cancel(self):
        pass


def test_recovery_never_restarts_after_unverified_cleanup():
    manager = active_manager()
    token = (4, 0, 1)
    manager._stack_recovery_generation = 4
    manager._stack_recovery_token = token
    manager._stack_recovery_timer = FakeTimer()
    manager.destroy_timer = lambda _timer: None
    manager.max_retries = 3
    manager.shutdown_all_processes_service = lambda: False
    launches = []
    manager.launch_new_stack = lambda map_name: launches.append(map_name) or True

    manager._run_stack_recovery(token, retry_map_switch=False)

    assert launches == []
    assert manager._navigation_cleanup_active
    assert manager.stack_fault_active
    assert manager.states == ['fault']


def launch_manager(failing_stage):
    manager = active_manager()
    start_timeouts = []

    def response(request):
        return SimpleNamespace(
            success=request.process_name != failing_stage,
            instance_id=(
                'session-map1'
                if request.process_name == 're_localization' else ''),
            active_map=(
                'map1' if request.process_name == 're_localization' else ''),
            session_start_ns=(
                100 if request.process_name == 're_localization' else 0),
        )

    manager.start_process_client = RecordingClient(response)
    manager._wait_for_future = lambda future, timeout_sec: (
        start_timeouts.append(timeout_sec) or future.result())
    manager.wait_for_map_to_odom_link_tf = lambda **_kwargs: failing_stage != 'tf'
    shutdowns = []
    manager.shutdown_process_by_name = lambda name: shutdowns.append(name) or True
    manager.get_process_status = all_stopped
    manager.CLEAN_STATUS_TIMEOUT_SEC = 1.0
    manager.CLEAN_STATUS_POLL_SEC = 0.0
    return manager, start_timeouts, shutdowns


@pytest.mark.parametrize(
    'failing_stage',
    ['re_localization', 'nav2_init_pose', 'tf', 'liosam', 'navigation2'],
)
def test_each_startup_failure_rolls_back_in_reverse_order_and_verifies_clean(
        failing_stage):
    manager, timeouts, shutdowns = launch_manager(failing_stage)

    assert not manager.launch_new_stack('map1')
    assert shutdowns == [
        'navigation2', 'liosam', 'nav2_init_pose', 're_localization']
    assert timeouts
    assert all(timeout >= 150.0 for timeout in timeouts)
    assert manager._last_stack_rollback_verified
    assert not manager._navigation_cleanup_active


def test_failed_startup_rollback_latches_fault_and_cleanup_gate():
    manager, _timeouts, _shutdowns = launch_manager('re_localization')
    manager.shutdown_all_processes_service = lambda: False

    assert not manager.launch_new_stack('map1')
    assert not manager._last_stack_rollback_verified
    assert manager.stack_fault_active
    assert manager._navigation_cleanup_active
    assert manager.states == ['fault']


@pytest.mark.parametrize('ambiguous_result', ['busy', 'timeout'])
def test_ambiguous_start_after_partial_success_defers_without_cleanup_or_abort(
        ambiguous_result):
    manager = active_manager()

    def start_response(request):
        if request.process_name == 're_localization':
            return SimpleNamespace(
                success=True,
                busy=False,
                message='started',
                instance_id='session-map1',
                active_map='map1',
                session_start_ns=100,
            )
        return SimpleNamespace(
            success=False,
            busy=True,
            message='another transaction owns ProcessManager',
            instance_id='',
            active_map='',
            session_start_ns=0,
        )

    manager.start_process_client = RecordingClient(start_response)

    def wait_for_start(future, timeout_sec):
        del timeout_sec
        second_request = len(manager.start_process_client.requests) == 2
        if ambiguous_result == 'timeout' and second_request:
            return None
        return future.result()

    manager._wait_for_future = wait_for_start
    manager.wait_for_map_to_odom_link_tf = lambda **_kwargs: (
        pytest.fail('TF wait must not run after an ambiguous start result'))
    cleanups = []
    aborts = []
    manager.shutdown_all_processes_service = (
        lambda: cleanups.append('shutdown') or True)
    manager.shutdown_process_by_name = (
        lambda name: cleanups.append(name) or True)
    manager.abort_navigation = lambda *args, **kwargs: (
        aborts.append((args, kwargs)) or True)

    assert manager.launch_new_stack('map1') is None
    assert [
        request.process_name
        for request in manager.start_process_client.requests
    ] == ['re_localization', 'nav2_init_pose']
    assert cleanups == []
    assert aborts == []


def test_navigation_defers_ambiguous_stack_start_without_abort():
    manager = active_manager()
    manager.get_process_status = all_stopped
    manager.launch_new_stack = lambda _map_name: None
    retries = []
    aborts = []
    manager._schedule_deferred_stack_start = retries.append
    manager.abort_navigation = lambda *args, **kwargs: (
        aborts.append((args, kwargs)) or True)

    manager.navigate_to_next_waypoint()

    assert manager._stack_start_deferred
    assert retries == [(1, 0)]
    assert aborts == []


def test_resumed_navigation_start_preserves_idempotent_localization_evidence():
    manager = active_manager()
    localization_result = object()
    manager._active_localization_session = 'session-map1'
    manager._active_localization_map = 'map1'
    manager._localization_session_start_ns = 100
    manager._expected_localization_session = ''
    manager._expected_localization_map = ''
    manager._expected_session_start_ns = 0
    manager._latest_localization_result = localization_result
    manager._pending_localization_result = None

    def start_response(request):
        return SimpleNamespace(
            success=True,
            busy=False,
            message='already healthy',
            instance_id=(
                'session-map1'
                if request.process_name == 're_localization' else ''),
            active_map=(
                'map1'
                if request.process_name == 're_localization' else ''),
            session_start_ns=(
                100 if request.process_name == 're_localization' else 0),
        )

    manager.start_process_client = RecordingClient(start_response)
    manager._wait_for_future = lambda future, timeout_sec: future.result()
    manager.wait_for_map_to_odom_link_tf = lambda **_kwargs: True
    manager._reset_localization_gate = lambda _map_name: pytest.fail(
        'resume must not clear active localization evidence')

    assert manager._launch_new_stack_owned('map1', resume=True)

    assert [
        request.process_name
        for request in manager.start_process_client.requests
    ] == ['re_localization', 'nav2_init_pose', 'liosam', 'navigation2']
    assert manager._active_localization_session == 'session-map1'
    assert manager._active_localization_map == 'map1'
    assert manager._localization_session_start_ns == 100
    assert manager._expected_localization_session == 'session-map1'
    assert manager._expected_localization_map == 'map1'
    assert manager._expected_session_start_ns == 100
    assert manager._latest_localization_result is localization_result


def test_deferred_navigation_start_deadline_terminates_instead_of_retrying(
        monkeypatch):
    manager = active_manager()
    manager._stack_start_deferred = True
    manager._stack_start_deferred_deadline = 9.0
    manager._last_reported_stack_state = 'STARTING'
    manager.get_process_status = lambda: {
        're_localization': True,
        'nav2_init_pose': False,
        'liosam': False,
        'navigation2': False,
    }
    manager._schedule_deferred_stack_start = lambda _context: pytest.fail(
        'expired deferred start must not schedule another retry')
    manager.launch_new_stack = lambda *_args, **_kwargs: pytest.fail(
        'expired deferred start must not launch again')
    manager.abort_navigation = lambda *_args, **_kwargs: pytest.fail(
        'deadline exhaustion must terminate, not enter recovery retries')
    faults = []
    manager.process_fault_callback = lambda message: faults.append(message.data)
    monkeypatch.setattr(
        'multi_map_navigation.navigation_manager.time.monotonic',
        lambda: 10.0,
    )

    manager.navigate_to_next_waypoint()

    assert len(faults) == 1
    assert 'ProcessManager' in faults[0]
    assert 'timeout' in faults[0].lower() or '超时' in faults[0]
    assert not manager._stack_start_deferred
    assert manager._stack_start_deferred_deadline is None


def test_cancelled_deferred_navigation_timer_callback_cannot_restart_stack(
        monkeypatch):
    manager = active_manager()
    manager.retry_delay = 0.1
    timers = []

    class ManualTimer:
        def __init__(self, _delay, callback):
            self.callback = callback
            self.daemon = False
            self.cancelled = False
            self.alive = False

        def is_alive(self):
            return self.alive and not self.cancelled

        def start(self):
            self.alive = True
            timers.append(self)

        def cancel(self):
            self.cancelled = True
            self.alive = False

    monkeypatch.setattr(
        'multi_map_navigation.navigation_manager.threading.Timer',
        ManualTimer,
    )
    resumed = []
    manager.navigate_to_next_waypoint = lambda **kwargs: resumed.append(kwargs)

    manager._schedule_deferred_stack_start((1, 0))
    assert len(timers) == 1
    stale_callback = timers[0].callback

    manager._reset_stack_recovery()
    stale_callback()

    assert timers[0].cancelled
    assert manager._stack_start_retry_timer is None
    assert resumed == []
