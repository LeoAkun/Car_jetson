"""Regression tests for the map-switch cleanup barrier."""

from collections import OrderedDict
import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from multi_map_navigation.map_switch_controller import MapSwitchController
from multi_map_navigation_msgs.msg import MapSwitchCancel, MapSwitchTrigger
import pytest


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


class CompletedFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class QueueClient:
    def __init__(self, responses):
        self.responses = list(responses)
        self.requests = []

    def call_async(self, request):
        self.requests.append(request)
        if not self.responses:
            raise AssertionError('unexpected service request')
        return CompletedFuture(self.responses.pop(0))


def switch_message(transaction_id='tx-map2', current_map='map1',
                   next_map='map2'):
    message = MapSwitchTrigger()
    message.transaction_id = transaction_id
    message.current_map = current_map
    message.next_map = next_map
    message.switch_pose = PoseStamped()
    return message


def cancel_message(transaction_id, reason='test_cancel'):
    message = MapSwitchCancel()
    message.transaction_id = transaction_id
    message.reason = reason
    return message


def response(success=True, message='ok', busy=False):
    return SimpleNamespace(
        success=success,
        message=message,
        busy=busy,
        instance_id='session-map2',
        active_map='map2',
        session_start_ns=100,
    )


def stopped_status():
    return SimpleNamespace(
        re_localization_running=False,
        liosam_running=False,
        nav2_init_pose_running=False,
        navigation2_running=False,
        message='stopped',
        stack_state='STOPPED',
        active_map='',
        localization_session_id='',
        session_start_ns=0,
    )


def bare_controller():
    controller = MapSwitchController.__new__(MapSwitchController)
    logger = FakeLogger()
    controller.get_logger = lambda: logger
    controller.process_shutdown_timeout = 0.1
    controller.map_switch_timeout = 0.1
    controller.current_map = 'map1'
    controller.is_switching = False
    controller._switch_lock = threading.Lock()
    controller._deferred_switch_lock = threading.Lock()
    controller._deferred_switch_timer = None
    controller._deferred_switch_message = None
    controller._deferred_switch_generation = 0
    controller._deferred_switch_deadline = None
    controller._map_switch_request_active = False
    controller._active_transaction_id = ''
    controller._closed_transactions = set()
    controller._pending_switch_triggers = OrderedDict()
    controller._localization_lock = threading.RLock()
    controller._active_localization_session = ''
    controller._active_localization_map = ''
    controller._localization_session_start_ns = 0
    controller._expected_localization_session = ''
    controller._expected_localization_map = ''
    controller._expected_session_start_ns = 0
    controller._latest_localization_result = None
    controller._pending_localization_result = None
    return controller


def test_shutdown_checks_every_response_and_uses_server_sized_timeout():
    controller = bare_controller()
    controller.shutdown_process_client = QueueClient([
        response(),
        response(False, 'liosam refused shutdown'),
        response(),
        response(),
    ])
    timeouts = []
    original_wait = controller._wait_for_future

    def record_timeout(future, timeout_sec):
        timeouts.append(timeout_sec)
        return original_wait(future, timeout_sec)

    controller._wait_for_future = record_timeout

    assert controller.shutdown_current_stack() is False
    assert [
        request.process_name
        for request in controller.shutdown_process_client.requests
    ] == ['navigation2', 'liosam', 'nav2_init_pose', 're_localization']
    assert len(timeouts) == 4
    assert all(timeout >= 15.0 for timeout in timeouts)


def test_clean_shutdown_fails_closed_when_status_is_unknown(monkeypatch):
    controller = bare_controller()
    controller.get_process_status_client = QueueClient([None] * 1000)

    # Keep this regression test fast while exercising repeated unknown replies.
    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.time.sleep',
        lambda _seconds: None,
    )

    assert controller.wait_for_clean_shutdown(0.01) is False
    assert controller.get_process_status_client.requests


def test_incomplete_status_cannot_be_treated_as_clean():
    controller = bare_controller()
    controller.get_process_status_client = QueueClient([
        SimpleNamespace(re_localization_running=False),
    ])

    assert controller.wait_for_clean_shutdown(0.1) is False


def test_navigation2_failure_rolls_back_and_verifies_clean():
    controller = bare_controller()
    controller.start_process_client = QueueClient([
        response(),
        response(),
        response(),
        response(False, 'nav2 lifecycle timeout'),
    ])
    controller.shutdown_process_client = QueueClient([response()] * 3)
    controller.get_process_status_client = QueueClient([stopped_status()])
    controller.wait_for_map_to_odom_link_tf = lambda timeout_sec: True

    wait_timeouts = []
    original_wait = controller._wait_for_future

    def record_timeout(future, timeout_sec):
        wait_timeouts.append(timeout_sec)
        return original_wait(future, timeout_sec)

    controller._wait_for_future = record_timeout

    assert controller.launch_new_stack('map2') is False
    assert [
        request.process_name
        for request in controller.start_process_client.requests
    ] == ['re_localization', 'nav2_init_pose', 'liosam', 'navigation2']
    assert [
        request.process_name
        for request in controller.shutdown_process_client.requests
    ] == ['liosam', 'nav2_init_pose', 're_localization']
    assert controller.get_process_status_client.requests

    start_timeouts = wait_timeouts[:4]
    assert start_timeouts[-1] >= 150.0


def test_tf_wait_exception_rolls_back_started_processes_and_checks_status():
    controller = bare_controller()
    controller.start_process_client = QueueClient([response(), response()])
    controller.shutdown_process_client = QueueClient([response()] * 2)
    controller.get_process_status_client = QueueClient([stopped_status()])

    def fail_tf_wait(timeout_sec):
        raise RuntimeError(f'TF failed before {timeout_sec}')

    controller.wait_for_map_to_odom_link_tf = fail_tf_wait

    assert controller.launch_new_stack('map2') is False
    assert [
        request.process_name
        for request in controller.shutdown_process_client.requests
    ] == ['nav2_init_pose', 're_localization']
    assert controller.get_process_status_client.requests


def test_switch_pose_is_sent_only_to_initial_pose_process():
    controller = bare_controller()
    controller.start_process_client = QueueClient([response()] * 4)
    controller.wait_for_map_to_odom_link_tf = lambda timeout_sec: True
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 4.5
    pose.pose.position.y = -1.25
    pose.pose.orientation.w = 1.0

    assert controller.launch_new_stack('map2', pose)

    requests = controller.start_process_client.requests
    assert [request.process_name for request in requests] == [
        're_localization', 'nav2_init_pose', 'liosam', 'navigation2']
    assert not requests[0].use_initial_pose
    assert requests[1].use_initial_pose
    assert requests[1].initial_pose.pose.position.x == 4.5
    assert requests[1].initial_pose.pose.position.y == -1.25
    assert not requests[2].use_initial_pose
    assert not requests[3].use_initial_pose


def test_ready_verification_failure_rolls_back_and_releases_switch_lock():
    controller = bare_controller()
    events = []
    controller.shutdown_current_stack = (
        lambda **_kwargs: events.append('shutdown') or True
    )
    controller.wait_for_clean_shutdown = (
        lambda _timeout, **_kwargs: events.append('clean') or True
    )
    controller.launch_new_stack = lambda _map: events.append('launch') or True
    controller.verify_stack_ready = (
        lambda **_kwargs: events.append('verify') or False)
    controller._rollback_processes = (
        lambda names, reason: events.append(
            ('rollback', tuple(names), reason)) or True
    )

    assert controller.execute_map_switch('map1', 'map2') is False
    assert controller.current_map == 'map1'
    assert controller.is_switching is False
    assert events[:4] == ['shutdown', 'clean', 'launch', 'verify']
    assert events[4][0] == 'rollback'
    assert events[4][1] == controller.STACK_START_ORDER
    assert controller._switch_lock.acquire(blocking=False)
    controller._switch_lock.release()


def test_concurrent_map_switch_is_rejected_before_stack_mutation():
    controller = bare_controller()
    mutations = []
    controller.shutdown_current_stack = (
        lambda: mutations.append('shutdown') or True
    )
    controller._switch_lock.acquire()
    try:
        assert controller.execute_map_switch('map1', 'map2') is False
    finally:
        controller._switch_lock.release()

    assert mutations == []
    assert controller.is_switching is False


@pytest.mark.parametrize('ambiguous_result', ['busy', 'timeout'])
def test_ambiguous_start_after_partial_success_does_not_rollback_or_shutdown(
        ambiguous_result):
    controller = bare_controller()
    controller.start_process_client = QueueClient([
        response(),
        response(
            success=False,
            message='another transaction owns ProcessManager',
            busy=True,
        ),
    ])

    original_wait = controller._wait_for_future

    def wait_for_start(future, timeout_sec):
        second_request = len(controller.start_process_client.requests) == 2
        if ambiguous_result == 'timeout' and second_request:
            return None
        return original_wait(future, timeout_sec)

    controller._wait_for_future = wait_for_start
    cleanups = []
    controller._rollback_processes = lambda names, reason: (
        cleanups.append(('rollback', tuple(names), reason)) or True)
    controller.shutdown_process_by_name = lambda name, **_kwargs: (
        cleanups.append(('shutdown', name)) or True)
    controller.wait_for_map_to_odom_link_tf = lambda timeout_sec: (
        pytest.fail('TF wait must not run after an ambiguous start result'))

    assert controller.launch_new_stack('map2') is None
    assert [
        request.process_name
        for request in controller.start_process_client.requests
    ] == ['re_localization', 'nav2_init_pose']
    assert cleanups == []


def test_map_switch_deferred_result_publishes_nothing_then_uses_resume(
        monkeypatch):
    controller = bare_controller()
    message = switch_message('tx-deferred')
    timers = []

    class ManualTimer:
        def __init__(self, _delay, callback):
            self.callback = callback
            self.daemon = False
            self._alive = False

        def is_alive(self):
            return self._alive

        def start(self):
            self._alive = True
            timers.append(self)

    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.threading.Timer',
        ManualTimer,
    )
    controller._execute_map_switch_transaction = (
        lambda *_args, **_kwargs: controller.DEFERRED)
    resumed = []
    controller._resume_deferred_map_switch_start = (
        lambda pending, generation: resumed.append(
            (pending, generation)) or True)
    published = []
    controller._publish_map_switch_completion = (
        lambda success, transaction_id: published.append(
            (transaction_id, success)))

    controller.map_switch_callback(message)

    assert published == []
    assert len(timers) == 1

    timers[0]._alive = False
    timers[0].callback()

    assert resumed == [(message, controller._deferred_switch_generation)]
    assert published == [('tx-deferred', True)]


def test_deferred_resume_does_not_repeat_stack_shutdown():
    controller = bare_controller()
    message = switch_message('tx-resume')
    events = []

    class OwnedTransaction:
        def acquire(self):
            events.append('acquire')
            return True

        def release(self):
            events.append('release')

    controller._stack_transaction_lock = OwnedTransaction()
    controller.shutdown_current_stack = lambda: (
        events.append('shutdown') or True)
    controller.launch_new_stack = lambda map_name, **_kwargs: (
        events.append(('launch', map_name)) or True)
    controller.verify_stack_ready = lambda **_kwargs: (
        events.append('verify') or True)

    assert controller._resume_deferred_map_switch_start(message) is True
    assert events == ['acquire', ('launch', 'map2'), 'verify', 'release']
    assert controller.current_map == 'map2'


def test_resumed_map_switch_preserves_idempotent_localization_evidence():
    controller = bare_controller()
    localization_result = object()
    controller._active_localization_session = 'session-map2'
    controller._active_localization_map = 'map2'
    controller._localization_session_start_ns = 100
    controller._latest_localization_result = localization_result
    controller.start_process_client = QueueClient([response()] * 4)
    controller.wait_for_map_to_odom_link_tf = lambda timeout_sec: True
    controller._reset_localization_gate = lambda _map_name: pytest.fail(
        'resume must not clear active localization evidence')

    assert controller.launch_new_stack('map2', resume=True)

    assert [
        request.process_name
        for request in controller.start_process_client.requests
    ] == ['re_localization', 'nav2_init_pose', 'liosam', 'navigation2']
    assert controller._active_localization_session == 'session-map2'
    assert controller._active_localization_map == 'map2'
    assert controller._localization_session_start_ns == 100
    assert controller._expected_localization_session == 'session-map2'
    assert controller._expected_localization_map == 'map2'
    assert controller._expected_session_start_ns == 100
    assert controller._latest_localization_result is localization_result


def test_deferred_map_switch_timeout_publishes_false_exactly_once(monkeypatch):
    controller = bare_controller()
    message = switch_message('tx-timeout')
    controller._map_switch_request_active = True
    controller._active_transaction_id = message.transaction_id
    controller._deferred_switch_generation = 7
    controller._deferred_switch_deadline = 10.0
    published = []
    controller._publish_map_switch_completion = (
        lambda success, transaction_id: published.append(
            (transaction_id, success)))
    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.time.monotonic',
        lambda: 10.0,
    )

    controller._schedule_deferred_map_switch(
        message, resume_start=True, generation=7)
    controller._schedule_deferred_map_switch(
        message, resume_start=True, generation=7)

    assert published == [('tx-timeout', False)]
    assert not controller._map_switch_request_active
    assert controller._deferred_switch_message is None
    assert controller._deferred_switch_timer is None
    assert controller._deferred_switch_deadline is None
    assert controller._deferred_switch_generation == 8


def test_new_trigger_cannot_replace_active_deferred_map_switch(monkeypatch):
    controller = bare_controller()
    first = switch_message('tx-first')
    second = switch_message('tx-second', 'map2', 'map3')
    timers = []

    class ManualTimer:
        def __init__(self, _delay, callback):
            self.callback = callback
            self.daemon = False
            self.alive = False

        def is_alive(self):
            return self.alive

        def start(self):
            self.alive = True
            timers.append(self)

        def cancel(self):
            self.alive = False

    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.threading.Timer',
        ManualTimer,
    )
    executions = []
    controller._execute_map_switch_transaction = (
        lambda current, next_map, _pose, **_kwargs:
        executions.append((current, next_map)) or controller.DEFERRED)
    published = []
    controller._publish_map_switch_completion = (
        lambda success, transaction_id: published.append(
            (transaction_id, success)))

    controller.map_switch_callback(first)
    generation = controller._deferred_switch_generation
    controller.map_switch_callback(second)

    assert executions == [('map1', 'map2')]
    assert len(timers) == 1
    assert controller._map_switch_request_active
    assert controller._deferred_switch_generation == generation
    assert controller._deferred_switch_message[0] is first
    assert published == []


def test_cancel_clears_deferred_map_switch_and_stale_callback(monkeypatch):
    controller = bare_controller()
    message = switch_message('tx-cancelled')
    timers = []

    class ManualTimer:
        def __init__(self, _delay, callback):
            self.callback = callback
            self.daemon = False
            self.alive = False
            self.cancelled = False

        def is_alive(self):
            return self.alive and not self.cancelled

        def start(self):
            self.alive = True
            timers.append(self)

        def cancel(self):
            self.cancelled = True
            self.alive = False

    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.threading.Timer',
        ManualTimer,
    )
    controller._map_switch_request_active = True
    controller._active_transaction_id = message.transaction_id
    controller._deferred_switch_generation = 11
    controller._resume_deferred_map_switch_start = lambda *_args: pytest.fail(
        'stale callback must not resume a cancelled switch')
    controller._execute_map_switch_transaction = lambda *_args: pytest.fail(
        'stale callback must not restart a cancelled transaction')
    published = []
    controller._publish_map_switch_completion = (
        lambda success, transaction_id: published.append(
            (transaction_id, success)))

    controller._schedule_deferred_map_switch(
        message, resume_start=True, generation=11)
    stale_callback = timers[0].callback

    controller.map_switch_cancel_callback(
        cancel_message(message.transaction_id))
    stale_callback()

    assert timers[0].cancelled
    assert controller._deferred_switch_timer is None
    assert controller._deferred_switch_message is None
    assert controller._deferred_switch_deadline is None
    assert not controller._map_switch_request_active
    assert controller._deferred_switch_generation == 12
    assert published == []


def test_cancel_before_trigger_blocks_late_trigger_for_same_transaction():
    controller = bare_controller()
    message = switch_message('tx-cancel-first')
    executions = []
    controller._execute_map_switch_transaction = (
        lambda *_args, **_kwargs: executions.append(True) or True)

    controller.map_switch_cancel_callback(
        cancel_message(message.transaction_id, 'navigation_complete'))
    controller.map_switch_callback(message)

    assert executions == []
    assert controller._closed_transactions == {message.transaction_id}
    assert not controller._map_switch_request_active
    assert controller._active_transaction_id == ''


def test_pending_trigger_runs_after_active_transaction_is_cancelled():
    controller = bare_controller()
    active_id = 'tx-active'
    pending = switch_message('tx-pending')
    controller._map_switch_request_active = True
    controller._active_transaction_id = active_id
    controller._deferred_switch_generation = 3
    executions = []
    published = []
    controller._execute_map_switch_transaction = (
        lambda current_map, next_map, _pose, **_kwargs:
        executions.append((current_map, next_map)) or True)
    controller._publish_map_switch_completion = (
        lambda success, transaction_id: published.append(
            (transaction_id, success)))

    controller.map_switch_callback(pending)

    assert executions == []
    assert list(controller._pending_switch_triggers) == [
        pending.transaction_id]

    controller.map_switch_cancel_callback(cancel_message(active_id))

    assert executions == [('map1', 'map2')]
    assert published == [(pending.transaction_id, True)]
    assert controller._pending_switch_triggers == OrderedDict()
    assert controller._closed_transactions == {
        active_id, pending.transaction_id}
    assert not controller._map_switch_request_active


def test_cancelled_pending_trigger_does_not_run_after_active_cancel():
    controller = bare_controller()
    active_id = 'tx-active'
    pending = switch_message('tx-pending-cancelled')
    controller._map_switch_request_active = True
    controller._active_transaction_id = active_id
    controller._deferred_switch_generation = 3
    executions = []
    controller._execute_map_switch_transaction = (
        lambda *_args, **_kwargs: executions.append(True) or True)

    controller.map_switch_callback(pending)
    controller.map_switch_cancel_callback(
        cancel_message(pending.transaction_id))
    controller.map_switch_cancel_callback(cancel_message(active_id))

    assert executions == []
    assert controller._pending_switch_triggers == OrderedDict()
    assert controller._closed_transactions == {
        active_id, pending.transaction_id}
    assert not controller._map_switch_request_active


def test_pending_triggers_are_dispatched_in_fifo_order_after_cancel():
    controller = bare_controller()
    active_id = 'tx-active'
    second = switch_message('tx-b', 'map1', 'map2')
    third = switch_message('tx-c', 'map2', 'map3')
    controller._map_switch_request_active = True
    controller._active_transaction_id = active_id
    controller._deferred_switch_generation = 3
    executions = []
    published = []
    controller._execute_map_switch_transaction = (
        lambda current_map, next_map, _pose, **_kwargs:
        executions.append((current_map, next_map)) or True)
    controller._publish_map_switch_completion = (
        lambda success, transaction_id: published.append(
            (transaction_id, success)))

    controller.map_switch_callback(second)
    controller.map_switch_callback(third)

    assert list(controller._pending_switch_triggers) == ['tx-b', 'tx-c']
    assert executions == []

    controller.map_switch_cancel_callback(cancel_message(active_id))

    assert executions == [('map1', 'map2'), ('map2', 'map3')]
    assert published == [('tx-b', True), ('tx-c', True)]
    assert controller._pending_switch_triggers == OrderedDict()
    assert controller._closed_transactions == {
        active_id, 'tx-b', 'tx-c'}
    assert not controller._map_switch_request_active


def test_cancel_during_execution_suppresses_late_transaction_result():
    controller = bare_controller()
    message = switch_message('tx-cancel-during-execution')
    published = []

    def execute(*_args, **_kwargs):
        controller.map_switch_cancel_callback(
            cancel_message(message.transaction_id))
        return False

    controller._execute_map_switch_transaction = execute
    controller._publish_map_switch_completion = (
        lambda success, transaction_id: published.append(
            (transaction_id, success)))

    controller.map_switch_callback(message)

    assert published == []
    assert controller._closed_transactions == {message.transaction_id}
    assert not controller._map_switch_request_active
    assert controller._active_transaction_id == ''


def test_first_map_switch_execution_uses_total_deadline_and_reports_timeout(
        monkeypatch):
    controller = bare_controller()
    message = switch_message('tx-first-timeout')
    controller.map_switch_timeout = 1.0
    controller._stack_transaction_lock = OwnedTransaction()
    now = [20.0]
    shutdown_deadlines = []
    published = []

    def expire_during_shutdown(**kwargs):
        shutdown_deadlines.append(kwargs['deadline_monotonic'])
        now[0] = kwargs['deadline_monotonic']
        return None

    controller.shutdown_current_stack = expire_during_shutdown
    controller.wait_for_clean_shutdown = lambda *_args, **_kwargs: pytest.fail(
        'deadline expiry must stop before clean-shutdown polling')
    controller.launch_new_stack = lambda *_args, **_kwargs: pytest.fail(
        'deadline expiry must stop before starting the replacement stack')
    controller._rollback_processes = lambda *_args, **_kwargs: pytest.fail(
        'deadline expiry is an ambiguous boundary and must not roll back')
    controller._publish_map_switch_completion = (
        lambda success, transaction_id: published.append(
            (transaction_id, success)))
    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.time.monotonic',
        lambda: now[0],
    )

    controller.map_switch_callback(message)

    assert shutdown_deadlines == [21.0]
    assert published == [(message.transaction_id, False)]
    assert controller._closed_transactions == {message.transaction_id}
    assert not controller._map_switch_request_active
    assert controller._active_transaction_id == ''


class OwnedTransaction:
    def __init__(self):
        self.releases = 0

    def acquire(self):
        return True

    def release(self):
        self.releases += 1


class BlockingFuture:
    def __init__(self):
        self.polled = threading.Event()
        self.cancelled = False

    def done(self):
        self.polled.set()
        return False

    def cancel(self):
        self.cancelled = True


class SingleFutureClient:
    def __init__(self, future):
        self.future = future
        self.requests = []

    def call_async(self, request):
        self.requests.append(request)
        return self.future


def configure_active_deferred_switch(controller, message, generation):
    controller._map_switch_request_active = True
    controller._active_transaction_id = message.transaction_id
    controller._deferred_switch_generation = generation
    controller._deferred_switch_deadline = time.monotonic() + 5.0
    controller._stack_transaction_lock = OwnedTransaction()


def run_resume_in_thread(controller, message, generation):
    outcome = {}

    def run():
        try:
            outcome['result'] = controller._resume_deferred_map_switch_start(
                message, generation)
        except Exception as exc:  # pragma: no cover - asserted below
            outcome['exception'] = exc

    worker = threading.Thread(target=run)
    worker.start()
    return worker, outcome


def test_cancel_interrupts_deferred_resume_service_wait_without_rollback():
    controller = bare_controller()
    message = switch_message('tx-service-wait')
    generation = 4
    configure_active_deferred_switch(controller, message, generation)
    future = BlockingFuture()
    controller.start_process_client = SingleFutureClient(future)
    rollbacks = []
    controller._rollback_processes = lambda names, reason: rollbacks.append(
        (tuple(names), reason))
    controller.verify_stack_ready = lambda: pytest.fail(
        'cancelled resume must not reach stack verification')

    worker, outcome = run_resume_in_thread(
        controller, message, generation)
    assert future.polled.wait(1.0)

    controller.map_switch_cancel_callback(
        cancel_message(message.transaction_id))
    worker.join(2.0)

    assert not worker.is_alive()
    assert 'exception' not in outcome
    assert outcome['result'] is None
    assert future.cancelled
    assert [request.process_name
            for request in controller.start_process_client.requests] == [
                're_localization']
    assert rollbacks == []
    assert controller.current_map == 'map1'


def test_cancel_interrupts_deferred_resume_tf_wait_without_rollback(
        monkeypatch):
    controller = bare_controller()
    message = switch_message('tx-tf-wait')
    generation = 6
    configure_active_deferred_switch(controller, message, generation)
    controller.start_process_client = QueueClient([response(), response()])
    entered_tf_wait = threading.Event()
    controller._localization_result_ready = lambda: (
        entered_tf_wait.set() or None)
    rollbacks = []
    controller._rollback_processes = lambda names, reason: rollbacks.append(
        (tuple(names), reason))
    controller.verify_stack_ready = lambda: pytest.fail(
        'cancelled resume must not reach stack verification')
    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.tf2_ros.Buffer',
        lambda: object(),
    )
    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.tf2_ros.TransformListener',
        lambda *_args, **_kwargs: object(),
    )

    worker, outcome = run_resume_in_thread(
        controller, message, generation)
    assert entered_tf_wait.wait(1.0)

    controller.map_switch_cancel_callback(
        cancel_message(message.transaction_id))
    worker.join(2.0)

    assert not worker.is_alive()
    assert 'exception' not in outcome
    assert outcome['result'] is None
    assert [request.process_name
            for request in controller.start_process_client.requests] == [
                're_localization', 'nav2_init_pose']
    assert rollbacks == []
    assert controller.current_map == 'map1'


def test_completed_future_at_deadline_cannot_start_next_process(monkeypatch):
    controller = bare_controller()
    message = switch_message('tx-deadline')
    generation = 9
    now = [9.0]
    controller._map_switch_request_active = True
    controller._active_transaction_id = message.transaction_id
    controller._deferred_switch_generation = generation
    controller._deferred_switch_deadline = 10.0

    class DeadlineClient:
        def __init__(self):
            self.requests = []

        def call_async(self, request):
            self.requests.append(request)
            now[0] = 10.0
            return CompletedFuture(response())

    controller.start_process_client = DeadlineClient()
    controller._rollback_processes = lambda *_args, **_kwargs: pytest.fail(
        'deadline expiry must not roll back an ambiguous completed future')
    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.time.monotonic',
        lambda: now[0],
    )

    result = controller.launch_new_stack(
        'map2',
        resume=True,
        generation=generation,
        deadline_monotonic=10.0,
    )

    assert result is None
    assert [request.process_name
            for request in controller.start_process_client.requests] == [
                're_localization']


@pytest.mark.parametrize('deadline_stage', ['launch', 'verify'])
def test_deferred_worker_finalizes_timeout_crossed_during_resume(
        monkeypatch, deadline_stage):
    controller = bare_controller()
    message = switch_message(f'tx-deadline-{deadline_stage}')
    generation = 12
    now = [9.0]
    controller._map_switch_request_active = True
    controller._active_transaction_id = message.transaction_id
    controller._deferred_switch_generation = generation
    controller._deferred_switch_deadline = 10.0
    controller._stack_transaction_lock = OwnedTransaction()
    timers = []

    class ManualTimer:
        def __init__(self, _delay, callback):
            self.callback = callback
            self.daemon = False

        def is_alive(self):
            return False

        def start(self):
            timers.append(self)

    def launch(_message, _generation):
        if deadline_stage == 'launch':
            now[0] = 10.0
        return True

    verify_calls = []

    def verify(**_kwargs):
        verify_calls.append(True)
        if deadline_stage == 'verify':
            now[0] = 10.0
        return True

    published = []
    controller._launch_resumed_stack = launch
    controller.verify_stack_ready = verify
    controller._publish_map_switch_completion = (
        lambda success, transaction_id: published.append(
            (transaction_id, success)))
    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.time.monotonic',
        lambda: now[0],
    )
    monkeypatch.setattr(
        'multi_map_navigation.map_switch_controller.threading.Timer',
        ManualTimer,
    )

    controller._schedule_deferred_map_switch(
        message, resume_start=True, generation=generation)
    assert len(timers) == 1
    timers[0].callback()

    assert verify_calls == ([] if deadline_stage == 'launch' else [True])
    assert published == [(message.transaction_id, False)]
    assert not controller._map_switch_request_active
    assert controller._active_transaction_id == ''
    assert controller._deferred_switch_generation == generation + 1
    assert controller._closed_transactions == {message.transaction_id}
    assert controller.current_map == 'map1'
