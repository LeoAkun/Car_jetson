"""
Offline regressions for session-bound relocalization startup readiness.

No test initializes rclpy, creates a ROS node, spawns a process, or contacts DDS.
The lifecycle/readiness responses and elapsed time are entirely in memory.
"""

import threading
from types import SimpleNamespace

import pytest
from lifecycle_msgs.msg import State, Transition

from multi_map_navigation import process_manager as process_manager_module
from multi_map_navigation.process_manager import ProcessManagerNode


SESSION = 'readiness-current-session'
MAP = 'map2'
SESSION_START = 1_795_248_900_000_000_000
GENERATION = 17


class FakeLogger:
    def __init__(self):
        self.messages = []

    def _record(self, message):
        self.messages.append(message)

    debug = _record
    info = _record
    warning = _record
    error = _record


class FakeProcess:
    pid = 987654

    def __init__(self):
        self.returncode = None

    def poll(self):
        return self.returncode


class VirtualTime:
    def __init__(self):
        self.now = 100.0
        self.sleep_calls = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        seconds = max(0.001, float(seconds))
        self.sleep_calls.append(seconds)
        self.now += seconds


class FakeFuture:
    def __init__(self, response=None, *, pending=False, error=None):
        self.response = response
        self.pending = pending
        self.error = error
        self.cancelled = False

    def done(self):
        return not self.pending or self.cancelled

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response

    def cancel(self):
        self.cancelled = True
        return True


class FakeClient:
    def __init__(self, handler, *, available=True):
        self.handler = handler
        self.available = available
        self.requests = []
        self.futures = []

    def service_is_ready(self):
        return self.available

    def wait_for_service(self, timeout_sec=None):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        value = self.handler(request)
        future = value if isinstance(value, FakeFuture) else FakeFuture(value)
        self.futures.append(future)
        return future


class FakeRelocalization:
    """Small deterministic server double with independent map readiness."""

    def __init__(self, state=State.PRIMARY_STATE_ACTIVE):
        self.state = state
        self.session = SESSION
        self.map_name = MAP
        self.session_start = SESSION_START
        self.target_ready = True
        self.change_success = True
        self.transitions = []
        self.readiness_calls = 0
        self.state_calls = 0
        self.on_readiness = None
        self.on_state = None
        self.on_change = None
        self.readiness_client = FakeClient(self.readiness)
        self.state_client = FakeClient(self.get_state)
        self.change_client = FakeClient(self.change_state)

    def readiness(self, _request):
        self.readiness_calls += 1
        response = SimpleNamespace(
            ready=(self.state == State.PRIMARY_STATE_ACTIVE and self.target_ready),
            session_id=self.session,
            map_id=self.map_name,
            session_start_ns=self.session_start,
            message='map and target ready' if self.target_ready else 'empty target',
        )
        if self.on_readiness is not None:
            self.on_readiness(self, response)
        return response

    def get_state(self, _request):
        self.state_calls += 1
        if self.on_state is not None:
            self.on_state(self)
        return SimpleNamespace(
            current_state=SimpleNamespace(id=self.state, label=str(self.state)))

    def change_state(self, request):
        transition = int(request.transition.id)
        self.transitions.append(transition)
        if self.on_change is not None:
            return self.on_change(self, transition)
        if self.change_success:
            if transition == Transition.TRANSITION_CONFIGURE:
                self.state = State.PRIMARY_STATE_INACTIVE
            elif transition == Transition.TRANSITION_ACTIVATE:
                self.state = State.PRIMARY_STATE_ACTIVE
            else:
                raise AssertionError(f'unexpected lifecycle transition {transition}')
        return SimpleNamespace(success=self.change_success)


@pytest.fixture
def readiness_case(monkeypatch):
    clock = VirtualTime()
    monkeypatch.setattr(process_manager_module.time, 'monotonic', clock.monotonic)
    monkeypatch.setattr(process_manager_module.time, 'sleep', clock.sleep)
    manager = ProcessManagerNode.__new__(ProcessManagerNode)
    manager._ensure_process_tracking()
    process = FakeProcess()
    manager.processes['re_localization'] = process
    manager.process_pgids['re_localization'] = process.pid
    manager.process_maps['re_localization'] = MAP
    manager.process_generations['re_localization'] = GENERATION
    manager.process_cancel_events['re_localization'] = threading.Event()
    manager.process_states['re_localization'] = 'STARTING'
    manager.health_failures['re_localization'] = 0
    manager.active_localization_session = SESSION
    manager.active_map = MAP
    manager.localization_session_start_ns = SESSION_START
    manager.max_startup_time = 0.5
    manager.lifecycle_query_timeout = 0.1
    manager.startup_delay = 0.0
    manager.logger = FakeLogger()
    manager.get_logger = lambda: manager.logger
    server = FakeRelocalization()
    manager.relocalization_readiness_client = server.readiness_client
    manager.relocalization_state_client = server.state_client
    manager.relocalization_change_client = server.change_client

    def wait_poll(name, generation, seconds):
        clock.sleep(seconds)
        return not manager._start_was_cancelled(name, generation)

    manager._wait_for_start_poll = wait_poll
    return SimpleNamespace(
        manager=manager, process=process, server=server, clock=clock)


def wait_ready(case):
    return case.manager._wait_for_relocalization_ready(
        case.process, GENERATION, SESSION, MAP, SESSION_START)


def assert_failed_without_transition(case):
    ready, reason = wait_ready(case)
    assert not ready
    assert reason
    assert case.server.transitions == []
    assert case.manager.process_states['re_localization'] != 'HEALTHY'


def test_active_current_session_with_ready_map_needs_no_transition(readiness_case):
    ready, reason = wait_ready(readiness_case)

    assert ready, reason
    assert readiness_case.server.readiness_calls >= 1
    assert readiness_case.server.transitions == []


def test_configure_then_activate_are_owned_by_current_start(readiness_case):
    readiness_case.server.state = State.PRIMARY_STATE_UNCONFIGURED

    ready, reason = wait_ready(readiness_case)

    assert ready, reason
    assert readiness_case.server.transitions == [
        Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE]


def test_inactive_current_server_is_activated_once(readiness_case):
    readiness_case.server.state = State.PRIMARY_STATE_INACTIVE

    ready, reason = wait_ready(readiness_case)

    assert ready, reason
    assert readiness_case.server.transitions == [Transition.TRANSITION_ACTIVATE]


def test_in_progress_activation_is_polled_without_duplicate_request(readiness_case):
    server = readiness_case.server
    server.state = State.TRANSITION_STATE_ACTIVATING

    def finish_activation(value):
        if value.state_calls >= 2:
            value.state = State.PRIMARY_STATE_ACTIVE

    server.on_state = finish_activation

    ready, reason = wait_ready(readiness_case)

    assert ready, reason
    assert server.transitions == []


def test_active_lifecycle_does_not_override_empty_registration_target(readiness_case):
    readiness_case.server.target_ready = False

    assert_failed_without_transition(readiness_case)


@pytest.mark.parametrize('field,value', (
    ('session', 'previous-session'),
    ('session', ''),
    ('map_name', 'map1'),
    ('session_start', SESSION_START - 1),
    ('session_start', 0),
))
def test_ready_response_must_identify_current_server(readiness_case, field, value):
    setattr(readiness_case.server, field, value)

    assert_failed_without_transition(readiness_case)


@pytest.mark.parametrize('state', (
    State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE,
))
def test_lifecycle_is_never_advanced_before_server_identity_is_proven(
        readiness_case, state):
    readiness_case.server.state = state
    readiness_case.server.session = 'old-server-on-shared-service-name'

    assert_failed_without_transition(readiness_case)


def test_missing_readiness_service_cannot_be_replaced_by_active_state(readiness_case):
    readiness_case.server.readiness_client.available = False

    assert_failed_without_transition(readiness_case)


def test_missing_state_service_cannot_be_replaced_by_ready_response(readiness_case):
    readiness_case.server.state_client.available = False

    assert_failed_without_transition(readiness_case)


def test_ready_response_does_not_override_actual_inactive_state(readiness_case):
    case = readiness_case
    case.server.state = State.PRIMARY_STATE_INACTIVE
    # Model a map-ready reply followed by a lifecycle deactivation.  Both
    # independent conditions must hold before the startup gate can return true.
    case.server.on_readiness = lambda _server, response: setattr(response, 'ready', True)

    ready, reason = wait_ready(case)

    assert ready, reason
    assert case.server.state == State.PRIMARY_STATE_ACTIVE
    assert case.server.transitions == [Transition.TRANSITION_ACTIVATE]


def test_delayed_current_identity_is_polled_without_touching_old_server(readiness_case):
    case = readiness_case
    case.server.state = State.PRIMARY_STATE_INACTIVE

    def stale_first_reply(server, response):
        if server.readiness_calls == 1:
            response.session_id = 'old-service-reply'

    def assert_identity_was_rechecked(server, transition):
        assert server.readiness_calls >= 2
        assert transition == Transition.TRANSITION_ACTIVATE
        server.state = State.PRIMARY_STATE_ACTIVE
        return SimpleNamespace(success=True)

    case.server.on_readiness = stale_first_reply
    case.server.on_change = assert_identity_was_rechecked

    ready, reason = wait_ready(case)

    assert ready, reason
    assert case.server.transitions == [Transition.TRANSITION_ACTIVATE]


def test_cancelled_start_cannot_query_or_activate_server(readiness_case):
    case = readiness_case
    case.server.state = State.PRIMARY_STATE_INACTIVE
    case.manager.process_cancel_events['re_localization'].set()

    assert_failed_without_transition(case)
    assert case.server.readiness_calls == 0
    assert case.server.state_calls == 0


def test_replaced_generation_cannot_activate_previous_server(readiness_case):
    case = readiness_case
    case.server.state = State.PRIMARY_STATE_INACTIVE
    replacement = FakeProcess()
    case.manager.processes['re_localization'] = replacement
    case.manager.process_generations['re_localization'] = GENERATION + 1
    case.manager.process_cancel_events['re_localization'] = threading.Event()

    assert_failed_without_transition(case)
    assert case.manager.processes['re_localization'] is replacement
    assert not case.manager.process_cancel_events['re_localization'].is_set()


def test_exited_parent_cannot_be_reported_ready(readiness_case):
    readiness_case.process.returncode = 2

    assert_failed_without_transition(readiness_case)


def test_cancel_arriving_with_positive_readiness_response_wins(readiness_case):
    case = readiness_case
    case.server.on_readiness = lambda _server, _response: (
        case.manager.process_cancel_events['re_localization'].set())

    assert_failed_without_transition(case)


def test_parent_exit_arriving_with_positive_readiness_response_wins(readiness_case):
    case = readiness_case
    case.server.on_readiness = lambda _server, _response: setattr(
        case.process, 'returncode', 2)

    assert_failed_without_transition(case)


@pytest.mark.parametrize('state,transition', (
    (State.PRIMARY_STATE_UNCONFIGURED, Transition.TRANSITION_CONFIGURE),
    (State.PRIMARY_STATE_INACTIVE, Transition.TRANSITION_ACTIVATE),
))
def test_failed_lifecycle_transition_has_bounded_retry_budget(
        readiness_case, state, transition):
    case = readiness_case
    case.server.state = state
    case.server.change_success = False

    ready, reason = wait_ready(case)

    assert not ready
    assert reason
    assert 1 <= len(case.server.transitions) <= 2
    assert set(case.server.transitions) == {transition}


def test_lost_activation_reply_is_resolved_by_state_not_duplicate_transition(
        readiness_case):
    case = readiness_case
    case.server.state = State.PRIMARY_STATE_INACTIVE
    case.manager.max_startup_time = 20.0
    pending = FakeFuture(pending=True)

    def activate_without_reply(server, transition):
        assert transition == Transition.TRANSITION_ACTIVATE
        server.state = State.PRIMARY_STATE_ACTIVE
        return pending

    case.server.on_change = activate_without_reply

    ready, reason = wait_ready(case)

    assert ready, reason
    assert pending.cancelled
    assert case.server.transitions == [Transition.TRANSITION_ACTIVATE]


def test_failed_reply_does_not_override_observed_active_state(readiness_case):
    case = readiness_case
    case.server.state = State.PRIMARY_STATE_INACTIVE

    def activate_with_failure_reply(server, transition):
        assert transition == Transition.TRANSITION_ACTIVATE
        server.state = State.PRIMARY_STATE_ACTIVE
        return SimpleNamespace(success=False)

    case.server.on_change = activate_with_failure_reply

    ready, reason = wait_ready(case)

    assert ready, reason
    assert case.server.transitions == [Transition.TRANSITION_ACTIVATE]


def test_unknown_lifecycle_state_is_not_blindly_activated(readiness_case):
    readiness_case.server.state = State.PRIMARY_STATE_UNKNOWN

    assert_failed_without_transition(readiness_case)


def test_finalized_lifecycle_fails_without_waiting_full_startup_timeout(readiness_case):
    case = readiness_case
    case.server.state = State.PRIMARY_STATE_FINALIZED
    started = case.clock.monotonic()

    assert_failed_without_transition(case)
    assert case.clock.monotonic() - started < case.manager.max_startup_time


def test_helper_cancels_hung_future_at_its_deadline(readiness_case):
    case = readiness_case
    future = FakeFuture(pending=True)
    client = FakeClient(lambda _request: future)
    deadline = case.clock.monotonic() + 0.1

    result = case.manager._call_relocalization_start_service(
        client, SimpleNamespace(), deadline, lambda: False)

    assert result is None
    assert future.cancelled
    assert case.clock.monotonic() >= deadline


def test_helper_cancels_hung_future_when_start_is_cancelled(readiness_case):
    case = readiness_case
    future = FakeFuture(pending=True)
    client = FakeClient(lambda _request: future)
    deadline = case.clock.monotonic() + 1.0
    cancel_at = case.clock.monotonic() + 0.05

    result = case.manager._call_relocalization_start_service(
        client, SimpleNamespace(), deadline,
        lambda: case.clock.monotonic() >= cancel_at)

    assert result is None
    assert future.cancelled
    assert case.clock.monotonic() < deadline


def test_helper_handles_service_result_exception_without_raising(readiness_case):
    case = readiness_case
    client = FakeClient(lambda _request: FakeFuture(error=RuntimeError('DDS reply failed')))

    result = case.manager._call_relocalization_start_service(
        client, SimpleNamespace(), case.clock.monotonic() + 0.1, lambda: False)

    assert result is None


def test_already_cancelled_helper_never_sends_request(readiness_case):
    case = readiness_case
    client = FakeClient(lambda _request: SimpleNamespace(ready=True))

    result = case.manager._call_relocalization_start_service(
        client, SimpleNamespace(), case.clock.monotonic() + 0.1, lambda: True)

    assert result is None
    assert client.requests == []


def prepare_fake_launch(case, monkeypatch):
    """Exercise launch orchestration without Popen, signals, or ROS publishers."""
    manager = case.manager
    manager.map_dir = '/offline/maps'
    manager.processes.clear()
    manager.process_generations.clear()
    manager.process_cancel_events.clear()
    manager.process_states.clear()
    manager.commands = []
    manager.rollback_calls = []
    manager.session_publications = []
    monkeypatch.setattr(process_manager_module.uuid, 'uuid4', lambda: SESSION)
    manager._localization_session_now_ns = lambda: SESSION_START
    manager._publish_localization_session = lambda active: (
        manager.session_publications.append(active))

    def spawn(name, map_name, command, **_kwargs):
        assert name == 're_localization'
        manager.commands.append(command)
        manager.processes[name] = case.process
        manager.process_generations[name] = GENERATION
        manager.process_cancel_events[name] = threading.Event()
        manager.process_states[name] = 'STARTING'
        manager.process_maps[name] = map_name
        return case.process, GENERATION

    manager._spawn_tracked_process = spawn
    manager._rollback_failed_launch = lambda *args, **kwargs: (
        manager.rollback_calls.append((args, kwargs)))
    return manager


@pytest.mark.parametrize('map_ready', (False, True))
def test_launch_success_is_committed_only_after_readiness_gate(
        readiness_case, monkeypatch, map_ready):
    case = readiness_case
    manager = prepare_fake_launch(case, monkeypatch)
    gate_calls = []

    def gate(*identity):
        assert manager.process_states['re_localization'] == 'STARTING'
        gate_calls.append(identity)
        return map_ready, '' if map_ready else 'registration target is empty'

    manager._wait_for_relocalization_ready = gate

    assert manager.launch_relocalization(MAP) is map_ready
    assert gate_calls == [(case.process, GENERATION, SESSION, MAP, SESSION_START)]
    assert 'autostart:=false' in manager.commands[0]
    assert manager.session_publications == [True]
    if map_ready:
        assert manager.process_states['re_localization'] == 'HEALTHY'
        assert manager.rollback_calls == []
    else:
        assert manager.process_states['re_localization'] != 'HEALTHY'
        assert len(manager.rollback_calls) == 1
        args, _kwargs = manager.rollback_calls[0]
        assert args[0] == 're_localization'
        assert 'registration target is empty' in args[1]
        assert args[2:] == (case.process, GENERATION)


def test_starting_instance_is_busy_not_idempotent_success(readiness_case):
    case = readiness_case
    response = SimpleNamespace()

    assert case.manager._populate_existing_start_response(
        're_localization', MAP, response)

    assert not response.success
    assert response.busy
    assert case.manager.process_states['re_localization'] == 'STARTING'


def test_cancel_between_readiness_and_healthy_commit_wins(readiness_case, monkeypatch):
    case = readiness_case
    manager = prepare_fake_launch(case, monkeypatch)

    def gate(*_identity):
        manager.process_cancel_events['re_localization'].set()
        return True, ''

    manager._wait_for_relocalization_ready = gate

    assert not manager.launch_relocalization(MAP)
    assert manager.process_states['re_localization'] != 'HEALTHY'


def test_replacement_between_readiness_and_commit_is_preserved(
        readiness_case, monkeypatch):
    case = readiness_case
    manager = prepare_fake_launch(case, monkeypatch)
    replacement = FakeProcess()
    replacement_cancel = threading.Event()

    def gate(*_identity):
        manager.processes['re_localization'] = replacement
        manager.process_generations['re_localization'] = GENERATION + 1
        manager.process_cancel_events['re_localization'] = replacement_cancel
        manager.process_states['re_localization'] = 'STARTING'
        return True, ''

    manager._wait_for_relocalization_ready = gate

    assert not manager.launch_relocalization(MAP)
    assert manager.processes['re_localization'] is replacement
    assert manager.process_generations['re_localization'] == GENERATION + 1
    assert manager.process_states['re_localization'] == 'STARTING'
    assert not replacement_cancel.is_set()
    assert manager.rollback_calls == []
