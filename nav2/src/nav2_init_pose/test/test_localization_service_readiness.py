"""Pure mocks: unavailable localization must not run an all-inf PSO search."""

import math
from types import SimpleNamespace

import pytest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from nav2_init_pose import nav2_init_pose as implementation


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def advance(self, seconds):
        self.now += seconds


class FakeFuture:
    def __init__(self, response, complete=True):
        self.response = response
        self.complete = complete
        self.cancel_called = False

    def done(self):
        return self.complete

    def result(self):
        return self.response

    def cancel(self):
        self.cancel_called = True


class FakeClient:
    def __init__(self, clock, response):
        self.clock = clock
        self.future = FakeFuture(response)
        self.available = True
        self.discovery_cost = 0.0
        self.request_cost = 0.0
        self.requests = []
        self.discovery_timeouts = []

    def wait_for_service(self, *, timeout_sec):
        self.discovery_timeouts.append(timeout_sec)
        self.clock.advance(self.discovery_cost if self.available else timeout_sec)
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        self.clock.advance(self.request_cost)
        return self.future


def response(message, *, success=False):
    return SimpleNamespace(
        success=success,
        message=message,
        cloud_stamp=Time(sec=1, nanosec=100),
        observation_id=1,
        session_id='test-session',
        map_id='map2',
        fitness_score=0.01,
        pose=PoseWithCovarianceStamped(),
    )


@pytest.fixture
def search(monkeypatch):
    clock = FakeClock()
    spin_timeouts = []
    spin_once_timeouts = []
    monkeypatch.setattr(
        implementation, 'time', SimpleNamespace(monotonic=lambda: clock.now))

    def spin_until_future_complete(_node, _future, *, timeout_sec):
        spin_timeouts.append(timeout_sec)

    def spin_once(_node, *, timeout_sec):
        spin_once_timeouts.append(timeout_sec)
        clock.advance(timeout_sec)

    # No context, executor, node or DDS participant is constructed in these tests.
    monkeypatch.setattr(implementation, 'rclpy', SimpleNamespace(
        spin_until_future_complete=spin_until_future_complete,
        spin_once=spin_once,
    ))
    node = implementation.PoseInitNode.__new__(implementation.PoseInitNode)
    node.max_localization_duration = 120.0
    node.localization_deadline = None
    node.localization_session_id = 'test-session'
    node.map_name = 'map2'
    node.session_start_ns = 1_000_000_000
    node.observation_id = 1
    node.observation_has_cloud = False
    node._observation_min_cloud_stamp = None
    node.localization_cloud_stamp = None
    node.use_initial_pose = False
    node.initial_x = node.initial_y = node.initial_yaw = 0.0
    node.latest_gps = SimpleNamespace(latitude=0.0, longitude=0.0)
    node.converter = SimpleNamespace(gps_to_map=lambda _lat, _lon: (0.0, 0.0))
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(
            nanoseconds=2_000_000_000, to_msg=lambda: Time(sec=2)))
    node.pso_particles = 30
    node.pso_search_radius = 10.0
    node.pso_w = 0.7
    node.pso_c1 = node.pso_c2 = 1.5
    node.pso_angle_step = 30
    node.pso_iterations = 6
    node.direct_accept_threshold = 0.025
    node.fitness_score_threshold = 0.05
    node.max_cloud_age = 120.0
    node._static_footprint_is_legal = lambda _pose: (True, 'free')
    failures = []
    messages = []
    logger = SimpleNamespace(
        info=messages.append, error=messages.append, warning=messages.append)
    node.get_logger = lambda: logger
    node.publish_localization_result = (
        lambda success, score, pose=None, reason='':
        failures.append((success, score, reason)))
    client = FakeClient(clock, response('registration did not converge'))
    node.re_localization_client = client
    return SimpleNamespace(
        node=node, client=client, clock=clock, failures=failures,
        spin_timeouts=spin_timeouts, spin_once_timeouts=spin_once_timeouts,
    )


@pytest.mark.parametrize('message', (
    'map or initial pose is not ready',
    'localization lifecycle is not active',
    'registration target is not ready',
))
@pytest.mark.parametrize('seeded', (False, True))
def test_explicit_not_ready_aborts_after_one_candidate(search, message, seeded):
    search.node.use_initial_pose = seeded
    search.client.future = FakeFuture(response(message))

    assert not search.node.execute_logic()

    assert len(search.client.requests) == 1
    assert len(search.failures) == 1
    success, score, reason = search.failures[0]
    assert success is False
    assert math.isinf(score)
    assert reason == 'localization service not ready: ' + message


def test_not_ready_during_confirmation_preserves_abort_reason(search):
    first_pose = Pose()
    first_pose.orientation.w = 1.0

    def first_candidate(_x, _y):
        search.node.localization_cloud_stamp = Time(sec=1, nanosec=100)
        return 0.03, 0.0, 0.0, 0.0, first_pose

    search.node.pso_search = first_candidate
    search.client.future = FakeFuture(
        response('localization lifecycle is not active'))

    assert not search.node.execute_logic()

    assert search.node.observation_id == 2
    assert len(search.client.requests) == 1
    assert len(search.failures) == 1
    assert search.failures[0][2] == (
        'localization service not ready: localization lifecycle is not active')


@pytest.mark.parametrize('message', (
    'registration did not converge',
    'no fresh point cloud received within timeout',
    'some other candidate rejection',
))
def test_candidate_failures_still_allow_next_candidate(search, message):
    search.node.localization_deadline = 120.0
    search.client.future = FakeFuture(response(message))
    score, pose, stamp = search.node._evaluate_candidate(0.0, 0.0, 0.0)
    assert math.isinf(score)
    assert pose is stamp is None

    search.client.future = FakeFuture(response('evaluated', success=True))
    score, pose, stamp = search.node._evaluate_candidate(0.0, 0.0, 30.0)

    assert score == 0.01
    assert pose is not None
    assert stamp.nanosec == 100
    assert len(search.client.requests) == 2
    assert not search.failures


def test_service_discovery_uses_one_total_deadline(search):
    search.client.available = False

    assert not search.node.execute_logic()

    assert not search.client.requests
    assert len(search.failures) == 1
    assert 'waiting for /re_localization service' in search.failures[0][2]
    assert search.clock.now == pytest.approx(120.0)
    assert all(0.0 < value <= 1.0 for value in search.client.discovery_timeouts)
    assert all(0.0 < value <= 0.1 for value in search.spin_once_timeouts)


def test_unanswered_request_aborts_and_cancels_local_future(search, monkeypatch):
    search.client.future.complete = False

    def timeout(_node, _future, *, timeout_sec):
        search.spin_timeouts.append(timeout_sec)
        search.clock.advance(timeout_sec)

    monkeypatch.setattr(implementation.rclpy, 'spin_until_future_complete', timeout)

    assert not search.node.execute_logic()

    assert len(search.client.requests) == 1
    assert search.spin_timeouts == [120.0]
    assert search.clock.now == 120.0
    assert search.client.future.cancel_called
    assert len(search.failures) == 1
    assert 'waiting for /re_localization response' in search.failures[0][2]


def test_response_timeout_uses_remaining_budget_after_discovery(search):
    search.client.discovery_cost = 0.25
    search.client.request_cost = 0.10
    search.client.future = FakeFuture(response('registration target is not ready'))

    assert not search.node.execute_logic()

    assert search.spin_timeouts == pytest.approx([119.65])
    assert 'registration target is not ready' in search.failures[0][2]


def test_late_completed_response_cannot_bypass_deadline(search, monkeypatch):
    def late_response(_node, _future, *, timeout_sec):
        search.clock.advance(timeout_sec + 0.01)

    monkeypatch.setattr(
        implementation.rclpy, 'spin_until_future_complete', late_response)
    search.client.future = FakeFuture(response('evaluated', success=True))

    assert not search.node.execute_logic()

    assert len(search.client.requests) == 1
    assert len(search.failures) == 1
    assert 'deadline' in search.failures[0][2]


def test_deadline_expiring_during_request_submission_cancels_future(search):
    search.client.request_cost = 120.01

    assert not search.node.execute_logic()

    assert len(search.client.requests) == 1
    assert not search.spin_timeouts
    assert search.client.future.cancel_called
    assert 'waiting for /re_localization response' in search.failures[0][2]
