import threading

from geometry_msgs.msg import Pose
from multi_map_navigation.map_odom_authority import MapOdomAuthority
from multi_map_navigation_msgs.msg import (
    LioMapOdomCorrection,
    LocalizationResult,
    LocalizationSession,
)


class FakeLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass

    def debug(self, _message):
        pass


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_authority():
    node = MapOdomAuthority.__new__(MapOdomAuthority)
    node._lock = threading.RLock()
    node._active_session_id = ''
    node._active_map_id = ''
    node._session_start_ns = 0
    node._accepted_pose = None
    node._current_map_odom = None
    node._pending_result = None
    node._map_lio_transform = None
    node._target_map_odom = None
    node._active_lio_identity = None
    node._last_correction_epoch = 0
    node._rejected_correction_epochs = set()
    node._dynamic_fault_latched = False
    node._last_update_ns = 0
    node._footprint_checkers = {}
    node._latest_odom_base = None
    node._map_directory = '/tmp'
    node._fault_pub = FakePublisher()
    node.get_logger = lambda: FakeLogger()
    return node


def make_session(session_id='new-session', map_id='map-a', start=100):
    message = LocalizationSession()
    message.session_id = session_id
    message.map_id = map_id
    message.session_start_ns = start
    message.active = True
    return message


def make_result(session_id='new-session', map_id='map-a', stamp=101):
    message = LocalizationResult()
    message.session_id = session_id
    message.map_id = map_id
    message.success = True
    message.fitness_score = 0.01
    message.first_fitness_score = 0.01
    message.observation_id = 1
    message.confirmation_count = 1
    message.cloud_stamp.sec = stamp // 1_000_000_000
    message.cloud_stamp.nanosec = stamp % 1_000_000_000
    message.pose = Pose()
    message.pose.position.x = 1.0
    message.pose.position.y = 2.0
    message.pose.orientation.w = 1.0
    return message


def make_correction(
        epoch=1, x=0.0, accepted=True, translation_residual=0.0,
        state=None):
    message = LioMapOdomCorrection()
    message.instance_id = 'lio-instance'
    message.generation = 7
    message.correction_epoch = epoch
    message.header.stamp.nanosec = 200
    message.lio_map_to_odom_target.translation.x = x
    message.lio_map_to_odom_target.rotation.w = 1.0
    message.translation_residual = translation_residual
    message.state = (
        message.STATE_SMALL_ACCEPTED if state is None else state)
    message.accepted = accepted
    message.reason = 'test'
    return message


def test_old_session_result_cannot_replace_current_transform():
    node = make_authority()
    node.localization_session_callback(make_session())

    node.localization_result_callback(
        make_result(session_id='old-session', stamp=150))

    assert node._accepted_pose is None


def test_cloud_from_before_session_is_rejected():
    node = make_authority()
    node.localization_session_callback(make_session(start=200))

    node.localization_result_callback(make_result(stamp=199))

    assert node._accepted_pose is None


def test_matching_result_becomes_authoritative_pose():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))

    result = make_result(stamp=101)
    node.localization_result_callback(result)

    assert node._accepted_pose is result.pose


def test_new_session_immediately_clears_previous_pose():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))
    node.localization_result_callback(make_result(stamp=101))
    assert node._accepted_pose is not None

    node.localization_session_callback(
        make_session(session_id='next-session', start=300))

    assert node._accepted_pose is None


def test_result_received_first_is_checked_after_session_discovery():
    node = make_authority()
    result = make_result(stamp=101)
    node.localization_result_callback(result)
    assert node._accepted_pose is None

    node.localization_session_callback(make_session(start=100))

    assert node._accepted_pose is result.pose


def test_inactive_current_session_stops_tf_publication():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))
    node.localization_result_callback(make_result(stamp=101))

    inactive = make_session(start=100)
    inactive.active = False
    node.localization_session_callback(inactive)

    assert node._accepted_pose is None
    assert node._active_session_id == ''


def test_valid_pose_rejects_non_unit_or_non_finite_quaternion():
    valid = Pose()
    valid.orientation.w = 1.0
    assert MapOdomAuthority._valid_pose(valid)

    invalid = Pose()
    invalid.orientation.w = float('nan')
    assert not MapOdomAuthority._valid_pose(invalid)

    zero = Pose()
    zero.orientation.w = 0.0
    assert not MapOdomAuthority._valid_pose(zero)


def test_hard_fitness_threshold_is_enforced_by_tf_authority():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))
    result = make_result(stamp=101)
    result.fitness_score = 0.050001

    node.localization_result_callback(result)

    assert node._accepted_pose is None


def test_weak_result_requires_a_newer_independent_observation():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))
    result = make_result(stamp=300)
    result.fitness_score = 0.02
    result.first_fitness_score = 0.032
    result.confirmation_count = 1

    node.localization_result_callback(result)
    assert node._accepted_pose is None

    result.confirmation_count = 2
    result.observation_id = 2
    result.first_cloud_stamp.sec = 0
    result.first_cloud_stamp.nanosec = 200
    node.localization_result_callback(result)
    assert node._accepted_pose is result.pose


def test_weak_final_score_cannot_hide_behind_strong_first_score():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))
    result = make_result(stamp=300)
    result.fitness_score = 0.03
    result.first_fitness_score = 0.01
    result.confirmation_count = 1

    node.localization_result_callback(result)

    assert node._accepted_pose is None


def test_first_lio_correction_establishes_map_lio_without_tf_jump():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))
    node.localization_result_callback(make_result(stamp=101))

    assert node.lio_correction_callback(make_correction(epoch=2, x=0.3))

    assert node._target_map_odom is None
    assert node._current_map_odom[0][0] == 1.0
    assert node._active_lio_identity == ('lio-instance', 7)


def test_small_next_epoch_becomes_rate_limited_target():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))
    node.localization_result_callback(make_result(stamp=101))
    assert node.lio_correction_callback(make_correction(epoch=2, x=0.3))

    assert node.lio_correction_callback(make_correction(epoch=3, x=0.34))

    assert abs(node._target_map_odom[0][0] - 1.04) < 1e-9


def test_transform_follow_rate_is_ten_centimeters_per_second():
    identity = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    target = ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    stepped = MapOdomAuthority._step_toward(identity, target, 0.1)

    assert abs(stepped[0][0] - 0.01) < 1e-9


def test_accepted_medium_dynamic_correction_becomes_rate_limited_target():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))
    node.localization_result_callback(make_result(stamp=101))
    assert node.lio_correction_callback(make_correction(epoch=2, x=0.0))

    correction = make_correction(
        epoch=3, x=0.08, translation_residual=0.08,
        state=LioMapOdomCorrection.STATE_MEDIUM_QUARANTINED)
    assert node.lio_correction_callback(correction)

    assert abs(node._target_map_odom[0][0] - 1.08) < 1e-9
    assert not node._dynamic_fault_latched
    assert not node._fault_pub.messages


def test_cumulative_target_backlog_is_not_treated_as_single_pose_jump():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))
    node.localization_result_callback(make_result(stamp=101))
    assert node.lio_correction_callback(make_correction(epoch=2, x=0.0))

    # This reproduces the latest real-vehicle failure: the accepted scan
    # correction itself was only 4.29 cm, while the rate-limited map->odom
    # target had accumulated a 34.31 cm backlog.
    assert node.lio_correction_callback(make_correction(
        epoch=19, x=0.3431, translation_residual=0.042856))

    assert abs(node._target_map_odom[0][0] - 1.3431) < 1e-9
    assert not node._dynamic_fault_latched
    assert not node._fault_pub.messages


def test_large_rejected_correction_fault_is_cleared_by_new_localization():
    node = make_authority()
    node.localization_session_callback(make_session(start=100))
    node.localization_result_callback(make_result(stamp=101))
    assert node.lio_correction_callback(make_correction(epoch=2, x=0.0))

    assert not node.lio_correction_callback(make_correction(
        epoch=3, x=0.31, accepted=False,
        translation_residual=0.31,
        state=LioMapOdomCorrection.STATE_LARGE_REJECTED))
    assert node._dynamic_fault_latched
    assert node._fault_pub.messages

    node.localization_result_callback(make_result(stamp=201))
    assert not node._dynamic_fault_latched
