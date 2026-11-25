"""Tests for adaptive localization confirmation policy."""

import math

import pytest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose

from nav2_init_pose.nav2_init_pose import PoseInitNode


class FakeLogger:
    def warning(self, _message):
        pass


def pose(x=0.0, y=0.0, yaw=0.0):
    value = Pose()
    value.position.x = x
    value.position.y = y
    value.orientation.z = math.sin(yaw / 2.0)
    value.orientation.w = math.cos(yaw / 2.0)
    return value


def stamp(nanosec):
    value = Time()
    value.nanosec = nanosec
    return value


def make_node(score=0.01):
    node = PoseInitNode.__new__(PoseInitNode)
    node.localization_deadline = float('inf')
    node.localization_cloud_stamp = stamp(100)
    node.first_localization_cloud_stamp = None
    node.first_fitness_score = float('inf')
    node.observation_id = 1
    node.observation_has_cloud = True
    node._observation_min_cloud_stamp = None
    node.confirmation_count = 0
    node.direct_accept_threshold = 0.025
    node.fitness_score_threshold = 0.05
    node.confirmation_yaw_tolerance = math.radians(3.0)
    node.max_cloud_age = 120.0
    node._cloud_age = lambda _stamp: 0.0
    node._static_footprint_is_legal = lambda _pose: (True, 'free')
    node.get_logger = lambda: FakeLogger()
    node.accepted = []
    node.rejected = []
    node.accept_localization = lambda result_pose, result_score, reason='': (
        node.accepted.append((result_pose, result_score, reason)))
    node.publish_localization_result = (
        lambda success, result_score, pose=None, reason='':
        node.rejected.append((success, result_score, reason)))
    return node


def test_strong_legal_result_is_accepted_without_second_cloud():
    node = make_node()
    node._local_confirm = lambda _pose: (_ for _ in ()).throw(
        AssertionError('strong result must not request another cloud'))

    accepted = node.validate_and_accept(
        pose(1.0, 2.0), 0.01, prior_x=1.0, prior_y=2.0)

    assert accepted
    assert node.confirmation_count == 1
    assert len(node.accepted) == 1


def test_weak_result_requires_consistent_newer_observation():
    node = make_node()
    node._local_confirm = lambda _pose: (
        0.03, pose(1.1, 2.0, math.radians(2.0)), stamp(200))

    accepted = node.validate_and_accept(
        pose(1.0, 2.0), 0.032, prior_x=1.0, prior_y=2.0)

    assert accepted
    assert node.observation_id == 2
    assert node.confirmation_count == 2
    assert node.first_localization_cloud_stamp.nanosec == 100
    assert node.localization_cloud_stamp.nanosec == 200


def test_poor_result_gets_one_fresh_retry_then_weak_confirmation():
    node = make_node()
    responses = iter((
        (0.03, pose(1.0, 2.0), stamp(200)),
        (0.025, pose(1.1, 2.0, math.radians(1.0)), stamp(300)),
    ))
    node._local_confirm = lambda _pose: next(responses)

    accepted = node.validate_and_accept(
        pose(1.0, 2.0), 0.2, prior_x=1.0, prior_y=2.0)

    assert accepted
    assert node.observation_id == 3
    assert node.confirmation_count == 2
    assert node.first_fitness_score == 0.03
    assert node.first_localization_cloud_stamp.nanosec == 200
    assert node.localization_cloud_stamp.nanosec == 300


def test_large_translation_confirmation_is_accepted_and_selected():
    node = make_node()
    confirmed_pose = pose(1.925, 2.0, math.radians(2.02))
    calls = []

    def confirm(reference):
        calls.append(reference)
        return 0.013867, confirmed_pose, stamp(200)

    node._local_confirm = confirm

    accepted = node.validate_and_accept(
        pose(1.0, 2.0), 0.042516, prior_x=1.0, prior_y=2.0)

    assert accepted
    assert len(calls) == 1
    assert node.confirmation_count == 2
    assert node.observation_id == 2
    assert node.accepted[0][0] is confirmed_pose
    assert node.accepted[0][1] == 0.013867
    assert node.localization_cloud_stamp.nanosec == 200
    assert not node.rejected


def test_inconsistent_yaw_is_rejected_after_retry_budget():
    node = make_node()
    node._local_confirm = lambda _pose: (
        0.02, pose(1.925, 2.0, math.radians(3.01)),
        stamp(200 + 100 * node.observation_id))

    accepted = node.validate_and_accept(
        pose(1.0, 2.0), 0.03, prior_x=1.0, prior_y=2.0)

    assert not accepted
    assert not node.accepted
    assert node.observation_id == 3
    assert len(node.rejected) == 1
    assert 'confirmation failed' in node.rejected[0][2]


def test_large_translation_still_requires_legal_footprint():
    node = make_node()
    node._static_footprint_is_legal = lambda value: (
        value.position.x < 1.5, 'occupied' if value.position.x >= 1.5 else 'free')
    node._local_confirm = lambda _pose: (0.02, pose(1.925, 2.0), stamp(200))

    accepted = node.validate_and_accept(
        pose(1.0, 2.0), 0.03, prior_x=1.0, prior_y=2.0)

    assert not accepted
    assert not node.accepted
    assert node.observation_id == 3


def test_large_translation_still_requires_prior_consistency():
    node = make_node()
    node._local_confirm = lambda _pose: (0.02, pose(16.1, 2.0), stamp(200))

    accepted = node.validate_and_accept(
        pose(1.0, 2.0), 0.03, prior_x=1.0, prior_y=2.0)

    assert not accepted
    assert not node.accepted
    assert node.observation_id == 3


@pytest.mark.parametrize('confirmation_score, expected', ((0.05, True), (0.0501, False)))
def test_large_translation_still_requires_fitness_threshold(confirmation_score, expected):
    node = make_node()
    node._local_confirm = lambda _pose: (
        confirmation_score, pose(1.925, 2.0), stamp(200))

    accepted = node.validate_and_accept(
        pose(1.0, 2.0), 0.03, prior_x=1.0, prior_y=2.0)

    assert accepted is expected
    assert bool(node.accepted) is expected


def test_yaw_retry_keeps_fixed_reference_and_advances_observation():
    node = make_node()
    first_pose = pose(1.0, 2.0)
    confirmed_pose = pose(1.925, 2.0, math.radians(2.02))
    observations = []

    def confirm(reference):
        observations.append((reference, node.observation_id,
                             node._observation_min_cloud_stamp.nanosec))
        if len(observations) == 1:
            return 0.02, pose(1.9, 2.0, math.radians(4.0)), stamp(200)
        return 0.013867, confirmed_pose, stamp(300)

    node._local_confirm = confirm

    accepted = node.validate_and_accept(
        first_pose, 0.03, prior_x=1.0, prior_y=2.0)

    assert accepted
    assert observations == [(first_pose, 2, 100), (first_pose, 3, 200)]
    assert node.accepted[0][0] is confirmed_pose
    assert node.first_localization_cloud_stamp.nanosec == 100
    assert node.localization_cloud_stamp.nanosec == 300
