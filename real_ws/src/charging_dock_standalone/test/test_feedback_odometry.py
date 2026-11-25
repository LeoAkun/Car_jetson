"""Feedback integration must not manufacture stationary or fresh odometry."""

import math

import pytest

from charging_dock_standalone.feedback_odometry import FeedbackIntegrator


def test_reverse_distance_and_imu_turn_use_measured_feedback():
    core = FeedbackIntegrator()
    assert core.update(1., 1., 1., .18, 3, .1).pose.x == 0.
    for n in range(1, 101):
        t = 1 + n * .02
        estimate = core.update(t, t, t, .18, 3, .1)
    assert estimate.pose.x == pytest.approx(-.5 * math.sin(.2))
    assert estimate.pose.y == pytest.approx(-.5 * (1 - math.cos(.2)))
    assert estimate.pose.yaw == pytest.approx(.2)


def test_zero_reported_speed_falls_back_to_signed_wheel_feedback():
    core = FeedbackIntegrator()
    assert core.update(1., 1., 1., 0., 3, 0., (-.04, -.04)).pose.x == 0.
    estimate = core.update(1.1, 1.1, 1.1, 0., 3, 0., (-.04, -.04))
    assert estimate.pose.x == pytest.approx(-.004)


def test_neutral_gear_does_not_hide_coasting_with_wheel_feedback():
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 3, 0., (-.04, -.04))
    estimate = core.update(1.1, 1.1, 1.1, 0., 2, 0., (-.02, -.02))
    assert estimate.speed == pytest.approx(-.02)
    assert estimate.pose.x == pytest.approx(-.003)


def test_nonzero_main_speed_during_neutral_braking_keeps_signed_wheel_odometry():
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 3, 0., (-.06, -.06))

    # run08: the quantized main speed rises after shifting to N, while the
    # measured wheels still show reverse coasting. Keep integrating that
    # motion until the wheels actually stop.
    estimate = core.update(1.1, 1.1, 1.1, .2, 2, 0., (-.065, -.065))
    assert estimate.speed == pytest.approx(-.065)
    assert estimate.pose.x == pytest.approx(-.00625)
    assert core.reason == 'valid'
    assert not core.failed

    estimate = core.update(1.2, 1.2, 1.2, .2, 2, 0., (-.03, -.03))
    assert estimate.pose.x == pytest.approx(-.011)
    estimate = core.update(1.3, 1.3, 1.3, 0., 2, 0., (0., 0.))
    assert estimate.pose.x == pytest.approx(-.0125)
    estimate = core.update(1.4, 1.4, 1.4, 0., 2, 0., (0., 0.))
    assert estimate.stamp == 1.4
    assert estimate.speed == 0.
    assert estimate.pose.x == pytest.approx(-.0125)
    assert not core.failed


def test_forward_roll_in_reverse_is_integrated_and_reverse_can_resume():
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 2, 0., (0., 0.))
    for t in (1.1, 1.2):
        estimate = core.update(t, t, t, 0., 3, 0., (.04, .04))
        assert estimate.speed == pytest.approx(.04)
        assert core.reason == 'forward_roll_in_reverse'
        assert not core.failed
    assert estimate.pose.x == pytest.approx(.006)

    # A reversal retains the measured positive travel rather than counting it
    # as docking progress; trapezoidal integration spans the sign change.
    estimate = core.update(1.3, 1.3, 1.3, 0., 3, 0., (-.04, -.04))
    assert estimate.pose.x == pytest.approx(.006)
    estimate = core.update(1.4, 1.4, 1.4, 0., 3, 0., (-.04, -.04))
    assert estimate.pose.x == pytest.approx(.002)
    assert estimate.speed == pytest.approx(-.04)
    assert core.reason == 'valid'
    assert not core.failed


def test_nonzero_main_speed_does_not_reverse_actual_forward_wheel_motion():
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 2, 0., (0., 0.))
    estimate = core.update(1.1, 1.1, 1.1, .2, 3, 0., (.04, .04))
    assert estimate.speed == pytest.approx(.04)
    assert estimate.pose.x == pytest.approx(.002)
    assert core.reason == 'forward_roll_in_reverse'
    assert not core.failed


def test_run05_opposite_wheel_signs_preserve_measured_forward_mean():
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 2, 0., (0., 0.))
    estimate = core.update(1.1, 1.1, 1.1, 0., 3, 0., (-.0051, .0305))
    assert estimate.speed == pytest.approx(.0127)
    assert estimate.pose.x == pytest.approx(.000635)
    assert core.reason == 'forward_roll_in_reverse'
    assert not core.failed


@pytest.mark.parametrize('wheels,gear', [
    ((math.nan, -.04), 3), ((-.04, math.inf), 3), ((-.6, -.6), 3),
    ((.6, .6), 3), ((-.04, -.04), 1),
    ((-.04, -.04), 0), ((.04, .04), 0),
])
def test_bad_wheel_feedback_latches_stop_instead_of_freezing_pose(wheels, gear):
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 2, 0., (0., 0.))
    assert core.update(1.02, 1.02, 1.02, 0., gear, 0., wheels) is None
    assert core.failed
    assert core.estimate.pose.x == 0.


def test_nonzero_vehicle_speed_continues_using_signed_wheel_source():
    core = FeedbackIntegrator()
    estimate = core.update(1., 1., 1., .36, 3, 0., (-.04, -.04))
    assert estimate.speed == pytest.approx(-.04)


def test_nonzero_vehicle_speed_without_wheels_preserves_main_speed_source():
    core = FeedbackIntegrator()
    estimate = core.update(1., 1., 1., .36, 3, 0.)
    assert estimate.speed == pytest.approx(-.1)


def test_nonzero_neutral_main_speed_without_wheels_still_latches_stop():
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 2, 0.)
    assert core.update(1.1, 1.1, 1.1, .2, 2, 0.) is None
    assert core.reason == 'moving_without_direction_feedback'
    assert core.failed
    assert core.estimate.stamp == 1.
    assert core.estimate.pose.x == 0.


@pytest.mark.parametrize('wheels', [
    (math.nan, -.04), (-.04, math.inf), (-.6, -.6),
])
def test_invalid_wheels_during_nonzero_neutral_coasting_latch_stop(wheels):
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 3, 0., (-.04, -.04))
    assert core.update(1.1, 1.1, 1.1, .2, 2, 0., wheels) is None
    assert core.reason == 'invalid_wheel_feedback'
    assert core.failed
    assert core.estimate.stamp == 1.
    assert core.estimate.pose.x == 0.


@pytest.mark.parametrize('gear,main_speed,wheels', [
    (3, 0., (-.08, .04)),
    (2, .2, (-.2, .2)),
    # Exact feedback that triggered run11's removed difference check.
    (3, .30000001192092896, (.01270250603556633, .31756266951560974)),
])
def test_different_wheel_speeds_integrate_signed_mean_without_latching(
        gear, main_speed, wheels):
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 2, 0., (0., 0.))
    estimate = core.update(1.02, 1.02, 1.02, main_speed, gear, 0., wheels)
    mean = sum(wheels) / 2
    assert estimate.speed == pytest.approx(mean)
    assert estimate.pose.x == pytest.approx(mean * .01)
    assert estimate.stamp == 1.02
    assert not core.failed
    assert core.reason == ('forward_roll_in_reverse' if gear == 3 and mean > .01
                           else 'valid')

    # A following normal measurement must still advance the odometry.
    recovered = core.update(1.04, 1.04, 1.04, 0., 3, 0., (-.02, -.02))
    assert recovered.speed == pytest.approx(-.02)
    assert recovered.pose.x == pytest.approx(mean * .01 + (mean - .02) * .01)
    assert not core.failed
    assert core.reason == 'valid'


def test_stale_wheels_during_nonzero_neutral_coasting_cannot_refresh_odometry():
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 3, 0., (-.04, -.04))
    assert core.update(1.2, 1., 1.2, .2, 2, 0., (-.04, -.04)) is None
    assert core.reason == 'can_or_imu_stale'
    assert core.failed
    assert core.estimate.stamp == 1.
    assert core.estimate.pose.x == 0.


def test_old_feedback_is_not_restamped_and_sensor_gap_latches_failure():
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 2, 0.)
    assert core.update(1.1, 1., 1., 0., 2, 0.) is None
    assert core.estimate.stamp == 1.
    assert core.update(1.2, 1., 1., 0., 2, 0.) is None
    assert core.failed
    assert core.update(1.21, 1.21, 1.21, 0., 2, 0.) is None


@pytest.mark.parametrize('speed,gear,omega', [
    (.1, 0, 0.), (.1, 2, 0.), (math.nan, 3, 0.), (0., 2, math.inf), (-.1, 3, 0.),
])
def test_invalid_feedback_after_initialization_cannot_move_the_pose(speed, gear, omega):
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 2, 0.)
    assert core.update(1.02, 1.02, 1.02, speed, gear, omega) is None
    assert core.failed
    assert core.estimate.pose.x == 0.


def test_sensor_arrival_skew_uses_older_measurement_time():
    core = FeedbackIntegrator()
    estimate = core.update(1.03, 1.02, 1., 0., 2, 0.)
    assert estimate.stamp == 1.


def test_backward_feedback_stamp_is_latched():
    core = FeedbackIntegrator()
    core.update(1., 1., 1., 0., 2, 0.)
    assert core.update(1.01, .99, 1.01, 0., 2, 0.) is None
    assert core.reason == 'feedback_time_discontinuity'
