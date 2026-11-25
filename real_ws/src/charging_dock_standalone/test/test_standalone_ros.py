"""Real standalone nodes, simulated feedback/plant, no map or hardware drivers."""

import math
import os
from pathlib import Path
import time

from geometry_msgs.msg import TransformStamped
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from charging_dock_msgs.msg import DockObservation
from tf2_ros import StaticTransformBroadcaster
from yunle_msgs.msg import Ecu, VehicleStatus
import yaml

from charging_dock.command_mux import CommandMux
from charging_dock.relative_docking_node import RelativeDockingNode
from charging_dock.relative_geometry import integrate, Pose2
from charging_dock_standalone.feedback_odometry import FeedbackOdometry
from charging_dock_standalone.runner import StandaloneDocking
from cmdvel_2_ecu.convert import CmdVelToEcu


pytestmark = pytest.mark.skipif(
    os.environ.get('DOCK_RUN_ROS_TESTS') != '1',
    reason='requires opt-in isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')


class Harness:
    def __init__(self, path, dry_run=True, distance=2.5, zero_vehicle_speed=False,
                 neutral_coast=False, hold_position=False):
        assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
        assert 100 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232
        shared = Path(__file__).resolve().parents[2] / 'charging_dock' / 'config'
        params = yaml.safe_load((shared / 'relative_docking.yaml').read_text())
        params['relative_docking_controller']['ros__parameters'].update({
            'allow_motion': not dry_run, 'calibrated': not dry_run,
            'local_frame': 'dock_odom', 'global_frame': 'dock_odom',
            'nominal_dock_distance': distance})
        params['command_mux']['ros__parameters'].update({
            'local_frame': 'dock_odom', 'nav_topic': '/unused_nav'})
        params['cmdvel_to_ecu'] = {'ros__parameters': {'cmd_vel_topic': '/cmd_vel_selected'}}
        params['standalone_docking'] = {'ros__parameters': {'dry_run': dry_run}}
        params['docking_feedback_odometry'] = {'ros__parameters': {
            'zero_speed_wheel_fallback': zero_vehicle_speed}}
        config = path / 'test.yaml'
        config.write_text(yaml.safe_dump(params))
        rclpy.init(args=['--ros-args', '--params-file', str(config)])
        self.executor = SingleThreadedExecutor()
        self.hub = Node('standalone_test_hub')
        self.mux, self.ecu, self.dock = CommandMux(), CmdVelToEcu(), RelativeDockingNode()
        self.odom, self.runner = FeedbackOdometry(), StandaloneDocking()
        self.nodes = [self.hub, self.mux, self.ecu, self.dock, self.odom, self.runner]
        for node in self.nodes:
            self.executor.add_node(node)
        self.sensor = Pose2(-.43, -.28, math.radians(225))
        self.back_plate = Pose2(-distance - .057)
        self.base = Pose2()
        self.dock_range_at_lock = None
        self.v = self.omega = 0.
        self.zero_vehicle_speed = zero_vehicle_speed
        self.neutral_coast = neutral_coast
        self.hold_position = hold_position
        self.neutral_coast_samples = []
        self.ecus = []
        self.hub.create_subscription(Ecu, '/ecu', self.ecus.append, 10)
        self.odom_states, self.emergency_stops = [], []
        self.hub.create_subscription(String, '/docking_feedback_odometry/state',
                                     lambda msg: self.odom_states.append(msg.data), 10)
        self.hub.create_subscription(Bool, '/charging_dock/emergency_stop',
                                     lambda msg: self.emergency_stops.append(msg.data), 10)
        self.imu_pub = self.hub.create_publisher(Imu, '/imu', 10)
        self.vehicle_pub = self.hub.create_publisher(VehicleStatus, '/vehicle_status', 10)
        self.scan_pub = self.hub.create_publisher(LaserScan, '/laser/scan_back', 10)
        self.observations = []
        self.hub.create_subscription(DockObservation, '/charging_dock/observation',
                                     self.observations.append, 10)
        self.broadcast = StaticTransformBroadcaster(self.hub)
        tf = TransformStamped()
        tf.header.stamp = self.hub.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'base_link', 'rear_laser'
        tf.transform.translation.x, tf.transform.translation.y = self.sensor.x, self.sensor.y
        tf.transform.rotation.z = math.sin(self.sensor.yaw / 2)
        tf.transform.rotation.w = math.cos(self.sensor.yaw / 2)
        self.broadcast.sendTransform(tf)
        self.imu_enabled = True
        self.wheel_enabled = True
        self.wheel_feedback_override = None
        self.no_dock_returns = False
        # These switches exercise the locked-final safety contract without
        # changing the vehicle/odometry feedback path.  The first keeps a
        # valid, fresh LaserScan stream while its finite FOV no longer covers
        # the dock; the second models a real lidar publication failure.
        self.scan_enabled = True
        self.scan_out_of_fov_after_lock = False
        # Disabled by default; the regression can impose a measured physical
        # forward roll while the feedback still reports reverse gear.
        self.forward_roll_until = 0.
        self.last_sensor_time = time.monotonic()
        self.hub.create_timer(.02, self.sensors)

    def sensors(self):
        now = time.monotonic()
        dt, self.last_sensor_time = now - self.last_sensor_time, now
        requested = steer = 0.
        if self.ecus and not self.ecus[-1].brake and not self.hold_position:
            ecu = self.ecus[-1]
            direction = -1 if ecu.shift == Ecu.SHIFT_R else 1
            requested = direction * math.floor(ecu.motor * 10 + .5) / 36
            steer = math.radians(-ecu.steer)
        forward_roll = now < self.forward_roll_until
        if forward_roll:
            self.v = .025
        else:
            self.v += min(1., dt / .12) * (requested - self.v)
        self.omega = self.v * math.tan(steer) / .5
        self.base = integrate(self.base, self.v, self.omega, dt)
        if (self.dock_range_at_lock is None
                and getattr(self.dock.core, 'locked_dock_pose_local', None) is not None):
            self.dock_range_at_lock = -self.back_plate.x - .057 + self.base.x
        stamp = self.hub.get_clock().now().to_msg()
        vehicle = VehicleStatus()
        vehicle.header.stamp, vehicle.is_autodrive = stamp, True
        vehicle.cur_speed = 0. if self.zero_vehicle_speed else abs(self.v) * 3.6
        vehicle.left_wheel_speed = vehicle.right_wheel_speed = (
            self.v if self.wheel_enabled else math.nan)
        if self.wheel_enabled and self.wheel_feedback_override is not None:
            vehicle.left_wheel_speed, vehicle.right_wheel_speed = self.wheel_feedback_override
        vehicle.shift_level = Ecu.SHIFT_R if forward_roll or self.v < 0 else Ecu.SHIFT_D
        if (self.neutral_coast and self.ecus
                and self.ecus[-1].shift == Ecu.SHIFT_N
                and not self.ecus[-1].brake
                and getattr(self.dock.core, 'locked_dock_pose_local', None) is None
                and self.v < -.005):
            # Reproduce run08: the converter has selected neutral zero drive,
            # but the plant still coasts backward and the unsigned main
            # speed briefly reads 0.2 km/h. Signed wheels retain direction.
            vehicle.shift_level = Ecu.SHIFT_N
            vehicle.cur_speed = .2
            self.neutral_coast_samples.append((self.v, self.base.x))
        self.vehicle_pub.publish(vehicle)
        if self.imu_enabled:
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, 'imu_link'
            imu.angular_velocity.z = self.omega
            self.imu_pub.publish(imu)
        if self.scan_enabled:
            # Independent ray/finite-plane intersections at the actual scan
            # height. Once the target is locked, a fresh restricted-FOV scan
            # deliberately contains no dock returns while preserving valid
            # LaserScan geometry and timestamps.
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'rear_laser'
            locked = getattr(self.dock.core, 'locked_dock_pose_local', None) is not None
            if self.scan_out_of_fov_after_lock and locked:
                count = 128
                scan.angle_min, scan.angle_increment = 0., .01
                scan.angle_max = scan.angle_min + scan.angle_increment * (count - 1)
                scan.range_min, scan.range_max = .05, 30.
                scan.ranges = np.full(count, np.inf).tolist()
            else:
                count = 1440
                scan.angle_min, scan.angle_increment = -math.pi, 2 * math.pi / count
                scan.angle_max = scan.angle_min + scan.angle_increment * (count - 1)
                scan.range_min, scan.range_max = .05, 30.
                sensor = self.base.compose(self.sensor).relative_to(self.back_plate)
                angles = scan.angle_min + np.arange(count) * scan.angle_increment + sensor.yaw
                with np.errstate(divide='ignore', invalid='ignore'):
                    distance = -sensor.x / np.cos(angles)
                    y = sensor.y + distance * np.sin(angles)
                scan.ranges = (np.full(count, np.inf) if self.no_dock_returns else
                               np.where((distance > .06) & (abs(y) <= .125),
                                        distance, np.inf)).tolist()
            self.scan_pub.publish(scan)

    def spin_until(self, condition, timeout=10.):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)
        assert condition(), (self.runner.state, self.runner.reason,
                             self.odom.core.reason, self.dock.core.state, self.dock.core.reason)

    def close(self):
        self.runner.client.destroy()
        self.dock.action.destroy()
        for node in self.nodes:
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()


@pytest.fixture
def make_harness(tmp_path):
    harnesses = []

    def create(**kwargs):
        harness = Harness(tmp_path, **kwargs)
        harnesses.append(harness)
        return harness

    yield create
    for harness in harnesses:
        harness.close()


def test_manual_p0_automatically_runs_static_precheck_without_map(make_harness):
    h = make_harness()
    h.spin_until(lambda: h.runner.state in ('READY', 'FAILED'))
    assert h.runner.state == 'READY', h.runner.reason
    assert h.runner.p0.header.frame_id == 'dock_odom'
    assert h.dock.tracker.anchor.x == pytest.approx(-2.557)
    assert h.ecus and all(m.motor == 0. and not m.brake for m in h.ecus)


def test_local_feedback_and_live_scans_complete_actual_control_loop(make_harness):
    # Leave room above the 0.60 m no-lock deadline for the sustained moving
    # alignment window and measured braking; starting on that deadline must
    # stop safely instead of bypassing the new first-stop qualification.
    h = make_harness(dry_run=False, distance=.65)
    h.spin_until(lambda: h.runner.state in ('DOCKED', 'FAILED'), timeout=20.)
    assert h.runner.state == 'DOCKED', h.runner.reason
    assert any(m.motor > .02 for m in h.ecus)
    assert h.ecus[-1].motor == 0.
    assert .45 <= .65 + h.base.x <= .47
    assert abs(h.base.y) < .015
    assert abs(h.base.yaw) < math.radians(1)


def test_zero_vehicle_speed_uses_wheels_beyond_p0_gate_to_complete_docking(make_harness):
    h = make_harness(dry_run=False, distance=.75, zero_vehicle_speed=True)
    h.spin_until(lambda: h.runner.state in ('DOCKED', 'FAILED'), timeout=30.)
    assert h.runner.state == 'DOCKED', h.runner.reason
    assert h.odom.core.estimate.pose.x < -.20
    assert .45 <= .75 + h.base.x <= .47
    assert h.ecus[-1].motor == 0. and not h.ecus[-1].brake


def test_asymmetric_signed_wheels_keep_odometry_valid_and_reverse_control_running(make_harness):
    h = make_harness(dry_run=False, zero_vehicle_speed=True)
    h.spin_until(lambda: any(m.motor > .02 for m in h.ecus)
                 and h.odom.core.estimate is not None
                 and h.odom.core.estimate.speed < -.01)
    odom_before = h.odom.core.estimate.pose.x
    h.wheel_feedback_override = (-.08, .04)
    assert abs(h.wheel_feedback_override[0] - h.wheel_feedback_override[1]) > .10
    h.spin_until(lambda: h.odom.core.estimate.speed == pytest.approx(-.02))
    until = time.monotonic() + .30
    h.spin_until(lambda: time.monotonic() >= until)

    assert h.odom.core.estimate.pose.x < odom_before
    assert h.odom.core.estimate.speed == pytest.approx(-.02)
    assert not h.odom.core.failed
    assert 'invalid_wheel_feedback' not in h.odom_states
    assert not any(h.emergency_stops)
    assert not h.mux.stop_latched
    assert h.runner.state == 'DOCKING'
    assert h.dock.last_output.velocity < 0.
    assert h.ecus[-1].shift == Ecu.SHIFT_R and not h.ecus[-1].brake

    # Return to normal signed feedback without resetting the attempt or odometry.
    h.wheel_feedback_override = None
    h.spin_until(lambda: h.odom.core.estimate.speed < -.03)
    assert not h.odom.core.failed
    assert not any(h.emergency_stops)
    assert h.runner.state == 'DOCKING'
    assert h.dock.last_output.velocity < 0.


def test_first_live_scan_alignment_stops_before_standalone_lock_confirmation(make_harness):
    # Keep the simulated vehicle parked to isolate scan noise from trajectory
    # corrections; fresh signed wheels, IMU, odometry and scans still run.
    h = make_harness(dry_run=False, distance=1., zero_vehicle_speed=True,
                     hold_position=True)
    h.back_plate = Pose2(-1.057, .02)
    states = []
    h.hub.create_subscription(String, '/charging_dock/state',
                              lambda msg: states.append(msg.data.split()[0]), 10)
    h.spin_until(lambda: h.dock.last_output is not None
                 and h.dock.last_output.velocity < 0., timeout=15.)
    assert h.runner.state == 'DOCKING'
    assert h.dock.get_parameter('lock_stop_on_first_alignment').value is True
    assert h.dock.get_parameter('lock_alignment_duration').value == .5
    until = time.monotonic() + .65
    h.spin_until(lambda: time.monotonic() >= until)
    assert 'LOCK_VERIFY' not in states

    h.back_plate = Pose2(-1.057)
    h.spin_until(lambda: h.dock.core.state == 'LOCK_VERIFY')
    assert h.dock.last_output.velocity == 0.
    assert h.dock.core.locked_dock_pose_local is None
    # The first aligned observation only stops the vehicle. The stopped scan
    # average must remain stable for the independent 0.5s confirmation.
    until = time.monotonic() + .20
    h.spin_until(lambda: time.monotonic() >= until)
    assert h.dock.core.state == 'LOCK_VERIFY'
    assert h.dock.core.locked_dock_pose_local is None
    h.spin_until(lambda: h.dock.core.state == 'LOCKED_FINAL_APPROACH')
    assert h.dock.core.locked_dock_pose_local is not None
    assert h.runner.state == 'DOCKING'
    assert not h.mux.stop_latched
    assert all(message.shift != Ecu.SHIFT_D for message in h.ecus)

    client = h.hub.create_client(Trigger, '/standalone_docking/stop')
    h.spin_until(client.service_is_ready)
    future = client.call_async(Trigger.Request())
    h.spin_until(future.done)
    assert future.result().success
    h.spin_until(lambda: h.ecus[-1].motor == 0. and not h.ecus[-1].brake)
    assert h.runner.state == 'STOPPED'


@pytest.mark.parametrize('stop', ['operator_stop', 'imu_loss'])
def test_unlimited_attempt_and_locked_final_survive_120s_age_and_preserve_stops(
        make_harness, stop):
    h = make_harness(dry_run=False, distance=.92, zero_vehicle_speed=True)
    assert h.dock.core.config.timeout == 0.
    assert h.dock.core.config.locked_final_timeout == 0.
    assert h.runner.values['action_timeout'] == 0.
    h.scan_out_of_fov_after_lock = True
    h.spin_until(lambda: h.dock.core.state == 'LOCKED_FINAL_APPROACH'
                 and h.ecus[-1].shift == Ecu.SHIFT_R and not h.ecus[-1].brake,
                 timeout=15.)
    assert h.dock.goal.request.timeout_sec == 0.
    locked_pose = h.dock.core.locked_dock_pose_local
    odom_x_before = h.odom.core.estimate.pose.x

    # Exercise the former total and locked-final deadlines without waiting
    # 120 real seconds. CAN/IMU/scans and TF keep their actual fresh clocks.
    h.runner.started -= 120.
    h.dock.requested_at -= 120.
    h.dock.core.started -= 120.
    h.dock.core.locked_final_started -= 120.
    until = time.monotonic() + .35
    h.spin_until(lambda: time.monotonic() >= until)
    assert h.runner.state == 'DOCKING', h.runner.reason
    assert h.dock.core.state == 'LOCKED_FINAL_APPROACH'
    assert h.dock.core.locked_dock_pose_local == locked_pose
    assert h.dock.last_output.velocity < 0.
    assert h.odom.core.estimate.pose.x < odom_x_before
    assert not h.odom.core.failed
    assert not h.mux.stop_latched

    if stop == 'operator_stop':
        client = h.hub.create_client(Trigger, '/standalone_docking/stop')
        h.spin_until(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        h.spin_until(future.done)
        assert future.result().success
    else:
        h.imu_enabled = False
    h.spin_until(lambda: h.runner.state in ('FAILED', 'STOPPED'))
    h.spin_until(lambda: h.ecus[-1].motor == 0. and not h.ecus[-1].brake)
    assert h.runner.state == ('STOPPED' if stop == 'operator_stop' else 'FAILED')
    assert h.mux.stop_latched
    if stop == 'imu_loss':
        assert h.odom.core.failed and h.odom.core.reason == 'can_or_imu_stale'


def test_locked_final_reaches_dock_when_complete_face_leaves_fov(make_harness):
    h = make_harness(dry_run=False, distance=1.15)
    h.scan_out_of_fov_after_lock = True
    h.spin_until(lambda: getattr(h.dock.core, 'locked_dock_pose_local', None) is not None,
                 timeout=30.)
    h.spin_until(lambda: h.dock_range_at_lock is not None)
    h.spin_until(lambda: (h.dock.latest_scan is not None
                          and h.dock.observation is not None
                          and not h.dock.observation.valid
                          and h.dock.core.state == 'LOCKED_FINAL_APPROACH'))
    assert .90 <= h.dock_range_at_lock <= 1.10
    scan = h.dock.latest_scan[0]
    assert len(scan.ranges) == 128
    assert all(math.isinf(value) for value in scan.ranges)
    scan_stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
    assert 0. <= h.dock._now() - scan_stamp <= h.dock.tracker.config.timeout
    h.spin_until(lambda: any(not obs.valid and obs.reason == 'no_complete_face_in_gate'
                             for obs in h.observations))
    h.spin_until(lambda: h.runner.state in ('DOCKED', 'FAILED'), timeout=40.)
    assert h.runner.state == 'DOCKED', h.runner.reason
    assert .45 <= 1.15 + h.base.x <= .47
    assert h.ecus[-1].motor == 0. and not h.ecus[-1].brake


def test_nonzero_main_speed_during_neutral_coast_keeps_odometry_and_completes(make_harness):
    h = make_harness(dry_run=False, distance=1.15, zero_vehicle_speed=True,
                     neutral_coast=True)
    h.scan_out_of_fov_after_lock = True
    h.spin_until(lambda: bool(h.neutral_coast_samples)
                 and h.odom.core.estimate is not None
                 and h.odom.core.estimate.speed < -.01, timeout=20.)
    odom_before_coast = h.odom.core.estimate.pose.x
    assert h.ecus[-1].shift == Ecu.SHIFT_N and not h.ecus[-1].brake
    assert not h.odom.core.failed

    h.spin_until(lambda: getattr(h.dock.core, 'locked_dock_pose_local', None) is not None,
                 timeout=10.)
    assert h.odom.core.estimate.pose.x < odom_before_coast
    assert h.neutral_coast_samples[0][0] < -.01
    assert abs(h.odom.core.estimate.pose.x - h.base.x) < .005
    assert not h.odom.core.failed
    assert 'moving_without_direction_feedback' not in h.odom_states
    assert not any(h.emergency_stops)

    h.spin_until(lambda: h.runner.state in ('DOCKED', 'FAILED'), timeout=40.)
    assert h.runner.state == 'DOCKED', h.runner.reason
    assert any(not obs.valid and obs.reason == 'no_complete_face_in_gate'
               for obs in h.observations)
    assert not h.odom.core.failed
    assert not any(h.emergency_stops)
    assert .45 <= 1.15 + h.base.x <= .47
    assert h.ecus[-1].motor == 0. and not h.ecus[-1].brake


def test_actual_stop_at_092m_sets_remaining_distance_with_1m_lock_preset(make_harness):
    # Model a vehicle already stopped at 0.92 m. Keep it there during moving
    # alignment qualification and stopped-pose verification, then release the
    # simulated plant for the actual odometry-only final approach.
    h = make_harness(dry_run=False, distance=.92, zero_vehicle_speed=True,
                     hold_position=True)
    assert h.dock.core.config.lock_distance == pytest.approx(1.)
    h.scan_out_of_fov_after_lock = True
    h.spin_until(lambda: getattr(h.dock.core, 'locked_dock_pose_local', None) is not None,
                 timeout=15.)
    h.spin_until(lambda: h.dock_range_at_lock is not None)

    locked_pose = h.dock.core.locked_dock_pose_local
    dock_at_stop = locked_pose.relative_to(h.odom.core.estimate.pose)
    remaining = h.dock.core.config.base_goal(dock_at_stop).inverse().x
    assert h.dock_range_at_lock == pytest.approx(.92, abs=.01)
    assert -dock_at_stop.x == pytest.approx(.92, abs=.01)
    assert remaining == pytest.approx(.46, abs=.01)
    assert h.dock.core.locked_distance == pytest.approx(.92, abs=.01)
    assert h.dock.core.locked_remaining_distance == pytest.approx(.46, abs=.01)
    assert abs(remaining - .54) > .05  # Never use the preset minus the final range.

    h.hold_position = False
    h.spin_until(lambda: any(not obs.valid and obs.reason == 'no_complete_face_in_gate'
                             for obs in h.observations))
    h.spin_until(lambda: h.runner.state in ('DOCKED', 'FAILED'), timeout=35.)
    assert h.runner.state == 'DOCKED', h.runner.reason
    assert h.dock.core.locked_dock_pose_local == locked_pose
    assert not any(h.emergency_stops)
    assert .45 <= .92 + h.base.x <= .47
    assert h.ecus[-1].motor == 0. and not h.ecus[-1].brake


def test_locked_final_accepts_measured_forward_roll_in_reverse_and_completes(make_harness):
    h = make_harness(dry_run=False, distance=1.15, zero_vehicle_speed=True)
    h.scan_out_of_fov_after_lock = True
    h.spin_until(lambda: h.dock.core.state == 'LOCKED_FINAL_APPROACH'
                 and h.dock_range_at_lock is not None
                 and h.ecus[-1].shift == Ecu.SHIFT_R and not h.ecus[-1].brake,
                 timeout=30.)
    locked_pose = h.dock.core.locked_dock_pose_local
    odom_x_before, body_x_before = h.odom.core.estimate.pose.x, h.base.x

    h.forward_roll_until = time.monotonic() + .24
    h.spin_until(lambda: 'forward_roll_in_reverse' in h.odom_states
                 and h.odom.core.estimate.pose.x > odom_x_before + .001,
                 timeout=2.)
    assert h.base.x > body_x_before
    assert h.odom.core.estimate.speed > 0.
    assert not h.odom.core.failed
    assert h.dock.core.locked_dock_pose_local == locked_pose
    assert not any(h.emergency_stops)

    h.spin_until(lambda: time.monotonic() > h.forward_roll_until
                 and h.odom.core.estimate.speed < -.01, timeout=3.)
    h.spin_until(lambda: h.runner.state in ('DOCKED', 'FAILED'), timeout=40.)
    assert h.runner.state == 'DOCKED', h.runner.reason
    assert h.dock.core.locked_dock_pose_local == locked_pose
    assert not any(h.emergency_stops)
    assert any(not obs.valid and obs.reason == 'no_complete_face_in_gate'
               for obs in h.observations)
    assert .45 <= 1.15 + h.base.x <= .47
    assert h.ecus[-1].motor == 0. and not h.ecus[-1].brake


def test_locked_final_fails_safe_when_lidar_stops_publishing(make_harness):
    h = make_harness(dry_run=False, distance=1.15)
    assert h.dock.core.config.locked_final_timeout == 0.
    h.spin_until(lambda: getattr(h.dock.core, 'locked_dock_pose_local', None) is not None,
                 timeout=30.)
    h.spin_until(lambda: h.dock_range_at_lock is not None)
    assert .90 <= h.dock_range_at_lock <= 1.10
    h.dock.core.locked_final_started -= 120.
    h.scan_enabled = False
    h.spin_until(lambda: h.runner.state in ('FAILED', 'STOPPED'), timeout=8.)
    h.spin_until(lambda: h.ecus[-1].motor == 0. and not h.ecus[-1].brake)
    assert h.runner.state == 'FAILED', h.runner.reason
    assert h.dock.core.state == 'FAILED'
    assert h.dock.core.reason == 'dock_observation_invalid'


def test_brief_scan_loss_holds_zero_drive_and_resumes_after_target_reacquisition(make_harness):
    # This loss occurs before the pose is locked. Once locked, the same loss
    # must be distinguished from a fresh scan whose finite FOV excludes the dock.
    h = make_harness(dry_run=False, distance=1.40)
    h.spin_until(lambda: any(m.motor > .02 for m in h.ecus))
    h.no_dock_returns = True
    h.spin_until(lambda: h.dock.core.state == 'OBSERVATION_HOLD')
    h.spin_until(lambda: h.ecus[-1].motor == 0. and not h.ecus[-1].brake)

    resume_at = time.monotonic() + .20
    h.spin_until(lambda: time.monotonic() >= resume_at)
    h.no_dock_returns = False
    h.spin_until(lambda: h.dock.core.state in ('APPROACH', 'FINAL_APPROACH'))
    h.spin_until(lambda: h.runner.state in ('DOCKED', 'FAILED'), timeout=40.)

    assert h.runner.state == 'DOCKED', h.runner.reason
    assert any(m.motor > .02 for m in h.ecus)
    assert h.ecus[-1].motor == 0. and not h.ecus[-1].brake


@pytest.mark.parametrize('fault', ['imu_loss', 'operator_stop', 'wheel_loss'])
def test_standalone_faults_stop_and_never_restart_automatically(make_harness, fault):
    h = make_harness(dry_run=False, zero_vehicle_speed=fault == 'wheel_loss')
    h.spin_until(lambda: any(m.motor > .02 for m in h.ecus))
    if fault == 'imu_loss':
        h.imu_enabled = False
    elif fault == 'wheel_loss':
        h.wheel_enabled = False
    else:
        client = h.hub.create_client(Trigger, '/standalone_docking/stop')
        h.spin_until(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        h.spin_until(future.done)
        assert future.result().success
    h.spin_until(lambda: h.runner.state in ('FAILED', 'STOPPED'))
    h.spin_until(lambda: h.ecus[-1].motor == 0. and not h.ecus[-1].brake)
    if fault == 'wheel_loss':
        assert h.odom.core.failed and h.odom.core.reason == 'invalid_wheel_feedback'
    assert h.mux.stop_latched
    h.imu_enabled = True
    deadline = time.monotonic() + .3
    h.spin_until(lambda: time.monotonic() >= deadline)
    assert h.ecus[-1].motor == 0.
    if fault == 'imu_loss':
        assert h.odom.core.failed
