# Copyright 2026 akun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Isolated-domain action/mux/converter tests; no chassis driver is launched."""

import math
import os
import time

import pytest
import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from yunle_msgs.msg import Ecu, VehicleStatus
from charging_dock_msgs.action import Dock
from charging_dock_msgs.msg import DockObservation

from charging_dock.command_mux import CommandMux
from charging_dock.relative_docking_node import RelativeDockingNode
from charging_dock.relative_geometry import integrate, Pose2
from cmdvel_2_ecu.convert import CmdVelToEcu
from relative_scan_fixture import docking_parameters, scan_faces


pytestmark = pytest.mark.skipif(
    os.environ.get('DOCK_RUN_ROS_TESTS') != '1',
    reason='opt in with an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')


class Harness:
    def __init__(self, tmp_path):
        assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
        assert 100 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232
        config = tmp_path / 'test.yaml'
        parameters = docking_parameters()
        parameters['relative_docking_controller']['ros__parameters'].update(
            allow_motion=True, calibrated=True)
        parameters['cmdvel_to_ecu'] = {'ros__parameters': {'cmd_vel_topic': '/cmd_vel_selected'}}
        config.write_text(yaml.safe_dump(parameters))
        rclpy.init(args=['--ros-args', '--params-file', str(config)])
        self.executor = SingleThreadedExecutor()
        self.hub = Node('relative_dock_test_hub')
        self.mux, self.ecu, self.dock = CommandMux(), CmdVelToEcu(), RelativeDockingNode()
        self.nodes = [self.hub, self.mux, self.ecu, self.dock]
        for node in self.nodes:
            self.executor.add_node(node)
        self.sensor = Pose2(-.43, -.28, math.radians(225))
        self.static = StaticTransformBroadcaster(self.hub)
        self.dynamic = TransformBroadcaster(self.hub)
        self.static.sendTransform([
            self.transform('map', 'odom', Pose2()),
            self.transform('base_link', 'rear_laser', self.sensor)])
        self.scan_pub = self.hub.create_publisher(LaserScan, '/laser/scan_back', 10)
        self.vehicle_pub = self.hub.create_publisher(VehicleStatus, '/vehicle_status', 10)
        self.nav_pub = self.hub.create_publisher(Twist, '/cmd_vel', 10)
        self.ecus = []
        self.hub.create_subscription(Ecu, '/ecu', self.ecus.append, 10)
        self.observations = []
        self.hub.create_subscription(DockObservation, '/charging_dock/observation',
                                     self.observations.append, 10)
        self.back_plate = Pose2(-2.557)
        self.base = Pose2()
        self.velocity = 0.
        self.vehicle_speed = 0.
        self.vehicle_gear = Ecu.SHIFT_R
        self.front_obstacle = False
        self.scanning = True
        self.rotation_rate = 0.
        self.last_sensor_time = time.monotonic()
        self.hub.create_timer(.04, self.sensors)
        self.client = ActionClient(self.hub, Dock, '/charging_dock/dock')
        self.spin_until(lambda: self.client.server_is_ready())

    def transform(self, parent, child, pose):
        t = TransformStamped()
        t.header.stamp = self.hub.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y = float(pose.x), float(pose.y)
        t.transform.rotation.z = math.sin(pose.yaw / 2)
        t.transform.rotation.w = math.cos(pose.yaw / 2)
        return t

    def sensors(self):
        now = time.monotonic()
        dt, self.last_sensor_time = now - self.last_sensor_time, now
        self.base = integrate(self.base, self.velocity, self.rotation_rate, dt)
        stamp = self.hub.get_clock().now().to_msg()
        pose = self.transform('odom', 'base_link', self.base)
        pose.header.stamp = stamp
        self.dynamic.sendTransform(pose)
        vehicle = VehicleStatus()
        vehicle.header.stamp = stamp
        vehicle.is_autodrive = True
        vehicle.cur_speed = self.vehicle_speed
        vehicle.shift_level = self.vehicle_gear
        self.vehicle_pub.publish(vehicle)
        if self.scanning:
            t = stamp.sec + stamp.nanosec / 1e9
            fake = scan_faces(t, base=self.base, sensor=self.sensor,
                              faces=((self.back_plate, .25),))
            if self.front_obstacle:
                # A measured point outside the current front shell, but
                # inside the forward stopping sweep at 0.04 m/s.
                point = Pose2(.461).relative_to(self.sensor)
                index = round((math.atan2(point.y, point.x) - fake.angle_min)
                              / fake.angle_increment) % len(fake.ranges)
                fake.ranges[index] = math.hypot(point.x, point.y)
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'rear_laser'
            for field in ('angle_min', 'angle_increment', 'range_min', 'range_max', 'ranges'):
                setattr(scan, field, getattr(fake, field))
            scan.angle_max = scan.angle_min + scan.angle_increment * (len(scan.ranges)-1)
            self.scan_pub.publish(scan)

    def spin_until(self, predicate, timeout=8.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.02)
        if not predicate():
            raise AssertionError(
                f'ROS condition timed out; mux={self.mux.dock_active}/'
                f'{self.mux.pending_mode}/{self.mux.stop_latched}, '
                f'owned={self.dock.owned}, vehicle={self.dock.vehicle}, '
                f'core={self.dock.core.state}/{self.dock.core.reason}, '
                f'tf_error={getattr(self.dock, "last_scan_tf_error", "")}, '
                f'p0={self.dock.p0_stable_since}, obs={self.dock.observation}, '
                f'finished={self.dock.finished and self.dock.finished.done()}')

    def start(self, dry_run, timeout_sec=10.):
        goal = Dock.Goal()
        goal.pre_dock_pose.header.frame_id = 'map'
        goal.pre_dock_pose.pose.orientation.w = 1.
        goal.dry_run = dry_run
        goal.timeout_sec = timeout_sec
        future = self.client.send_goal_async(goal)
        self.spin_until(future.done)
        assert future.result().accepted
        return future.result()

    def close(self):
        self.client.destroy()
        self.dock.action.destroy()
        for node in self.nodes:
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()


@pytest.fixture
def harness(tmp_path):
    h = Harness(tmp_path)
    try:
        yield h
    finally:
        h.close()


def test_real_ros_action_dry_run_only_reports_ready_after_measured_handoff(harness):
    h = harness
    handle = h.start(True)
    result = handle.get_result_async()
    h.spin_until(result.done)
    assert result.result().status == 4
    assert result.result().result.dry_run_ready
    assert not result.result().result.success
    assert h.mux.dock_active and h.ecu.docking_mode
    assert h.ecus and all(m.motor == 0 and not m.brake for m in h.ecus)
    assert h.dock.tracker.anchor.x == pytest.approx(-2.557)
    valid = [o for o in h.observations if o.valid]
    assert valid and valid[-1].pose.position.x == pytest.approx(-2.5, abs=.002)
    assert valid[-1].visible_width == pytest.approx(.25, abs=.025)


def test_final_back_plate_window_is_projected_once_before_measured_stop(harness):
    h = harness
    # Begin at an already aligned, near-dock P0 to exercise final ROS handling.
    # Full 2.5m approaches with this geometry run in the numerical plant tests.
    h.dock.values['nominal_dock_distance'] = .46
    h.back_plate = Pose2(-.517)
    handle = h.start(False)
    result = handle.get_result_async()
    h.spin_until(result.done)
    assert result.result().result.success
    assert not result.result().result.charging_confirmed
    assert abs(result.result().result.final_longitudinal_error) < .002
    assert h.ecus and all(m.motor == 0 and not m.brake for m in h.ecus)
    assert h.dock.tracker.anchor.x == pytest.approx(-.517)
    assert h.dock.observation.pose.x == pytest.approx(-.46, abs=.002)


@pytest.mark.parametrize('fault', ['cancel', 'scan_loss', 'manual_control'])
def test_real_ros_motion_faults_stop_and_never_fall_back_to_nav(harness, fault):
    h = harness
    handle = h.start(False)
    result = handle.get_result_async()
    h.spin_until(lambda: any(m.motor > .02 for m in h.ecus))
    nav = Twist()
    nav.linear.x = .4
    h.nav_pub.publish(nav)
    if fault == 'cancel':
        cancel = handle.cancel_goal_async()
        h.spin_until(cancel.done)
    elif fault == 'scan_loss':
        h.scanning = False
    else:
        # Exercise the independent mux latch even if the action node has not
        # received the next vehicle status yet.
        h.mux.autodrive = False
        h.mux._publish_selected()
    h.spin_until(result.done)
    assert not result.result().result.success
    assert result.result().status == (5 if fault == 'cancel' else 6)
    if fault == 'scan_loss':
        assert result.result().result.message == 'dock_observation_reacquire_timeout'
    h.spin_until(lambda: h.ecus[-1].motor == 0 and not h.ecus[-1].brake)
    assert h.mux.dock_active
    assert h.ecus[-1].motor == 0


@pytest.mark.parametrize('stop', ['cancel', 'scan_loss'])
def test_default_unlimited_action_survives_120s_age_and_preserves_stops(harness, stop):
    h = harness
    assert h.dock.core.config.timeout == 0.
    handle = h.start(False, timeout_sec=0.)
    result = handle.get_result_async()
    h.spin_until(lambda: any(m.motor > .02 for m in h.ecus))

    # Age only attempt clocks. Sensor stamps and the executor clock remain
    # live, so this exercises the old 90 s deadline without waiting 90 s or
    # accidentally bypassing freshness and per-phase protections.
    h.dock.requested_at -= 120.
    h.dock.core.started -= 120.
    until = time.monotonic() + .35
    h.spin_until(lambda: time.monotonic() >= until)
    assert not result.done()
    assert h.dock.core.state in ('APPROACH', 'FINAL_APPROACH')
    assert h.dock.last_output.velocity < 0.
    assert not h.mux.stop_latched

    if stop == 'cancel':
        cancel = handle.cancel_goal_async()
        h.spin_until(cancel.done)
    else:
        h.scanning = False
    h.spin_until(result.done)
    assert not result.result().result.success
    assert result.result().status == (5 if stop == 'cancel' else 6)
    if stop == 'scan_loss':
        assert result.result().result.message == 'dock_observation_reacquire_timeout'
    h.spin_until(lambda: h.ecus[-1].motor == 0. and not h.ecus[-1].brake)


def test_explicit_positive_action_timeout_remains_enabled(harness):
    h = harness
    assert h.dock.core.config.timeout == 0.
    handle = h.start(False, timeout_sec=10.)
    result = handle.get_result_async()
    h.spin_until(lambda: any(m.motor > .02 for m in h.ecus))
    h.dock.requested_at -= 11.
    h.spin_until(result.done)
    assert result.result().status == 6
    assert result.result().result.message == 'action_timeout'
    h.spin_until(lambda: h.ecus[-1].motor == 0. and not h.ecus[-1].brake)


@pytest.mark.parametrize('stop', ['cancel', 'scan_loss'])
def test_default_unlimited_locked_final_survives_120s_age_and_preserves_stops(harness, stop):
    h = harness
    assert h.dock.core.config.timeout == 0.
    assert h.dock.core.config.locked_final_timeout == 0.
    h.dock.values['nominal_dock_distance'] = 1.
    h.back_plate = Pose2(-1.057)
    handle = h.start(False, timeout_sec=0.)
    result = handle.get_result_async()
    h.spin_until(lambda: h.dock.core.state == 'LOCKED_FINAL_APPROACH')
    assert h.dock.core.locked_dock_pose_local is not None

    # Age only attempt and phase clocks. External odometry, CAN feedback,
    # and scans continue with fresh timestamps and measured slow motion.
    h.dock.requested_at -= 120.
    h.dock.core.started -= 120.
    h.dock.core.locked_final_started -= 120.
    h.velocity = -.005
    h.vehicle_speed = .018
    until = time.monotonic() + .4
    h.spin_until(lambda: time.monotonic() >= until)
    assert not result.done()
    assert h.dock.core.state == 'LOCKED_FINAL_APPROACH'
    assert h.dock.last_output.velocity < 0.
    assert h.dock.last_longitudinal_speed < -.004
    assert h.base.x < -.001
    assert not h.mux.stop_latched

    h.velocity = 0.
    h.vehicle_speed = 0.
    if stop == 'cancel':
        cancel = handle.cancel_goal_async()
        h.spin_until(cancel.done)
    else:
        h.scanning = False
    h.spin_until(result.done)
    assert not result.result().result.success
    assert result.result().status == (5 if stop == 'cancel' else 6)
    if stop == 'scan_loss':
        assert result.result().result.message == 'dock_observation_invalid'
    h.spin_until(lambda: h.ecus[-1].motor == 0. and not h.ecus[-1].brake)


def test_can_zero_speed_does_not_allow_handoff_while_tf_is_rotating(harness):
    h = harness
    h.rotation_rate = .03
    handle = h.start(True)
    deadline = time.monotonic() + 1.2
    h.spin_until(lambda: time.monotonic() >= deadline)
    assert h.mux.pending_mode is True
    assert not h.mux.dock_active
    assert not h.ecu.docking_mode
    assert all(m.motor == 0 for m in h.ecus)
    cancel = handle.cancel_goal_async()
    h.spin_until(cancel.done)
    result = handle.get_result_async()
    h.spin_until(result.done)
    assert result.result().status == 5


def test_actual_forward_roll_allows_reverse_control_with_external_odometry(harness):
    h = harness
    handle = h.start(False)
    h.spin_until(lambda: any(m.motor > .02 for m in h.ecus))
    h.velocity = .025
    h.spin_until(lambda: h.dock.last_longitudinal_speed > .02)
    until = time.monotonic() + .24
    h.spin_until(lambda: time.monotonic() >= until)
    assert h.base.x > .005
    assert h.dock.core.state in ('APPROACH', 'FINAL_APPROACH')
    assert not h.mux.stop_latched
    assert h.dock.last_output.velocity < 0.
    assert all(m.shift != Ecu.SHIFT_D for m in h.ecus)
    h.velocity = 0.
    cancel = handle.cancel_goal_async()
    h.spin_until(cancel.done)
    result = handle.get_result_async()
    h.spin_until(result.done)
    assert result.result().status == 5


def test_neutral_coast_with_nonzero_main_speed_waits_then_locks(harness):
    h = harness
    h.dock.values['nominal_dock_distance'] = 1.
    h.back_plate = Pose2(-1.057)
    handle = h.start(False, timeout_sec=0.)
    h.spin_until(lambda: h.dock.core.state == 'LOCK_VERIFY')
    coast_start = len(h.ecus)
    # The formal package receives independent odometry. Gear N and the
    # unsigned main speed are not used to infer the direction of TF motion.
    h.vehicle_gear = Ecu.SHIFT_N
    h.velocity = -.06
    h.vehicle_speed = .2
    h.spin_until(lambda: h.dock.last_longitudinal_speed < -.05)
    until = time.monotonic() + .16
    h.spin_until(lambda: time.monotonic() >= until)
    assert h.dock.core.state == 'LOCK_VERIFY'
    assert h.dock.core.reason == 'waiting_for_measured_stop_before_lock'
    assert h.dock.core.locked_dock_pose_local is None
    assert h.dock.last_output.velocity == 0.
    assert not h.mux.stop_latched
    assert h.ecus[coast_start:]
    assert all(m.motor == 0. and not m.brake and m.shift == Ecu.SHIFT_N
               for m in h.ecus[coast_start:])

    # The main speed must also settle before stationary scans can lock the
    # target, even if external odometry already reports no displacement.
    h.velocity = 0.
    h.spin_until(lambda: abs(h.dock.last_longitudinal_speed) < .001)
    assert h.dock.core.state == 'LOCK_VERIFY'
    assert h.dock.core.locked_dock_pose_local is None
    h.vehicle_speed = 0.
    h.spin_until(lambda: h.dock.core.state == 'LOCKED_FINAL_APPROACH')
    assert .96 < h.dock.core.locked_distance < 1.
    assert h.dock.core.locked_remaining_distance == pytest.approx(
        h.dock.core.locked_distance - .46, abs=.002)
    assert not h.mux.stop_latched

    cancel = handle.cancel_goal_async()
    h.spin_until(cancel.done)
    result = handle.get_result_async()
    h.spin_until(result.done)
    assert result.result().status == 5


def test_first_live_scan_alignment_stops_before_formal_lock_confirmation(harness):
    h = harness
    h.dock.values['nominal_dock_distance'] = 1.
    h.back_plate = Pose2(-1.057, .02)
    states = []
    h.hub.create_subscription(String, '/charging_dock/state',
                              lambda msg: states.append(msg.data.split()[0]), 10)
    handle = h.start(False, timeout_sec=0.)
    h.spin_until(lambda: h.dock.last_output is not None
                 and h.dock.last_output.velocity < 0.)
    assert h.dock.get_parameter('lock_stop_on_first_alignment').value is True
    assert h.dock.get_parameter('lock_alignment_duration').value == .5
    until = time.monotonic() + .65
    h.spin_until(lambda: time.monotonic() >= until)
    assert 'LOCK_VERIFY' not in states

    # Use actual finite-face scans, including the production tracker filter.
    # The first valid aligned estimate requests a stop immediately.
    h.back_plate = Pose2(-1.057)
    h.spin_until(lambda: h.dock.core.state == 'LOCK_VERIFY')
    assert h.dock.last_output.velocity == 0.
    assert h.dock.core.locked_dock_pose_local is None
    until = time.monotonic() + .20
    h.spin_until(lambda: time.monotonic() >= until)
    assert h.dock.core.state == 'LOCK_VERIFY'
    assert h.dock.core.locked_dock_pose_local is None
    h.spin_until(lambda: h.dock.core.state == 'LOCKED_FINAL_APPROACH')
    assert h.dock.core.locked_dock_pose_local is not None
    assert not h.mux.stop_latched
    assert all(message.shift != Ecu.SHIFT_D for message in h.ecus)

    cancel = handle.cancel_goal_async()
    h.spin_until(cancel.done)
    result = handle.get_result_async()
    h.spin_until(result.done)
    assert result.result().status == 5


def test_actual_forward_roll_checks_front_stopping_sweep_during_reverse_command(harness):
    h = harness
    handle = h.start(False)
    h.spin_until(lambda: any(m.motor > .02 for m in h.ecus))
    # TF provides direction. Retain the larger measured CAN magnitude for
    # stopping distance instead of assuming its reverse gear determines sign.
    h.velocity = .025
    h.vehicle_speed = .144  # 0.04 m/s, larger than the TF estimate.
    h.front_obstacle = True
    h.spin_until(lambda: h.dock.core.state == 'STOPPING')
    assert h.dock.core.reason == 'swept_footprint_obstacle'
    assert h.dock.last_longitudinal_speed > 0.
    assert h.dock.last_output.velocity == 0.
    h.velocity = 0.
    h.vehicle_speed = 0.
    result = handle.get_result_async()
    h.spin_until(result.done)
    assert result.result().result.message == 'swept_footprint_obstacle'
