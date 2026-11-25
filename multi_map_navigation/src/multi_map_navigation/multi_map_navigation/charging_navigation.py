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

"""Asynchronous handoff from a charging P0 waypoint to relative docking."""

import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from charging_dock_msgs.action import Dock
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger


class ChargingNavigation:
    """Keep localization and the route alive until docking is explicitly released."""

    def init_charging_navigation(self):
        self.declare_parameter('docking_dry_run', True)
        self.declare_parameter('docking_require_charging', False)
        self.declare_parameter('docking_timeout', 0.0)
        self.docking_client = ActionClient(self, Dock, '/charging_dock/dock')
        self.docking_release_client = self.create_client(
            SetBool, '/command_mux/set_dock_active')
        self.docking_stop_pub = self.create_publisher(
            Bool, '/charging_dock/emergency_stop', 10)
        self.docking_generation = 0
        self.docking_hold = False
        self.docking_complete = False
        self.docking_in_progress = False
        self.docking_handle = None
        self.docking_deadline = None
        self.docking_release_future = None
        self.docking_release_deadline = None
        self.create_service(Trigger, '~/retry_docking', self.retry_docking)
        self.create_service(Trigger, '~/release_docking', self.release_docking)
        self.create_timer(0.25, self.poll_docking_release)
        self.create_timer(0.25, self.resume_after_docking_release)
        self.create_timer(0.25, self.check_docking_timeout)

    def navigation_behavior_tree(self, waypoint):
        filename = ('charging_nav_to_pose.xml' if self.is_charge_point(waypoint)
                    else 'general_nav_to_pose.xml')
        return os.path.join(get_package_share_directory('nav2'), 'config', filename)

    def start_docking(self, waypoint):
        with self.state_lock:
            if getattr(self, 'docking_in_progress', False):
                return
            self.docking_hold = True
            self.docking_complete = False
            self.docking_in_progress = True
            self.docking_generation += 1
            generation = self.docking_generation
            self.current_goal_handle = None
            self.pending_replan_endpoints = None
            self.docking_deadline = None
        try:
            timeout = float(self.get_parameter('docking_timeout').value)
        except (TypeError, ValueError, OverflowError):
            timeout = math.nan
        if not math.isfinite(timeout) or timeout < 0:
            self.docking_failed(generation, 'docking_timeout 必须为有限非负数；0 表示不限制总时长')
            return
        if not self.docking_client.server_is_ready():
            self.docking_failed(generation, '对接 Action 不可用；保持当前航点')
            return
        goal = Dock.Goal()
        goal.pre_dock_pose = PoseStamped()
        goal.pre_dock_pose.header.frame_id = 'map'
        goal.pre_dock_pose.header.stamp = self.get_clock().now().to_msg()
        goal.pre_dock_pose.pose.position.x = waypoint.x
        goal.pre_dock_pose.pose.position.y = waypoint.y
        goal.pre_dock_pose.pose.orientation.z = math.sin(waypoint.yaw / 2)
        goal.pre_dock_pose.pose.orientation.w = math.cos(waypoint.yaw / 2)
        goal.dock_id = 0
        goal.dry_run = self.get_parameter('docking_dry_run').value
        goal.require_charging = self.get_parameter('docking_require_charging').value
        goal.timeout_sec = timeout
        # Zero disables both the controller's total timer and this outer deadline.
        # A positive, explicitly configured limit retains response grace time.
        if timeout > 0:
            self.docking_deadline = time.monotonic() + timeout + 10.
        self.publish_robot_state('docking')
        try:
            future = self.docking_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.docking_response(f, generation))
        except Exception as exc:
            self.docking_failed(generation, str(exc))

    def docking_response(self, future, generation):
        try:
            handle = future.result()
            with self.state_lock:
                if generation != self.docking_generation:
                    if handle and handle.accepted:
                        handle.cancel_goal_async()
                    return
                if handle is None or not handle.accepted:
                    self.docking_failed(generation, '对接目标被拒绝')
                    return
                self.docking_handle = handle
            handle.get_result_async().add_done_callback(
                lambda f: self.docking_result(f, generation))
        except Exception as exc:
            self.docking_failed(generation, str(exc))

    def docking_result(self, future, generation):
        try:
            wrapper = future.result()
            with self.state_lock:
                if generation != self.docking_generation:
                    return
                self.docking_in_progress = False
                self.docking_deadline = None
                self.docking_handle = None
                result = wrapper.result
                if wrapper.status == 4 and result.success:
                    self.docking_complete = True
                    self.publish_robot_state('docked')
                    self.get_logger().info('对接完成，保持停车和定位；手动脱离后显式释放')
                elif wrapper.status == 4 and result.dry_run_ready:
                    self.publish_robot_state('docking_ready')
                    self.get_logger().info('对接预检通过；dry-run 不推进航点')
                else:
                    self.docking_failed(generation, result.message)
        except Exception as exc:
            self.docking_failed(generation, str(exc))

    def docking_failed(self, generation, reason):
        with self.state_lock:
            if generation != self.docking_generation:
                return
            self.docking_in_progress = False
            self.docking_deadline = None
            self.docking_complete = False
            self.docking_handle = None
        self.get_logger().error(f'对接未完成，保持航点和停车: {reason}')
        self.publish_robot_state('docking_failed')

    def retry_docking(self, _request, response):
        with self.state_lock:
            valid = (self.docking_hold and not self.docking_in_progress
                     and not self.docking_complete and not self.stack_fault_active
                     and self.docking_release_deadline is None)
            response.success = bool(valid)
            response.message = '重新执行当前充电点' if valid else '当前状态不能重试对接'
            if valid:
                self.start_docking(self.waypoint_list[self.current_waypoint_index])
        return response

    def release_docking(self, _request, response):
        with self.state_lock:
            valid = (self.docking_hold and self.docking_complete
                     and not self.docking_in_progress and not self.stack_fault_active)
            response.success = bool(valid)
            response.message = ('已请求释放；车辆须已安全脱离充电桩，实测停车后切换控制权'
                                if valid else '仅完成实车对接后可释放；预检和失败不推进航点')
            if valid and self.docking_release_deadline is None:
                self.docking_release_deadline = time.monotonic() + 8.0
        return response

    def poll_docking_release(self):
        with self.state_lock:
            if self.docking_release_deadline is None:
                return
            if self.stack_fault_active or time.monotonic() > self.docking_release_deadline:
                self.docking_release_deadline = None
                self.docking_release_future = None
                self.get_logger().error('控制权释放未确认，继续保持当前航点')
                return
            future = self.docking_release_future
            if future is not None:
                if not future.done():
                    return
                self.docking_release_future = None
                try:
                    confirmed = future.result().success
                except Exception:
                    confirmed = False
                if confirmed:
                    self.docking_release_deadline = None
                    self.docking_hold = self.docking_complete = False
                    self._advance_current_waypoint()
                    # Stack service calls can block: do not hold state_lock.
                    self.docking_resume_pending = True
                    return
            if self.docking_release_client.service_is_ready():
                request = SetBool.Request()
                request.data = False
                self.docking_release_future = self.docking_release_client.call_async(request)

    def resume_after_docking_release(self):
        with self.state_lock:
            resume = getattr(self, 'docking_resume_pending', False)
            self.docking_resume_pending = False
            resume = resume and not self.stack_fault_active
        if resume:
            self.navigate_to_next_waypoint()

    def stop_docking_for_fault(self):
        """Independent mux stop remains effective even if the controller dies."""
        if not hasattr(self, 'docking_stop_pub'):
            return
        with self.state_lock:
            self.docking_generation += 1
            handle = self.docking_handle
            self.docking_handle = None
            self.docking_in_progress = False
            self.docking_release_deadline = None
            self.docking_release_future = None
        self.docking_stop_pub.publish(Bool(data=True))
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'取消对接 Action 失败: {exc}')

    def check_docking_timeout(self):
        with self.state_lock:
            expired = (self.docking_in_progress and self.docking_deadline is not None
                       and time.monotonic() > self.docking_deadline)
            if not expired:
                return
            self.stop_docking_for_fault()
            self.docking_failed(self.docking_generation, '对接 Action 响应超时，已独立锁定停车')
