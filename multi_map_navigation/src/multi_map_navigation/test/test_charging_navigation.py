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

"""Charging waypoint lifecycle regressions without starting navigation processes."""

from pathlib import Path
from types import SimpleNamespace as NS
import threading
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Time
import pytest
import yaml
import multi_map_navigation.charging_navigation as charging_navigation
from multi_map_navigation.navigation_manager import NavigationManager
from multi_map_navigation.route_planner import RoutePlannerNode
from multi_map_navigation_msgs.msg import Waypoint


class Future:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value

    def done(self):
        return True


def manager():
    node = NavigationManager.__new__(NavigationManager)
    node.state_lock = threading.RLock()
    node.docking_generation = 1
    node.docking_hold = True
    node.docking_in_progress = True
    node.docking_complete = False
    node.docking_handle = None
    node.current_waypoint_index = 0
    node.current_map = 'test'
    point = Waypoint()
    point.type = 3
    node.waypoint_list = [point]
    node.stack_fault_active = False
    node.docking_release_deadline = None
    node.docking_release_future = None
    node.get_logger = lambda: NS(info=lambda _: None, error=lambda _: None,
                                 warning=lambda _: None)
    node.states = []
    node.publish_robot_state = node.states.append
    return node


def docking_start_manager(timeout):
    node = manager()
    node.docking_in_progress = False
    parameters = {'docking_timeout': timeout, 'docking_dry_run': False,
                  'docking_require_charging': False}
    node.get_parameter = lambda name: NS(value=parameters[name])
    node.get_clock = lambda: NS(now=lambda: NS(to_msg=Time))
    goals, stops = [], []

    def send_goal(goal):
        goals.append(goal)
        return NS(add_done_callback=lambda _: None)

    node.docking_client = NS(
        server_is_ready=lambda: True, send_goal_async=send_goal)
    node.docking_stop_pub = NS(publish=lambda message: stops.append(message.data))
    return node, goals, stops


def test_docking_default_and_shipped_configuration_disable_total_timeout(monkeypatch):
    node = manager()
    defaults = {}
    node.declare_parameter = lambda name, value: defaults.setdefault(name, value)
    node.create_client = lambda *_: NS()
    node.create_publisher = lambda *_: NS()
    node.create_service = lambda *_: NS()
    node.create_timer = lambda *_: NS()
    monkeypatch.setattr(charging_navigation, 'ActionClient', lambda *_: NS())
    node.init_charging_navigation()
    assert defaults['docking_timeout'] == 0.0
    config_path = Path(__file__).resolve().parents[1] / 'config/navigation_config.yaml'
    config = yaml.safe_load(config_path.read_text())
    assert config['navigation_manager']['ros__parameters']['docking_timeout'] == 0.0


def test_disabled_docking_timeout_keeps_action_running_past_old_limit(monkeypatch):
    now = [1000.]
    monkeypatch.setattr(charging_navigation.time, 'monotonic', lambda: now[0])
    node, goals, stops = docking_start_manager(0.)
    node.start_docking(node.waypoint_list[0])
    assert len(goals) == 1
    assert goals[0].timeout_sec == 0.
    assert node.docking_deadline is None
    for elapsed in (90., 100., 1000.):
        now[0] = 1000. + elapsed
        node.check_docking_timeout()
        assert node.docking_in_progress
        assert node.states == ['docking']
        assert not stops


def test_explicit_positive_docking_timeout_still_stops_at_its_deadline(monkeypatch):
    now = [1000.]
    monkeypatch.setattr(charging_navigation.time, 'monotonic', lambda: now[0])
    node, goals, stops = docking_start_manager(30.)
    node.start_docking(node.waypoint_list[0])
    assert goals[0].timeout_sec == 30.
    assert node.docking_deadline == 1040.
    now[0] = 1039.
    node.check_docking_timeout()
    assert node.docking_in_progress
    assert not stops
    now[0] = 1041.
    node.check_docking_timeout()
    assert not node.docking_in_progress
    assert node.docking_hold
    assert node.current_waypoint_index == 0
    assert node.states == ['docking', 'docking_failed']
    assert stops == [True]


@pytest.mark.parametrize('timeout', [
    -1., float('nan'), float('inf'), -float('inf'), None, 'invalid',
])
def test_invalid_docking_timeout_fails_without_sending_or_advancing(timeout):
    node, goals, stops = docking_start_manager(timeout)
    node.start_docking(node.waypoint_list[0])
    assert not goals
    assert not stops
    assert not node.docking_in_progress
    assert node.docking_deadline is None
    assert node.docking_hold
    assert node.current_waypoint_index == 0
    assert node.states == ['docking_failed']


def test_final_charge_waypoint_must_dock_before_route_completion():
    node = manager()
    node.docking_hold = False
    started = []
    node.start_docking = started.append
    node.complete_navigation = lambda: pytest.fail('P0 is not docking completion')
    node.on_goal_reached()
    assert started == node.waypoint_list
    assert node.current_waypoint_index == 0


@pytest.mark.parametrize('success,ready,status,state', [
    (True, False, 4, 'docked'), (False, True, 4, 'docking_ready'),
    (False, False, 6, 'docking_failed'), (False, False, 5, 'docking_failed'),
])
def test_result_keeps_route_and_localization_until_explicit_release(success, ready, status, state):
    node = manager()
    result = NS(success=success, dry_run_ready=ready, message='test')
    node.docking_result(Future(NS(status=status, result=result)), 1)
    assert node.current_waypoint_index == 0
    assert node.docking_hold
    assert node.docking_complete == success
    assert node.states == [state]
    response = node.release_docking(None, NS())
    assert response.success == success


def test_stale_result_cannot_complete_new_or_faulted_action():
    node = manager()
    node.docking_generation = 2
    node.docking_result(Future(NS(status=4, result=NS(success=True))), 1)
    assert not node.docking_complete
    assert node.states == []


def test_late_goal_response_is_canceled_after_fault():
    node = manager()
    node.docking_generation = 2
    canceled = []
    handle = NS(accepted=True, cancel_goal_async=lambda: canceled.append(True))
    node.docking_response(Future(handle), 1)
    assert canceled == [True]


def test_release_advances_once_only_after_mux_confirmation():
    node = manager()
    node.docking_complete = True
    node.docking_in_progress = False
    node.docking_release_deadline = float('inf')
    node.docking_release_future = Future(NS(success=False))
    node.docking_release_client = NS(service_is_ready=lambda: False)
    resumed = []
    node.navigate_to_next_waypoint = lambda: resumed.append(node.current_waypoint_index)
    node.poll_docking_release()
    assert node.current_waypoint_index == 0
    node.docking_release_future = Future(NS(success=True))
    node.poll_docking_release()
    assert node.current_waypoint_index == 1
    assert not resumed
    node.resume_after_docking_release()
    node.resume_after_docking_release()
    assert resumed == [1]


def test_fault_latches_independent_stop_and_cancels_docking_handle():
    node = manager()
    stops, cancels = [], []
    node.docking_stop_pub = NS(publish=lambda m: stops.append(m.data))
    node.docking_handle = NS(cancel_goal_async=lambda: cancels.append(True))
    node.stop_docking_for_fault()
    assert stops == [True]
    assert cancels == [True]
    assert node.docking_generation == 2


@pytest.mark.parametrize('state', ['docking', 'docked', 'docking_ready'])
def test_expected_docking_stop_does_not_request_remote_driving(state):
    from multi_map_navigation.status_reporter import StatusReporter
    node = StatusReporter.__new__(StatusReporter)
    node.robot_status = {}
    node.robot_state_callback(NS(data=state))
    assert node.robot_status['task_status'] == 1
    # Returns before timers or external request paths are needed.
    node.check_vehicle_problem()


@pytest.mark.parametrize('state', ['docking', 'docked', 'docking_ready', 'docking_failed'])
def test_route_cannot_be_overwritten_during_docking_hold(state):
    node = RoutePlannerNode.__new__(RoutePlannerNode)
    node.state_lock = threading.Lock()
    node.robot_state_callback(NS(data=state))
    assert node.is_running


def test_all_installed_trees_select_an_explicit_goal_checker():
    root = next(p for p in Path(__file__).resolve().parents
                if (p / 'nav2/src/nav2/config').is_dir())
    config_dir = root / 'nav2/src/nav2/config'
    config = yaml.safe_load((config_dir / 'nav2_real_params.yaml').read_text())
    checkers = config['controller_server']['ros__parameters']['goal_checker_plugins']
    for name in ('general_nav_to_pose', 'general_nav_through_poses', 'charging_nav_to_pose'):
        tree = ET.parse(config_dir / (name + '.xml'))
        follows = tree.findall('.//FollowPath')
        assert follows
        expected = 'charging_goal_checker' if name.startswith(
            'charging') else 'general_goal_checker'
        for follow in follows:
            assert follow.attrib['goal_checker_id'] == expected
            assert expected in checkers
