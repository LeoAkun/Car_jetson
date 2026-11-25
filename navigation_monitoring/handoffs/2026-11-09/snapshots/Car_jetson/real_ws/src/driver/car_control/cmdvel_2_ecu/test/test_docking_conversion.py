"""Exercise conversion without a ROS context, node, executor, or publisher."""

import ast
import inspect
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from rclpy.node import Node
import yaml

from cmdvel_2_ecu.convert import CmdVelToEcu, MPS_TO_KMH


@pytest.fixture(autouse=True)
def prohibit_ros_initialization(monkeypatch):
    def prohibited(*_args, **_kwargs):
        raise AssertionError('Conversion tests must not initialize ROS or create a node')

    monkeypatch.setattr(rclpy, 'init', prohibited)
    monkeypatch.setattr(Node, '__init__', prohibited)


def _converter(docking_mode=False):
    # Call the conversion method with plain data instead of constructing its Node.
    clock = SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123, nanosec=456)))
    return SimpleNamespace(
        get_clock=lambda: clock,
        L=0.5,
        MAX_STEER_DEG=30.0,
        LOW_SPEED_THRESH=0.8 / 3.6,
        STOP_THRESH=0.02 / 3.6,
        Vel_Offeset=0.03 / 3.6,
        OMEGA_DEADZONE=0.020,
        OMEGA_FULL_STEER=5.0,
        rear_wheel_flag=True,
        navigation_max_speed=2.0 / 3.6,
        docking_mode=docking_mode,
        docking_max_speed=0.10,
        docking_max_steer_deg=8.0,
        docking_stop_threshold=0.001,
        docking_ecu_speed_scale=3.6,
    )


def _convert(converter, velocity, omega=0.0):
    msg = Twist()
    msg.linear.x = velocity
    msg.angular.z = omega
    return CmdVelToEcu.cmdvel_2_ecu(converter, msg)


def _declared_numeric_default(name):
    # Read constructor defaults without constructing a Node or executing __init__.
    tree = ast.parse(Path(inspect.getfile(CmdVelToEcu)).read_text())

    def number(expression):
        if isinstance(expression, ast.Constant) and type(expression.value) in (int, float):
            return expression.value
        if isinstance(expression, ast.Name) and expression.id == 'MPS_TO_KMH':
            return MPS_TO_KMH
        if isinstance(expression, ast.BinOp) and isinstance(expression.op, ast.Div):
            return number(expression.left) / number(expression.right)
        raise AssertionError(f'Unexpected numeric default expression: {ast.dump(expression)}')

    for call in ast.walk(tree):
        if (isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
                and call.func.attr == 'declare_parameter' and len(call.args) >= 2
                and isinstance(call.args[0], ast.Constant) and call.args[0].value == name):
            return number(call.args[1])
    raise AssertionError(f'No declared default for {name}')


def test_bare_node_default_retains_point_three_kmh_floor_without_yaml():
    assert MPS_TO_KMH == 3.6
    converter = _converter()
    converter.LOW_SPEED_THRESH = _declared_numeric_default('LOW_SPEED_THRESH')
    assert converter.LOW_SPEED_THRESH == pytest.approx(0.3 / 3.6)
    ecu = _convert(converter, 0.03 / 3.6)
    assert ecu.motor == pytest.approx(0.3, abs=1e-6)


@pytest.mark.parametrize('velocity, expected_kmh', [(0.9 / 3.6, 0.93), (-0.9 / 3.6, 0.90)])
def test_navigation_converts_si_speed_and_preserves_kmh_offset(velocity, expected_kmh):
    ecu = _convert(_converter(), velocity)
    assert ecu.motor == pytest.approx(expected_kmh, abs=1e-6)
    assert ecu.shift == (ecu.SHIFT_D if velocity > 0 else ecu.SHIFT_R)
    assert not ecu.brake


@pytest.mark.parametrize('direction', [-1, 1])
def test_navigation_retains_point_eight_kmh_low_speed_compensation(direction):
    converter = _converter()
    ecu = _convert(converter, direction * 0.03 / 3.6)
    assert ecu.motor == pytest.approx(0.8, abs=1e-6)
    assert ecu.shift == (ecu.SHIFT_D if direction > 0 else ecu.SHIFT_R)
    assert not ecu.brake


@pytest.mark.parametrize('docking_mode', [False, True])
@pytest.mark.parametrize('fraction', [-1.0, -0.5, -0.0, 0.0, 0.5, 1.0])
def test_stop_deadband_includes_both_boundaries_and_selects_neutral(docking_mode, fraction):
    converter = _converter(docking_mode)
    threshold = (converter.docking_stop_threshold
                 if docking_mode else converter.STOP_THRESH)
    ecu = _convert(converter, fraction * threshold)
    assert ecu.motor == 0.0
    assert ecu.steer == 0.0
    assert ecu.shift == ecu.SHIFT_N
    assert ecu.brake


@pytest.mark.parametrize('direction', [-1, 1])
def test_navigation_moves_immediately_above_stop_threshold_in_both_directions(direction):
    converter = _converter()
    velocity = direction * math.nextafter(converter.STOP_THRESH, math.inf)
    ecu = _convert(converter, velocity)
    assert ecu.motor == pytest.approx(0.8, abs=1e-6)
    assert ecu.shift == (ecu.SHIFT_D if direction > 0 else ecu.SHIFT_R)
    assert not ecu.brake


@pytest.mark.parametrize('velocity', [-100.0, -2.0 / 3.6, 2.0 / 3.6, 100.0])
def test_navigation_caps_speed_after_compensation_at_two_kmh(velocity):
    ecu = _convert(_converter(), velocity)
    assert ecu.motor == pytest.approx(2.0, abs=1e-6)
    assert ecu.shift == (ecu.SHIFT_D if velocity > 0 else ecu.SHIFT_R)
    assert not ecu.brake


@pytest.mark.parametrize('velocity', [-0.25, -0.10, 0.10, 0.25])
def test_ackermann_steering_uses_si_command_before_motor_floor_and_offset(velocity):
    converter = _converter()
    omega = 0.02
    ecu = _convert(converter, velocity, omega)
    expected = math.degrees(math.atan(converter.L * -omega / velocity))
    assert ecu.steer == pytest.approx(expected, abs=1e-6)


@pytest.mark.parametrize('docking_mode', [False, True])
@pytest.mark.parametrize('direction', [-1, 1])
def test_steering_uses_clamped_command_speed(docking_mode, direction):
    converter = _converter(docking_mode)
    limit = (converter.docking_max_speed if docking_mode
             else converter.navigation_max_speed)
    omega = 0.01
    ecu = _convert(converter, direction * 100.0, omega)
    expected = math.degrees(math.atan(converter.L * -omega / (direction * limit)))
    assert ecu.steer == pytest.approx(expected, abs=1e-6)


@pytest.mark.parametrize('docking_mode', [False, True])
@pytest.mark.parametrize('omega', [-2.0, 2.0])
def test_existing_steer_limits_are_retained(docking_mode, omega):
    converter = _converter(docking_mode)
    ecu = _convert(converter, 0.01, omega)
    limit = (converter.docking_max_steer_deg if docking_mode
             else converter.MAX_STEER_DEG)
    assert ecu.steer == pytest.approx(-math.copysign(limit, omega), abs=1e-6)


@pytest.mark.parametrize('docking_mode', [False, True])
@pytest.mark.parametrize('invalid', [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize('axis', ['velocity', 'omega'])
def test_nonfinite_input_cannot_generate_a_motion_command(docking_mode, invalid, axis):
    velocity, omega = (invalid, 0.1) if axis == 'velocity' else (0.1, invalid)
    ecu = _convert(_converter(docking_mode), velocity, omega)
    assert ecu.motor == 0.0
    assert ecu.steer == 0.0
    assert ecu.shift == ecu.SHIFT_N
    assert ecu.brake


@pytest.mark.parametrize('velocity', [-0.03, 0.03])
def test_docking_preserves_centimetre_speed_without_navigation_compensation(velocity):
    ecu = _convert(_converter(True), velocity)
    assert ecu.motor == pytest.approx(0.03 * 3.6, abs=1e-6)
    # JD01 setSpeed() rounds positive km/h * 10 to a CAN integer. This
    # centimetre-speed command must survive that real protocol quantization.
    assert math.floor(ecu.motor * 10 + 0.5) == 1
    assert ecu.shift == (ecu.SHIFT_D if velocity > 0 else ecu.SHIFT_R)
    assert not ecu.brake


@pytest.mark.parametrize('velocity', [-1.0, -0.1, 0.1, 1.0])
def test_docking_retains_independent_si_speed_limit(velocity):
    ecu = _convert(_converter(True), velocity)
    assert ecu.motor == pytest.approx(0.1 * 3.6, abs=1e-6)
    assert ecu.shift == (ecu.SHIFT_D if velocity > 0 else ecu.SHIFT_R)


def test_real_navigation_configuration_uses_two_kmh_limit_and_retains_other_calibration():
    project = next(
        parent for parent in Path(__file__).resolve().parents
        if (parent / 'nav2/src/nav2/config/nav2_real_params.yaml').is_file())
    nav_config = yaml.safe_load(
        (project / 'nav2/src/nav2/config/nav2_real_params.yaml').read_text())
    config_directory = Path(__file__).resolve().parents[1] / 'config'
    controller = nav_config['controller_server']['ros__parameters']['FollowPath']
    smoother = nav_config['velocity_smoother']['ros__parameters']
    assert _declared_numeric_default('navigation_max_speed') == pytest.approx(2.0 / 3.6)

    for filename, historical_stop in [('config.yaml', 0.02), ('docking_config.yaml', 0.01)]:
        parameters = yaml.safe_load(
            (config_directory / filename).read_text())['/cmdvel_to_ecu']['ros__parameters']
        assert parameters['navigation_max_speed'] == pytest.approx(2.0 / 3.6)
        assert parameters['LOW_SPEED_THRESH'] == pytest.approx(0.8 / 3.6)
        assert parameters['STOP_THRESH'] == pytest.approx(historical_stop / 3.6)
        assert parameters['Vel_Offeset'] == pytest.approx(0.03 / 3.6)
    assert controller['vx_max'] == pytest.approx(2.0 / 3.6)
    assert controller['vx_min'] == pytest.approx(-2.0 / 3.6)
    assert controller['vx_std'] == pytest.approx(1.5 / 3.6)
    assert controller['ax_max'] == pytest.approx(2.5 / 3.6)
    assert controller['ax_min'] == pytest.approx(-2.5 / 3.6)
    assert controller['wz_max'] == 5.0
    assert controller['wz_std'] == 0.5
    assert smoother['max_velocity'] == pytest.approx([2.0 / 3.6, 0.0, 2.0])
    assert smoother['min_velocity'] == pytest.approx([-2.0 / 3.6, 0.0, -2.0])
    assert smoother['max_accel'] == pytest.approx([2.5 / 3.6, 0.0, 3.2])
    assert smoother['max_decel'] == pytest.approx([-2.5 / 3.6, 0.0, -3.2])
