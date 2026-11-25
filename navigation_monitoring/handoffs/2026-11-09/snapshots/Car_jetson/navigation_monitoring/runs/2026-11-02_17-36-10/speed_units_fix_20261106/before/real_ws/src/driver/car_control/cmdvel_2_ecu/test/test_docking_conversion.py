import math

import rclpy
from geometry_msgs.msg import Twist

from cmdvel_2_ecu.convert import CmdVelToEcu


def _node(docking_mode):
    rclpy.init()
    node = CmdVelToEcu()
    node.docking_mode = docking_mode
    node.STOP_THRESH = 0.01
    return node


def test_docking_mode_preserves_centimetre_speed():
    node = _node(True)
    try:
        msg = Twist()
        msg.linear.x = -0.03
        ecu = node.cmdvel_2_ecu(msg)
        assert math.isclose(ecu.motor, 0.03 * 3.6, abs_tol=1e-6)
        # Msg.cpp::setSpeed rounds to a 0.1 km/h CAN integer. Previously .03
        # went to zero; the converted docking command must survive encoding.
        assert round(ecu.motor * 10) >= 1
        assert ecu.shift == ecu.SHIFT_R
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_normal_mode_preserves_production_speed_calibration():
    node = _node(False)
    node.STOP_THRESH = 0.02
    node.LOW_SPEED_THRESH = 0.8
    node.Vel_Offeset = 0.03
    try:
        msg = Twist()
        msg.linear.x = 0.9
        ecu = node.cmdvel_2_ecu(msg)
        assert math.isclose(ecu.motor, 0.93, abs_tol=1e-6)

        msg.linear.x = -0.9
        ecu = node.cmdvel_2_ecu(msg)
        assert math.isclose(ecu.motor, 0.9, abs_tol=1e-6)

        msg.linear.x = 0.01
        ecu = node.cmdvel_2_ecu(msg)
        assert math.isclose(ecu.motor, 0.0, abs_tol=1e-6)
        assert ecu.brake
    finally:
        node.destroy_node()
        rclpy.shutdown()
