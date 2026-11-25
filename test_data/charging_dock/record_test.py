"""Record one authorized vehicle test; start through the package service."""

import argparse
import json
import math
from pathlib import Path
import time

from charging_dock_msgs.msg import DockObservation
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from yunle_msgs.msg import Ecu, VehicleStatus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--manual-start', action='store_true',
                        help='Record without automatically calling the start service')
    parser.add_argument('--duration', type=float, default=105.,
                        help='Recording deadline in seconds; request stop on expiry')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    path = args.output_dir
    trace_path = path / 'motion_trace.jsonl'
    summary_path = path / 'motion_summary.json'
    if trace_path.exists() or summary_path.exists():
        raise FileExistsError(f'recording output already exists: {path}')

    rclpy.init()
    node = Node('docking_test_recorder')
    started = time.monotonic()
    stream = trace_path.open('x', encoding='utf-8')
    latest, counts, last_printed = {}, {}, {}
    transitions = []
    last_scan = -math.inf
    terminal_at = None

    def received(topic, msg):
        nonlocal last_scan, terminal_at
        elapsed = time.monotonic() - started
        latest[topic] = msg
        counts[topic] = counts.get(topic, 0) + 1
        if topic == '/laser/scan_back':
            if elapsed - last_scan < .19:
                return
            last_scan = elapsed
        stream.write(json.dumps({'t': elapsed, 'topic': topic,
                                 'message': message_to_ordereddict(msg)}) + '\n')
        if isinstance(msg, String) and last_printed.get(topic) != msg.data:
            last_printed[topic] = msg.data
            transitions.append({'t': elapsed, 'topic': topic, 'state': msg.data})
            print(f'{elapsed:.2f}s {topic}: {msg.data}', flush=True)
        if topic == '/standalone_docking/state' and msg.data.split()[0] in (
                'FAILED', 'STOPPED', 'DOCKED'):
            if terminal_at is None:
                terminal_at = elapsed

    specs = [
        ('/standalone_docking/state', String), ('/charging_dock/state', String),
        ('/docking_feedback_odometry/state', String),
        ('/charging_dock/observation', DockObservation),
        ('/vehicle_status', VehicleStatus), ('/charging_dock/local_odom', Odometry),
        ('/cmd_vel_dock', Twist), ('/cmd_vel_selected', Twist), ('/ecu', Ecu),
        ('/charging_dock/emergency_stop', Bool), ('/laser/scan_back', LaserScan),
    ]
    subscriptions = [node.create_subscription(
        kind, topic, lambda message, topic=topic: received(topic, message),
        qos_profile_sensor_data if kind is LaserScan else 10) for topic, kind in specs]
    start = node.create_client(Trigger, '/standalone_docking/start')
    stop = node.create_client(Trigger, '/standalone_docking/stop')
    service_called = False
    next_report = 0.
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            elapsed = time.monotonic() - started
            if (not args.manual_start and not service_called
                    and elapsed >= 2 and start.service_is_ready()):
                if latest.get('/standalone_docking/state', String()).data.startswith('IDLE'):
                    future = start.call_async(Trigger.Request())
                    service_called = True
                    future.add_done_callback(lambda result: print(
                        'START ' + str(result.result()), flush=True))
            if elapsed >= next_report:
                vehicle = latest.get('/vehicle_status')
                obs = latest.get('/charging_dock/observation')
                ecu = latest.get('/ecu')
                print(json.dumps({'t': round(elapsed, 2),
                                  'cur_speed': vehicle.cur_speed if vehicle else None,
                                  'left_wheel_speed': vehicle.left_wheel_speed if vehicle else None,
                                  'right_wheel_speed': vehicle.right_wheel_speed if vehicle else None,
                                  'gear': vehicle.shift_level if vehicle else None,
                                  'autodrive': vehicle.is_autodrive if vehicle else None,
                                  'ecu_motor': ecu.motor if ecu else None,
                                  'brake': ecu.brake if ecu else None,
                                  'dock_x': obs.pose.position.x if obs and obs.valid else None,
                                  'dock_y': obs.pose.position.y if obs and obs.valid else None,
                                  'quality': obs.quality if obs else None}), flush=True)
                stream.flush()
                next_report = elapsed + 2.
            if terminal_at is not None and elapsed - terminal_at >= 2:
                break
            if elapsed > args.duration:
                if stop.service_is_ready():
                    response = stop.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(node, response, timeout_sec=3)
                print('RECORDER DEADLINE: requested stop', flush=True)
                break
    finally:
        stream.close()
        vehicle, ecu = latest.get('/vehicle_status'), latest.get('/ecu')
        summary = {'counts': counts, 'transitions': transitions,
                   'final_vehicle': message_to_ordereddict(vehicle) if vehicle else None,
                   'final_ecu': message_to_ordereddict(ecu) if ecu else None,
                   'final_state': last_printed.get('/standalone_docking/state'),
                   'start_service_called': service_called}
        with summary_path.open('x', encoding='utf-8') as output:
            json.dump(summary, output, indent=2)
        print('SUMMARY ' + json.dumps(summary), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
