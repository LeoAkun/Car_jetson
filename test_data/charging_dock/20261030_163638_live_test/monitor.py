"""Record one authorized vehicle test; start through the package service."""

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
    rclpy.init()
    node = Node('docking_test_recorder')
    started = time.monotonic()
    path = Path(__file__).parent
    stream = (path / 'motion_trace.jsonl').open('w', encoding='utf-8')
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
            if not service_called and elapsed >= 2 and start.service_is_ready():
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
                                  'speed_kmh': vehicle.cur_speed if vehicle else None,
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
            if elapsed > 105:
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
        (path / 'motion_summary.json').write_text(json.dumps(summary, indent=2))
        print('SUMMARY ' + json.dumps(summary), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
