#!/usr/bin/env python3
"""Probe StatusReporter with two minimal ROS nodes in isolated domain 226.

No StatusReporter constructor, real process-manager endpoint, MQTT, HTTP, or
vehicle process is used. The fake shutdown service only returns success.
"""

import json
from pathlib import Path
import time
from unittest.mock import patch

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from multi_map_navigation.status_reporter import StatusReporter
from multi_map_navigation_msgs.srv import ShutdownProcess


class Logger:
    def __init__(self):
        self.messages = []

    def __getattr__(self, name):
        return lambda message: self.messages.append((name, message))


def main():
    context = Context()
    context.init(domain_id=226)
    executor = SingleThreadedExecutor(context=context)
    reporter = StatusReporter.__new__(StatusReporter)
    Node.__init__(reporter, 'status_callback_audit', context=context)
    server = Node('mock_shutdown_callback_audit', context=context)
    logger = Logger()
    reporter.get_logger = lambda: logger
    calls, waits, futures, completed = [], [], [], []

    def reply(request, response):
        calls.append(request.process_name)
        response.success = True
        return response

    service_name = '/navigation_followup_audit/mock_shutdown'
    server.create_service(ShutdownProcess, service_name, reply)
    reporter.shutdown_process_client = reporter.create_client(ShutdownProcess, service_name)
    original_call = reporter.shutdown_process_client.call_async

    def tracked_call(request):
        future = original_call(request)
        futures.append(future)
        return future

    reporter.shutdown_process_client.call_async = tracked_call
    executor.add_node(reporter)
    executor.add_node(server)
    original_spin = rclpy.spin_until_future_complete

    def fast_spin(node, future, timeout_sec):
        # Same executor/default callback groups as production; shorten each wait.
        original_spin(node, future, executor=executor, timeout_sec=0.05)
        waits.append(future.done())

    def invoke():
        timer.cancel()
        reporter.shutdown_all_auto_driving_processes()
        completed.append(True)

    try:
        assert reporter.shutdown_process_client.wait_for_service(timeout_sec=2.0)
        timer = reporter.create_timer(0.01, invoke)
        deadline = time.monotonic() + 3.0
        with patch('multi_map_navigation.status_reporter.rclpy.spin_until_future_complete', fast_spin):
            while not completed and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
        assert completed and len(waits) == 4
        assert waits == [False] * 4, waits
        deadline = time.monotonic() + 1.0
        while not all(future.done() for future in futures) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert len(calls) == 4 and all(future.done() for future in futures)
        assert all(future.result().success for future in futures)
        result = {'domain_id': 226, 'fake_server_successes': len(calls),
                  'future_done_inside_timer_waits': waits,
                  'future_done_after_timer_returned': [future.done() for future in futures],
                  'reported_failures': [message for level, message in logger.messages
                                        if level == 'error'],
                  'vehicle_processes_started': False}
        Path('/home/akun/workspace/navigation_status_callback_results_2026-10-30.json').write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        executor.remove_node(reporter)
        executor.remove_node(server)
        executor.shutdown()
        reporter.destroy_node = Node.destroy_node.__get__(reporter)
        reporter.destroy_node()
        server.destroy_node()
        context.shutdown()


if __name__ == '__main__':
    main()
