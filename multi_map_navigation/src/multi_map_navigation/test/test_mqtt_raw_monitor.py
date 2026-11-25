# Copyright 2026 contributors
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

"""Exercise read-only MQTT monitoring without a production broker or navigation."""

import base64
import json
import logging
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
import yaml

from multi_map_navigation import mqtt_raw_monitor as monitor


def config_document():
    return {
        'mqtt_task_receiver': {'ros__parameters': {
            'broker_url': '127.0.0.1', 'port': 1883, 'username': 'test-user',
            'password': 'test-secret', 'client_id': 'receiver',
            'task_topic': 'robot/test/down/cmd', 'qos': 1, 'reconnect_delay': 0.1,
        }},
        'status_reporter': {'ros__parameters': {'client_id': 'reporter'}},
    }


def mqtt_message(payload, **kwargs):
    values = dict(topic='robot/test/down/cmd', payload=payload,
                  qos=1, retain=False, dup=False)
    values.update(kwargs)
    return SimpleNamespace(**values)


@pytest.fixture
def node_factory(tmp_path, monkeypatch):
    path = tmp_path / 'config' / 'mqtt_config.yaml'
    path.parent.mkdir()
    path.write_text(yaml.safe_dump(config_document()), encoding='utf-8')
    monkeypatch.setattr(monitor, 'get_package_share_directory', lambda _: str(tmp_path))
    client = Mock()
    client.subscribe.return_value = (monitor.mqtt.MQTT_ERR_SUCCESS, 42)
    constructor = Mock(return_value=client)
    monkeypatch.setattr(monitor.mqtt, 'Client', constructor)
    nodes = []

    def make_node(loop_start_result=None, **parameters):
        client.loop_start.return_value = loop_start_result
        args = ['--ros-args']
        for key, value in parameters.items():
            args.extend(['-p', f'{key}:={value}'])
        rclpy.init(args=args)
        node = monitor.MQTTRawMonitor()
        node.get_logger = Mock(return_value=Mock())
        nodes.append(node)
        return node, client, constructor

    yield make_node
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_default_connection_reuses_config_and_isolates_client(node_factory):
    node, client, constructor = node_factory()
    assert node.topic == 'robot/test/down/cmd'
    assert len(node.client_id) == 23
    assert node.client_id.startswith('mqtt_mon_')
    constructor.assert_called_once_with(
        client_id=node.client_id, clean_session=True, protocol=monitor.mqtt.MQTTv311)
    client.username_pw_set.assert_called_once_with('test-user', 'test-secret')
    client.connect_async.assert_called_once_with('127.0.0.1', 1883, keepalive=60)
    client.reconnect_delay_set.assert_called_once_with(min_delay=0.1, max_delay=60)
    client.publish.assert_not_called()
    # Only Node's standard parameter event publisher is present; no task publisher.
    publisher_names = [name for name, _ in node.get_publisher_names_and_types_by_node(
        node.get_name(), node.get_namespace())]
    assert '/start_end_graph' not in publisher_names


def test_network_loop_also_accepts_explicit_success_code(node_factory):
    node, client, _ = node_factory(loop_start_result=monitor.mqtt.MQTT_ERR_SUCCESS)
    assert not node.subscription_ready
    client.loop_start.assert_called_once()


def test_network_loop_failure_cleans_up_connection(node_factory):
    with pytest.raises(RuntimeError, match='网络线程'):
        node_factory(loop_start_result=monitor.mqtt.MQTT_ERR_INVAL)


@pytest.mark.parametrize('client_id', ['receiver', 'reporter'])
def test_rejects_existing_business_client_ids(node_factory, client_id):
    with pytest.raises(ValueError, match='Client ID'):
        node_factory(client_id=client_id)


@pytest.mark.parametrize('topic', [
    '$share/workers/robot/test/down/cmd', '$queue/robot/test/down/cmd',
    '', 'robot/a+b/down', 'robot/#/down', 'robot/a#', 'robot/\x00/down',
])
def test_rejects_shared_and_invalid_topic_filters(topic):
    with pytest.raises(ValueError):
        monitor.validate_topic_filter(topic)


@pytest.mark.parametrize('topic', ['robot/+/down/cmd', 'robot/#', 'robot/test/down/cmd'])
def test_accepts_normal_topic_filters(topic):
    assert monitor.validate_topic_filter(topic) == topic


def test_ready_only_after_successful_suback_and_resubscribes(node_factory):
    node, client, _ = node_factory()
    node._on_connect(client, None, {}, 0)
    client.subscribe.assert_called_once_with(node.topic, qos=1)
    assert not node.subscription_ready
    node._on_subscribe(client, None, 42, [128])
    assert not node.subscription_ready
    node.get_logger().error.assert_called()
    node._on_subscribe(client, None, 42, [1])
    assert node.subscription_ready
    node._on_disconnect(client, None, 1)
    assert not node.subscription_ready
    node._on_connect(client, None, {}, 0)
    assert client.subscribe.call_count == 2
    node._on_subscribe(client, None, 43, [0])
    assert node.subscription_ready
    client.publish.assert_not_called()


def test_rejected_connection_does_not_subscribe(node_factory):
    node, client, _ = node_factory()
    node._on_connect(client, None, {}, 5)
    assert not node.subscription_ready
    client.subscribe.assert_not_called()
    node._on_connect_fail(client, None)
    node.get_logger().warning.assert_called()


def test_utf8_json_preserves_original_spacing_and_bytes():
    payload = ' \n{ "operation": "送餐", "points": [1, 2] }\r\n'.encode()
    record = monitor.make_message_record(mqtt_message(payload, retain=True, dup=True))
    assert record['payload_text'] == payload.decode()
    assert base64.b64decode(record['payload_base64']) == payload
    assert record['payload_bytes'] == len(payload)
    assert record['json_valid']
    assert record['retained'] and record['duplicate']
    assert record['received_at'].endswith('+00:00')


@pytest.mark.parametrize('payload', [b'{not valid JSON', b'', b'\xff\xfe\x00'])
def test_bad_json_or_encoding_does_not_lose_raw_bytes(payload):
    record = monitor.make_message_record(mqtt_message(payload))
    assert not record['json_valid']
    assert base64.b64decode(record['payload_base64']) == payload
    assert 'json_error' in record or 'decode_error' in record


def test_deep_json_is_preserved_without_stopping_network_callback(node_factory):
    node, client, _ = node_factory()
    payload = b'[' * 1100 + b'0' + b']' * 1100
    node._on_message(client, None, mqtt_message(payload))
    node._on_message(client, None, mqtt_message(b'{}'))
    assert node.received_count == 2
    assert not monitor.make_message_record(mqtt_message(payload))['json_valid']


def test_pretty_preview_and_nul_payload_are_safe_for_ros_logging(node_factory):
    node, client, _ = node_factory(pretty_print='true')
    output = []

    def ros_safe_log(message):
        message.encode('utf-8')
        assert '\x00' not in message
        output.append(message)

    node.get_logger().info.side_effect = ros_safe_log
    node._on_message(client, None, mqtt_message(b'{"x":"\\ud800"}'))
    node._on_message(client, None, mqtt_message(b'one\x00two'))
    assert any('\\ud800' in message for message in output)
    assert any('one\\x00two' in message for message in output)
    assert node.received_count == 2


def test_messages_print_and_persist_even_when_json_is_invalid(node_factory, tmp_path):
    log_file = tmp_path / 'messages' / 'raw.jsonl'
    node, client, _ = node_factory(log_file=str(log_file), pretty_print='true')
    payloads = [' \n{"operation":"测试"}\n'.encode(), b'{invalid', b'\xff\x00']
    for payload in payloads:
        node._on_message(client, None, mqtt_message(payload))
    records = [json.loads(line) for line in log_file.read_text().splitlines()]
    assert [base64.b64decode(row['payload_base64']) for row in records] == payloads
    assert [row['json_valid'] for row in records] == [True, False, False]
    assert records[0]['payload_text'] == payloads[0].decode()
    output = '\n'.join(call.args[0] for call in node.get_logger().info.call_args_list)
    assert payloads[0].decode() in output
    assert '{invalid' in output
    assert 'JSON 格式化预览' in output
    assert node.received_count == 3
    client.publish.assert_not_called()


def test_log_rotation_keeps_configured_backups(node_factory, tmp_path):
    log_file = tmp_path / 'raw.jsonl'
    node, client, _ = node_factory(
        log_file=str(log_file), log_max_bytes=400, log_backup_count=2)
    for number in range(8):
        node._on_message(client, None, mqtt_message(json.dumps({'n': number}).encode()))
    files = list(tmp_path.glob('raw.jsonl*'))
    assert len(files) == 3
    assert json.loads(log_file.read_text())['payload_text'] == '{"n": 7}'
    for path in files:
        for line in path.read_text().splitlines():
            assert 'payload_base64' in json.loads(line)


def test_file_write_failure_does_not_stop_receiving(node_factory, tmp_path, monkeypatch):
    node, client, _ = node_factory(log_file=str(tmp_path / 'raw.jsonl'))
    monkeypatch.setattr(node._file_handler, 'handle', Mock(side_effect=OSError('disk full')))
    for _ in range(2):
        node._on_message(client, None, mqtt_message(b'{"ok":true}'))
    assert node.received_count == 2
    assert node.get_logger().error.call_count == 2
    assert node.get_logger().info.call_count == 2


def test_rotating_handler_surfaces_internal_logging_errors(tmp_path):
    handler = monitor.StrictRotatingFileHandler(tmp_path / 'raw.jsonl')
    try:
        handler.stream.close()
        with pytest.raises(ValueError):
            handler.handle(logging.LogRecord('monitor', 20, '', 0, 'test', (), None))
    finally:
        handler.stream = None
        handler.close()


def test_clean_shutdown_stops_network_and_closes_log(node_factory, tmp_path):
    node, client, _ = node_factory(log_file=str(tmp_path / 'raw.jsonl'))
    handler = node._file_handler
    node.destroy_node()
    client.disconnect.assert_called_once()
    client.loop_stop.assert_called_once()
    assert handler.stream is None
    node._on_message(client, None, mqtt_message(b'{}'))
    assert node.received_count == 0


@pytest.mark.parametrize('key,value', [
    ('port', 0), ('port', True), ('qos', 3), ('qos', True),
    ('reconnect_delay', 0), ('reconnect_delay', float('nan')),
    ('broker_url', ''), ('username', None),
])
def test_invalid_connection_settings_fail_early(tmp_path, key, value):
    document = config_document()
    document['mqtt_task_receiver']['ros__parameters'][key] = value
    path = tmp_path / 'mqtt_config.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        monitor.load_mqtt_settings(path)


def test_yaml_parse_error_does_not_disclose_configuration(tmp_path):
    path = tmp_path / 'broken.yaml'
    path.write_text('password: [top-secret\n')
    with pytest.raises(ValueError) as error:
        monitor.load_mqtt_settings(path)
    assert 'top-secret' not in str(error.value)
