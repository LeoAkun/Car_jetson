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

"""Observe original MQTT task payloads without forwarding or executing tasks."""

import base64
from datetime import datetime, timezone
import json
import logging
from logging.handlers import RotatingFileHandler
import math
from pathlib import Path
import uuid

from ament_index_python.packages import get_package_share_directory
import paho.mqtt.client as mqtt
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml


def load_mqtt_settings(path):
    """Reuse the task receiver's connection settings without logging secrets."""
    try:
        with Path(path).expanduser().open(encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError('无法读取 MQTT 配置文件，请检查路径、权限和 YAML 格式') from exc
    if not isinstance(document, dict):
        raise ValueError('MQTT 配置文件必须是 YAML 映射')
    receiver = document.get('mqtt_task_receiver', {})
    if not isinstance(receiver, dict):
        raise ValueError('缺少 mqtt_task_receiver.ros__parameters 配置')
    params = receiver.get('ros__parameters')
    if not isinstance(params, dict):
        raise ValueError('缺少 mqtt_task_receiver.ros__parameters 配置')
    settings = {
        'broker_url': params.get('broker_url'),
        'port': params.get('port', 1883),
        'username': params.get('username', ''),
        'password': params.get('password', ''),
        'task_topic': params.get('task_topic'),
        'qos': params.get('qos', 1),
        'reconnect_delay': params.get('reconnect_delay', 5.0),
    }
    if not isinstance(settings['broker_url'], str) or not settings['broker_url'].strip():
        raise ValueError('broker_url 必须是非空服务器地址')
    if type(settings['port']) is not int or not 1 <= settings['port'] <= 65535:
        raise ValueError('MQTT port 必须是 1 到 65535 的整数')
    if type(settings['qos']) is not int or settings['qos'] not in (0, 1, 2):
        raise ValueError('MQTT qos 必须是 0、1 或 2')
    delay = settings['reconnect_delay']
    if (isinstance(delay, bool) or not isinstance(delay, (int, float))
            or not math.isfinite(delay) or delay <= 0):
        raise ValueError('reconnect_delay 必须是大于 0 的有限秒数')
    if any(not isinstance(settings[key], str) for key in ('username', 'password')):
        raise ValueError('MQTT username 和 password 必须是字符串')
    reserved = set()
    for name in ('mqtt_task_receiver', 'status_reporter'):
        section = document.get(name, {})
        if isinstance(section, dict):
            section_params = section.get('ros__parameters', {})
            if isinstance(section_params, dict):
                client_id = section_params.get('client_id')
                if isinstance(client_id, str) and client_id:
                    reserved.add(client_id)
    settings['reserved_client_ids'] = reserved
    return settings


def validate_topic_filter(topic):
    """Allow ordinary subscriptions, never shared subscriptions that consume tasks."""
    if not isinstance(topic, str) or not topic or '\x00' in topic:
        raise ValueError('监听 topic 必须是非空 MQTT 主题过滤器')
    if len(topic.encode('utf-8')) > 65535:
        raise ValueError('监听 topic 超过 MQTT 长度限制')
    if topic.startswith(('$share/', '$queue/')):
        raise ValueError('监听节点不能使用共享订阅，请使用普通任务 topic')
    levels = topic.split('/')
    for index, level in enumerate(levels):
        if ('+' in level and level != '+') or (
                '#' in level and (level != '#' or index != len(levels) - 1)):
            raise ValueError('监听 topic 的 MQTT 通配符位置无效')
    return topic


def make_message_record(message):
    """Keep exact payload bytes even when UTF-8 decoding or JSON parsing fails."""
    payload = bytes(message.payload)
    record = {
        'received_at': datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
        'topic': message.topic,
        'qos': int(message.qos),
        'retained': bool(message.retain),
        'duplicate': bool(message.dup),
        'payload_bytes': len(payload),
        'payload_base64': base64.b64encode(payload).decode('ascii'),
        'payload_text': None,
        'json_valid': False,
    }
    try:
        record['payload_text'] = payload.decode('utf-8')
    except UnicodeDecodeError as exc:
        record['decode_error'] = str(exc)
        return record
    try:
        json.loads(record['payload_text'])
        record['json_valid'] = True
    except (ValueError, RecursionError) as exc:
        record['json_error'] = str(exc)
    return record


class StrictRotatingFileHandler(RotatingFileHandler):
    """Surface write failures to the node instead of silently dropping records."""

    def handleError(self, record):  # noqa: N802 (logging API spelling)
        """Let the callback report a file error and keep receiving MQTT messages."""
        raise


class MQTTRawMonitor(Node):
    """Use a separate MQTT session to view task traffic without ROS task output."""

    def __init__(self):
        super().__init__('mqtt_raw_monitor')
        self.mqtt_client = None
        self._file_handler = None
        self._stopping = False
        self.subscription_ready = False
        self.received_count = 0
        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _configure(self):
        default_config = str(Path(get_package_share_directory(
            'multi_map_navigation')) / 'config' / 'mqtt_config.yaml')
        defaults = {
            'config_file': default_config,
            'topic': '',
            'client_id': '',
            'pretty_print': False,
            'log_file': '',
            'log_max_bytes': 10 * 1024 * 1024,
            'log_backup_count': 3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        parameters = {name: self.get_parameter(name).value for name in defaults}
        settings = load_mqtt_settings(parameters['config_file'])
        self.topic = validate_topic_filter(parameters['topic'] or settings['task_topic'])
        self.client_id = parameters['client_id'] or 'mqtt_mon_' + uuid.uuid4().hex[:14]
        if self.client_id in settings['reserved_client_ids']:
            raise ValueError('监听 Client ID 不能与任务接收或状态上报节点相同')
        self.qos = settings['qos']
        self.pretty_print = parameters['pretty_print']
        if (parameters['log_max_bytes'] <= 0 or parameters['log_backup_count'] < 1):
            raise ValueError('log_max_bytes 必须大于 0，log_backup_count 必须至少为 1')
        if parameters['log_file']:
            log_path = Path(parameters['log_file']).expanduser().resolve()
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self._file_handler = StrictRotatingFileHandler(
                log_path, maxBytes=parameters['log_max_bytes'],
                backupCount=parameters['log_backup_count'], encoding='utf-8')
            self._file_handler.setFormatter(logging.Formatter('%(message)s'))
            self.get_logger().info(f'原始消息 JSONL 日志: {log_path}')

        self.mqtt_client = mqtt.Client(
            client_id=self.client_id, clean_session=True, protocol=mqtt.MQTTv311)
        if settings['username']:
            self.mqtt_client.username_pw_set(settings['username'], settings['password'])
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_connect_fail = self._on_connect_fail
        self.mqtt_client.on_subscribe = self._on_subscribe
        self.mqtt_client.on_disconnect = self._on_disconnect
        self.mqtt_client.on_message = self._on_message
        delay = settings['reconnect_delay']
        self.mqtt_client.reconnect_delay_set(min_delay=delay, max_delay=max(60, delay))
        self.get_logger().info(
            f'只读监听 {settings["broker_url"]}:{settings["port"]} '
            f'| topic={self.topic} | client_id={self.client_id}')
        self.mqtt_client.connect_async(settings['broker_url'], settings['port'], keepalive=60)
        result = self.mqtt_client.loop_start()
        # Paho 1.x returns None on success; newer versions may return a result code.
        if result not in (None, mqtt.MQTT_ERR_SUCCESS):
            raise RuntimeError('无法启动 MQTT 网络线程')

    def _on_connect(self, client, userdata, flags, rc):
        if self._stopping:
            return
        self.subscription_ready = False
        if rc != 0:
            self.get_logger().error(f'MQTT 连接被拒绝，返回码={rc}，将自动重试')
            return
        result, _ = client.subscribe(self.topic, qos=self.qos)
        if result != mqtt.MQTT_ERR_SUCCESS:
            self.get_logger().error(f'发送 MQTT 订阅失败，返回码={result}')
        else:
            self.get_logger().info('MQTT 已连接，等待服务器确认订阅')

    def _on_connect_fail(self, client, userdata):
        if not self._stopping:
            self.get_logger().warning('MQTT 连接失败，将自动重试')

    def _on_subscribe(self, client, userdata, mid, granted_qos):
        if self._stopping:
            return
        self.subscription_ready = bool(granted_qos) and all(
            qos in (0, 1, 2) for qos in granted_qos)
        if self.subscription_ready:
            self.get_logger().info(f'订阅就绪: {self.topic} | QoS={list(granted_qos)}')
        else:
            self.get_logger().error(f'服务器拒绝订阅: {self.topic} | 返回码={list(granted_qos)}')

    def _on_disconnect(self, client, userdata, rc):
        self.subscription_ready = False
        if not self._stopping:
            self.get_logger().warning(f'MQTT 已断开，返回码={rc}，将自动重连并重新订阅')

    def _on_message(self, client, userdata, message):
        if self._stopping:
            return
        record = make_message_record(message)
        self.received_count += 1
        if self._file_handler is not None:
            try:
                self._file_handler.handle(logging.LogRecord(
                    'mqtt_raw_monitor', logging.INFO, '', 0,
                    json.dumps(record, ensure_ascii=False), (), None))
            except Exception as exc:
                self.get_logger().error(f'写入原始消息日志失败: {type(exc).__name__}')
        payload_display = record['payload_text']
        if payload_display is None:
            payload_display = '[非 UTF-8]'
        elif '\x00' in payload_display:
            # rcutils uses C strings; escape NUL so the rest is not truncated.
            payload_display = repr(payload_display)
        self.get_logger().info(
            f'收到 MQTT #{self.received_count} | time={record["received_at"]} '
            f'| topic={record["topic"]} | qos={record["qos"]} '
            f'| retain={record["retained"]} | dup={record["duplicate"]} '
            f'| bytes={record["payload_bytes"]} | json_valid={record["json_valid"]}\n'
            f'原始 payload:\n{payload_display}')
        if record['payload_text'] is None:
            self.get_logger().warning(f'非 UTF-8 消息，原始字节 Base64: {record["payload_base64"]}')
        elif not record['json_valid']:
            self.get_logger().warning(f'JSON 解析失败，原文已保留: {record["json_error"]}')
        elif self.pretty_print:
            preview = json.dumps(json.loads(record['payload_text']), ensure_ascii=False, indent=2)
            self.get_logger().info('JSON 格式化预览:\n' + preview.encode(
                'utf-8', errors='backslashreplace').decode('utf-8'))

    def destroy_node(self):
        """Stop the network loop before closing the file and ROS node."""
        if self._stopping:
            return True
        self._stopping = True
        self.subscription_ready = False
        try:
            if self.mqtt_client is not None:
                try:
                    self.mqtt_client.disconnect()
                finally:
                    self.mqtt_client.loop_stop()
        except Exception as exc:
            self.get_logger().warning(f'关闭 MQTT 连接时出现异常: {type(exc).__name__}')
        finally:
            if self._file_handler is not None:
                try:
                    self._file_handler.close()
                except (OSError, ValueError) as exc:
                    self.get_logger().warning(f'关闭日志文件时出现异常: {type(exc).__name__}')
        return super().destroy_node()


def main(args=None):
    """Run only the monitor; never start navigation or a task receiver."""
    rclpy.init(args=args)
    node = None
    result = 0
    try:
        node = MQTTRawMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        rclpy.logging.get_logger('mqtt_raw_monitor').error(
            f'监听节点启动或运行失败: {type(exc).__name__}: {exc}')
        result = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
