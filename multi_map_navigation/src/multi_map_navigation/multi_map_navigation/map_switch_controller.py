#!/usr/bin/env python3
"""
地图切换控制器
开发者B - 导航模块

通过协调进程关闭和启动来处理地图切换操作
"""

from collections import OrderedDict
import threading
import time
import math

from geometry_msgs.msg import TransformStamped
from multi_map_navigation_msgs.msg import (
    LocalizationResult,
    LocalizationSession,
    MapSwitchCancel,
    MapSwitchResult,
    MapSwitchTrigger,
)
from multi_map_navigation_msgs.srv import (
    GetProcessStatus,
    ShutdownProcess,
    StartProcess,
)
import rclpy
import rclpy.duration
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)
from multi_map_navigation.stack_transaction_lock import StackTransactionLock


class MapSwitchController(Node):
    DEFERRED = object()
    DEFERRED_TRANSACTION = object()
    STACK_START_ORDER = (
        're_localization',
        'nav2_init_pose',
        'liosam',
        'navigation2',
    )
    STACK_SHUTDOWN_ORDER = tuple(reversed(STACK_START_ORDER))
    PROCESS_STATUS_TIMEOUT_SEC = 5.0
    # ProcessManager 的普通启动上限为 60 秒，失败回滚最多再用 20 秒。
    START_PROCESS_TIMEOUT_SEC = 90.0
    # Nav2 启动上限为 150 秒，失败回滚最多再用 20 秒。
    NAVIGATION2_START_TIMEOUT_SEC = 180.0
    DEFERRED_SWITCH_TIMEOUT_SEC = 300.0

    def __init__(self):
        super().__init__('map_switch_controller')

        # 声明参数
        self.declare_parameter('map_switch_timeout', 300.0)
        self.declare_parameter('process_shutdown_timeout', 30.0)

        # 获取参数
        self.map_switch_timeout = self.get_parameter(
            'map_switch_timeout'
        ).value
        self.process_shutdown_timeout = self.get_parameter(
            'process_shutdown_timeout'
        ).value

        # 使用 ReentrantCallbackGroup 允许回调内调用服务
        self.cb_group = ReentrantCallbackGroup()

        # 进程管理服务客户端
        self.start_process_client = self.create_client(
            StartProcess,
            '/process_manager/start_process',
            callback_group=self.cb_group,
        )
        self.shutdown_process_client = self.create_client(
            ShutdownProcess,
            '/process_manager/shutdown_process',
            callback_group=self.cb_group,
        )
        self.get_process_status_client = self.create_client(
            GetProcessStatus,
            '/process_manager/get_status',
            callback_group=self.cb_group,
        )

        # 当前地图跟踪
        self.current_map = None
        self.is_switching = False
        # 回调组必须保持 Reentrant，服务 future 才能在地图切换回调期间完成；
        # 单独用锁确保不会并发执行两次切换事务。
        self._switch_lock = threading.Lock()
        self._stack_transaction_lock = StackTransactionLock()
        self._deferred_switch_lock = threading.Lock()
        self._deferred_switch_timer = None
        self._deferred_switch_message = None
        self._deferred_switch_generation = 0
        self._deferred_switch_deadline = None
        self._map_switch_request_active = False
        self._active_transaction_id = ''
        self._closed_transactions = set()
        self._pending_switch_triggers = OrderedDict()
        self._localization_lock = threading.RLock()
        self._active_localization_session = ''
        self._active_localization_map = ''
        self._localization_session_start_ns = 0
        self._expected_localization_session = ''
        self._expected_localization_map = ''
        self._expected_session_start_ns = 0
        self._latest_localization_result = None
        self._pending_localization_result = None

        # ROS2订阅器用于地图切换触发
        self.map_switch_sub = self.create_subscription(
            MapSwitchTrigger,
            '/trigger_map_switch',
            self.map_switch_callback,
            10,
            callback_group=self.cb_group
        )
        self.map_switch_cancel_sub = self.create_subscription(
            MapSwitchCancel,
            '/map_switch_cancel',
            self.map_switch_cancel_callback,
            10,
            callback_group=self.cb_group,
        )

        localization_qos = QoSProfile(depth=1)
        localization_qos.reliability = ReliabilityPolicy.RELIABLE
        localization_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.localization_session_sub = self.create_subscription(
            LocalizationSession,
            '/localization_session',
            self.localization_session_callback,
            localization_qos,
            callback_group=self.cb_group,
        )
        self.localization_result_sub = self.create_subscription(
            LocalizationResult,
            '/localization_result',
            self.localization_result_callback,
            localization_qos,
            callback_group=self.cb_group,
        )

        # ROS2发布器用于地图切换完成
        self.map_switch_complete_pub = self.create_publisher(
            MapSwitchResult,
            '/map_switch_complete',
            10
        )

        self.get_logger().info('地图切换控制器已初始化')

    def _wait_for_future(self, future, timeout_sec: float):
        """等待 future 完成，由 MultiThreadedExecutor 在其他线程处理回调"""
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            cancel = getattr(future, 'cancel', None)
            if callable(cancel):
                cancel()
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'服务请求 future 异常: {exc}')
            return None

    def _deferred_switch_is_current(
        self, generation=None, deadline_monotonic=None
    ):
        if (deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic):
            return False
        if generation is None:
            return True
        lock = getattr(self, '_deferred_switch_lock', None)
        if lock is None:
            return True
        with lock:
            return (
                generation == getattr(self, '_deferred_switch_generation', 0)
                and bool(getattr(self, '_map_switch_request_active', False))
            )

    def _deferred_switch_deadline_for(self, generation):
        with self._deferred_switch_lock:
            if (generation != getattr(
                    self, '_deferred_switch_generation', 0)
                    or not getattr(self, '_map_switch_request_active', False)):
                return time.monotonic()
            return getattr(self, '_deferred_switch_deadline', None)

    def _wait_for_future_until(
        self, future, timeout_sec: float, *, generation=None,
        deadline_monotonic=None,
    ):
        wait_deadline = time.monotonic() + timeout_sec
        if deadline_monotonic is not None:
            wait_deadline = min(wait_deadline, deadline_monotonic)
        while not future.done() and time.monotonic() < wait_deadline:
            if not self._deferred_switch_is_current(
                    generation, deadline_monotonic):
                cancel = getattr(future, 'cancel', None)
                if callable(cancel):
                    cancel()
                return None
            time.sleep(0.05)
        if not self._deferred_switch_is_current(
                generation, deadline_monotonic):
            cancel = getattr(future, 'cancel', None)
            if callable(cancel):
                cancel()
            return None
        if not future.done():
            cancel = getattr(future, 'cancel', None)
            if callable(cancel):
                cancel()
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'服务请求 future 异常: {exc}')
            return None

    def _get_stack_transaction_lock(self):
        lock = getattr(self, '_stack_transaction_lock', None)
        if lock is None:
            lock = StackTransactionLock()
            self._stack_transaction_lock = lock
        return lock

    def map_switch_callback(self, msg: MapSwitchTrigger):
        """
        地图切换触发回调

        参数:
            msg: 包含切换信息的MapSwitchTrigger消息
        """
        self.get_logger().info(
            f'触发地图切换: {msg.current_map} -> {msg.next_map}'
        )

        transaction_id = getattr(msg, 'transaction_id', '')
        if not transaction_id:
            self.get_logger().error('地图切换请求缺少 transaction_id，拒绝执行')
            return

        with self._deferred_switch_lock:
            if transaction_id in getattr(
                    self, '_closed_transactions', set()):
                self.get_logger().warning(
                    f'忽略已结束的地图切换事务: {transaction_id}')
                return
            pending = getattr(self, '_pending_switch_triggers', None)
            if pending is None:
                pending = OrderedDict()
                self._pending_switch_triggers = pending
            if getattr(self, '_map_switch_request_active', False):
                pending[transaction_id] = msg
                self.get_logger().warning(
                    '上一地图切换事务仍在退出，暂存当前触发')
                return
            if pending:
                pending[transaction_id] = msg
                closed = getattr(self, '_closed_transactions', set())
                while pending:
                    pending_id, candidate = pending.popitem(last=False)
                    if pending_id not in closed:
                        msg = candidate
                        break
            generation = self._admit_map_switch_locked(msg)

        self._run_admitted_map_switch(msg, generation)

    def _admit_map_switch_locked(self, message):
        """Reserve one transaction while holding _deferred_switch_lock."""
        self._map_switch_request_active = True
        self._active_transaction_id = message.transaction_id
        self._deferred_switch_generation = getattr(
            self, '_deferred_switch_generation', 0) + 1
        timeout = max(
            0.1,
            float(getattr(
                self, 'map_switch_timeout', self.DEFERRED_SWITCH_TIMEOUT_SEC)),
        )
        self._deferred_switch_deadline = time.monotonic() + timeout
        self._deferred_switch_timer = getattr(
            self, '_deferred_switch_timer', None)
        self._deferred_switch_message = None
        return self._deferred_switch_generation

    def _run_admitted_map_switch(self, msg, generation):
        """Execute a transaction already reserved by the admission lock."""
        transaction_id = msg.transaction_id
        success = self._execute_map_switch_transaction(
            msg.current_map,
            msg.next_map,
            msg.switch_pose,
            generation=generation,
        )
        with self._deferred_switch_lock:
            if generation != self._deferred_switch_generation:
                return
            deadline = self._deferred_switch_deadline
            timed_out = (
                success is None
                and deadline is not None
                and time.monotonic() >= deadline
                and transaction_id == self._active_transaction_id
                and self._map_switch_request_active
            )
        if success is self.DEFERRED:
            self._schedule_deferred_map_switch(
                msg, resume_start=True, generation=generation)
            return
        if success is self.DEFERRED_TRANSACTION:
            self._schedule_deferred_map_switch(
                msg, resume_start=False, generation=generation)
            return
        if success is None:
            if self._finalize_map_switch_transaction(
                    generation, transaction_id):
                if timed_out:
                    self.get_logger().error('地图切换达到总超时时间')
                    self._publish_map_switch_completion(
                        False, transaction_id)
                self._dispatch_next_pending_switch()
            return
        if self._finalize_map_switch_transaction(
                generation, transaction_id):
            self._publish_map_switch_completion(success, transaction_id)
            self._dispatch_next_pending_switch()

    def _remember_closed_transaction_locked(self, transaction_id):
        closed = getattr(self, '_closed_transactions', None)
        if closed is None:
            closed = set()
            self._closed_transactions = closed
        closed.add(transaction_id)

    def _finalize_map_switch_transaction(self, generation, transaction_id):
        """Atomically finish only the transaction that produced a result."""
        with self._deferred_switch_lock:
            if (generation != getattr(
                    self, '_deferred_switch_generation', 0)
                    or transaction_id != getattr(
                        self, '_active_transaction_id', '')
                    or not getattr(self, '_map_switch_request_active', False)):
                return False
            self._remember_closed_transaction_locked(transaction_id)
            self._map_switch_request_active = False
            self._active_transaction_id = ''
            self._deferred_switch_deadline = None
            self._deferred_switch_message = None
            return True

    def _dispatch_next_pending_switch(self):
        """Admit a trigger that raced ahead of the previous cancellation."""
        message = None
        generation = None
        with self._deferred_switch_lock:
            if getattr(self, '_map_switch_request_active', False):
                return
            pending = getattr(self, '_pending_switch_triggers', None)
            closed = getattr(self, '_closed_transactions', set())
            while pending:
                transaction_id, candidate = pending.popitem(last=False)
                if transaction_id not in closed:
                    message = candidate
                    generation = self._admit_map_switch_locked(candidate)
                    break
        if message is not None:
            self._run_admitted_map_switch(message, generation)

    def map_switch_cancel_callback(self, message: MapSwitchCancel):
        """Cancel exactly one transaction, including a trigger arriving later."""
        transaction_id = getattr(message, 'transaction_id', '')
        if not transaction_id:
            self.get_logger().warning('忽略缺少 transaction_id 的地图切换取消')
            return
        with self._deferred_switch_lock:
            self._remember_closed_transaction_locked(transaction_id)
            pending = getattr(self, '_pending_switch_triggers', None)
            if pending is not None:
                pending.pop(transaction_id, None)
            if transaction_id != getattr(
                    self, '_active_transaction_id', ''):
                return
            self._deferred_switch_generation = getattr(
                self, '_deferred_switch_generation', 0) + 1
            timer = self._deferred_switch_timer
            self._deferred_switch_timer = None
            self._deferred_switch_message = None
            self._deferred_switch_deadline = None
            self._map_switch_request_active = False
            self._active_transaction_id = ''
            if timer is not None:
                timer.cancel()
        self._dispatch_next_pending_switch()

    def _publish_map_switch_completion(self, success, transaction_id):
        """Publish one terminal result for the active map-switch request."""
        completion_msg = MapSwitchResult()
        completion_msg.transaction_id = transaction_id
        completion_msg.success = success
        self.map_switch_complete_pub.publish(completion_msg)

        if success:
            self.get_logger().info('地图切换成功完成')
        else:
            self.get_logger().error('地图切换失败')

    def _schedule_deferred_map_switch(
        self, message, *, resume_start, generation=None
    ):
        """Retry a ProcessManager BUSY result without publishing failure."""
        timed_out = False
        with self._deferred_switch_lock:
            if not hasattr(self, '_deferred_switch_generation'):
                self._deferred_switch_generation = 0
            if not hasattr(self, '_deferred_switch_deadline'):
                self._deferred_switch_deadline = None
            if not hasattr(self, '_map_switch_request_active'):
                self._map_switch_request_active = False
            if generation is None:
                generation = getattr(
                    self, '_deferred_switch_generation', 0)
            if generation != self._deferred_switch_generation:
                return
            now = time.monotonic()
            if self._deferred_switch_deadline is None:
                timeout = max(
                    0.1,
                    float(getattr(
                        self, 'map_switch_timeout',
                        self.DEFERRED_SWITCH_TIMEOUT_SEC)),
                )
                self._deferred_switch_deadline = now + timeout
            if now >= self._deferred_switch_deadline:
                self._deferred_switch_deadline = None
                self._deferred_switch_message = None
                self._map_switch_request_active = False
                transaction_id = getattr(
                    message, 'transaction_id', '')
                if transaction_id:
                    self._remember_closed_transaction_locked(transaction_id)
                self._active_transaction_id = ''
                self._deferred_switch_generation += 1
                timed_out = True
            if timed_out:
                timer = None
            else:
                timer = self._deferred_switch_timer
            if not timed_out and timer is not None and timer.is_alive():
                return
            if timed_out:
                self._deferred_switch_timer = None
            else:
                self._deferred_switch_message = (
                    message, resume_start, generation)

            if timed_out:
                retry = None
            else:

                def retry():
                    with self._deferred_switch_lock:
                        pending = self._deferred_switch_message
                        self._deferred_switch_message = None
                        self._deferred_switch_timer = None
                    if pending is None:
                        return
                    pending_message, start_only, pending_generation = pending
                    if pending_generation != self._deferred_switch_generation:
                        return
                    if start_only:
                        result = self._resume_deferred_map_switch_start(
                            pending_message, pending_generation)
                    else:
                        result = self._execute_map_switch_transaction(
                            pending_message.current_map,
                            pending_message.next_map,
                            pending_message.switch_pose,
                            generation=pending_generation,
                        )

                    with self._deferred_switch_lock:
                        if (pending_generation !=
                                self._deferred_switch_generation):
                            return
                    if result is self.DEFERRED_TRANSACTION:
                        self._schedule_deferred_map_switch(
                            pending_message, resume_start=False,
                            generation=pending_generation)
                    elif result is self.DEFERRED:
                        self._schedule_deferred_map_switch(
                            pending_message, resume_start=True,
                            generation=pending_generation)
                    elif result is None:
                        self._schedule_deferred_map_switch(
                            pending_message,
                            resume_start=start_only,
                            generation=pending_generation,
                        )
                    elif result is not None:
                        if self._finalize_map_switch_transaction(
                                pending_generation,
                                pending_message.transaction_id):
                            self._publish_map_switch_completion(
                                result, pending_message.transaction_id)
                            self._dispatch_next_pending_switch()

                timer = threading.Timer(0.5, retry)
                timer.daemon = True
                self._deferred_switch_timer = timer
                timer.start()

        if timed_out:
            self.get_logger().error(
                '等待 ProcessManager 地图切换事务结束超时')
            self._publish_map_switch_completion(False, transaction_id)
            self._dispatch_next_pending_switch()

    def _resume_deferred_map_switch_start(self, message, generation=None):
        """Resume at stack start; the previous attempt already shut it down."""
        if (generation is not None and
                generation != self._deferred_switch_generation):
            return None
        if not self._switch_lock.acquire(blocking=False):
            return self.DEFERRED
        transaction_lock = self._get_stack_transaction_lock()
        if not transaction_lock.acquire():
            self._switch_lock.release()
            return self.DEFERRED
        self.is_switching = True
        try:
            launched = self._launch_resumed_stack(message, generation)
            if launched is None:
                if not self._deferred_switch_is_current(generation):
                    return None
                return self.DEFERRED
            if not launched:
                return False
            if (generation is not None
                    and not self._deferred_switch_is_current(
                        generation,
                        self._deferred_switch_deadline_for(generation))):
                return None
            deadline = (
                self._deferred_switch_deadline_for(generation)
                if generation is not None else None
            )
            if generation is None:
                ready = self.verify_stack_ready()
            else:
                ready = self.verify_stack_ready(
                    generation=generation,
                    deadline_monotonic=deadline,
                )
            if ready is None:
                return None
            if not ready:
                self._rollback_processes(
                    self.STACK_START_ORDER,
                    reason='延迟启动后的导航栈就绪验证失败',
                )
                return False
            if (generation is not None
                    and not self._deferred_switch_is_current(
                        generation,
                        self._deferred_switch_deadline_for(generation))):
                return None
            self.current_map = message.next_map
            return True
        except Exception as exc:
            self.get_logger().error(f'恢复延迟地图切换失败: {exc}')
            return False
        finally:
            self.is_switching = False
            transaction_lock.release()
            self._switch_lock.release()

    def _launch_resumed_stack(self, message, generation):
        """Resume a partial startup without clearing localization evidence."""
        deadline = (
            self._deferred_switch_deadline_for(generation)
            if generation is not None else None
        )
        if self._valid_switch_pose(message.switch_pose):
            return self.launch_new_stack(
                message.next_map,
                message.switch_pose,
                resume=True,
                generation=generation,
                deadline_monotonic=deadline,
            )
        return self.launch_new_stack(
            message.next_map,
            resume=True,
            generation=generation,
            deadline_monotonic=deadline,
        )

    def execute_map_switch(self, current_map: str, next_map: str) -> bool:
        """
        执行地图切换过程

        参数:
            current_map: 当前地图名称
            next_map: 下一个地图名称

        返回:
            切换成功返回True，否则返回False
        """
        result = self._execute_map_switch_transaction(current_map, next_map)
        return result is True

    def _execute_map_switch_transaction(
        self, current_map: str, next_map: str, switch_pose=None,
        generation=None,
    ):
        """执行一次获得串行许可的地图切换；忙时返回 None。"""
        if not self._deferred_switch_is_current(generation):
            return None
        if not self._switch_lock.acquire(blocking=False):
            if (generation is not None
                    and self._deferred_switch_is_current(generation)):
                self.get_logger().warning(
                    '上一地图切换事务仍在退出，延迟当前切换请求')
                return self.DEFERRED_TRANSACTION
            self.get_logger().warning('地图切换已在进行中，拒绝并发切换请求')
            return None

        transaction_lock = self._get_stack_transaction_lock()
        if not transaction_lock.acquire():
            self._switch_lock.release()
            if not self._deferred_switch_is_current(generation):
                return None
            self.get_logger().warning(
                '另一个控制器正在变更导航栈，延迟地图切换')
            return self.DEFERRED_TRANSACTION

        self.is_switching = True

        try:
            deadline = (
                self._deferred_switch_deadline_for(generation)
                if generation is not None else None
            )
            if not self._deferred_switch_is_current(generation, deadline):
                return None
            # 步骤1: 关闭当前堆栈
            self.get_logger().info('步骤1: 正在关闭当前导航堆栈')
            if generation is None:
                shutdown_result = self.shutdown_current_stack()
            else:
                shutdown_result = self.shutdown_current_stack(
                    generation=generation,
                    deadline_monotonic=deadline,
                )
            if shutdown_result is None:
                return None
            if not shutdown_result:
                self.get_logger().error('关闭当前堆栈失败')
                return False
            if not self._deferred_switch_is_current(generation, deadline):
                return None

            # 步骤2: 等待完全关闭
            self.get_logger().info('步骤2: 正在等待完全关闭')
            if generation is None:
                clean = self.wait_for_clean_shutdown(
                    self.process_shutdown_timeout)
            else:
                clean = self.wait_for_clean_shutdown(
                    self.process_shutdown_timeout,
                    generation=generation,
                    deadline_monotonic=deadline,
                )
            if clean is None:
                return None
            if not clean:
                self.get_logger().warn('完全关闭超时')
                return False
            if not self._deferred_switch_is_current(generation, deadline):
                return None

            # 步骤3: 启动新堆栈
            self.get_logger().info(f'步骤3: 正在为地图启动新导航堆栈: {next_map}')
            if generation is None:
                if self._valid_switch_pose(switch_pose):
                    launched = self.launch_new_stack(next_map, switch_pose)
                else:
                    launched = self.launch_new_stack(next_map)
            elif self._valid_switch_pose(switch_pose):
                launched = self.launch_new_stack(
                    next_map, switch_pose, generation=generation,
                    deadline_monotonic=deadline)
            else:
                launched = self.launch_new_stack(
                    next_map, generation=generation,
                    deadline_monotonic=deadline)
            if launched is None:
                self.get_logger().warning(
                    'ProcessManager 正忙，地图切换稍后重试且不回滚')
                return self.DEFERRED
            if not launched:
                self.get_logger().error('启动新堆栈失败')
                return False
            if not self._deferred_switch_is_current(generation, deadline):
                return None

            # 步骤4: 验证堆栈已就绪
            self.get_logger().info('步骤4: 正在验证新堆栈已就绪')
            if generation is None:
                ready = self.verify_stack_ready()
            else:
                ready = self.verify_stack_ready(
                    generation=generation,
                    deadline_monotonic=deadline,
                )
            if ready is None:
                return None
            if not ready:
                self.get_logger().error('新堆栈验证失败')
                self._rollback_processes(
                    self.STACK_START_ORDER,
                    reason='新堆栈就绪验证失败',
                )
                return False
            if not self._deferred_switch_is_current(generation, deadline):
                return None
            self.get_logger().info('新堆栈启动成功')

            # 更新当前地图
            self.current_map = next_map

            self.get_logger().info('Navigation2关键节点和生命周期状态均已就绪')
            return True

        except Exception as e:
            self.get_logger().error(f'地图切换期间发生异常: {e}')
            self._rollback_processes(
                self.STACK_START_ORDER,
                reason='地图切换发生异常',
            )
            return False
        finally:
            self.is_switching = False
            transaction_lock.release()
            self._switch_lock.release()

    def shutdown_current_stack(
        self, *, generation=None, deadline_monotonic=None
    ):
        """
        关闭当前导航堆栈

        返回:
            关闭成功返回True，否则返回False
        """
        success = True
        # process_manager 最多花 10 秒优雅关闭进程。这里额外保留 5 秒，
        # 避免客户端先超时后继续启动下一套堆栈。
        response_timeout = max(15.0, self.process_shutdown_timeout + 5.0)

        for process_name in self.STACK_SHUTDOWN_ORDER:
            if not self._deferred_switch_is_current(
                    generation, deadline_monotonic):
                return None
            result = self.shutdown_process_by_name(
                process_name,
                timeout_sec=response_timeout,
                generation=generation,
                deadline_monotonic=deadline_monotonic,
            )
            if result is None:
                return None
            if not result:
                success = False

        return success

    def wait_for_clean_shutdown(
        self, timeout: float, *, generation=None, deadline_monotonic=None
    ):
        """
        等待所有进程完全关闭

        参数:
            timeout: 最大等待时间（秒）

        返回:
            所有进程关闭返回True，超时返回False
        """
        deadline = time.monotonic() + timeout
        if deadline_monotonic is not None:
            deadline = min(deadline, deadline_monotonic)

        while time.monotonic() < deadline:
            if not self._deferred_switch_is_current(
                    generation, deadline_monotonic):
                return None
            req = GetProcessStatus.Request()
            try:
                future = self.get_process_status_client.call_async(req)
            except Exception as exc:
                self.get_logger().error(f'请求进程状态失败: {exc}')
                time.sleep(0.5)
                continue

            if generation is None and deadline_monotonic is None:
                status = self._wait_for_future(
                    future,
                    timeout_sec=self.PROCESS_STATUS_TIMEOUT_SEC,
                )
            else:
                status = self._wait_for_future_until(
                    future,
                    timeout_sec=self.PROCESS_STATUS_TIMEOUT_SEC,
                    generation=generation,
                    deadline_monotonic=deadline_monotonic,
                )
            if status is None:
                if not self._deferred_switch_is_current(
                        generation, deadline_monotonic):
                    return None
                # 状态未知时必须失败关闭，不能把“没有答复”解释成全部停止。
                self.get_logger().warning('进程状态未知，继续等待干净关闭')
                time.sleep(0.5)
                continue

            status_fields = (
                're_localization_running',
                'liosam_running',
                'nav2_init_pose_running',
                'navigation2_running',
            )
            if not all(hasattr(status, field) for field in status_fields):
                self.get_logger().error('进程状态响应不完整，拒绝判定为已关闭')
                return False

            all_stopped = not any(
                bool(getattr(status, field)) for field in status_fields
            )
            if all_stopped:
                self.get_logger().info('所有进程已完全关闭')
                return True
            self.get_logger().warning(f'status: {status}')
            time.sleep(0.5)

        if not self._deferred_switch_is_current(
                generation, deadline_monotonic):
            return None
        self.get_logger().warning('等待完全关闭超时')
        return False

    @staticmethod
    def _valid_switch_pose(switch_pose) -> bool:
        if switch_pose is None:
            return False
        if switch_pose.header.frame_id != 'map':
            return False
        pose = switch_pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return False
        norm_sq = sum(value * value for value in (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ))
        return 0.5 <= norm_sq <= 1.5

    def launch_new_stack(
        self, map_name: str, initial_pose=None, *, resume: bool = False,
        generation=None, deadline_monotonic=None,
    ):
        """
        为指定地图启动新导航堆栈

        参数:
            map_name: 要启动的地图名称

        返回:
            启动成功返回True，否则返回False
        """
        attempted = []
        if not self._deferred_switch_is_current(
                generation, deadline_monotonic):
            return None
        if not resume:
            self._reset_localization_gate(map_name)

        result = self._start_process(
            're_localization', map_name, self.START_PROCESS_TIMEOUT_SEC,
            generation=generation,
            deadline_monotonic=deadline_monotonic,
        )
        if result is None:
            return None
        if not result:
            self._rollback_processes(attempted, reason='启动重定位失败')
            return False
        attempted.append('re_localization')

        result = self._start_process(
            'nav2_init_pose', map_name, self.START_PROCESS_TIMEOUT_SEC,
            initial_pose=initial_pose,
            generation=generation,
            deadline_monotonic=deadline_monotonic,
        )
        if result is None:
            return None
        if not result:
            self._rollback_processes(attempted, reason='启动初始位姿节点失败')
            return False
        attempted.append('nav2_init_pose')

        # 等待tf变换完成
        self.get_logger().info('等待重定位完成并发布 map -> odom TF...')
        try:
            if generation is None and deadline_monotonic is None:
                tf_ready = self.wait_for_map_to_odom_link_tf(
                    timeout_sec=600.0)
            else:
                tf_ready = self.wait_for_map_to_odom_link_tf(
                    timeout_sec=600.0,
                    generation=generation,
                    deadline_monotonic=deadline_monotonic,
                )
        except Exception as exc:
            self.get_logger().error(f'等待 map -> odom TF 异常: {exc}')
            tf_ready = False
        if tf_ready is None:
            return None
        if not tf_ready:
            self.get_logger().error('map -> odom TF 长时间未出现，启动失败')
            self._rollback_processes(attempted, reason='重定位 TF 等待失败')
            return False

        result = self._start_process(
            'liosam', '', self.START_PROCESS_TIMEOUT_SEC,
            generation=generation,
            deadline_monotonic=deadline_monotonic,
        )
        if result is None:
            return None
        if not result:
            self._rollback_processes(attempted, reason='启动 LIO-SAM 失败')
            return False
        attempted.append('liosam')

        result = self._start_process(
            'navigation2', map_name, self.NAVIGATION2_START_TIMEOUT_SEC,
            generation=generation,
            deadline_monotonic=deadline_monotonic,
        )
        if result is None:
            return None
        if not result:
            self._rollback_processes(attempted, reason='启动 Navigation2 失败')
            return False
        attempted.append('navigation2')

        return True

    def _start_process(
        self, process_name: str, map_name: str, timeout_sec: float,
        initial_pose=None, *, generation=None, deadline_monotonic=None,
    ):
        """启动单个进程并严格检查服务响应。"""
        if not self._deferred_switch_is_current(
                generation, deadline_monotonic):
            return None
        req = StartProcess.Request()
        req.process_name = process_name
        req.map_name = map_name
        req.use_initial_pose = bool(
            process_name == 'nav2_init_pose'
            and self._valid_switch_pose(initial_pose))
        if req.use_initial_pose:
            req.initial_pose = initial_pose
        try:
            future = self.start_process_client.call_async(req)
        except Exception as exc:
            self.get_logger().error(f'请求启动 {process_name} 失败: {exc}')
            return False

        if generation is None and deadline_monotonic is None:
            response = self._wait_for_future(future, timeout_sec=timeout_sec)
        else:
            response = self._wait_for_future_until(
                future,
                timeout_sec=timeout_sec,
                generation=generation,
                deadline_monotonic=deadline_monotonic,
            )
        if response is None:
            self.get_logger().warning(
                f'启动 {process_name} 结果未知（{timeout_sec:.1f}s），禁止回滚'
            )
            return None
        if getattr(response, 'busy', False):
            self.get_logger().warning(
                f'启动 {process_name} 暂忙: {response.message}')
            return None
        if getattr(response, 'success', False) is not True:
            message = getattr(response, 'message', '无错误信息')
            self.get_logger().error(f'启动 {process_name} 失败: {message}')
            return False
        if process_name == 're_localization':
            instance_id = getattr(response, 'instance_id', '')
            active_map = getattr(response, 'active_map', '')
            session_start_ns = int(
                getattr(response, 'session_start_ns', 0))
            if (not instance_id or active_map != map_name
                    or session_start_ns <= 0):
                self.get_logger().error(
                    're_localization 启动响应缺少本轮会话证据')
                return False
            with self._localization_lock:
                changed = (
                    self._active_localization_session != instance_id or
                    self._active_localization_map != active_map or
                    self._localization_session_start_ns != session_start_ns)
                if changed:
                    self._latest_localization_result = None
                self._active_localization_session = instance_id
                self._active_localization_map = active_map
                self._localization_session_start_ns = session_start_ns
                self._expected_localization_session = instance_id
                self._expected_localization_map = active_map
                self._expected_session_start_ns = session_start_ns
                pending = self._pending_localization_result
                self._pending_localization_result = None
                if pending is not None:
                    self._accept_localization_result_locked(pending)
        return True

    def _reset_localization_gate(self, map_name: str):
        if not hasattr(self, '_localization_lock'):
            self._localization_lock = threading.RLock()
        with self._localization_lock:
            self._active_localization_session = ''
            self._active_localization_map = map_name or ''
            self._localization_session_start_ns = 0
            self._expected_localization_session = ''
            self._expected_localization_map = map_name or ''
            self._expected_session_start_ns = 0
            self._latest_localization_result = None
            self._pending_localization_result = None

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def localization_session_callback(self, message: LocalizationSession):
        with self._localization_lock:
            if not message.active:
                if (not message.session_id
                        or message.session_id == self._active_localization_session):
                    self._active_localization_session = ''
                    self._localization_session_start_ns = 0
                    self._latest_localization_result = None
                    self._pending_localization_result = None
                return
            changed = (
                message.session_id != self._active_localization_session
                or message.map_id != self._active_localization_map
                or int(message.session_start_ns) != self._localization_session_start_ns
            )
            if changed:
                self._latest_localization_result = None
            self._active_localization_session = message.session_id
            self._active_localization_map = message.map_id
            self._localization_session_start_ns = int(message.session_start_ns)
            pending = self._pending_localization_result
            self._pending_localization_result = None
            if pending is not None:
                self._accept_localization_result_locked(pending)

    def localization_result_callback(self, message: LocalizationResult):
        with self._localization_lock:
            if not self._active_localization_session:
                self._pending_localization_result = message
                return
            self._accept_localization_result_locked(message)

    def _accept_localization_result_locked(self, message):
        if (message.session_id != self._active_localization_session
                or message.map_id != self._active_localization_map):
            return False
        if not message.success:
            self._latest_localization_result = message
            return False
        if (self._stamp_to_ns(message.cloud_stamp)
                < self._localization_session_start_ns):
            return False
        if not math.isfinite(float(message.fitness_score)):
            return False
        if float(message.fitness_score) > 0.05:
            return False
        self._latest_localization_result = message
        return True

    def _localization_result_ready(self):
        with self._localization_lock:
            result = self._latest_localization_result
            expected_session = self._expected_localization_session
            expected_map = self._expected_localization_map
            expected_start = self._expected_session_start_ns
            active_session = self._active_localization_session
            active_map = self._active_localization_map
            active_start = self._localization_session_start_ns
        if (not expected_session or expected_start <= 0
                or active_session != expected_session
                or active_map != expected_map
                or active_start != expected_start or result is None):
            return None
        if (result.session_id != expected_session
                or result.map_id != expected_map or not result.success
                or self._stamp_to_ns(result.cloud_stamp) < expected_start):
            return None
        return result

    @staticmethod
    def _pose_matches_transform(pose, transform, tolerance=0.20):
        dx = pose.position.x - transform.transform.translation.x
        dy = pose.position.y - transform.transform.translation.y
        dz = pose.position.z - transform.transform.translation.z
        if math.sqrt(dx * dx + dy * dy + dz * dz) > tolerance:
            return False
        a = pose.orientation
        b = transform.transform.rotation
        norm_a = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z + a.w * a.w)
        norm_b = math.sqrt(b.x * b.x + b.y * b.y + b.z * b.z + b.w * b.w)
        if norm_a < 1e-6 or norm_b < 1e-6:
            return False
        dot = abs(
            a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
        return dot / (norm_a * norm_b) >= math.cos(math.radians(10.0))

    def _transform_is_from_current_session(self, transform):
        stamp_ns = self._stamp_to_ns(transform.header.stamp)
        with self._localization_lock:
            session_start_ns = self._expected_session_start_ns
        if session_start_ns <= 0 or stamp_ns < session_start_ns:
            return False
        age_ns = self.get_clock().now().nanoseconds - stamp_ns
        return -500_000_000 <= age_ns <= 2_000_000_000

    def shutdown_process_by_name(
        self, process_name: str, timeout_sec: float = 15.0, *,
        generation=None, deadline_monotonic=None,
    ):
        """通过服务关闭进程"""
        if not self._deferred_switch_is_current(
                generation, deadline_monotonic):
            return None
        try:
            req = ShutdownProcess.Request()
            req.process_name = process_name
            future = self.shutdown_process_client.call_async(req)
            if generation is None and deadline_monotonic is None:
                response = self._wait_for_future(
                    future, timeout_sec=timeout_sec)
            else:
                response = self._wait_for_future_until(
                    future,
                    timeout_sec=timeout_sec,
                    generation=generation,
                    deadline_monotonic=deadline_monotonic,
                )
            if response is None:
                if not self._deferred_switch_is_current(
                        generation, deadline_monotonic):
                    return None
                self.get_logger().error(
                    f'关闭 {process_name} 超时（{timeout_sec:.1f}s）'
                )
                return False
            if getattr(response, 'success', False) is not True:
                message = getattr(response, 'message', '无错误信息')
                self.get_logger().error(f'关闭 {process_name} 失败: {message}')
                return False
            return True
        except Exception as e:
            self.get_logger().error(f'关闭进程{process_name}异常: {e}')
            return False

    def _rollback_processes(self, process_names, reason: str) -> bool:
        """按启动顺序的逆序回滚，并确认进程状态确实全部停止。"""
        names = list(dict.fromkeys(process_names))
        self.get_logger().warning(
            f'{reason}，开始逆序回滚: {list(reversed(names))}'
        )

        responses_ok = True
        response_timeout = max(15.0, self.process_shutdown_timeout + 5.0)
        for process_name in reversed(names):
            if not self.shutdown_process_by_name(
                process_name,
                timeout_sec=response_timeout,
            ):
                responses_ok = False

        clean = self.wait_for_clean_shutdown(self.process_shutdown_timeout)
        if not clean:
            self.get_logger().error(f'{reason}后的回滚未能确认系统已清空')
        elif not responses_ok:
            self.get_logger().error(
                f'{reason}后的进程状态已清空，但至少一个关闭响应失败'
            )
        return responses_ok and clean

    def wait_for_map_to_odom_link_tf(
        self,
        timeout_sec: float = 300.0,  # 5分钟
        check_interval: float = 0.5,
        generation=None,
        deadline_monotonic=None,
    ):
        """
        等待 map -> odom TF 变换可用
        """
        self.get_logger().info("等待本轮定位结果及 map -> odom TF 变换就绪...")

        tf_buffer = tf2_ros.Buffer()
        self._map_switch_tf_listener = tf2_ros.TransformListener(
            tf_buffer,
            self,
        )

        wait_deadline = time.monotonic() + timeout_sec
        if deadline_monotonic is not None:
            wait_deadline = min(wait_deadline, deadline_monotonic)

        consecutive_matches = 0
        while time.monotonic() < wait_deadline:
            if not self._deferred_switch_is_current(
                    generation, deadline_monotonic):
                self.get_logger().warning(
                    '地图切换已取消或延迟启动已到截止时间')
                return None
            result = self._localization_result_ready()
            if result is None:
                consecutive_matches = 0
                time.sleep(check_interval)
                continue
            try:
                # 尝试查询最新的变换（time=0 表示 latest）
                trans: TransformStamped = tf_buffer.lookup_transform(
                    target_frame="map",
                    source_frame="odom",
                    time=rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(
                        seconds=0.1
                    ),  # 小超时避免阻塞
                )
                if (self._transform_is_from_current_session(trans)
                        and self._pose_matches_transform(result.pose, trans)):
                    consecutive_matches += 1
                    if consecutive_matches >= 3:
                        self.get_logger().info(
                            '本轮定位结果与 map -> odom TF 连续一致，TF 已就绪')
                        return True
                else:
                    consecutive_matches = 0
                    self.get_logger().warning(
                        'TF 与本轮定位结果不一致，拒绝使用该 TF')

            except (
                LookupException,
                ConnectivityException,
                ExtrapolationException,
            ):
                # 正常等待中，不报错
                self.get_logger().debug("map -> odom TF 尚未可用，继续等待...")

            except Exception as ex:
                self.get_logger().warn(f"TF 查询异常: {ex}")

            # 避免 CPU 空转
            time.sleep(check_interval)
        if (deadline_monotonic is not None
                and time.monotonic() >= deadline_monotonic):
            self.get_logger().warning('地图切换延迟启动已到截止时间')
            return None
        self.get_logger().error(f"等待 map -> odom TF 超时 ({timeout_sec}s)")
        return False

    def verify_stack_ready(
        self, *, generation=None, deadline_monotonic=None
    ):
        """
        验证导航堆栈已就绪

        返回:
            堆栈就绪返回True，否则返回False
        """
        # 检查所有进程是否正在运行
        if not self._deferred_switch_is_current(
                generation, deadline_monotonic):
            return None
        req = GetProcessStatus.Request()
        future = self.get_process_status_client.call_async(req)
        if generation is None and deadline_monotonic is None:
            status = self._wait_for_future(
                future,
                timeout_sec=self.PROCESS_STATUS_TIMEOUT_SEC,
            )
        else:
            status = self._wait_for_future_until(
                future,
                timeout_sec=self.PROCESS_STATUS_TIMEOUT_SEC,
                generation=generation,
                deadline_monotonic=deadline_monotonic,
            )

        if status is None:
            if not self._deferred_switch_is_current(
                    generation, deadline_monotonic):
                return None
            self.get_logger().error('无法获取进程状态')
            return False

        if not status.re_localization_running:
            self.get_logger().error('重定位进程未运行')
            return False

        if not status.liosam_running:
            self.get_logger().error('LIO-SAM进程未运行')
            return False

        if not status.nav2_init_pose_running:
            self.get_logger().error('nav2_init_pose进程未运行')
            return False

        if not status.navigation2_running:
            self.get_logger().error('Navigation2进程未运行')
            return False

        with self._localization_lock:
            expected_map = self._expected_localization_map
            expected_session = self._expected_localization_session
        if getattr(status, 'stack_state', '') != 'RUNNING':
            self.get_logger().error('进程状态不是完整 RUNNING，拒绝完成切换')
            return False
        if getattr(status, 'active_map', '') != expected_map:
            self.get_logger().error(
                '进程状态地图与本次切换不一致: '
                f'{getattr(status, "active_map", "")!r} != {expected_map!r}')
            return False
        if (getattr(status, 'localization_session_id', '')
                != expected_session):
            self.get_logger().error('进程状态定位会话与本次切换不一致')
            return False

        self.get_logger().info('所有进程已验证并正在运行')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MapSwitchController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
