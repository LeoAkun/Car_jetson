import threading

from multi_map_navigation.process_manager import ProcessManagerNode
from geometry_msgs.msg import PoseStamped


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))

    def error(self, message):
        self.messages.append(('error', message))


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class FakeProcess:
    def __init__(self, pid, returncode=None):
        self.pid = pid
        self.returncode = returncode

    def poll(self):
        return self.returncode


def make_manager():
    manager = ProcessManagerNode.__new__(ProcessManagerNode)
    manager.processes = {}
    manager.process_states = {}
    manager.health_failures = {}
    manager.fault_reasons = {}
    manager.confirmed_healthy_processes = set()
    manager.health_failure_threshold = 1
    manager.shutdown_timeout = 0.0
    manager.startup_delay = 0.0
    manager.map_dir = '/maps'
    manager.fault_pub = FakePublisher()
    manager.logger = FakeLogger()
    manager.get_logger = lambda: manager.logger
    return manager


def seed_healthy_relocalization(
        manager, process, map_name, session_id, session_start_ns,
        generation=1):
    manager._ensure_process_tracking()
    manager.processes['re_localization'] = process
    manager.process_pgids['re_localization'] = process.pid
    manager.process_maps['re_localization'] = map_name
    manager.process_started_at['re_localization'] = 1.0
    manager.process_generations['re_localization'] = generation
    manager.process_cancel_events['re_localization'] = threading.Event()
    manager.process_states['re_localization'] = 'HEALTHY'
    manager.health_failures['re_localization'] = 0
    manager.confirmed_healthy_processes.add('re_localization')
    manager.active_localization_session = session_id
    manager.active_map = map_name
    manager.localization_session_start_ns = session_start_ns


def test_live_same_name_is_rejected_without_overwriting(monkeypatch):
    manager = make_manager()
    existing = FakeProcess(101, returncode=1)
    manager.processes = {'re_localization': existing}
    manager.process_pgids = {'re_localization': 501}
    manager.process_maps = {'re_localization': 'old_map'}
    manager._live_process_group_members = lambda _pgid: {202}

    def unexpected_popen(*_args, **_kwargs):
        raise AssertionError('a live process group must not be overwritten')

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        unexpected_popen,
    )

    result = manager._spawn_tracked_process(
        're_localization', 'new_map', ['ignored'], start_new_session=True)

    assert result is None
    assert manager.processes['re_localization'] is existing
    assert manager.process_pgids['re_localization'] == 501
    assert manager.process_maps['re_localization'] == 'old_map'


def test_untracked_residual_process_blocks_new_launch(monkeypatch):
    manager = make_manager()
    manager.external_process_guard_enabled = True
    manager._find_untracked_processes = lambda _name: [
        (4242, '/old/install/re_localization/pcl_localization_node')]

    def unexpected_popen(*_args, **_kwargs):
        raise AssertionError('must not stack a new process over a residual')

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        unexpected_popen,
    )

    result = manager._spawn_tracked_process(
        're_localization', 'map_new', ['ignored'], start_new_session=True)

    assert result is None
    assert manager.process_states['re_localization'] == 'FAILED'
    assert 'pid=[4242]' in manager.fault_reasons['re_localization']


def test_untracked_residual_cannot_be_reported_as_shutdown_success():
    manager = make_manager()
    manager.external_process_guard_enabled = True
    manager._find_untracked_processes = lambda _name: [
        (5252, '/old/nav2_container --ros-args')]

    assert not manager.shutdown_process('navigation2')
    assert manager.process_states['navigation2'] == 'FAILED'
    assert 'pid=[5252]' in manager.fault_reasons['navigation2']


def test_spawn_immediately_records_pgid_and_map(monkeypatch):
    manager = make_manager()
    child = FakeProcess(303)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        lambda *_args, **_kwargs: child,
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid',
        lambda pid: 703 if pid == 303 else None,
    )

    result = manager._spawn_tracked_process(
        'navigation2', 'map_7', ['command'], start_new_session=True)

    assert result == (child, manager.process_generations['navigation2'])
    assert manager.processes['navigation2'] is child
    assert manager.process_pgids['navigation2'] == 703
    assert manager.process_maps['navigation2'] == 'map_7'
    assert manager.process_started_at['navigation2'] > 0
    assert manager.process_states['navigation2'] == 'STARTING'


def test_group_wait_does_not_treat_parent_exit_as_group_exit(monkeypatch):
    manager = make_manager()
    parent = FakeProcess(401, returncode=0)
    members = iter(({402}, {402}, set()))
    manager.shutdown_timeout = 1.0
    manager._live_process_group_members = lambda _pgid: next(members)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.monotonic',
        lambda: 10.0,
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.sleep', lambda _delay: None)

    assert manager._wait_for_process_group_exit(801, parent)


def test_shutdown_kills_group_and_retains_record_if_member_survives(
        monkeypatch):
    manager = make_manager()
    parent = FakeProcess(501, returncode=0)
    manager.processes = {'navigation2': parent}
    manager.process_pgids = {'navigation2': 901}
    manager.process_maps = {'navigation2': 'map_a'}
    manager.process_started_at = {'navigation2': 1.0}
    manager._live_process_group_members = lambda _pgid: {502}
    manager._wait_for_process_group_exit = (
        lambda _pgid, _process, _deadline=None: False)
    signals = []
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.killpg',
        lambda pgid, sig: signals.append((pgid, sig)),
    )

    assert not manager.shutdown_process('navigation2')
    assert [sig for _, sig in signals] == [15, 9]
    assert manager.processes['navigation2'] is parent
    assert manager.process_pgids['navigation2'] == 901
    assert manager.process_maps['navigation2'] == 'map_a'
    assert manager.process_states['navigation2'] == 'FAILED'
    assert '仍有非僵尸进程 [502]' in manager.fault_reasons['navigation2']


def test_launch_health_failure_rolls_back_only_its_own_group(monkeypatch):
    manager = make_manager()
    child = FakeProcess(601)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        lambda *_args, **_kwargs: child,
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid', lambda _pid: 1001)
    manager.wait_for_process_healthy = lambda _name, _generation: False
    shutdown_calls = []
    manager.shutdown_process = (
        lambda name, preserve_fault=False, **_kwargs:
        shutdown_calls.append((name, preserve_fault)) or True
    )

    assert not manager.launch_liosam()
    assert shutdown_calls == [('liosam', True)]
    assert manager.processes['liosam'] is child
    assert manager.process_states['liosam'] == 'FAILED'


def test_localization_session_is_unique_and_forwarded_to_init_pose(
        monkeypatch):
    manager = make_manager()
    commands = []
    children = iter((FakeProcess(701), FakeProcess(702)))

    def fake_popen(command, **_kwargs):
        commands.append(command)
        return next(children)

    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen', fake_popen)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid', lambda pid: pid)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.uuid.uuid4',
        lambda: 'session-123',
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.time_ns',
        lambda: 123456789,
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.sleep', lambda _delay: None)

    def mark_running(name, *_instance):
        manager.process_states[name] = 'HEALTHY'
        return True

    manager.is_process_running = mark_running
    manager._wait_for_relocalization_ready = lambda *_args: (True, '')

    assert manager.launch_relocalization('map_b')
    assert manager.active_localization_session == 'session-123'
    assert manager.active_map == 'map_b'
    assert manager.localization_session_start_ns == 123456789
    assert 'localization_session_id:=session-123' in commands[0]
    assert 'map_id:=map_b' in commands[0]
    assert 'session_start_ns:=123456789' in commands[0]
    assert 'autostart:=false' in commands[0]

    assert manager.launch_nav2_init_pose('map_b')
    assert 'localization_session_id:=session-123' in commands[1]
    assert 'map_name:=map_b' in commands[1]
    assert 'session_start_ns:=123456789' in commands[1]


def test_each_relocalization_start_gets_a_new_session(monkeypatch):
    manager = make_manager()
    children = [FakeProcess(801), FakeProcess(802)]
    session_ids = iter(('session-one', 'session-two'))
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        lambda *_args, **_kwargs: children.pop(0),
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid', lambda pid: pid)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.uuid.uuid4',
        lambda: next(session_ids),
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.sleep', lambda _delay: None)
    manager._wait_for_relocalization_ready = lambda *_args: (True, '')
    manager._live_process_group_members = lambda _pgid: set()

    assert manager.launch_relocalization('map_one')
    first_session = manager.active_localization_session
    manager.processes['re_localization'].returncode = 0

    assert manager.launch_relocalization('map_two')
    assert first_session == 'session-one'
    assert manager.active_localization_session == 'session-two'
    assert manager.active_map == 'map_two'


def test_map_switch_pose_is_forwarded_to_local_search_parameters(monkeypatch):
    manager = make_manager()
    seed_healthy_relocalization(
        manager,
        FakeProcess(900),
        'map_next',
        'session-seeded',
        987654321,
    )
    commands = []
    child = FakeProcess(901)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        lambda command, **_kwargs: commands.append(command) or child,
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid', lambda pid: pid)
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.time.sleep', lambda _delay: None)
    manager.is_process_running = lambda _name, *_instance: True

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.pose.position.x = 12.5
    initial_pose.pose.position.y = -3.25
    initial_pose.pose.orientation.z = 0.70710678
    initial_pose.pose.orientation.w = 0.70710678

    assert manager.launch_nav2_init_pose('map_next', initial_pose)
    command = commands[0]
    assert 'use_initial_pose:=true' in command
    assert 'initial_x:=12.5' in command
    assert 'initial_y:=-3.25' in command
    yaw_arg = next(value for value in command if value.startswith('initial_yaw:='))
    assert abs(float(yaw_arg.split(':=', 1)[1]) - 1.5707963268) < 1e-6


def test_init_pose_fails_and_cleans_only_itself_when_relocalization_stops(
        monkeypatch):
    manager = make_manager()
    relocalization = FakeProcess(1001)
    init_pose = FakeProcess(1002)
    seed_healthy_relocalization(
        manager,
        relocalization,
        'map_stop',
        'session-stop',
        1234,
        generation=11,
    )
    manager._live_process_group_members = lambda _pgid: set()
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        lambda *_args, **_kwargs: init_pose,
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid', lambda pid: pid)

    real_spawn = manager._spawn_tracked_process

    def spawn_then_stop_dependency(*args, **kwargs):
        spawned = real_spawn(*args, **kwargs)
        assert spawned is not None
        assert manager.shutdown_process(
            're_localization',
            expected_process=relocalization,
            expected_generation=11,
        )
        return spawned

    manager._spawn_tracked_process = spawn_then_stop_dependency

    assert not manager.launch_nav2_init_pose('map_stop')
    assert 'nav2_init_pose' not in manager.processes
    assert manager.process_states['nav2_init_pose'] == 'FAILED'
    assert '停止或被替换' in manager.fault_reasons['nav2_init_pose']
    assert 're_localization' not in manager.processes
    assert manager.process_states['re_localization'] == 'STOPPED'


def test_init_pose_fails_and_does_not_clean_relocalization_replacement(
        monkeypatch):
    manager = make_manager()
    old_relocalization = FakeProcess(1101)
    replacement = FakeProcess(1102)
    init_pose = FakeProcess(1103)
    seed_healthy_relocalization(
        manager,
        old_relocalization,
        'map_replace',
        'session-old',
        5678,
        generation=21,
    )
    manager._live_process_group_members = lambda _pgid: set()
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.subprocess.Popen',
        lambda *_args, **_kwargs: init_pose,
    )
    monkeypatch.setattr(
        'multi_map_navigation.process_manager.os.getpgid', lambda pid: pid)

    real_spawn = manager._spawn_tracked_process

    def spawn_then_replace_dependency(*args, **kwargs):
        spawned = real_spawn(*args, **kwargs)
        assert spawned is not None
        with manager.process_lifecycle_lock:
            manager.processes['re_localization'] = replacement
            manager.process_pgids['re_localization'] = replacement.pid
            manager.process_generations['re_localization'] = 22
            manager.process_cancel_events['re_localization'] = threading.Event()
            manager.process_states['re_localization'] = 'HEALTHY'
            manager.active_localization_session = 'session-new'
            manager.localization_session_start_ns = 6789
        return spawned

    manager._spawn_tracked_process = spawn_then_replace_dependency

    assert not manager.launch_nav2_init_pose('map_replace')
    assert 'nav2_init_pose' not in manager.processes
    assert manager.process_states['nav2_init_pose'] == 'FAILED'
    assert manager.processes['re_localization'] is replacement
    assert manager.process_generations['re_localization'] == 22
    assert manager.process_states['re_localization'] == 'HEALTHY'
