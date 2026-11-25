import networkx as nx

from multi_map_navigation.route_planner import RoutePlannerNode


class FakeLogger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


def make_planner(nodes):
    planner = RoutePlannerNode.__new__(RoutePlannerNode)
    planner.G = nx.Graph()
    planner.G.add_nodes_from(nodes)
    logger = FakeLogger()
    planner.get_logger = lambda: logger
    return planner, logger


def test_cross_map_edge_requires_declared_switch_endpoint():
    ordinary_a = {'map_name': 'map_a', 'type': 1, 'next_map_name': ''}
    ordinary_b = {'map_name': 'map_b', 'type': 1, 'next_map_name': ''}
    switch = {
        'map_name': 'map_a',
        'type': 4,
        'next_map_name': 'map_b',
    }

    assert not RoutePlannerNode._edge_preserves_map_transition(
        ordinary_a, ordinary_b)
    assert RoutePlannerNode._edge_preserves_map_transition(switch, ordinary_b)


def test_valid_route_changes_map_at_switch_waypoint():
    planner, _logger = make_planner([
        ('a', {'map_name': 'map_a', 'type': 1}),
        ('switch', {
            'map_name': 'map_a', 'type': 4,
            'next_map_name': 'map_b'}),
        ('b', {'map_name': 'map_b', 'type': 1}),
    ])

    assert planner.validate_route_map_transitions(['a', 'switch', 'b'])


def test_route_with_silent_map_jump_is_rejected():
    planner, logger = make_planner([
        ('a', {'map_name': 'map_a', 'type': 1}),
        ('b', {'map_name': 'map_b', 'type': 1}),
    ])

    assert not planner.validate_route_map_transitions(['a', 'b'])
    assert logger.errors


def test_switch_must_lead_to_its_declared_next_map():
    planner, logger = make_planner([
        ('switch', {
            'map_name': 'map_a', 'type': 4,
            'next_map_name': 'map_b'}),
        ('wrong', {'map_name': 'map_c', 'type': 1}),
    ])

    assert not planner.validate_route_map_transitions(['switch', 'wrong'])
    assert logger.errors
