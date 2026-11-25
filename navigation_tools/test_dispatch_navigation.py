import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).with_name("dispatch_navigation.py")
SPEC = importlib.util.spec_from_file_location("dispatch_navigation", MODULE_PATH)
dispatch_navigation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dispatch_navigation)


@pytest.fixture(scope="module")
def catalog():
    return dispatch_navigation.load_catalog()


def test_catalog_is_valid_and_contains_four_maps(catalog):
    assert set(catalog["maps"]) == {"3b2mus", "map1", "map2", "map3"}


def test_resolve_by_id_with_map(catalog):
    map_name, point = dispatch_navigation.resolve_point(catalog, "103", "map1")
    assert map_name == "map1"
    assert point["name"] == "三角T字路口点"


def test_resolve_by_unique_name(catalog):
    map_name, point = dispatch_navigation.resolve_point(catalog, "机车博物馆路口")
    assert map_name == "3b2mus"
    assert point["id"] == 108


def test_known_vehicle_route(catalog):
    map_data = catalog["maps"]["3b2mus"]
    path, weight, _ = dispatch_navigation.safe_shortest_path(map_data, 106, 108)
    assert path == [106, 107, 108]
    assert weight == pytest.approx(45.0)


@pytest.mark.parametrize(
    ("map_name", "start_id", "goal_id"),
    [
        ("map1", 103, 85),
        ("map2", 60, 67),
        ("map3", 93, 99),
    ],
)
def test_each_historical_map_has_a_dispatchable_route(
    catalog, map_name, start_id, goal_id
):
    path, weight, _ = dispatch_navigation.safe_shortest_path(
        catalog["maps"][map_name], start_id, goal_id
    )
    assert path[0] == start_id
    assert path[-1] == goal_id
    assert weight > 0.0


def test_unsupported_type_is_not_used_as_intermediate(catalog):
    map_data = catalog["maps"]["3b2mus"]
    path, _, _ = dispatch_navigation.safe_shortest_path(map_data, 111, 109)
    assert 114 not in path


def test_direction_labels():
    assert dispatch_navigation.point_direction(0.0).startswith("东")
    assert dispatch_navigation.point_direction(1.57).startswith("北")
    assert dispatch_navigation.point_direction(-1.57).startswith("南")
