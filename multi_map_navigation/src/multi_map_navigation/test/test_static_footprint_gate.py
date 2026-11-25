from pathlib import Path

from multi_map_navigation.static_footprint_gate import StaticMapFootprintChecker


def write_map(tmp_path, rows):
    height = len(rows)
    width = len(rows[0])
    pgm_path = Path(tmp_path) / 'map.pgm'
    pgm_path.write_bytes(
        f'P5\n{width} {height}\n255\n'.encode() + bytes(
            pixel for row in rows for pixel in row))
    yaml_path = Path(tmp_path) / 'map.yaml'
    yaml_path.write_text(
        'image: map.pgm\n'
        'mode: trinary\n'
        'resolution: 1.0\n'
        'origin: [0.0, 0.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8')
    return StaticMapFootprintChecker(str(yaml_path))


def test_free_footprint_is_legal(tmp_path):
    checker = write_map(tmp_path, [[255] * 7 for _ in range(7)])

    legal, _ = checker.check_rectangular_footprint(
        3.5, 3.5, 0.0, half_length=0.4, half_width=0.3, padding=0.0)

    assert legal


def test_occupied_cell_under_footprint_is_illegal(tmp_path):
    rows = [[255] * 7 for _ in range(7)]
    # PGM rows are top-to-bottom; row 3 maps to occupancy cell y=3.
    rows[3][3] = 0
    checker = write_map(tmp_path, rows)

    legal, reason = checker.check_rectangular_footprint(
        3.5, 3.5, 0.0, half_length=0.4, half_width=0.3, padding=0.0)

    assert not legal
    assert 'occupied' in reason


def test_unknown_cell_and_outside_map_are_illegal(tmp_path):
    rows = [[255] * 7 for _ in range(7)]
    rows[3][3] = 128
    checker = write_map(tmp_path, rows)

    unknown_legal, unknown_reason = checker.check_rectangular_footprint(
        3.5, 3.5, 0.0, half_length=0.4, half_width=0.3, padding=0.0)
    outside_legal, outside_reason = checker.check_rectangular_footprint(
        -0.2, 3.5, 0.0, half_length=0.4, half_width=0.3, padding=0.0)

    assert not unknown_legal
    assert 'unknown' in unknown_reason
    assert not outside_legal
    assert 'unknown' in outside_reason
