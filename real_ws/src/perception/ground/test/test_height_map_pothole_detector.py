import os
import sys

import numpy as np
import pytest


sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ground.height_map_pothole_detector import HeightMapPotholeDetector  # noqa: E402


def make_detector(**overrides):
    params = dict(
        x_min=0.0,
        x_max=1.0,
        y_min=0.0,
        y_max=1.0,
        z_min=-1.0,
        z_max=1.0,
        resolution=0.1,
        min_depth=0.1,
        max_depth=5.0,
        low_threshold=0.08,
        min_cell_points=1,
        min_area_cells=1,
        max_area_cells=300,
        step=1,
        cell_height_percentile=35.0,
        ground_fit_max_iterations=10,
        ground_fit_distance_threshold=0.025,
        ground_fit_min_inlier_ratio=0.45,
        ground_candidate_percentile=70.0,
        ground_candidate_min_cells=3,
        ground_fallback_percentile=75.0,
        max_pothole_depth=0.45,
        min_mean_drop=0.04,
        max_pit_cell_height_spread=0.08,
        ground_contact_threshold=0.05,
        edge_drop_threshold=0.05,
        detect_negative_edges=True,
        include_edges_in_output=True,
        edge_component_confidence_scale=0.6,
        min_edge_component_cells=3,
        min_edge_component_ratio=0.28,
        detect_unknown_holes=False,
        unknown_hole_kernel_cells=5,
        unknown_hole_min_ground_neighbors=5,
        unknown_hole_border_margin_cells=2,
        min_component_pit_cells=3,
        reject_border_components=True,
        reject_near_border=False,
        component_border_margin_cells=1,
        ground_ring_width_cells=2,
        min_ground_ring_cells=0,
        min_ground_ring_ratio=0.0,
        min_raw_mask_area_cells=1,
        mask_dilation_cells=0,
        morph_kernel_cells=3,
        morph_open_iterations=0,
        morph_close_iterations=0,
        confidence_full_area_cells=16,
        min_valid_depth_ratio=0.0,
        min_valid_cell_ratio=0.0,
        min_valid_cells=0,
        random_seed=0,
        auto_roi_smoothing_alpha=1.0,
        max_roi_delta_per_frame=0.0,
        debug_view=False,
    )
    params.update(overrides)
    return HeightMapPotholeDetector(**params)


def square_component(shape=(10, 10), top=3, left=3, size=4):
    mask = np.zeros(shape, dtype=np.uint8)
    mask[top:top + size, left:left + size] = 255
    return mask


def mask_from_cells(shape, cells):
    mask = np.zeros(shape, dtype=np.uint8)
    for y, x in cells:
        mask[y, x] = 255
    return mask


def diff_for_component(mask, base_drop=0.04, peak_drop=0.12):
    diff = np.zeros_like(mask, dtype=np.float32)
    diff[mask > 0] = base_drop
    ys, xs = np.where(mask > 0)
    if len(xs) > 0:
        diff[ys[len(ys) // 2], xs[len(xs) // 2]] = peak_drop
    return diff


def set_masks(detector, low_mask, pit_mask=None, edge_mask=None):
    shape = low_mask.shape
    detector.last_masks = {
        "pit_bottom": (
            np.zeros(shape, dtype=np.uint8)
            if pit_mask is None else
            pit_mask.astype(np.uint8)
        ),
        "negative_edges": (
            np.zeros(shape, dtype=np.uint8)
            if edge_mask is None else
            edge_mask.astype(np.uint8)
        ),
        "ground_like": np.zeros(shape, dtype=np.uint8),
        "valid": np.ones(shape, dtype=np.uint8),
    }


def test_shallow_real_sample_edge_only_component_is_accepted_and_downweighted():
    detector = make_detector()
    low_cells = [(y, x) for y in range(3, 7) for x in range(3, 16)]
    low_mask = mask_from_cells((12, 20), low_cells)
    diff_map = diff_for_component(low_mask)

    edge_cells = (
        [(3, x) for x in range(3, 16)] +
        [(4, x) for x in range(3, 7)]
    )
    edge_mask = mask_from_cells(low_mask.shape, edge_cells)

    set_masks(detector, low_mask, pit_mask=None, edge_mask=edge_mask)
    potholes, accepted_mask, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )
    assert len(potholes) == 1
    edge_confidence = potholes[0]["confidence"]
    assert accepted_mask.sum() > 0
    assert detector.last_debug["accepted_edge_only_components"] == 1
    assert detector.last_debug["rejected_pit_bottom_components"] == 0

    pit_mask = low_mask.copy()
    set_masks(detector, low_mask, pit_mask=pit_mask, edge_mask=edge_mask)
    potholes, _, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )
    assert len(potholes) == 1
    pit_confidence = potholes[0]["confidence"]

    assert edge_confidence == pytest.approx(pit_confidence * 0.6, rel=1e-6)
    assert pit_confidence > edge_confidence


def test_deep_long_real_sample_edge_only_component_is_accepted_and_downweighted():
    detector = make_detector()
    low_cells = [(y, x) for y in range(4, 14) for x in range(3, 18)]
    low_mask = mask_from_cells((20, 24), low_cells)
    diff_map = diff_for_component(low_mask)

    edge_cells = low_cells[:50]
    edge_mask = mask_from_cells(low_mask.shape, edge_cells)

    set_masks(detector, low_mask, pit_mask=None, edge_mask=edge_mask)
    potholes, accepted_mask, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )

    assert len(potholes) == 1
    assert accepted_mask.sum() > 0
    assert detector.last_debug["accepted_edge_only_components"] == 1
    equivalent_radius = np.sqrt((150 * detector.resolution ** 2) / np.pi)
    assert potholes[0]["radius"] > equivalent_radius


def test_edge_weak_ratio_is_rejected():
    detector = make_detector()
    low_mask = square_component()
    diff_map = diff_for_component(low_mask, base_drop=0.03, peak_drop=0.06)

    edge_mask = np.zeros_like(low_mask, dtype=np.uint8)
    edge_mask[3:5, 3] = 255

    set_masks(detector, low_mask, pit_mask=None, edge_mask=edge_mask)
    potholes, accepted_mask, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )

    assert potholes == []
    assert np.count_nonzero(accepted_mask) == 0
    assert detector.last_debug["rejected_pit_bottom_components"] == 1
    assert detector.last_debug["accepted_edge_only_components"] == 0


def test_edge_low_drop_is_rejected():
    detector = make_detector()
    low_cells = [(y, x) for y in range(3, 7) for x in range(3, 16)]
    low_mask = mask_from_cells((12, 20), low_cells)
    diff_map = diff_for_component(low_mask, base_drop=0.03, peak_drop=0.05)

    edge_cells = [(3, x) for x in range(3, 16)] + [(4, x) for x in range(3, 7)]
    edge_mask = mask_from_cells(low_mask.shape, edge_cells)

    set_masks(detector, low_mask, pit_mask=None, edge_mask=edge_mask)
    potholes, accepted_mask, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )

    assert potholes == []
    assert np.count_nonzero(accepted_mask) == 0
    assert detector.last_debug["rejected_pit_bottom_components"] == 1


def test_near_border_only_is_allowed_by_default():
    detector = make_detector()
    low_mask = square_component(left=0)
    diff_map = diff_for_component(low_mask)

    set_masks(detector, low_mask, pit_mask=low_mask, edge_mask=None)
    potholes, accepted_mask, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )

    assert len(potholes) == 1
    assert np.count_nonzero(accepted_mask) > 0
    assert detector.last_debug["rejected_border_components"] == 0


def test_near_border_only_can_be_rejected_when_compat_flag_is_set():
    detector = make_detector(reject_near_border=True)
    low_mask = square_component(left=0)
    diff_map = diff_for_component(low_mask)

    set_masks(detector, low_mask, pit_mask=low_mask, edge_mask=None)
    potholes, accepted_mask, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )

    assert potholes == []
    assert np.count_nonzero(accepted_mask) == 0
    assert detector.last_debug["rejected_border_components"] == 1


def test_far_border_component_is_rejected():
    detector = make_detector()
    low_mask = square_component(left=6)
    diff_map = diff_for_component(low_mask)

    set_masks(detector, low_mask, pit_mask=low_mask, edge_mask=None)
    potholes, accepted_mask, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )

    assert potholes == []
    assert np.count_nonzero(accepted_mask) == 0
    assert detector.last_debug["rejected_border_components"] == 1


def test_side_border_component_is_accepted_with_confidence_degradation():
    detector = make_detector()
    low_mask = square_component(top=0, left=0)
    diff_map = diff_for_component(low_mask)

    set_masks(detector, low_mask, pit_mask=low_mask, edge_mask=None)
    potholes, accepted_mask, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )

    assert len(potholes) == 1
    assert np.count_nonzero(accepted_mask) > 0
    assert detector.last_debug["rejected_border_components"] == 0
    assert detector.last_debug["border_degraded_components"] == 1
    assert potholes[0]["confidence"] == pytest.approx(0.4484375, rel=1e-6)


def test_pit_bottom_normal_component_is_accepted():
    detector = make_detector()
    low_mask = square_component(size=5)
    diff_map = diff_for_component(low_mask)

    pit_mask = low_mask.copy()
    set_masks(detector, low_mask, pit_mask=pit_mask, edge_mask=None)
    potholes, accepted_mask, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )

    assert len(potholes) == 1
    assert np.count_nonzero(accepted_mask) > 0
    assert detector.last_debug["accepted_components"] == 1


def test_flat_ground_no_pothole():
    detector = make_detector()
    low_mask = np.zeros((10, 10), dtype=np.uint8)
    diff_map = np.zeros_like(low_mask, dtype=np.float32)

    set_masks(detector, low_mask, pit_mask=None, edge_mask=None)
    potholes, accepted_mask, _ = detector.extract_potholes_from_grid(
        low_mask,
        diff_map,
        None
    )

    assert potholes == []
    assert np.count_nonzero(accepted_mask) == 0
