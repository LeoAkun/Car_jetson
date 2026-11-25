#!/usr/bin/env python3

import json
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import Point, PolygonStamped, Point32
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, String

from cv_bridge import CvBridge

import tf2_ros
from tf_transformations import quaternion_matrix

from ground.dropoff_detector import GroundRayDropoffDetector


class HeightMapPotholeDetector:
    """
    深度图 + 局部高度地图坑洞检测器。

    核心流程：
        1. 深度图反投影为相机点云
        2. 通过 TF 转到 base_link
        3. 在机器人前方 ROI 内建立局部高度栅格
        4. 使用 RANSAC 拟合地面平面
        5. 提取低于地面平面的连通低洼区域

    输入：
        depth_img: 深度图，单位 m
        K: 相机内参
        R, t: camera_frame 到 base_link 的变换

    输出：
        potholes:
            [
                {
                    "x": base_link 下坑洞中心 x,
                    "y": base_link 下坑洞中心 y,
                    "radius": 坑洞半径,
                    "area": 面积
                    "z": base_link 下估计地面高度
                }
            ]
    """

    def __init__(
        self,
        x_min=0.45,
        x_max=3.0,
        y_min=-1.0,
        y_max=1.0,
        z_min=-1.0,
        z_max=1.0,
        resolution=0.05,
        min_depth=0.1,
        max_depth=5.0,
        low_threshold=0.03,
        min_cell_points=2,
        min_area_cells=4,
        max_area_cells=800,
        step=2,
        cell_height_percentile=35.0,
        ground_fit_max_iterations=80,
        ground_fit_distance_threshold=0.025,
        ground_fit_min_inlier_ratio=0.45,
        ground_candidate_percentile=70.0,
        ground_candidate_min_cells=20,
        ground_fallback_percentile=75.0,
        max_pothole_depth=0.45,
        min_mean_drop=0.04,
        max_pit_cell_height_spread=0.08,
        ground_contact_threshold=0.05,
        edge_drop_threshold=0.05,
        detect_negative_edges=True,
        include_edges_in_output=False,
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
        side_border_confidence_scale=0.7,
        ground_ring_width_cells=2,
        min_ground_ring_cells=4,
        min_ground_ring_ratio=0.15,
        min_raw_mask_area_cells=2,
        mask_dilation_cells=1,
        morph_kernel_cells=3,
        morph_open_iterations=0,
        morph_close_iterations=1,
        confidence_full_area_cells=0,
        min_valid_depth_ratio=0.0,
        min_valid_cell_ratio=0.0,
        min_valid_cells=0,
        random_seed=-1,
        auto_roi_smoothing_alpha=1.0,
        max_roi_delta_per_frame=0.0,
        debug_view=False,
        ground_reference_enabled=False,
        ground_reference_z=-0.670,
        ground_reference_tolerance=0.030,
        ground_max_slope=0.35,
    ):
        # base_link 下检测范围
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.z_min = z_min
        self.z_max = z_max

        # 高度地图分辨率
        self.resolution = resolution

        # 深度有效范围，单位 m
        self.min_depth = min_depth
        self.max_depth = max_depth

        # 低于地面多少认为是坑洞，单位 m
        self.low_threshold = low_threshold

        # 一个栅格内最少点数
        self.min_cell_points = min_cell_points

        # 最小坑洞面积，单位：栅格数量
        self.min_area_cells = min_area_cells
        self.max_area_cells = max_area_cells

        # 深度图降采样步长
        self.step = step

        self.cell_height_percentile = float(
            np.clip(cell_height_percentile, 0.0, 100.0)
        )
        self.ground_fit_max_iterations = int(ground_fit_max_iterations)
        self.ground_fit_distance_threshold = float(ground_fit_distance_threshold)
        self.ground_fit_min_inlier_ratio = float(ground_fit_min_inlier_ratio)
        self.ground_candidate_percentile = float(
            np.clip(ground_candidate_percentile, 0.0, 100.0)
        )
        self.ground_candidate_min_cells = int(ground_candidate_min_cells)
        self.ground_fallback_percentile = float(
            np.clip(ground_fallback_percentile, 0.0, 100.0)
        )
        self.max_pothole_depth = float(max_pothole_depth)
        self.min_mean_drop = float(min_mean_drop)
        self.max_pit_cell_height_spread = float(max_pit_cell_height_spread)
        self.ground_contact_threshold = float(ground_contact_threshold)
        self.edge_drop_threshold = float(edge_drop_threshold)
        self.detect_negative_edges = bool(detect_negative_edges)
        self.include_edges_in_output = bool(include_edges_in_output)
        self.edge_component_confidence_scale = float(
            np.clip(edge_component_confidence_scale, 0.0, 1.0)
        )
        self.min_edge_component_cells = int(min_edge_component_cells)
        self.min_edge_component_ratio = float(
            np.clip(min_edge_component_ratio, 0.0, 1.0)
        )
        self.detect_unknown_holes = bool(detect_unknown_holes)
        self.unknown_hole_kernel_cells = self._make_odd(
            unknown_hole_kernel_cells,
            minimum=3
        )
        self.unknown_hole_min_ground_neighbors = int(unknown_hole_min_ground_neighbors)
        self.unknown_hole_border_margin_cells = int(unknown_hole_border_margin_cells)
        self.min_component_pit_cells = int(min_component_pit_cells)
        self.reject_border_components = bool(reject_border_components)
        self.reject_near_border = bool(reject_near_border)
        self.component_border_margin_cells = int(component_border_margin_cells)
        self.side_border_confidence_scale = float(
            np.clip(side_border_confidence_scale, 0.0, 1.0)
        )
        self.ground_ring_width_cells = int(ground_ring_width_cells)
        self.min_ground_ring_cells = int(min_ground_ring_cells)
        self.min_ground_ring_ratio = float(min_ground_ring_ratio)
        self.min_raw_mask_area_cells = int(min_raw_mask_area_cells)
        self.mask_dilation_cells = int(mask_dilation_cells)
        self.morph_kernel_cells = self._make_odd(
            morph_kernel_cells,
            minimum=1
        )
        self.morph_open_iterations = int(morph_open_iterations)
        self.morph_close_iterations = int(morph_close_iterations)
        self.confidence_full_area_cells = float(confidence_full_area_cells)
        if self.confidence_full_area_cells <= 0.0:
            self.confidence_full_area_cells = min(
                float(self.max_area_cells),
                max(float(self.min_area_cells) * 3.0, 1.0)
            )
        self.min_valid_depth_ratio = float(min_valid_depth_ratio)
        self.min_valid_cell_ratio = float(min_valid_cell_ratio)
        self.min_valid_cells = int(min_valid_cells)
        self.rng = (
            np.random.RandomState(int(random_seed))
            if int(random_seed) >= 0 else
            np.random.RandomState()
        )
        self.auto_roi_smoothing_alpha = float(
            np.clip(auto_roi_smoothing_alpha, 0.0, 1.0)
        )
        self.max_roi_delta_per_frame = float(max_roi_delta_per_frame)
        self.last_debug = {}
        self.last_masks = {}
        self.last_detection_fields = {}
        self.last_ground_plane = None
        self.ground_reference_enabled = bool(ground_reference_enabled)
        self.ground_reference_z = float(ground_reference_z)
        self.ground_reference_tolerance = float(ground_reference_tolerance)
        self.ground_max_slope = float(ground_max_slope)

        # 是否显示调试窗口
        self.debug_view = debug_view

    @staticmethod
    def _make_odd(value, minimum=1):
        value = max(int(value), int(minimum))
        if value % 2 == 0:
            value += 1
        return value

    def process(self, depth_img, K, R, t):
        """
        主处理函数
        """
        self.last_detection_fields = {}
        self.last_masks = {}
        self.last_ground_plane = None
        points_camera = self.depth_to_camera_points(depth_img, K)

        if points_camera is None or len(points_camera) == 0:
            self.last_debug['quality_reject_reason'] = 'no_valid_depth'
            return [], None, None, None, None, None

        depth_valid_ratio = self.last_debug.get("depth_valid_ratio", 1.0)
        if depth_valid_ratio < self.min_valid_depth_ratio:
            self.last_debug.update({
                "quality_reject_reason": "depth_valid_ratio",
            })
            # Still try to verify the remaining upper-ground support. An edge
            # can have a sound near side while most farther rays have no return.

        points_base = self.transform_points(points_camera, R, t)

        points_roi = self.filter_roi_base(points_base)

        if points_roi is None or len(points_roi) < 50:
            self.last_debug.update({
                "roi_points": 0 if points_roi is None else len(points_roi),
                "quality_reject_reason": "roi_points",
            })
            return [], None, None, None, None, None

        height_map, count_map, spread_map = self.build_height_map(points_roi)

        valid_cells = (
            np.isfinite(height_map) &
            (count_map >= self.min_cell_points)
        )
        valid_cell_count = int(np.count_nonzero(valid_cells))
        valid_cell_ratio = (
            valid_cell_count / float(height_map.size)
            if height_map.size > 0 else
            0.0
        )
        self.last_debug.update({
            "valid_cell_ratio": valid_cell_ratio,
            "valid_cell_count": valid_cell_count,
        })
        if (
            valid_cell_count < self.min_valid_cells or
            valid_cell_ratio < self.min_valid_cell_ratio
        ):
            self.last_debug.setdefault('quality_reject_reason', 'valid_cells')

        ground_map, ground_z = self.estimate_ground_map(height_map, count_map)

        if ground_map is None:
            self.last_debug.setdefault('quality_reject_reason', 'ground_fit')
            return [], None, height_map, None, None, None
        if self.last_debug.get('quality_reject_reason'):
            # Grid quality gates the pit branch; the raw-ray branch has its own
            # quality gates and uses only this frame's verified upper plane.
            empty_mask = np.zeros_like(height_map, dtype=np.uint8)
            return [], empty_mask, height_map, empty_mask, None, None

        low_mask, diff_map = self.detect_low_cells(
            height_map,
            count_map,
            spread_map,
            ground_map
        )

        low_mask = self.postprocess_mask(low_mask)
        self.last_debug["mask_cells"] = int(np.count_nonzero(low_mask))

        potholes, accepted_mask, rejection_debug_image = self.extract_potholes_from_grid(
            low_mask,
            diff_map,
            ground_map
        )

        self.last_debug["potholes"] = len(potholes)
        self.last_debug["accepted_cells"] = (
            int(np.count_nonzero(accepted_mask))
            if accepted_mask is not None else 0
        )

        for p in potholes:
            p["z"] = float(ground_z)

        if self.debug_view:
            self.visualize_height_map(diff_map, accepted_mask, ground_z)

        return potholes, accepted_mask, height_map, low_mask, diff_map, rejection_debug_image

    def depth_to_camera_points(self, depth_img, K):
        """
        深度图反投影到相机坐标系

        相机光学坐标系一般为：
            X: 图像右方
            Y: 图像下方
            Z: 相机前方
        """
        H, W = depth_img.shape

        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        step = self.step

        us, vs = np.meshgrid(
            np.arange(0, W, step),
            np.arange(0, H, step)
        )

        ds = depth_img[vs, us]

        valid = (
            np.isfinite(ds) &
            (ds > self.min_depth) &
            (ds < self.max_depth)
        )
        self.last_debug = {
            "depth_valid_ratio": (
                float(np.count_nonzero(valid)) / float(valid.size)
                if valid.size > 0 else
                0.0
            ),
            "depth_valid_pixels": int(np.count_nonzero(valid)),
            "depth_sampled_pixels": int(valid.size),
        }

        if np.count_nonzero(valid) == 0:
            return np.empty((0, 3), dtype=np.float32)

        us_valid = us[valid].astype(np.float32)
        vs_valid = vs[valid].astype(np.float32)
        ds_valid = ds[valid].astype(np.float32)

        X = (us_valid - cx) * ds_valid / fx
        Y = (vs_valid - cy) * ds_valid / fy
        Z = ds_valid

        points_camera = np.stack([X, Y, Z], axis=1).astype(np.float32)

        return points_camera

    def transform_points(self, points, R, t):
        """
        点云从 camera_frame 转到 base_link
        """
        return (R @ points.T + t.reshape(3, 1)).T

    def filter_roi_base(self, points_base):
        """
        筛选机器人前方区域
        base_link 坐标：
            x: 前方
            y: 左方
            z: 上方
        """
        x = points_base[:, 0]
        y = points_base[:, 1]
        z = points_base[:, 2]

        mask = (
            (x > self.x_min) & (x < self.x_max) &
            (y > self.y_min) & (y < self.y_max) &
            (z > self.z_min) & (z < self.z_max)
        )

        return points_base[mask]

    def update_roi_from_camera_view(
        self,
        K,
        image_shape,
        R,
        t,
        ground_z,
        margin,
        min_span,
        x_limits,
        y_limits
    ):
        """
        根据相机视锥和 base_link 下的地面高度自动计算检测 ROI。

        相机安装位置和俯角来自 TF，因此 xacro/launch 改相机位姿后，
        检测范围会跟着变化。
        """
        height, width = image_shape
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        us = np.linspace(0.0, float(width - 1), 7, dtype=np.float32)
        vs = np.linspace(0.0, float(height - 1), 7, dtype=np.float32)
        intersections = []

        for v in vs:
            for u in us:
                ray_camera = np.array(
                    [(u - cx) / fx, (v - cy) / fy, 1.0],
                    dtype=np.float32
                )
                ray_base = R @ ray_camera
                if ray_base[2] >= -1e-3:
                    continue

                scale = (float(ground_z) - float(t[2])) / float(ray_base[2])
                if scale <= 0.0:
                    continue

                point = t + scale * ray_base
                intersections.append(point)

        if len(intersections) < 3:
            return False

        points = np.array(intersections, dtype=np.float32)
        x_min = max(float(x_limits[0]), float(np.min(points[:, 0]) - margin))
        x_max = min(float(x_limits[1]), float(np.max(points[:, 0]) + margin))
        y_min = max(float(y_limits[0]), float(np.min(points[:, 1]) - margin))
        y_max = min(float(y_limits[1]), float(np.max(points[:, 1]) + margin))

        if x_max - x_min < min_span or y_max - y_min < min_span:
            return False

        x_min = self._smooth_roi_value(self.x_min, x_min)
        x_max = self._smooth_roi_value(self.x_max, x_max)
        y_min = self._smooth_roi_value(self.y_min, y_min)
        y_max = self._smooth_roi_value(self.y_max, y_max)

        if x_max - x_min < min_span or y_max - y_min < min_span:
            return False

        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

        self.last_debug.update({
            "auto_roi": True,
            "roi_x_min": x_min,
            "roi_x_max": x_max,
            "roi_y_min": y_min,
            "roi_y_max": y_max,
        })

        return True

    def _smooth_roi_value(self, previous, target):
        alpha = self.auto_roi_smoothing_alpha
        value = float(previous) + alpha * (float(target) - float(previous))
        max_delta = self.max_roi_delta_per_frame
        if max_delta > 0.0:
            value = float(np.clip(
                value,
                float(previous) - max_delta,
                float(previous) + max_delta
            ))
        return value

    def build_height_map(self, points_base):
        """
        构建局部高度地图。

        每个栅格记录：
            height_map: 指定百分位高度，避免单个低噪声点触发误检
            count_map: 点数量
            spread_map: 栅格内高度 90%-10% 分位差，竖直边缘通常更大
        """
        width = int((self.x_max - self.x_min) / self.resolution)
        height = int((self.y_max - self.y_min) / self.resolution)

        height_map = np.full((height, width), np.nan, dtype=np.float32)
        spread_map = np.full((height, width), np.nan, dtype=np.float32)
        count_map = np.zeros((height, width), dtype=np.int32)

        xs = points_base[:, 0]
        ys = points_base[:, 1]
        zs = points_base[:, 2]

        ix = ((xs - self.x_min) / self.resolution).astype(np.int32)
        iy = ((ys - self.y_min) / self.resolution).astype(np.int32)

        valid = (
            (ix >= 0) & (ix < width) &
            (iy >= 0) & (iy < height)
        )

        if np.count_nonzero(valid) == 0:
            return height_map, count_map, spread_map

        ix = ix[valid]
        iy = iy[valid]
        zs = zs[valid]
        linear = iy * width + ix

        # Sorting by cell and height lets all per-cell percentiles be computed
        # in one array operation; fine grids must not run Python per cell.
        order = np.lexsort((zs, linear))
        linear_sorted = linear[order]
        z_sorted = zs[order]

        unique, starts, counts = np.unique(
            linear_sorted,
            return_index=True,
            return_counts=True
        )

        count_map.flat[unique] = counts
        enough = counts >= self.min_cell_points
        cells = unique[enough]
        starts = starts[enough]
        counts = counts[enough]

        def percentile(q):
            ranks = (counts - 1) * (q / 100.0)
            lo = np.floor(ranks).astype(np.int64)
            hi = np.ceil(ranks).astype(np.int64)
            fraction = ranks - lo
            return (z_sorted[starts + lo] * (1.0 - fraction) +
                    z_sorted[starts + hi] * fraction)

        height_map.flat[cells] = percentile(self.cell_height_percentile)
        spread_map.flat[cells] = percentile(90.0) - percentile(10.0)

        return height_map, count_map, spread_map

    def estimate_ground_map(self, height_map, count_map):
        """
        使用栅格高度点 RANSAC 拟合局部地面平面 z = ax + by + c。
        """
        valid = np.isfinite(height_map) & (count_map >= self.min_cell_points)

        if np.count_nonzero(valid) < 10:
            return None, None

        ys, xs = np.where(valid)
        x_coords = self.x_min + (xs.astype(np.float32) + 0.5) * self.resolution
        y_coords = self.y_min + (ys.astype(np.float32) + 0.5) * self.resolution
        z_coords = height_map[ys, xs].astype(np.float32)

        if self.ground_reference_enabled:
            # A lower floor dominating the image must not become the new road
            # plane. The reference is installation/test-fixture calibration.
            candidate_mask = (
                np.abs(z_coords - self.ground_reference_z) <=
                self.ground_reference_tolerance
            )
            if np.count_nonzero(candidate_mask) < max(10, self.ground_candidate_min_cells):
                self.last_debug.update({
                    'ground_fit_status': 'reference_unavailable',
                    'quality_reject_reason': 'ground_reference',
                })
                return None, None
        else:
            candidate_mask = self.select_ground_candidates(z_coords)
        fit_x = x_coords[candidate_mask]
        fit_y = y_coords[candidate_mask]
        fit_z = z_coords[candidate_mask]

        plane = self.fit_ground_plane(fit_x, fit_y, fit_z)
        if plane is None:
            if self.ground_reference_enabled:
                self.last_debug['ground_fit_status'] = 'fit_failed'
                return None, None
            ground_z = float(np.percentile(z_coords, self.ground_fallback_percentile))
            plane = (0.0, 0.0, ground_z)
            fit_status = "fallback_percentile"
        else:
            fit_status = "ransac_candidates"

        self.last_debug.update({
            "ground_fit_status": fit_status,
            "ground_candidate_cells": int(np.count_nonzero(candidate_mask)),
        })

        a, b, c = plane
        if not np.all(np.isfinite(plane)) or math.hypot(a, b) > self.ground_max_slope:
            self.last_debug.update({
                'ground_fit_status': 'slope_rejected',
                'quality_reject_reason': 'ground_slope',
            })
            return None, None
        self.last_ground_plane = tuple(float(v) for v in plane)
        width = height_map.shape[1]
        grid_height = height_map.shape[0]
        grid_xs = self.x_min + (
            np.arange(width, dtype=np.float32) + 0.5
        ) * self.resolution
        grid_ys = self.y_min + (
            np.arange(grid_height, dtype=np.float32) + 0.5
        ) * self.resolution
        xx, yy = np.meshgrid(grid_xs, grid_ys)
        ground_map = (a * xx + b * yy + c).astype(np.float32)
        ground_z = float(np.nanmedian(ground_map[valid]))

        return ground_map, ground_z

    def select_ground_candidates(self, z_coords):
        """
        Prefer higher local surfaces as ground candidates.

        Potholes are negative obstacles, so fitting all cells can select the pit
        bottom when it occupies much of the ROI. Using the upper-height cells
        biases the fit toward the surrounding road surface.
        """
        if len(z_coords) < self.ground_candidate_min_cells:
            return np.ones_like(z_coords, dtype=bool)

        threshold = np.percentile(z_coords, self.ground_candidate_percentile)
        candidates = z_coords >= threshold

        if np.count_nonzero(candidates) < self.ground_candidate_min_cells:
            order = np.argsort(z_coords)
            candidates = np.zeros_like(z_coords, dtype=bool)
            candidates[order[-self.ground_candidate_min_cells:]] = True

        return candidates

    def fit_ground_plane(self, xs, ys, zs):
        point_count = len(zs)
        if point_count < 10:
            return None

        best_inliers = None
        best_error = None

        for _ in range(max(self.ground_fit_max_iterations, 1)):
            sample_ids = self.rng.choice(point_count, size=3, replace=False)
            A = np.column_stack((
                xs[sample_ids],
                ys[sample_ids],
                np.ones(3, dtype=np.float32)
            ))

            if np.linalg.cond(A) > 1e4:
                continue

            try:
                a, b, c = np.linalg.solve(A, zs[sample_ids])
            except np.linalg.LinAlgError:
                continue

            predicted = a * xs + b * ys + c
            residual = np.abs(zs - predicted)
            inliers = residual < self.ground_fit_distance_threshold
            inlier_count = int(np.count_nonzero(inliers))
            if inlier_count < 10:
                continue

            mean_error = float(np.mean(residual[inliers]))
            if (
                best_inliers is None or
                inlier_count > int(np.count_nonzero(best_inliers)) or
                (
                    inlier_count == int(np.count_nonzero(best_inliers)) and
                    (best_error is None or mean_error < best_error)
                )
            ):
                best_inliers = inliers
                best_error = mean_error

        if best_inliers is None:
            return None

        inlier_ratio = np.count_nonzero(best_inliers) / float(point_count)
        if inlier_ratio < self.ground_fit_min_inlier_ratio:
            return None

        A = np.column_stack((
            xs[best_inliers],
            ys[best_inliers],
            np.ones(np.count_nonzero(best_inliers), dtype=np.float32)
        ))
        a, b, c = np.linalg.lstsq(A, zs[best_inliers], rcond=None)[0]

        return float(a), float(b), float(c)

    def detect_low_cells(self, height_map, count_map, spread_map, ground_map):
        """
        检测低洼栅格

        如果：
            ground_plane_z - cell_z > low_threshold

        则认为该栅格可能是坑洞。
        """
        roi_debug = {
            key: value for key, value in self.last_debug.items()
            if (
                key == "auto_roi" or
                key.startswith("depth_") or
                key.startswith("valid_cell_") or
                key.startswith("roi_") or
                key.startswith("ground_")
            )
        }
        valid = np.isfinite(height_map) & (count_map >= self.min_cell_points)
        flat_cell = (
            np.isfinite(spread_map) &
            (spread_map <= self.max_pit_cell_height_spread)
        )
        ground_like = (
            valid &
            (np.abs(height_map - ground_map) < self.ground_contact_threshold)
        )

        low_mask = np.zeros_like(height_map, dtype=np.uint8)
        diff_map = np.zeros_like(height_map, dtype=np.float32)
        diff_map[valid] = np.maximum(ground_map[valid] - height_map[valid], 0.0)

        pit_bottom = (
            valid &
            flat_cell &
            (diff_map > self.low_threshold) &
            (diff_map < self.max_pothole_depth)
        )
        edge_mask = self.detect_negative_edges_from_grid(
            height_map,
            valid,
            diff_map,
            ground_map
        )
        unknown_mask = self.detect_unknown_holes_from_grid(
            height_map,
            valid,
            diff_map,
            ground_map
        )

        final_cells = pit_bottom | unknown_mask
        if self.include_edges_in_output:
            final_cells |= edge_mask

        low_mask[final_cells] = 255

        self.last_masks = {
            "pit_bottom": pit_bottom.astype(np.uint8) * 255,
            "negative_edges": edge_mask.astype(np.uint8) * 255,
            "unknown_holes": unknown_mask.astype(np.uint8) * 255,
            "ground_like": ground_like.astype(np.uint8) * 255,
            "valid": valid.astype(np.uint8) * 255,
        }

        self.last_debug = {
            "valid_cells": int(np.count_nonzero(valid)),
            "pit_bottom_cells": int(np.count_nonzero(pit_bottom)),
            "flat_cells": int(np.count_nonzero(flat_cell & valid)),
            "edge_cells": int(np.count_nonzero(edge_mask)),
            "unknown_cells": int(np.count_nonzero(unknown_mask)),
            "raw_mask_cells": int(np.count_nonzero(low_mask)),
            "max_drop": float(np.max(diff_map[valid])) if np.any(valid) else 0.0,
        }
        self.last_debug.update(roi_debug)

        return low_mask, diff_map

    def detect_negative_edges_from_grid(self, height_map, valid, diff_map, ground_map):
        if not self.detect_negative_edges:
            return np.zeros_like(valid, dtype=bool)

        ground_like = (
            valid &
            (np.abs(height_map - ground_map) < self.ground_contact_threshold)
        )
        edge_mask = np.zeros_like(valid, dtype=bool)

        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue

                cur_y = slice(max(0, dy), valid.shape[0] + min(0, dy))
                cur_x = slice(max(0, dx), valid.shape[1] + min(0, dx))
                nb_y = slice(max(0, -dy), valid.shape[0] + min(0, -dy))
                nb_x = slice(max(0, -dx), valid.shape[1] + min(0, -dx))

                drop = height_map[nb_y, nb_x] - height_map[cur_y, cur_x]
                edge = (
                    valid[cur_y, cur_x] &
                    ground_like[nb_y, nb_x] &
                    (drop > self.edge_drop_threshold) &
                    (diff_map[cur_y, cur_x] < self.max_pothole_depth)
                )
                edge_mask[cur_y, cur_x] |= edge

        # diff_map is a metric measurement. Edge membership must not inflate
        # a measured 4/5 cm drop to an artificial confidence threshold.
        return edge_mask

    def detect_unknown_holes_from_grid(self, height_map, valid, diff_map, ground_map):
        if not self.detect_unknown_holes:
            return np.zeros_like(valid, dtype=bool)

        ground_like = (
            valid &
            (np.abs(height_map - ground_map) < self.ground_contact_threshold)
        ).astype(np.uint8)
        kernel = np.ones(
            (self.unknown_hole_kernel_cells, self.unknown_hole_kernel_cells),
            np.uint8
        )
        ground_neighbor_count = cv2.filter2D(
            ground_like,
            ddepth=cv2.CV_16S,
            kernel=kernel,
            borderType=cv2.BORDER_CONSTANT
        )

        unknown_mask = (
            (~valid) &
            (ground_neighbor_count >= self.unknown_hole_min_ground_neighbors)
        )

        margin = max(self.unknown_hole_border_margin_cells, 0)
        if margin > 0:
            unknown_mask[:margin, :] = False
            unknown_mask[-margin:, :] = False
            unknown_mask[:, :margin] = False
            unknown_mask[:, -margin:] = False

        unknown_score = max(self.low_threshold * 1.5, self.min_mean_drop * 2.0)
        diff_map[unknown_mask] = np.maximum(diff_map[unknown_mask], unknown_score)
        return unknown_mask

    def postprocess_mask(self, low_mask):
        """
        形态学滤波：
            默认保留坑洞边缘，闭运算和膨胀连接危险边界
        """
        if low_mask is None:
            return None

        if self.morph_open_iterations > 0:
            kernel = np.ones(
                (self.morph_kernel_cells, self.morph_kernel_cells),
                np.uint8
            )
            low_mask = cv2.morphologyEx(
                low_mask,
                cv2.MORPH_OPEN,
                kernel,
                iterations=self.morph_open_iterations
            )

        low_mask = self.remove_small_mask_components(
            low_mask,
            self.min_raw_mask_area_cells
        )

        if self.morph_close_iterations > 0:
            kernel = np.ones(
                (self.morph_kernel_cells, self.morph_kernel_cells),
                np.uint8
            )
            low_mask = cv2.morphologyEx(
                low_mask,
                cv2.MORPH_CLOSE,
                kernel,
                iterations=self.morph_close_iterations
            )

        if self.mask_dilation_cells > 0:
            dilation_size = self.mask_dilation_cells * 2 + 1
            dilation_kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE,
                (dilation_size, dilation_size)
            )
            low_mask = cv2.dilate(low_mask, dilation_kernel)

        return low_mask

    def remove_small_mask_components(self, low_mask, min_area_cells):
        if min_area_cells <= 1:
            return low_mask

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            low_mask,
            connectivity=8
        )
        filtered = np.zeros_like(low_mask)
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= min_area_cells:
                filtered[labels == i] = 255
        return filtered

    def extract_potholes_from_grid(self, low_mask, diff_map=None, ground_map=None):
        """
        连通域分析，提取坑洞中心、最终有效 mask 和拒绝原因图。
        """
        if low_mask is None:
            return [], None, None

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            low_mask,
            connectivity=8
        )

        potholes = []
        accepted_mask = np.zeros_like(low_mask, dtype=np.uint8)
        rejection_debug = np.zeros((*low_mask.shape, 3), dtype=np.uint8)
        self.last_detection_fields = {
            "confidence": np.zeros_like(low_mask, dtype=np.float32),
            "mean_drop": np.zeros_like(low_mask, dtype=np.float32),
            "max_drop": np.zeros_like(low_mask, dtype=np.float32),
            "component_id": np.zeros_like(low_mask, dtype=np.float32),
            "radius": np.zeros_like(low_mask, dtype=np.float32),
        }

        reject_counts = {
            "area": 0,
            "border": 0,
            "border_degraded": 0,
            "pit_bottom": 0,
            "edge_only": 0,
            "ground_ring": 0,
            "depth": 0,
            "accepted": 0,
        }

        for i in range(1, num_labels):
            area_cells = stats[i, cv2.CC_STAT_AREA]
            component = labels == i

            if area_cells < self.min_area_cells or area_cells > self.max_area_cells:
                rejection_debug[component] = [128, 128, 128]
                reject_counts["area"] += 1
                continue

            if diff_map is not None and np.any(component):
                mean_drop = float(np.mean(diff_map[component]))
                max_drop = float(np.max(diff_map[component]))
            else:
                mean_drop = 0.0
                max_drop = 0.0

            pit_bottom_mask = self.last_masks.get("pit_bottom")
            edge_mask = self.last_masks.get("negative_edges")
            pit_cells = 0
            edge_cells = 0
            if pit_bottom_mask is not None:
                pit_cells = int(np.count_nonzero(component & (pit_bottom_mask > 0)))
            if edge_mask is not None:
                edge_cells = int(np.count_nonzero(component & (edge_mask > 0)))

            edge_ratio = edge_cells / float(area_cells) if area_cells > 0 else 0.0
            edge_only_pass = (
                self.include_edges_in_output and
                pit_bottom_mask is not None and
                self.min_component_pit_cells > 0 and
                pit_cells < self.min_component_pit_cells and
                edge_cells >= self.min_edge_component_cells and
                edge_ratio >= self.min_edge_component_ratio and
                max_drop >= self.low_threshold and
                mean_drop >= self.min_mean_drop * 0.5
            )
            if (
                pit_bottom_mask is not None and
                self.min_component_pit_cells > 0 and
                pit_cells < self.min_component_pit_cells and
                not edge_only_pass
            ):
                rejection_debug[component] = [0, 165, 255]
                reject_counts["pit_bottom"] += 1
                continue

            border_contact = self.component_border_contact(component)
            if border_contact["reject"]:
                rejection_debug[component] = [0, 255, 255]
                reject_counts["border"] += 1
                continue

            if not self.component_has_ground_ring(component):
                rejection_debug[component] = [255, 0, 255]
                reject_counts["ground_ring"] += 1
                continue

            cx_cell, cy_cell = centroids[i]

            center_x = self.x_min + (cx_cell + 0.5) * self.resolution
            center_y = self.y_min + (cy_cell + 0.5) * self.resolution

            area_m2 = area_cells * self.resolution * self.resolution

            equivalent_radius = math.sqrt(area_m2 / math.pi)
            component_rows, component_cols = np.where(component)
            if len(component_cols) > 0:
                component_x = self.x_min + (component_cols + 0.5) * self.resolution
                component_y = self.y_min + (component_rows + 0.5) * self.resolution
                bounding_radius = float(np.max(np.hypot(
                    component_x - center_x,
                    component_y - center_y
                )))
                bounding_radius += self.resolution * 0.5
                radius = max(equivalent_radius, bounding_radius)
            else:
                radius = equivalent_radius

            if mean_drop < self.min_mean_drop and max_drop < self.low_threshold:
                rejection_debug[component] = [255, 0, 0]
                reject_counts["depth"] += 1
                continue

            if ground_map is not None and np.any(component):
                ground_z = float(np.mean(ground_map[component]))
            else:
                ground_z = 0.0

            area_score = min(
                area_cells / max(float(self.confidence_full_area_cells), 1.0),
                1.0
            )
            drop_score = min(mean_drop / max(self.low_threshold * 2.0, 1e-6), 1.0)
            confidence = float(0.5 * area_score + 0.5 * drop_score)
            if edge_only_pass:
                confidence *= self.edge_component_confidence_scale
            side_border_degraded = bool(border_contact["side_degrade"])
            if side_border_degraded:
                confidence *= self.side_border_confidence_scale

            potholes.append({
                "x": float(center_x),
                "y": float(center_y),
                "radius": float(radius),
                "area": float(area_m2),
                "mean_drop": mean_drop,
                "max_drop": max_drop,
                "confidence": confidence,
                "z": ground_z
            })
            accepted_mask[component] = 255
            self.last_detection_fields["confidence"][component] = confidence
            self.last_detection_fields["mean_drop"][component] = mean_drop
            self.last_detection_fields["max_drop"][component] = max_drop
            self.last_detection_fields["component_id"][component] = float(i)
            self.last_detection_fields["radius"][component] = float(radius)
            rejection_debug[component] = (
                [0, 128, 255] if side_border_degraded else [0, 0, 255]
            )
            reject_counts["accepted"] += 1
            if edge_only_pass:
                reject_counts["edge_only"] += 1
            if side_border_degraded:
                reject_counts["border_degraded"] += 1

        self.last_debug.update({
            "rejected_area_components": reject_counts["area"],
            "rejected_border_components": reject_counts["border"],
            "border_degraded_components": reject_counts["border_degraded"],
            "rejected_pit_bottom_components": reject_counts["pit_bottom"],
            "accepted_edge_only_components": reject_counts["edge_only"],
            "rejected_ground_ring_components": reject_counts["ground_ring"],
            "rejected_depth_components": reject_counts["depth"],
            "accepted_components": reject_counts["accepted"],
        })

        return potholes, accepted_mask, rejection_debug

    def component_border_contact(self, component):
        contact = {
            "near_x": False,
            "far_x": False,
            "side_y": False,
            "reject": False,
            "side_degrade": False,
        }
        if not self.reject_border_components:
            return contact

        margin = max(self.component_border_margin_cells, 0)
        if margin <= 0:
            return contact

        height, width = component.shape
        touches_near_x = bool(np.any(component[:, :margin]))
        touches_far_x = bool(np.any(component[:, width - margin:]))
        touches_min_y = bool(np.any(component[:margin, :]))
        touches_max_y = bool(np.any(component[height - margin:, :]))

        touches_sides = touches_min_y or touches_max_y
        contact.update({
            "near_x": touches_near_x,
            "far_x": touches_far_x,
            "side_y": touches_sides,
        })

        if touches_far_x:
            contact["reject"] = True
        elif touches_near_x and self.reject_near_border:
            contact["reject"] = True
        elif touches_sides:
            contact["side_degrade"] = True

        return contact

    def component_has_ground_ring(self, component):
        if self.min_ground_ring_cells <= 0 and self.min_ground_ring_ratio <= 0.0:
            return True

        ground_like = self.last_masks.get("ground_like")
        valid = self.last_masks.get("valid")
        if ground_like is None or valid is None:
            return True

        width_cells = max(int(self.ground_ring_width_cells), 1)
        kernel_size = width_cells * 2 + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size)
        )
        dilated = cv2.dilate(component.astype(np.uint8), kernel) > 0
        ring = dilated & (~component)
        valid_ring_cells = int(np.count_nonzero(ring & (valid > 0)))
        ground_ring_cells = int(np.count_nonzero(ring & (ground_like > 0)))

        if ground_ring_cells < self.min_ground_ring_cells:
            return False

        if valid_ring_cells == 0:
            return False

        ratio = ground_ring_cells / float(valid_ring_cells)
        return ratio >= self.min_ground_ring_ratio

    def visualize_height_map(self, diff_map, low_mask, ground_z=None):
        """
        可视化相对地面下沉量
        红色区域表示低洼区域
        """
        if diff_map is None:
            return

        valid = np.isfinite(diff_map)

        if np.count_nonzero(valid) == 0:
            return

        max_v = max(float(np.nanmax(diff_map[valid])), 1e-6)
        norm = np.zeros_like(diff_map, dtype=np.uint8)

        norm[valid] = np.clip(diff_map[valid] / max_v * 255.0, 0, 255).astype(
            np.uint8
        )

        color = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

        if low_mask is not None:
            color[low_mask > 0] = [0, 0, 255]

        # 放大显示
        color_big = cv2.resize(
            color,
            None,
            fx=5,
            fy=5,
            interpolation=cv2.INTER_NEAREST
        )

        cv2.imshow("height_map_pothole", color_big)
        cv2.waitKey(1)


class HeightMapPotholeNode(Node):
    def __init__(self):
        super().__init__('height_map_pothole_detector')

        # -------------------------
        # 参数声明
        # -------------------------
        self.declare_parameter(
            'depth_topic',
            '/camera_sensor/depth/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera_sensor/depth/camera_info'
        )
        self.declare_parameter(
            'camera_frame',
            'camera_optical_link'
        )
        self.declare_parameter(
            'base_frame',
            'base_link'
        )
        self.declare_parameter(
            'pothole_topic',
            '/potholes'
        )
        self.declare_parameter(
            'marker_topic',
            '/pothole_markers'
        )
        self.declare_parameter(
            'height_diff_image_topic',
            '/height_map/depth_diff_image'
        )
        self.declare_parameter(
            'pothole_mask_image_topic',
            '/height_map/pothole_mask'
        )
        self.declare_parameter(
            'pothole_candidate_mask_image_topic',
            '/height_map/pothole_candidate_mask'
        )
        self.declare_parameter(
            'pit_bottom_mask_image_topic',
            '/height_map/pit_bottom_mask'
        )
        self.declare_parameter(
            'negative_edge_mask_image_topic',
            '/height_map/negative_edge_mask'
        )
        self.declare_parameter(
            'unknown_hole_mask_image_topic',
            '/height_map/unknown_hole_mask'
        )
        self.declare_parameter(
            'ground_like_mask_image_topic',
            '/height_map/ground_like_mask'
        )
        self.declare_parameter(
            'pothole_rejection_debug_image_topic',
            '/height_map/pothole_rejection_debug'
        )
        self.declare_parameter(
            'pothole_detections_topic',
            '/pothole_detections'
        )
        self.declare_parameter('publish_pothole_polygon', True)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('publish_debug_images', False)

        self.declare_parameter('x_min', 0.50)
        self.declare_parameter('x_max', 4.00)
        self.declare_parameter('y_min', -1.50)
        self.declare_parameter('y_max', 1.50)
        self.declare_parameter('z_min', -1.20)
        self.declare_parameter('z_max', -0.35)
        self.declare_parameter('auto_roi_from_camera', True)
        self.declare_parameter('roi_ground_z', -0.670)
        self.declare_parameter('roi_margin', 0.05)
        self.declare_parameter('roi_min_span', 0.25)

        self.declare_parameter('resolution', 0.025)
        self.declare_parameter('min_depth', 0.1)
        self.declare_parameter('max_depth', 5.0)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_valid_depth_ratio', 0.10)
        self.declare_parameter('min_valid_cell_ratio', 0.02)
        self.declare_parameter('min_valid_cells', 20)
        self.declare_parameter('low_threshold', 0.035)
        self.declare_parameter('min_cell_points', 2)
        self.declare_parameter('min_area_cells', 6)
        self.declare_parameter('max_area_cells', 12000)
        self.declare_parameter('confidence_full_area_cells', 48)
        self.declare_parameter('step', 2)
        self.declare_parameter('cell_height_percentile', 35.0)
        self.declare_parameter('ground_fit_max_iterations', 40)
        self.declare_parameter('ground_fit_distance_threshold', 0.012)
        self.declare_parameter('ground_fit_min_inlier_ratio', 0.45)
        self.declare_parameter('ground_candidate_percentile', 70.0)
        self.declare_parameter('ground_candidate_min_cells', 20)
        self.declare_parameter('ground_fallback_percentile', 75.0)
        self.declare_parameter('max_pothole_depth', 0.45)
        self.declare_parameter('min_mean_drop', 0.025)
        self.declare_parameter('max_pit_cell_height_spread', 0.04)
        self.declare_parameter('ground_contact_threshold', 0.020)
        self.declare_parameter('edge_drop_threshold', 0.04)
        self.declare_parameter('detect_negative_edges', True)
        self.declare_parameter('include_edges_in_output', False)
        self.declare_parameter('edge_component_confidence_scale', 0.6)
        self.declare_parameter('min_edge_component_cells', 3)
        self.declare_parameter('min_edge_component_ratio', 0.28)
        self.declare_parameter('detect_unknown_holes', False)
        self.declare_parameter('unknown_hole_kernel_cells', 5)
        self.declare_parameter('unknown_hole_min_ground_neighbors', 5)
        self.declare_parameter('unknown_hole_border_margin_cells', 2)
        self.declare_parameter('min_component_pit_cells', 3)
        self.declare_parameter('reject_border_components', True)
        self.declare_parameter('reject_near_border', False)
        self.declare_parameter('component_border_margin_cells', 1)
        self.declare_parameter('side_border_confidence_scale', 0.7)
        self.declare_parameter('ground_ring_width_cells', 2)
        self.declare_parameter('min_ground_ring_cells', 4)
        self.declare_parameter('min_ground_ring_ratio', 0.15)
        self.declare_parameter('min_raw_mask_area_cells', 3)
        self.declare_parameter('mask_dilation_cells', 0)
        self.declare_parameter('morph_kernel_cells', 3)
        self.declare_parameter('morph_open_iterations', 0)
        self.declare_parameter('morph_close_iterations', 1)
        self.declare_parameter('random_seed', -1)
        self.declare_parameter('auto_roi_smoothing_alpha', 1.0)
        self.declare_parameter('max_roi_delta_per_frame', 0.0)
        self.declare_parameter('publish_pothole_detections', True)
        self.declare_parameter('virtual_obstacle_height', 0.08)
        self.declare_parameter('cloud_z_mode', 'ground')
        self.declare_parameter('cloud_z_value', 0.0)
        self.declare_parameter('publish_detection_metrics', False)
        self.declare_parameter('cloud_cell_stride', 1)
        self.declare_parameter('max_cloud_points', 5000)
        self.declare_parameter('tf_timeout', 0.2)
        self.declare_parameter('tf_cache_enabled', True)
        self.declare_parameter('tf_cache_max_age', 0.0)
        self.declare_parameter('camera_frame_check_enabled', True)
        self.declare_parameter('log_detections', True)
        self.declare_parameter('debug_log_period', 30)
        self.declare_parameter('debug_view', False)
        extra_defaults = {
            'ground_reference_enabled': True,
            'ground_reference_z': -0.670,
            'ground_reference_tolerance': 0.030,
            'ground_max_slope': 0.35,
            'dropoff_enabled': True,
            'dropoff_topic': '/dropoff_detections',
            'dropoff_marker_topic': '/dropoff_markers',
            'status_topic': '/pothole_status',
            'depth_timeout': 0.5,
            'dropoff_min_drop': 0.04,
            'dropoff_min_width': 0.12,
            'dropoff_support_pixels': 3,
            'dropoff_gap_pixels': 2,
            'dropoff_confirmation_frames': 3,
            'dropoff_max_range': 2.5,
            'dropoff_border_pixels': 8,
            'dropoff_min_valid_ratio': 0.01,
            'dropoff_plane_tolerance': 0.020,
        }
        for name, value in extra_defaults.items():
            self.declare_parameter(name, value)
        extra = {name: self.get_parameter(name).value for name in extra_defaults}
        self.dropoff_enabled = extra['dropoff_enabled']
        self.depth_timeout = max(float(extra['depth_timeout']), 0.1)
        self.last_depth_received = None
        self.last_source_stamp_seconds = None
        self.camera_info_shape = None

        # -------------------------
        # 参数读取
        # -------------------------
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.pothole_topic = self.get_parameter('pothole_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.height_diff_image_topic = self.get_parameter(
            'height_diff_image_topic'
        ).value
        self.pothole_mask_image_topic = self.get_parameter(
            'pothole_mask_image_topic'
        ).value
        self.pothole_candidate_mask_image_topic = self.get_parameter(
            'pothole_candidate_mask_image_topic'
        ).value
        self.pit_bottom_mask_image_topic = self.get_parameter(
            'pit_bottom_mask_image_topic'
        ).value
        self.negative_edge_mask_image_topic = self.get_parameter(
            'negative_edge_mask_image_topic'
        ).value
        self.unknown_hole_mask_image_topic = self.get_parameter(
            'unknown_hole_mask_image_topic'
        ).value
        self.ground_like_mask_image_topic = self.get_parameter(
            'ground_like_mask_image_topic'
        ).value
        self.pothole_rejection_debug_image_topic = self.get_parameter(
            'pothole_rejection_debug_image_topic'
        ).value
        self.pothole_detections_topic = self.get_parameter(
            'pothole_detections_topic'
        ).value
        self.publish_pothole_polygon_enabled = self.get_parameter(
            'publish_pothole_polygon'
        ).value
        self.publish_markers_enabled = self.get_parameter('publish_markers').value
        self.publish_debug_images_enabled = self.get_parameter(
            'publish_debug_images'
        ).value

        x_min = self.get_parameter('x_min').value
        x_max = self.get_parameter('x_max').value
        y_min = self.get_parameter('y_min').value
        y_max = self.get_parameter('y_max').value
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        self.auto_roi_from_camera = self.get_parameter(
            'auto_roi_from_camera'
        ).value
        self.roi_ground_z = float(self.get_parameter('roi_ground_z').value)
        self.roi_margin = float(self.get_parameter('roi_margin').value)
        self.roi_min_span = float(self.get_parameter('roi_min_span').value)
        self.roi_x_limits = (float(x_min), float(x_max))
        self.roi_y_limits = (float(y_min), float(y_max))

        resolution = self.get_parameter('resolution').value
        min_depth = self.get_parameter('min_depth').value
        max_depth = self.get_parameter('max_depth').value
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        min_valid_depth_ratio = self.get_parameter('min_valid_depth_ratio').value
        min_valid_cell_ratio = self.get_parameter('min_valid_cell_ratio').value
        min_valid_cells = self.get_parameter('min_valid_cells').value
        low_threshold = self.get_parameter('low_threshold').value
        min_cell_points = self.get_parameter('min_cell_points').value
        min_area_cells = self.get_parameter('min_area_cells').value
        max_area_cells = self.get_parameter('max_area_cells').value
        confidence_full_area_cells = self.get_parameter(
            'confidence_full_area_cells'
        ).value
        step = self.get_parameter('step').value
        cell_height_percentile = self.get_parameter(
            'cell_height_percentile'
        ).value
        ground_fit_max_iterations = self.get_parameter(
            'ground_fit_max_iterations'
        ).value
        ground_fit_distance_threshold = self.get_parameter(
            'ground_fit_distance_threshold'
        ).value
        ground_fit_min_inlier_ratio = self.get_parameter(
            'ground_fit_min_inlier_ratio'
        ).value
        ground_candidate_percentile = self.get_parameter(
            'ground_candidate_percentile'
        ).value
        ground_candidate_min_cells = self.get_parameter(
            'ground_candidate_min_cells'
        ).value
        ground_fallback_percentile = self.get_parameter(
            'ground_fallback_percentile'
        ).value
        max_pothole_depth = self.get_parameter('max_pothole_depth').value
        min_mean_drop = self.get_parameter('min_mean_drop').value
        max_pit_cell_height_spread = self.get_parameter(
            'max_pit_cell_height_spread'
        ).value
        ground_contact_threshold = self.get_parameter(
            'ground_contact_threshold'
        ).value
        edge_drop_threshold = self.get_parameter('edge_drop_threshold').value
        detect_negative_edges = self.get_parameter('detect_negative_edges').value
        include_edges_in_output = self.get_parameter(
            'include_edges_in_output'
        ).value
        edge_component_confidence_scale = self.get_parameter(
            'edge_component_confidence_scale'
        ).value
        min_edge_component_cells = self.get_parameter(
            'min_edge_component_cells'
        ).value
        min_edge_component_ratio = self.get_parameter(
            'min_edge_component_ratio'
        ).value
        detect_unknown_holes = self.get_parameter('detect_unknown_holes').value
        unknown_hole_kernel_cells = self.get_parameter(
            'unknown_hole_kernel_cells'
        ).value
        unknown_hole_min_ground_neighbors = self.get_parameter(
            'unknown_hole_min_ground_neighbors'
        ).value
        unknown_hole_border_margin_cells = self.get_parameter(
            'unknown_hole_border_margin_cells'
        ).value
        min_component_pit_cells = self.get_parameter(
            'min_component_pit_cells'
        ).value
        reject_border_components = self.get_parameter(
            'reject_border_components'
        ).value
        reject_near_border = self.get_parameter(
            'reject_near_border'
        ).value
        component_border_margin_cells = self.get_parameter(
            'component_border_margin_cells'
        ).value
        side_border_confidence_scale = self.get_parameter(
            'side_border_confidence_scale'
        ).value
        ground_ring_width_cells = self.get_parameter(
            'ground_ring_width_cells'
        ).value
        min_ground_ring_cells = self.get_parameter(
            'min_ground_ring_cells'
        ).value
        min_ground_ring_ratio = self.get_parameter(
            'min_ground_ring_ratio'
        ).value
        min_raw_mask_area_cells = self.get_parameter(
            'min_raw_mask_area_cells'
        ).value
        mask_dilation_cells = self.get_parameter('mask_dilation_cells').value
        morph_kernel_cells = self.get_parameter('morph_kernel_cells').value
        morph_open_iterations = self.get_parameter('morph_open_iterations').value
        morph_close_iterations = self.get_parameter('morph_close_iterations').value
        random_seed = self.get_parameter('random_seed').value
        auto_roi_smoothing_alpha = self.get_parameter(
            'auto_roi_smoothing_alpha'
        ).value
        max_roi_delta_per_frame = self.get_parameter(
            'max_roi_delta_per_frame'
        ).value
        self.publish_pothole_detections_enabled = self.get_parameter(
            'publish_pothole_detections'
        ).value
        self.virtual_obstacle_height = self.get_parameter(
            'virtual_obstacle_height'
        ).value
        self.cloud_z_mode = str(self.get_parameter('cloud_z_mode').value)
        self.cloud_z_value = float(self.get_parameter('cloud_z_value').value)
        self.publish_detection_metrics_enabled = self.get_parameter(
            'publish_detection_metrics'
        ).value
        self.cloud_cell_stride = max(
            1,
            int(self.get_parameter('cloud_cell_stride').value)
        )
        self.max_cloud_points = max(
            1,
            int(self.get_parameter('max_cloud_points').value)
        )
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.tf_cache_enabled = self.get_parameter('tf_cache_enabled').value
        self.tf_cache_max_age = float(self.get_parameter('tf_cache_max_age').value)
        self.camera_frame_check_enabled = self.get_parameter(
            'camera_frame_check_enabled'
        ).value
        self.log_detections = self.get_parameter('log_detections').value
        self.debug_log_period = max(
            1,
            int(self.get_parameter('debug_log_period').value)
        )
        debug_view = self.get_parameter('debug_view').value
        self.frame_count = 0

        # -------------------------
        # 工具
        # -------------------------
        self.bridge = CvBridge()
        self.K = None
        self.camera_info_received = False
        self.camera_info_frame = ''
        self.frame_check_warnings = set()
        self.cached_transforms = {}

        # -------------------------
        # TF
        # -------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # -------------------------
        # 检测器
        # -------------------------
        self.detector = HeightMapPotholeDetector(
            x_min=x_min,
            x_max=x_max,
            y_min=y_min,
            y_max=y_max,
            z_min=z_min,
            z_max=z_max,
            resolution=resolution,
            min_depth=min_depth,
            max_depth=max_depth,
            min_valid_depth_ratio=min_valid_depth_ratio,
            min_valid_cell_ratio=min_valid_cell_ratio,
            min_valid_cells=min_valid_cells,
            low_threshold=low_threshold,
            min_cell_points=min_cell_points,
            min_area_cells=min_area_cells,
            max_area_cells=max_area_cells,
            confidence_full_area_cells=confidence_full_area_cells,
            step=step,
            cell_height_percentile=cell_height_percentile,
            ground_fit_max_iterations=ground_fit_max_iterations,
            ground_fit_distance_threshold=ground_fit_distance_threshold,
            ground_fit_min_inlier_ratio=ground_fit_min_inlier_ratio,
            ground_candidate_percentile=ground_candidate_percentile,
            ground_candidate_min_cells=ground_candidate_min_cells,
            ground_fallback_percentile=ground_fallback_percentile,
            max_pothole_depth=max_pothole_depth,
            min_mean_drop=min_mean_drop,
            max_pit_cell_height_spread=max_pit_cell_height_spread,
            ground_contact_threshold=ground_contact_threshold,
            edge_drop_threshold=edge_drop_threshold,
            detect_negative_edges=detect_negative_edges,
            include_edges_in_output=include_edges_in_output,
            edge_component_confidence_scale=edge_component_confidence_scale,
            min_edge_component_cells=min_edge_component_cells,
            min_edge_component_ratio=min_edge_component_ratio,
            detect_unknown_holes=detect_unknown_holes,
            unknown_hole_kernel_cells=unknown_hole_kernel_cells,
            unknown_hole_min_ground_neighbors=unknown_hole_min_ground_neighbors,
            unknown_hole_border_margin_cells=unknown_hole_border_margin_cells,
            min_component_pit_cells=min_component_pit_cells,
            reject_border_components=reject_border_components,
            reject_near_border=reject_near_border,
            component_border_margin_cells=component_border_margin_cells,
            side_border_confidence_scale=side_border_confidence_scale,
            ground_ring_width_cells=ground_ring_width_cells,
            min_ground_ring_cells=min_ground_ring_cells,
            min_ground_ring_ratio=min_ground_ring_ratio,
            min_raw_mask_area_cells=min_raw_mask_area_cells,
            mask_dilation_cells=mask_dilation_cells,
            morph_kernel_cells=morph_kernel_cells,
            morph_open_iterations=morph_open_iterations,
            morph_close_iterations=morph_close_iterations,
            random_seed=random_seed,
            auto_roi_smoothing_alpha=auto_roi_smoothing_alpha,
            max_roi_delta_per_frame=max_roi_delta_per_frame,
            debug_view=debug_view,
            **{name: extra[name] for name in (
                'ground_reference_enabled', 'ground_reference_z',
                'ground_reference_tolerance', 'ground_max_slope',
            )},
        )
        self.dropoff_detector = GroundRayDropoffDetector(
            **{name.removeprefix('dropoff_'): extra[name] for name in (
                'dropoff_min_drop', 'dropoff_min_width', 'dropoff_support_pixels',
                'dropoff_gap_pixels', 'dropoff_confirmation_frames',
                'dropoff_max_range', 'dropoff_border_pixels',
                'dropoff_min_valid_ratio', 'dropoff_plane_tolerance',
            )},
            max_depth=max_depth, x_min=x_min, x_max=x_max,
            y_min=y_min, y_max=y_max,
        )

        # -------------------------
        # 订阅和发布
        # -------------------------
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.pothole_pub = self.create_publisher(
            PolygonStamped,
            self.pothole_topic,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.marker_topic,
            10
        )

        self.height_diff_image_pub = self.create_publisher(
            Image,
            self.height_diff_image_topic,
            10
        )

        self.pothole_mask_image_pub = self.create_publisher(
            Image,
            self.pothole_mask_image_topic,
            10
        )

        self.pothole_candidate_mask_image_pub = self.create_publisher(
            Image,
            self.pothole_candidate_mask_image_topic,
            10
        )

        self.pit_bottom_mask_image_pub = self.create_publisher(
            Image,
            self.pit_bottom_mask_image_topic,
            10
        )

        self.negative_edge_mask_image_pub = self.create_publisher(
            Image,
            self.negative_edge_mask_image_topic,
            10
        )

        self.unknown_hole_mask_image_pub = self.create_publisher(
            Image,
            self.unknown_hole_mask_image_topic,
            10
        )

        self.ground_like_mask_image_pub = self.create_publisher(
            Image,
            self.ground_like_mask_image_topic,
            10
        )

        self.pothole_rejection_debug_image_pub = self.create_publisher(
            Image,
            self.pothole_rejection_debug_image_topic,
            10
        )

        self.pothole_detections_pub = self.create_publisher(
            PointCloud2,
            self.pothole_detections_topic,
            10
        )
        self.dropoff_pub = self.create_publisher(PointCloud2, extra['dropoff_topic'], 10)
        self.dropoff_marker_pub = self.create_publisher(
            MarkerArray, extra['dropoff_marker_topic'], 10)
        self.status_pub = self.create_publisher(String, extra['status_topic'], 10)
        self.health_timer = self.create_timer(0.2, self.check_depth_timeout)

        self.get_logger().info('HeightMapPotholeNode started.')
        self.get_logger().info(f'depth_topic: {self.depth_topic}')
        self.get_logger().info(f'camera_info_topic: {self.camera_info_topic}')
        self.get_logger().info(f'camera_frame: {self.camera_frame}')
        self.get_logger().info(f'base_frame: {self.base_frame}')
        self.get_logger().info(f'pothole_topic: {self.pothole_topic}')
        self.get_logger().info(f'marker_topic: {self.marker_topic}')
        self.get_logger().info(
            f'height_diff_image_topic: {self.height_diff_image_topic}'
        )
        self.get_logger().info(
            f'pothole_mask_image_topic: {self.pothole_mask_image_topic}'
        )
        self.get_logger().info(
            'pothole_candidate_mask_image_topic: '
            f'{self.pothole_candidate_mask_image_topic}'
        )
        self.get_logger().info(
            f'pit_bottom_mask_image_topic: {self.pit_bottom_mask_image_topic}'
        )
        self.get_logger().info(
            f'negative_edge_mask_image_topic: {self.negative_edge_mask_image_topic}'
        )
        self.get_logger().info(
            f'unknown_hole_mask_image_topic: {self.unknown_hole_mask_image_topic}'
        )
        self.get_logger().info(
            f'ground_like_mask_image_topic: {self.ground_like_mask_image_topic}'
        )
        self.get_logger().info(
            'pothole_rejection_debug_image_topic: '
            f'{self.pothole_rejection_debug_image_topic}'
        )
        self.get_logger().info(
            f'pothole_detections_topic: {self.pothole_detections_topic}'
        )

    def camera_info_callback(self, msg):
        """
        获取相机内参
        """
        K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        if not np.all(np.isfinite(K)) or K[0, 0] <= 0 or K[1, 1] <= 0:
            self.K = None
            self.publish_unavailable('invalid_camera_info', msg.header.stamp)
            return
        self.K = K
        self.camera_info_shape = (int(msg.height), int(msg.width))
        self.camera_info_frame = msg.header.frame_id

        if not self.camera_info_received:
            self.camera_info_received = True
            self.get_logger().info(f'CameraInfo received. K =\n{self.K}')

    def depth_callback(self, msg):
        """
        深度图回调
        """
        self.last_depth_received = self.get_clock().now()
        self.last_source_stamp_seconds = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.K is None:
            self.get_logger().warn('CameraInfo not received yet.')
            self.publish_unavailable('camera_info_missing', msg.header.stamp)
            return

        try:
            depth_img = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            self.publish_unavailable('depth_decode', msg.header.stamp)
            return

        if depth_img.ndim != 2 or depth_img.shape != self.camera_info_shape:
            self.publish_unavailable('depth_camera_info_shape_mismatch', msg.header.stamp)
            return

        # -------------------------
        # 深度单位处理
        # -------------------------
        if depth_img.dtype == np.uint16:
            # 常见情况：16UC1，单位 mm；实机可通过 depth_scale 调整。
            depth_img = depth_img.astype(np.float32) * self.depth_scale
        elif depth_img.dtype == np.float32:
            # 常见情况：32FC1，单位 m
            depth_img = depth_img.astype(np.float32)
        else:
            self.get_logger().warn(
                f'Unsupported depth image dtype: {depth_img.dtype}'
            )
            self.publish_unavailable('unsupported_depth_encoding', msg.header.stamp)
            return

        # -------------------------
        # 查 TF：base_link <- camera_frame
        # -------------------------
        stamp = Time.from_msg(msg.header.stamp)

        source_frame = msg.header.frame_id or self.camera_frame
        self.check_camera_frame_consistency(source_frame)
        if self.camera_info_frame and self.camera_info_frame != source_frame:
            self.publish_unavailable('depth_camera_info_frame_mismatch', msg.header.stamp)
            return

        R, t = self.get_tf_transform(
            target_frame=self.base_frame,
            source_frame=source_frame,
            stamp=stamp
        )

        tf_cache_used = False
        if R is None or t is None:
            R, t = self.get_cached_tf(source_frame)
            if R is None or t is None:
                self.get_logger().warn('TF not available.')
                self.publish_unavailable('tf_missing', msg.header.stamp)
                return
            tf_cache_used = True

        if self.auto_roi_from_camera:
            self.detector.update_roi_from_camera_view(
                self.K,
                depth_img.shape,
                R,
                t,
                self.roi_ground_z,
                self.roi_margin,
                self.roi_min_span,
                self.roi_x_limits,
                self.roi_y_limits
            )

        # -------------------------
        # 检测坑洞
        # -------------------------
        (
            potholes,
            accepted_mask,
            height_map,
            low_mask,
            diff_map,
            rejection_debug_image,
        ) = self.detector.process(
            depth_img,
            self.K,
            R,
            t
        )
        self.detector.last_debug.update({
            "tf_cache_used": tf_cache_used,
        })
        dropoffs = []
        if self.dropoff_enabled:
            for name in ('x_min', 'x_max', 'y_min', 'y_max'):
                setattr(self.dropoff_detector, name, getattr(self.detector, name))
            # Percentile fallback is not verified upper-ground support.
            plane = self.detector.last_ground_plane
            if self.detector.last_debug.get('ground_fit_status') == 'fallback_percentile':
                plane = None
            dropoffs = self.dropoff_detector.process(
                depth_img, self.K, R, t, plane,
                stamp_seconds=stamp.nanoseconds * 1e-9,
            )
        reason = self.detector.last_debug.get('quality_reject_reason')
        if dropoffs:
            state, reason = 'suspected_dropoff', 'upper_edge_evidence'
        elif reason:
            state = 'unavailable'
        elif potholes:
            state, reason = 'detected_pothole', 'visible_low_cells'
        elif self.dropoff_enabled and self.dropoff_detector.last_status == 'unavailable':
            state = 'unavailable'
            reason = self.dropoff_detector.last_debug.get('reason')
        else:
            state, reason = 'observing', 'no_confirmed_detection'
        self.publish_dropoffs(dropoffs, msg.header.stamp)
        self.publish_status(state, reason, len(potholes), len(dropoffs))
        self.frame_count += 1

        # -------------------------
        # 发布结果
        # -------------------------
        self.publish_potholes(potholes, msg.header.stamp)
        self.publish_pothole_markers(potholes, accepted_mask, msg.header.stamp)
        self.publish_height_debug_images(
            height_map,
            accepted_mask,
            low_mask,
            diff_map,
            rejection_debug_image,
            msg.header.stamp
        )
        self.publish_pothole_detections(accepted_mask, msg.header.stamp)
        self.log_debug_stats()

    def publish_status(self, state, reason, pit_count=0, edge_count=0):
        self.status_pub.publish(String(data=json.dumps({
            'state': state, 'reason': reason,
            'pit_count': int(pit_count), 'edge_count': int(edge_count),
            'ground_fit': self.detector.last_debug.get('ground_fit_status', 'unavailable'),
            'edge_candidates': self.dropoff_detector.last_debug.get('candidate_edges', 0),
            'unknown_edges': self.dropoff_detector.last_debug.get('unknown_edges', 0),
            'pit_quality_reject': self.detector.last_debug.get('quality_reject_reason'),
            'stamp_seconds': self.get_clock().now().nanoseconds * 1e-9,
            'source_stamp_seconds': self.last_source_stamp_seconds,
        })))

    def publish_unavailable(self, reason, stamp):
        self.dropoff_detector.reset()
        self.detector.last_ground_plane = None
        self.detector.last_detection_fields = {}
        self.detector.last_masks = {}
        self.detector.last_debug = {'quality_reject_reason': reason}
        self.publish_dropoffs([], stamp)
        self.publish_potholes([], stamp)
        self.publish_pothole_markers([], None, stamp)
        self.publish_pothole_detections(None, stamp)
        self.publish_status('unavailable', reason)

    def check_depth_timeout(self):
        now = self.get_clock().now()
        if self.last_depth_received is None:
            self.publish_unavailable('waiting_for_depth', now.to_msg())
        else:
            age = (now - self.last_depth_received).nanoseconds * 1e-9
            if age < 0.0 or age > self.depth_timeout:
                self.publish_unavailable('depth_timeout', now.to_msg())

    def publish_dropoffs(self, detections, stamp):
        rows = []
        markers = MarkerArray()
        clear = Marker()
        clear.header = Header(stamp=stamp, frame_id=self.base_frame)
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        plane = self.detector.last_ground_plane
        for index, detection in enumerate(detections, 1):
            x, y = detection['x'], detection['y']
            z = plane[0] * x + plane[1] * y + plane[2]
            rows.append((x, y, z, detection['confidence'], detection['width'],
                         float(detection['depth_known']),
                         detection['drop'] if detection['depth_known'] else -1.0))
            marker = Marker()
            marker.header = clear.header
            marker.ns, marker.id = 'upper_edges', index
            marker.type, marker.action = Marker.LINE_LIST, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.025
            marker.color.r, marker.color.a = 1.0, 0.95
            marker.color.g = 0.0 if detection['depth_known'] else 0.6
            marker.lifetime.nanosec = 300000000
            half_width = detection['width'] * 0.5
            marker.points = [Point(x=float(x), y=float(y + side * half_width),
                                   z=float(z + 0.02)) for side in (-1, 1)]
            markers.markers.append(marker)
        self.dropoff_pub.publish(self.make_cloud_message(
            rows, ['x', 'y', 'z', 'confidence', 'width', 'depth_known', 'drop'], stamp))
        if self.publish_markers_enabled:
            self.dropoff_marker_pub.publish(markers)

    def check_camera_frame_consistency(self, depth_frame):
        if not self.camera_frame_check_enabled:
            return

        frames = {
            "depth": depth_frame,
            "camera_info": self.camera_info_frame,
            "param": self.camera_frame,
        }
        non_empty = {name: frame for name, frame in frames.items() if frame}
        if len(set(non_empty.values())) <= 1:
            return

        key = tuple(sorted(non_empty.items()))
        if key in self.frame_check_warnings:
            return

        self.frame_check_warnings.add(key)
        self.get_logger().warn(
            'Camera frame mismatch: '
            f'depth={frames["depth"]}, '
            f'camera_info={frames["camera_info"]}, '
            f'param={frames["param"]}. '
            'Depth optical frame must match CameraInfo calibration.'
        )

    def log_debug_stats(self):
        if not self.log_detections:
            return

        if self.frame_count % self.debug_log_period != 0:
            return

        stats = self.detector.last_debug
        self.get_logger().info(
            'Pothole debug: '
            f'depth_valid_ratio={stats.get("depth_valid_ratio", 0.0):.2f}, '
            f'valid_cells={stats.get("valid_cells", 0)}, '
            f'valid_cell_ratio={stats.get("valid_cell_ratio", 0.0):.2f}, '
            f'flat_cells={stats.get("flat_cells", 0)}, '
            f'pit_bottom_cells={stats.get("pit_bottom_cells", 0)}, '
            f'edge_cells={stats.get("edge_cells", 0)}, '
            f'unknown_cells={stats.get("unknown_cells", 0)}, '
            f'mask_cells={stats.get("mask_cells", 0)}, '
            f'max_drop={stats.get("max_drop", 0.0):.3f}, '
            f'roi=[{stats.get("roi_x_min", self.detector.x_min):.2f}, '
            f'{stats.get("roi_x_max", self.detector.x_max):.2f}]x'
            f'[{stats.get("roi_y_min", self.detector.y_min):.2f}, '
            f'{stats.get("roi_y_max", self.detector.y_max):.2f}], '
            f'potholes={stats.get("potholes", 0)}, '
            f'accepted_cells={stats.get("accepted_cells", 0)}, '
            f'detection_points={stats.get("detection_points", 0)}, '
            f'rejects=[area:{stats.get("rejected_area_components", 0)}, '
            f'border:{stats.get("rejected_border_components", 0)}, '
            f'border_degraded:{stats.get("border_degraded_components", 0)}, '
            f'pit:{stats.get("rejected_pit_bottom_components", 0)}, '
            f'ring:{stats.get("rejected_ground_ring_components", 0)}, '
            f'depth:{stats.get("rejected_depth_components", 0)}], '
            f'ground_fit={stats.get("ground_fit_status", "unknown")}, '
            f'ground_candidates={stats.get("ground_candidate_cells", 0)}, '
            f'quality_reject={stats.get("quality_reject_reason", "none")}, '
            f'tf_cache_used={stats.get("tf_cache_used", False)}'
        )

    def get_tf_transform(self, target_frame, source_frame, stamp):
        """
        获取 target_frame <- source_frame 的 TF

        返回：
            R, t

        使得：
            point_target = R @ point_source + t
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout)
            )

            t = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ], dtype=np.float32)

            q = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ]

            R = quaternion_matrix(q)[:3, :3].astype(np.float32)

            if self.tf_cache_enabled:
                self.cached_transforms[(target_frame, source_frame)] = {
                    "R": R,
                    "t": t,
                    "cache_time": self.get_clock().now(),
                }

            return R, t

        except Exception as e:
            self.get_logger().warn(
                f'TF lookup failed: {target_frame} <- {source_frame}, error: {e}'
            )
            return None, None

    def get_cached_tf(self, source_frame):
        if not self.tf_cache_enabled:
            return None, None

        cached = self.cached_transforms.get((self.base_frame, source_frame))
        if cached is None:
            return None, None

        if self.tf_cache_max_age > 0.0:
            age = (
                self.get_clock().now() - cached["cache_time"]
            ).nanoseconds / 1e9
            if age > self.tf_cache_max_age:
                return None, None

        if (
            self.log_detections and
            self.frame_count % self.debug_log_period == 0
        ):
            self.get_logger().warn(
                f'Using cached TF: {self.base_frame} <- {source_frame}'
            )

        return cached["R"], cached["t"]

    def publish_potholes(self, potholes, stamp):
        """
        发布坑洞

        使用 PolygonStamped 临时表示：
            point.x = 坑洞中心 x
            point.y = 坑洞中心 y
            point.z = 坑洞半径

        frame_id = base_link
        """
        if not self.publish_pothole_polygon_enabled:
            return

        msg = PolygonStamped()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame

        for p in potholes:
            pt = Point32()
            pt.x = float(p["x"])
            pt.y = float(p["y"])
            pt.z = float(p["radius"])
            msg.polygon.points.append(pt)

        self.pothole_pub.publish(msg)

        if (
            self.log_detections and
            len(potholes) > 0 and
            self.frame_count % self.debug_log_period == 0
        ):
            for i, p in enumerate(potholes):
                self.get_logger().info(
                    f'Pothole {i}: '
                    f'x={p["x"]:.2f}, '
                    f'y={p["y"]:.2f}, '
                    f'radius={p["radius"]:.2f}, '
                    f'area={p["area"]:.3f}, '
                    f'mean_drop={p.get("mean_drop", 0.0):.3f}, '
                    f'max_drop={p.get("max_drop", 0.0):.3f}, '
                    f'confidence={p.get("confidence", 0.0):.2f}'
                )


    def publish_pothole_markers(self, potholes, accepted_mask, stamp):
        """
        发布 RViz 可视化结果。

        accepted_mask 用 CUBE_LIST 显示真实栅格形状，文本 marker
        显示每个连通域的摘要指标。
        """
        if not self.publish_markers_enabled:
            return

        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.stamp = stamp
        clear_marker.header.frame_id = self.base_frame
        clear_marker.ns = 'potholes'
        clear_marker.id = 0
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        cell_marker = self.make_accepted_cell_marker(accepted_mask, stamp)
        if cell_marker is not None:
            marker_array.markers.append(cell_marker)

        for i, p in enumerate(potholes, start=1):
            text_marker = Marker()
            text_marker.header.stamp = stamp
            text_marker.header.frame_id = self.base_frame
            text_marker.ns = 'pothole_labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(p["x"])
            text_marker.pose.position.y = float(p["y"])
            text_marker.pose.position.z = float(p.get("z", 0.0)) + 0.15
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.08
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.95
            text_marker.text = (
                f'drop={p.get("mean_drop", 0.0):.2f}m '
                f'area={p.get("area", 0.0):.2f} '
                f'conf={p.get("confidence", 0.0):.2f}'
            )
            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 300000000
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def make_accepted_cell_marker(self, accepted_mask, stamp):
        if accepted_mask is None or np.count_nonzero(accepted_mask) == 0:
            return None

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = 'pothole_cells'
        marker.id = 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(self.detector.resolution)
        marker.scale.y = float(self.detector.resolution)
        marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.02
        marker.color.a = 0.65
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 300000000

        ys, xs = np.where(accepted_mask > 0)
        for y_idx, x_idx in zip(ys, xs):
            point = Point()
            point.x = (
                self.detector.x_min +
                (float(x_idx) + 0.5) * self.detector.resolution
            )
            point.y = (
                self.detector.y_min +
                (float(y_idx) + 0.5) * self.detector.resolution
            )
            plane = self.detector.last_ground_plane
            point.z = (plane[0] * point.x + plane[1] * point.y + plane[2] + 0.01
                       if plane is not None else 0.01)
            marker.points.append(point)

        return marker

    def publish_pothole_detections(self, accepted_mask, stamp):
        """
        Publish current-frame validated pothole cells for the costmap plugin.
        """
        if not self.publish_pothole_detections_enabled:
            return

        detection_points = self.mask_to_detection_points(accepted_mask)

        self.detector.last_debug.update({
            "detection_points": int(len(detection_points)),
        })
        self.publish_cloud_points(detection_points, stamp)

    def mask_to_detection_points(self, accepted_mask):
        if accepted_mask is None or np.count_nonzero(accepted_mask) == 0:
            field_count = 8 if self.publish_detection_metrics_enabled else 3
            return np.empty((0, field_count), dtype=np.float32)

        ys, xs = np.where(accepted_mask > 0)
        points = []
        stride = max(1, self.cloud_cell_stride)
        detection_fields = self.detector.last_detection_fields

        for y_idx, x_idx in zip(ys, xs):
            if stride > 1 and (x_idx % stride != 0 or y_idx % stride != 0):
                continue

            x = self.detector.x_min + (float(x_idx) + 0.5) * self.detector.resolution
            y = self.detector.y_min + (float(y_idx) + 0.5) * self.detector.resolution
            mean_drop = self._detection_field_value(
                detection_fields,
                "mean_drop",
                y_idx,
                x_idx
            )
            z = self.cloud_point_z(mean_drop)

            if self.publish_detection_metrics_enabled:
                points.append((
                    x,
                    y,
                    z,
                    self._detection_field_value(
                        detection_fields,
                        "confidence",
                        y_idx,
                        x_idx
                    ),
                    mean_drop,
                    self._detection_field_value(
                        detection_fields,
                        "max_drop",
                        y_idx,
                        x_idx
                    ),
                    self._detection_field_value(
                        detection_fields,
                        "component_id",
                        y_idx,
                        x_idx
                    ),
                    self._detection_field_value(
                        detection_fields,
                        "radius",
                        y_idx,
                        x_idx
                    ),
                ))
            else:
                points.append((x, y, z))
            if len(points) >= self.max_cloud_points:
                break

        return np.array(points, dtype=np.float32)

    def _detection_field_value(self, fields, name, y_idx, x_idx):
        field = fields.get(name)
        if field is None:
            return 0.0
        return float(field[y_idx, x_idx])

    def cloud_point_z(self, mean_drop):
        mode = self.cloud_z_mode
        if mode == 'virtual_height':
            if self.cloud_z_value != 0.0:
                return self.cloud_z_value
            return float(self.virtual_obstacle_height)
        if mode == 'configured':
            return self.cloud_z_value
        if mode == 'drop_depth':
            return -float(mean_drop)
        return 0.0

    def publish_cloud_points(self, points, stamp):
        field_names = ['x', 'y', 'z']
        if self.publish_detection_metrics_enabled:
            field_names.extend([
                'confidence',
                'mean_drop',
                'max_drop',
                'component_id',
                'radius',
            ])
        self.pothole_detections_pub.publish(self.make_cloud_message(points, field_names, stamp))

    def make_cloud_message(self, points, field_names, stamp):
        msg = PointCloud2()
        msg.header = Header(stamp=stamp, frame_id=self.base_frame)
        msg.height = 1
        msg.fields = [
            PointField(
                name=name,
                offset=index * 4,
                datatype=PointField.FLOAT32,
                count=1
            )
            for index, name in enumerate(field_names)
        ]
        msg.is_bigendian = False
        msg.point_step = 4 * len(field_names)
        msg.is_dense = True
        msg.width = int(len(points))
        msg.row_step = msg.point_step * msg.width
        point_array = np.asarray(points, dtype='<f4')
        if point_array.size == 0:
            msg.data = b''
        else:
            point_array = np.ascontiguousarray(point_array.reshape((-1, len(field_names))))
            msg.data = point_array.tobytes()
        return msg

    def publish_height_debug_images(
        self,
        height_map,
        accepted_mask,
        low_mask,
        diff_map,
        rejection_debug_image,
        stamp
    ):
        """
        发布 RViz Image 可直接显示的高度差调试图。

        depth_diff_image:
            彩色图，黄色覆盖候选区域，红色覆盖最终通过区域。

        pothole_mask:
            单通道图，255 表示最终通过质量过滤的坑洞格子。
        """
        if not self.publish_debug_images_enabled:
            return

        if height_map is None:
            return

        valid = np.isfinite(height_map)
        if np.count_nonzero(valid) == 0:
            return

        if diff_map is None:
            ground_z = np.nanmedian(height_map[valid])
            diff = np.zeros_like(height_map, dtype=np.float32)
            diff[valid] = np.maximum(ground_z - height_map[valid], 0.0)
        else:
            diff = diff_map

        max_diff = max(float(np.nanmax(diff[valid])), 1e-6)
        diff_u8 = np.zeros_like(height_map, dtype=np.uint8)
        diff_u8[valid] = np.clip(diff[valid] / max_diff * 255.0, 0, 255).astype(
            np.uint8
        )

        diff_color = cv2.applyColorMap(diff_u8, cv2.COLORMAP_JET)
        if low_mask is not None:
            diff_color[low_mask > 0] = [0, 255, 255]
        if accepted_mask is not None:
            diff_color[accepted_mask > 0] = [0, 0, 255]

        diff_msg = self.bridge.cv2_to_imgmsg(diff_color, encoding='bgr8')
        diff_msg.header.stamp = stamp
        diff_msg.header.frame_id = self.base_frame
        self.height_diff_image_pub.publish(diff_msg)

        if accepted_mask is not None:
            mask_msg = self.bridge.cv2_to_imgmsg(accepted_mask, encoding='mono8')
            mask_msg.header.stamp = stamp
            mask_msg.header.frame_id = self.base_frame
            self.pothole_mask_image_pub.publish(mask_msg)

        if low_mask is not None:
            candidate_msg = self.bridge.cv2_to_imgmsg(low_mask, encoding='mono8')
            candidate_msg.header.stamp = stamp
            candidate_msg.header.frame_id = self.base_frame
            self.pothole_candidate_mask_image_pub.publish(candidate_msg)

        if rejection_debug_image is not None:
            rejection_msg = self.bridge.cv2_to_imgmsg(
                rejection_debug_image,
                encoding='bgr8'
            )
            rejection_msg.header.stamp = stamp
            rejection_msg.header.frame_id = self.base_frame
            self.pothole_rejection_debug_image_pub.publish(rejection_msg)

        self.publish_debug_mask_image(
            self.detector.last_masks.get("pit_bottom"),
            self.pit_bottom_mask_image_pub,
            stamp
        )
        self.publish_debug_mask_image(
            self.detector.last_masks.get("negative_edges"),
            self.negative_edge_mask_image_pub,
            stamp
        )
        self.publish_debug_mask_image(
            self.detector.last_masks.get("unknown_holes"),
            self.unknown_hole_mask_image_pub,
            stamp
        )
        self.publish_debug_mask_image(
            self.detector.last_masks.get("ground_like"),
            self.ground_like_mask_image_pub,
            stamp
        )

    def publish_debug_mask_image(self, mask, publisher, stamp):
        if mask is None:
            return

        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header.stamp = stamp
        mask_msg.header.frame_id = self.base_frame
        publisher.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)

    node = HeightMapPotholeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info('Ctrl+C received, shutting down...')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
