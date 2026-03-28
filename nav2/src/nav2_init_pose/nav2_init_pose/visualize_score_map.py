# 可视化热力图
# python3 visualize_score_map.py --center_x -105.0 --center_y 284.0 --radius 15.0 --step 1.0 --angle_step 30

#!/usr/bin/env python3
"""可视化 fitness_score 在 XY 平面上的分布（3D 曲面图）

用法:
  # 采集数据并绘图
  python3 visualize_score_map.py --center_x <x> --center_y <y> --radius <r> --step <s>

  # 使用已有数据文件绘图（跳过 ROS 采集）
  python3 visualize_score_map.py --load score_data.npz

参数说明:
  --center_x   搜索中心 x（地图坐标，米）
  --center_y   搜索中心 y（地图坐标，米）
  --radius     搜索半径（米），默认 10.0
  --step       网格步长（米），默认 1.0
  --angle_step 角度搜索步长（度），默认 30
  --save       保存数据到文件（默认 score_data.npz）
  --load       从文件加载数据，跳过 ROS 采集
"""

import sys
import argparse
import numpy as np
import matplotlib
import os
if os.environ.get('DISPLAY'):
    matplotlib.use('TkAgg')
else:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from matplotlib import cm


def parse_args():
    parser = argparse.ArgumentParser(description='Fitness Score 3D 可视化')
    parser.add_argument('--center_x', type=float, default=None)
    parser.add_argument('--center_y', type=float, default=None)
    parser.add_argument('--radius',   type=float, default=10.0)
    parser.add_argument('--step',     type=float, default=1.0)
    parser.add_argument('--angle_step', type=int, default=30)
    parser.add_argument('--save',     type=str,   default='score_data.npz')
    parser.add_argument('--load',     type=str,   default=None)
    return parser.parse_args()


def collect_scores(center_x, center_y, radius, step, angle_step):
    """通过 ROS2 服务扫描 XY 网格，返回 (xs, ys, scores) 三个数组。"""
    import rclpy
    import math
    import transforms3d.euler as euler_lib
    from re_localization.srv import ReLocalization
    from rclpy.node import Node

    class ScoreCollector(Node):
        def __init__(self):
            super().__init__('score_visualizer')
            self.client = self.create_client(ReLocalization, '/re_localization')
            self.get_logger().info('等待 /re_localization 服务...')
            while not self.client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info('服务未就绪，继续等待...')
            self.get_logger().info('服务已连接！')

        def euler_to_quaternion(self, yaw_deg):
            yaw = math.radians(yaw_deg)
            w, x, y, z = euler_lib.euler2quat(0.0, 0.0, yaw, axes='sxyz')
            return x, y, z, w

        def query_score(self, x, y, angle_step):
            """查询指定位置的最佳 fitness_score（遍历所有角度取最小值）"""
            best_score = float('inf')
            request = ReLocalization.Request()
            for yaw in range(0, 360, angle_step):
                qx, qy, qz, qw = self.euler_to_quaternion(yaw)
                request.initial_pose.header.frame_id = 'map'
                request.initial_pose.header.stamp = self.get_clock().now().to_msg()
                request.initial_pose.pose.pose.position.x = float(x)
                request.initial_pose.pose.pose.position.y = float(y)
                request.initial_pose.pose.pose.position.z = 0.0
                request.initial_pose.pose.pose.orientation.x = qx
                request.initial_pose.pose.pose.orientation.y = qy
                request.initial_pose.pose.pose.orientation.z = qz
                request.initial_pose.pose.pose.orientation.w = qw

                future = self.client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                try:
                    resp = future.result()
                    if resp is not None and resp.success:
                        if resp.fitness_score < best_score:
                            best_score = resp.fitness_score
                except Exception as e:
                    self.get_logger().warn(f'服务调用异常: {e}')
            return best_score

    rclpy.init()
    node = ScoreCollector()

    # 生成网格点
    xs_1d = np.arange(center_x - radius, center_x + radius + step * 0.5, step)
    ys_1d = np.arange(center_y - radius, center_y + radius + step * 0.5, step)
    total = len(xs_1d) * len(ys_1d)
    print(f'网格大小: {len(xs_1d)} x {len(ys_1d)} = {total} 个点')
    print(f'每个点查询 {360 // angle_step} 个角度，共 {total * (360 // angle_step)} 次服务调用')

    xs_list, ys_list, scores_list = [], [], []
    count = 0
    for x in xs_1d:
        for y in ys_1d:
            count += 1
            score = node.query_score(x, y, angle_step)
            xs_list.append(x)
            ys_list.append(y)
            scores_list.append(score)
            print(f'[{count}/{total}] ({x:.2f}, {y:.2f}) -> score={score:.4f}')

    node.destroy_node()
    rclpy.shutdown()

    return np.array(xs_list), np.array(ys_list), np.array(scores_list)


def plot_3d(xs, ys, scores, center_x=None, center_y=None):
    """绘制 3D 曲面图和热力图"""
    # 去除 inf 值（服务未响应的点）
    valid = np.isfinite(scores)
    if not np.any(valid):
        print('没有有效的分数数据，无法绘图。')
        return

    # 剔除异常大值（超过有效值 99 百分位的 5 倍视为异常）
    p99 = np.percentile(scores[valid], 99)
    clamp_max = min(p99 * 5, scores[valid].max())
    scores_plot = np.clip(scores, 0, clamp_max)

    # 构建规则网格（用于曲面图）
    xs_unique = np.unique(xs)
    ys_unique = np.unique(ys)
    X, Y = np.meshgrid(xs_unique, ys_unique)
    Z = np.full(X.shape, np.nan)

    for i, x in enumerate(xs_unique):
        for j, y in enumerate(ys_unique):
            mask = (xs == x) & (ys == y)
            if np.any(mask):
                Z[j, i] = scores_plot[mask][0]

    fig = plt.figure(figsize=(16, 7))
    fig.suptitle('Fitness Score 空间分布', fontsize=14)

    # --- 左图：3D 曲面 ---
    ax1 = fig.add_subplot(121, projection='3d')
    surf = ax1.plot_surface(X, Y, Z, cmap=cm.viridis_r,
                            linewidth=0, antialiased=True, alpha=0.85)
    # 在曲面上叠加散点，便于看到采样点
    sc = ax1.scatter(xs, ys, scores_plot, c=scores_plot,
                     cmap=cm.viridis_r, s=20, zorder=5)
    if center_x is not None and center_y is not None:
        ax1.scatter([center_x], [center_y],
                    [scores_plot[np.argmin(np.hypot(xs - center_x, ys - center_y))]],
                    c='red', s=100, marker='*', zorder=10, label='搜索中心')
        ax1.legend()
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Fitness Score')
    ax1.set_title('3D 曲面')
    fig.colorbar(surf, ax=ax1, shrink=0.5, label='Score（越小越好）')

    # --- 右图：俯视热力图 ---
    ax2 = fig.add_subplot(122)
    # 找最优点
    best_idx = np.nanargmin(Z)
    best_j, best_i = np.unravel_index(best_idx, Z.shape)
    best_x = xs_unique[best_i]
    best_y = ys_unique[best_j]
    best_score = Z[best_j, best_i]

    im = ax2.pcolormesh(X, Y, Z, cmap='viridis_r', shading='auto')
    ax2.scatter([best_x], [best_y], c='red', s=150, marker='*',
                zorder=5, label=f'最优点 ({best_x:.2f},{best_y:.2f})\nscore={best_score:.4f}')
    if center_x is not None and center_y is not None:
        ax2.scatter([center_x], [center_y], c='orange', s=100, marker='o',
                    zorder=5, label=f'搜索中心 ({center_x:.2f},{center_y:.2f})')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('俯视热力图（颜色越深分数越低越好）')
    ax2.legend(loc='upper right', fontsize=8)
    fig.colorbar(im, ax=ax2, label='Score（越小越好）')

    plt.tight_layout()
    plt.show()
    print(f'最优位置: ({best_x:.2f}, {best_y:.2f})，最优 fitness_score: {best_score:.4f}')


def main():
    args = parse_args()

    if args.load:
        print(f'从文件加载数据: {args.load}')
        data = np.load(args.load)
        xs, ys, scores = data['xs'], data['ys'], data['scores']
        cx = float(data['center_x']) if 'center_x' in data else None
        cy = float(data['center_y']) if 'center_y' in data else None
    else:
        if args.center_x is None or args.center_y is None:
            print('错误：需要提供 --center_x 和 --center_y，或者使用 --load 加载已有数据。')
            sys.exit(1)
        cx, cy = args.center_x, args.center_y
        xs, ys, scores = collect_scores(cx, cy, args.radius, args.step, args.angle_step)
        np.savez(args.save, xs=xs, ys=ys, scores=scores, center_x=cx, center_y=cy)
        print(f'数据已保存到: {args.save}')

    plot_3d(xs, ys, scores, cx, cy)


if __name__ == '__main__':
    main()
