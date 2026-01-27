import open3d as o3d
import numpy as np

raw_map_path = "/home/akun/workspace/CAR/LIO-SAM/pcd_global/real/cloudGlobal2.pcd"
clean_map_path = "/home/akun/workspace/CAR/utils/pcd_filter/map_clean/cloudGlobal_clean2.pcd"

def main():
    # 1️⃣ 读取原始点云
    raw_map_pcd = o3d.io.read_point_cloud(raw_map_path)
    print(f"原始点云数量: {len(raw_map_pcd.points)}")

    # 2️⃣ 下采样（体素滤波）
    voxel_size = 0.02  # 5cm
    pcd_down = raw_map_pcd.voxel_down_sample(voxel_size)
    print(f"下采样后点云数量: {len(pcd_down.points)}")

    # 3️⃣ 去除离群点（统计滤波）
    pcd_down, _ = pcd_down.remove_statistical_outlier(nb_neighbors=15, std_ratio=0.5)

    # 4️⃣ 去除地面（RANSAC）
    plane_model, inliers = pcd_down.segment_plane(distance_threshold=0.02,
                                                 ransac_n=3,
                                                 num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"地面方程: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")

    # 去掉地面点，仅保留上部结构
    pcd_no_ground = pcd_down.select_by_index(inliers, invert=True)

    # 5️⃣ 法线估计
    pcd_no_ground.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(30))
    normals = np.asarray(pcd_no_ground.normals)

    # 6️⃣ 仅保留垂直方向的点（墙体）
    # 垂直意味着法线方向几乎水平 → z 分量较小
    vertical_mask = np.abs(normals[:, 2]) < 0.2  # 调整阈值：越小越严格
    vertical_pcd = pcd_no_ground.select_by_index(np.where(vertical_mask)[0])
    print(f"垂直结构点数: {len(vertical_pcd.points)}")

    # 7️⃣ 对垂直结构聚类，去除孤立小簇
    labels = np.array(vertical_pcd.cluster_dbscan(eps=0.35, min_points=100, print_progress=True))
    max_label = labels.max()
    print(f"检测到 {max_label + 1} 个聚类")

    # 8️⃣ 仅保留较大簇（大面积墙体）
    counts = np.bincount(labels[labels >= 0])
    large_clusters = [i for i, c in enumerate(counts) if c > 1000]
    vertical_pcd = vertical_pcd.select_by_index(np.where(np.isin(labels, large_clusters))[0])
    print(f"保留的大簇数量: {len(large_clusters)}")

    # 🔟 保存最终结果
    o3d.io.write_point_cloud(clean_map_path, vertical_pcd)
    print(f"✅ 已保存垂直墙体点云：{clean_map_path}")

