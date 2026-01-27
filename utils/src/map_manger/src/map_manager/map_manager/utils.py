import json, os, glob, yaml, shutil, geomag, math
import osmnx as ox
import geopandas as gpd
import numpy as np
from shapely.geometry import box, Polygon, MultiPolygon, GeometryCollection
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
from pyproj import Transformer
from pyproj import CRS, Transformer

# 公共工具类，如可视化
class PublicUtils:

    # 根据json可视化地图
    @classmethod
    def visualize_from_json(cls, json_path, highlight_map=None, point=None):
        import json
        import geopandas as gpd
        import matplotlib.pyplot as plt
        from shapely.geometry import Polygon, Point

        # 读取 JSON
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        polygons = []
        map_names = []

        for entry in data:
            corners = entry["parts"][0]["corner_coordinates"]
            poly = Polygon([(c["lon"], c["lat"]) for c in corners])
            polygons.append(poly)
            map_names.append(entry["map"])

        gdf = gpd.GeoDataFrame(geometry=polygons, crs="EPSG:4326")

        # === 可视化 ===
        fig, ax = plt.subplots(figsize=(10, 10))

        # --- 画所有格子（绿色边框）---
        gdf.plot(ax=ax, facecolor="none", edgecolor="green", linewidth=1)

        # --- 标注每个格子名（精确居中） ---
        for geom, name in zip(gdf.geometry, map_names):
            c = geom.centroid
            ax.text(
                c.x, c.y, name,
                fontsize=9,
                ha='center', va='center',
                color="black",
                bbox=dict(facecolor="white", alpha=0.6, edgecolor="none")
            )

        # ====== 若用户提供 highlight_map ======
        if highlight_map is not None:
            if highlight_map in map_names:
                idx = map_names.index(highlight_map)
                gdf.iloc[[idx]].plot(
                    ax=ax,
                    facecolor="none",
                    edgecolor="red",
                    linewidth=2
                )

        # ====== 若用户提供 point ======
        if point is not None:
            lon, lat = point
            ax.plot(lon, lat, "ro")

        plt.title("Grid Visualization from JSON")
        plt.show()

# 地图管理工具类
class MapManagerUtils:

    # 给定经纬度，判断所属的地图
    @staticmethod
    def assign_points_to_maps(json_path, location):
        # 1. JSON
        with open(json_path, "r", encoding="utf-8") as f:
            map_data = json.load(f)
        
        # 2. 构建 map 文件对应的 Polygon
        longitude, latitude = location
        map_polygons = {}
        for entry in map_data:
            map = entry["map"]
            # 这里只取每个 map 的第一部分
            corners = entry["parts"][0]["corner_coordinates"]
            polygon = Polygon([(c["lon"], c["lat"]) for c in corners])
            map_polygons[map] = polygon   

        # 3. 定义要判断的点
        point = Point(longitude, latitude) 

        # 4. 判断这个点在哪个地图内
        for map, poly in map_polygons.items():
            if poly.contains(point):
                print(f"[INFO] [MapManagerUtils] 点{point}在地图 {map} 内")
                return map
        else:
            print(f"[WARN] [MapManagerUtils] 点不在任何地图内")
            return None

    # 给定目标点经纬度、原点经纬度，将目标点经纬度转为nav2坐标系
    @staticmethod
    def location_to_nav2_xy(map_folder : str, map_name : str, target_point_location : tuple[float, float]):
        
        with open( os.path.join(map_folder, map_name + ".yaml") ) as f:
            origin_data = yaml.safe_load(f)
            lon_origin = origin_data["origin"]["longitude"]
            lat_origin = origin_data["origin"]["latitude"]
        lon_target, lat_target = target_point_location

        # 将目标点转为以origin为原点的真北坐标系（X正东，Y正北）
        local_crs = CRS.from_proj4(
            f"+proj=tmerc +lat_0={lat_origin} +lon_0={lon_origin} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs")
        wgs84_crs = CRS("EPSG:4326")
        transformer = Transformer.from_crs(wgs84_crs, local_crs, always_xy=True)
        east_local, north_local = transformer.transform(lon_target, lat_target) # x表示东，y表示北
        
        # 计算地图map原点（机器人启动时指向IMU的磁北与正北之间）的磁偏角
        dec= geomag.declination( dlat=lat_origin, dlon=lon_origin)

        # 将角度转换为弧度
        rotation_rad = math.radians(dec)

        # 应用旋转矩阵，将目标点转为以origin为原点的磁北坐标系，即map坐标系，因为机器人建图启动时是以imu磁北方向为正方向
        east_nav2 = east_local * math.cos(rotation_rad) - north_local * math.sin(rotation_rad)
        north_nav2 = east_local * math.sin(rotation_rad) + north_local * math.cos(rotation_rad)

        # 转换为ros导航坐标系(nav2)，x前侧即磁北，y左侧即磁西
        x_nav2 = north_nav2
        y_nav2 = -east_nav2
        
        return x_nav2, y_nav2

    # 给定map_name后，获取周围8连通的地图名 (更新以使用 index_x, index_y)
    @staticmethod
    def get_neighbor_maps(json_path: str, map_name: str) -> list[str]:
        """
        给定地图名，获取周围八连通（包括对角线）的地图名列表。
        使用 JSON 中的 'index_x' 和 'index_y' 字段进行计算。

        :param json_path: 包含所有地图网格信息的 JSON 文件路径。
        :param map_name: 目标地图的名称（例如 'map12'）。
        :return: 周围存在的地图名列表。
        """
        # 1. 读取 JSON 数据
        with open(json_path, "r", encoding="utf-8") as f:
            map_data = json.load(f)
        
        # 2. 建立 map_name 到 (index_x, index_y) 的映射，并构建 (index_x, index_y) 到 map_name 的反向查找表
        target_map_info = None
        # 使用 (index_x, index_y) 作为 key
        index_to_map = {} 
        
        for entry in map_data:
            m_name = entry.get("map")
            m_x = entry.get("index_x")
            m_y = entry.get("index_y")
            
            # 必须确保 index_x 和 index_y 字段存在
            if m_name is None or m_x is None or m_y is None:
                print("[WARN] [MapManagerUtils] JSON数据缺少 'map', 'index_x' 或 'index_y' 字段，无法进行邻居查找。")
                return []
            
            index_to_map[(m_x, m_y)] = m_name
            
            if m_name == map_name:
                target_map_info = entry

        # 3. 检查目标地图是否存在
        if target_map_info is None:
            print(f"[WARN] [MapManagerUtils] 未找到地图: {map_name}")
            return []

        # 4. 获取目标地图的索引
        target_x = target_map_info["index_x"]
        target_y = target_map_info["index_y"]
        
        neighbor_maps = []
        
        # 定义 8 个方向的偏移量： (d_x, d_y)
        # d_x: -1 (左), 0 (中), 1 (右)
        # d_y: -1 (下), 0 (中), 1 (上)
        offsets = [
            (-1, -1), (0, -1), (1, -1),  # 左下，下，右下
            (-1, 0), (1, 0),             # 左，右
            (-1, 1), (0, 1), (1, 1)      # 左上，上，右上
        ]

        print(f"[INFO] 目标地图 {map_name} 的索引为: (x={target_x}, y={target_y})")

        # 5. 遍历 8 个方向，查找邻居
        for d_x, d_y in offsets:
            neighbor_x = target_x + d_x
            neighbor_y = target_y + d_y
            
            neighbor_index = (neighbor_x, neighbor_y)
            
            # 检查这个计算出的 (x, y) 索引在地图索引字典中是否存在
            if neighbor_index in index_to_map:
                neighbor_name = index_to_map[neighbor_index]
                neighbor_maps.append(neighbor_name)
                print(f"[INFO] 发现邻居: {neighbor_name}，索引: {neighbor_index}")
            else:
                # 位于边界，忽略该方向
                pass

        print(f"✔ 地图 {map_name} 的 8 连通邻居: {neighbor_maps}")
        return neighbor_maps
    
# 给全局地图划分格子
class GeneratorGridJson:
    def __init__(self, place_name, grid_size, json_path):
        self.place_name = place_name
        self.grid_size = grid_size
        self.json_path = json_path

        self.campus = None
        self.grid_gdf_wgs = None
        self.final_json = None
        self.campus_utm = None

    # 加载地图
    def load_campus(self):
        print("加载地图边界...")
        self.campus = ox.geocode_to_gdf(self.place_name)
        self.campus_utm = self.campus.to_crs(self.campus.estimate_utm_crs())
        print("地图加载完成")

    # 生成网格 (核心修改)
    def generate_grid(self):
        campus_poly = self.campus_utm.geometry.iloc[0]
        minx, miny, maxx, maxy = self.campus_utm.total_bounds

        x_edges = np.arange(minx, maxx, self.grid_size)
        y_edges = np.arange(miny, maxy, self.grid_size)

        grid_cells_with_info = []

        print("生成并裁剪网格中...")

        # --- 遍历并记录索引 ---
        for col_idx, x in enumerate(x_edges):
            for row_idx, y in enumerate(y_edges):
                cell = box(x, y, x + self.grid_size, y + self.grid_size)

                if cell.intersects(campus_poly):
                    clipped = cell.intersection(campus_poly)
                    if not clipped.is_empty:
                        # 存储几何图形以及行/列索引
                        grid_cells_with_info.append({
                            'geometry': clipped,
                            'row': row_idx,
                            'col': col_idx
                        })

        # 从字典列表创建 GeoDataFrame
        if not grid_cells_with_info:
            print("❌ 未生成任何有效网格")
            self.grid_gdf_wgs = gpd.GeoDataFrame(
                {'geometry': [], 'row': [], 'col': []}, 
                crs=self.campus_utm.crs
            )
        else:
            grid_gdf = gpd.GeoDataFrame(grid_cells_with_info, crs=self.campus_utm.crs)
            self.grid_gdf_wgs = grid_gdf.to_crs(epsg=4326)

        print(f"✔ 生成了 {len(self.grid_gdf_wgs)} 个网格")

    # 提取角点网格
    def __extract_corners(self, geom):
        if isinstance(geom, Polygon):
            parts = [geom]
        elif isinstance(geom, MultiPolygon):
            parts = list(geom.geoms)
        elif isinstance(geom, GeometryCollection):
            parts = [g for g in geom.geoms if isinstance(g, (Polygon, MultiPolygon))]
        else:
            return []

        part_list = []
        for p in parts:
            exterior = list(p.exterior.coords)[:-1]
            corners = [{"lat": float(lat), "lon": float(lon)} for lon, lat in exterior]
            part_list.append({
                "num_vertices": len(corners),
                "corner_coordinates": corners
            })

        return part_list

    # 构建 JSON
    def build_json(self):
        print("构建 JSON 中...")
        json_dir = os.path.dirname(self.json_path)
        if json_dir:  # 只有当目录路径不为空时才创建
            os.makedirs(json_dir, exist_ok=True)

        json_list = []

        # 遍历 GeoDataFrame，注意我们现在可以访问 row 和 col
        for idx, row in self.grid_gdf_wgs.iterrows():
            geom = row.geometry
            entry = {
                "map": f"map{idx}",
                "index_x": int(row['col']), # 添加列索引
                "index_y": int(row['row']), # 添加行索引
                "parts": self.__extract_corners(geom)
            }
            json_list.append(entry)

        # 保存 JSON
        save_path = self.json_path
        with open(save_path, "w", encoding="utf-8") as f:
            json.dump(json_list, f, indent=2, ensure_ascii=False)

        self.final_json = json_list

        print(f"✔ JSON 已保存到: {save_path}")

        return json_list, len(json_list)

# 生成假地图数据
class GeneratorFalseData:
    def __init__(self, pcd_folder, pgm_folder, origin_location_folder):
        self.pcd_folder = pcd_folder
        self.pgm_folder = pgm_folder
        self.origin_location_folder = origin_location_folder
        os.makedirs(self.pcd_folder, exist_ok=True)

    def __generate_pgm(self, map_idx):
        """
        生成空的pgm文件
        :param map_idx: 地图索引
        """
        # 创建地图目录
        map_dir = os.path.join(self.pgm_folder, f"map{map_idx}")
        os.makedirs(map_dir, exist_ok=True)
        
        # 生成空的pgm文件
        pgm_path = os.path.join(map_dir, f"map{map_idx}.pgm")
        with open(pgm_path, 'w') as f:
            f.write("P2\n")
            f.write("# Created by map_manager\n")
            f.write("1 1\n")
            f.write("255\n")
            f.write("205\n")  # 灰色值，表示未知区域
        
        return pgm_path
    
    def __generate_yaml(self, map_idx):
        """
        生成yaml文件
        :param map_idx: 地图索引
        """
        # 创建地图目录
        map_dir = os.path.join(self.pgm_folder, f"map{map_idx}")
        os.makedirs(map_dir, exist_ok=True)
        
        # 生成yaml文件
        yaml_path = os.path.join(map_dir, f"map{map_idx}.yaml")
        with open(yaml_path, 'w') as f:
            f.write("image: map{}.pgm\n".format(map_idx))
            f.write("resolution: 0.050000\n")
            f.write("origin: [0.0, 0.0, 0.0]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.196\n")
        
        return yaml_path
    
    def generate_pcd_pgm(self, num_maps):
        """
        根据 map 数量生成空 PCD 文件，并删除旧的 map*.pcd 文件
        :param num_maps: map 文件数量
        """
        print(f"开始生成 {num_maps} 个空 PCD 文件...")

        # 删除旧的map_pcd 的内容
        old_files = glob.glob(os.path.join(self.pcd_folder, "map*.pcd"))
        for fpath in old_files:
            os.remove(fpath)
        
        # 删除旧的map_pgm目录内容
        old_dirs = glob.glob(os.path.join(self.pgm_folder, "map*"))
        for fpath in old_dirs:
            # 检查是否是目录，如果是，使用 rmtree 删除；如果不是文件，使用 os.remove
            if os.path.isdir(fpath):
                shutil.rmtree(fpath)
            elif os.path.isfile(fpath):
                os.remove(fpath) # 保险起见，虽然按命名规则不该有文件，但仍可处理

        # 生成空 PCD 文件
        for idx in range(num_maps):
            file_path = os.path.join(self.pcd_folder, f"map{idx}.pcd")
            with open(file_path, 'w') as f:
                f.write(
                    "# .PCD v0.7 - Point Cloud Data file\n"
                    "VERSION 0.7\n"
                    "FIELDS x y z\n"
                    "SIZE 4 4 4\n"
                    "TYPE F F F\n"
                    "COUNT 1 1 1\n"
                    "WIDTH 0\n"
                    "HEIGHT 1\n"
                    "VIEWPOINT 0 0 0 1 0 0 0\n"
                    "POINTS 0\n"
                    "DATA ascii\n"
                )

            # 生成对应的pgm和yaml文件
            pgm_path = self.__generate_pgm(idx)
            yaml_path = self.__generate_yaml(idx)

        print(f"✔ 所有空 PCD 文件生成完成, 路径为{self.pcd_folder}")
        print(f"✔ 所有空 PGM 和 YAML 文件生成完成{self.pgm_folder}")
    
    def generate_origin_location(self, num_maps):
        """
        生成每张地图的地理原点坐标 YAML 文件。
        文件格式:
        origin:
        latitude: ***
        longitude: ***

        会确保：
        - 若目录不存在 -> 创建
        - 若目录已存在 -> 清空目录后再生成

        :param num_maps: 地图数量
        """

        print(f"开始生成 {num_maps} 个地图的 geo 原点坐标文件...")

        # === 先判断目录是否存在 ===
        if os.path.exists(self.origin_location_folder):
            print(f"目录 {self.origin_location_folder} 已存在，正在清空...")
            shutil.rmtree(self.origin_location_folder)   # 删除整个目录
        else:
            print(f"目录 {self.origin_location_folder} 不存在，将创建...")

        # 重新创建目录
        os.makedirs(self.origin_location_folder, exist_ok=True)

        # === 生成文件 ===
        for idx in range(num_maps):

            # 示例经纬度，可换成真实值
            lat = 30.7644525 + idx * 0.00010
            lon = 103.9831145 + idx * 0.00010

            origin_yaml_path = os.path.join(self.origin_location_folder, f"map{idx}.yaml")

            data = {
                "origin": {
                    "latitude": float(lat),
                    "longitude": float(lon)
                }
            }

            with open(origin_yaml_path, "w", encoding="utf-8") as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

        print(f"✔ 所有地图的 geo 原点坐标文件生成完成, 路径为{self.origin_location_folder}")
        