import rclpy
import rclpy.node
import rclpy.action
from global_route_msg.srv import GlobalPlanRoute
from ament_index_python.packages import get_package_share_directory
from .utils import MapManagerUtils, PublicUtils
import yaml, os
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap

import time

class MapManager(rclpy.node.Node):
    def __init__(self, name, mode = "test"):
        super().__init__(node_name=name)
        
        self.mode= mode

        with open(get_package_share_directory("map_manager") + "/config/generate_gird_config.yaml", 
                "r", 
                encoding="utf-8") as f:
            data = yaml.safe_load(f)['generate_gird_config']

        # 地图与边界坐标对应关系的json文件路径
        self.map_json = data['json_path']

        # 地图路径
        self.map_folder = data['map_folder']

        # 全局路径规划器客户端， 用于申请全局路径
        self.global_planner_client = self.create_client(GlobalPlanRoute, 'CacuGlobalPlanService')

        # 全局路径点
        self.points = []

        # nav2动作客户端, 用于发送导航请求
        self.nav2_action_client = rclpy.action.ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # 导入地图客户端，用于更换nav2地图
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
        self.current_map = ""
        self.nav_result = -1

    # 向global_planner发送，请求获取全局路径
    def send_global_planner_request(self, start_name: str, dest_name: str):

        # 等待global_planner服务可用
        while not self.global_planner_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务 [CacuGlobalPlanService] ...')

        req = GlobalPlanRoute.Request()

        req.start_place_name = start_name
        req.dest_place_name = dest_name
        
        # 异步发送请求
        future = self.global_planner_client.call_async(req)

        # 阻塞直到响应完成
        rclpy.spin_until_future_complete(self, future)  
        res = future.result()

        # 保存全局路径 longtitude=103.9831145      latitude = 30.7644525 
        self.points = list(res.points)
        self.get_logger().info(f"接收到路径点数量: {len(self.points)}")        

    # 向navigaion2发送动作请求
    def send_nav2_goal(self, msg : tuple[float, float, float, float]):

        # 等待nav2服务可用
        while not self.nav2_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('等待动作服务 [navigation2] ...')

        x, y, z, w = msg
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.w = w
        goal_msg.behavior_tree = ""

        # 阻塞等待导航结果
        future = self.nav2_action_client.send_goal_async(
            goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('导航动作被拒绝')
            return False
        result_future = goal_handle.get_result_async()
        self.get_logger().info('开始导航，等待结果...')

        # 阻塞等待结果 Future 完成
        rclpy.spin_until_future_complete(self, result_future)
        result_response = result_future.result()
        if result_response.status == 4: # 成功状态
            self.get_logger().info('✅ 导航成功到达目标点。')
        else:
            self.get_logger().error(f'❌ 导航失败，状态: {result_response.status}')

    def __load_map(self, map_name):
        # 加载地图, 如果地图已经加载，则不需要再加载
        self.get_logger().info(f"当前系统中的地图:{self.current_map}, 待加载的地图:{map_name}")
        if map_name == self.current_map:
            self.get_logger().info(f"相同地图, 无需再加载")
        else:
            map_req = LoadMap.Request()
            
            if self.mode == "test":
                map_req.map_url = os.path.join("/home/akun/workspace/CAR/utils/pcd2pgm/pgm/",  map_name, map_name + "_map.yaml")
            else:
                map_req.map_url = os.path.join(self.map_folder, "map_pgm", map_name, map_name + ".yaml")

            times = 5.0
            self.get_logger().info(f"暂停{times}秒, 等待 RViz 渲染...")
            time.sleep(times)

            # 等待地图加载服务完成
            future = self.map_client.call_async(map_req)
            rclpy.spin_until_future_complete(self, future) # 内部会spin
            self.current_map = map_name

            self.get_logger().info(f"地图切换成功, 请求返回结果为:{future.result().result}")
        
    # 地图管理
    def map_manager(self):

        # 请求全局路径规划
        self.send_global_planner_request("西南交大犀浦3号教学楼", "鸿哲斋7号楼")
        
        print(self.points)
        # 用于测试
        self.count = 0
        
        # 依次判断各点所属地图，并依次进行导航
        # for index, point in enumerate(self.points):
        #     target_point_location = (point.longtitude, point.latitude)
            
        #     print(target_point_location)

        #     if self.mode == "test":
        #         # 测试用：用于测试地图切换服务是否正常
        #         if self.count % 2 == 0:
        #             map_name = "sim"
        #         else:
        #             map_name = "real"
        #     else:
        #         # 将点与地图进行匹配，获取所属地图名称
        #         map_name = MapManagerUtils.assign_points_to_maps(self.map_json, target_point_location)

        #     # 方案一：分段建图，导航时进行拼接，建图需要有重叠区域
        #     # 找到8连通地图
        #     neighbor = MapManagerUtils.get_neighbor_maps(self.map_json, map_name)
            
            # 拼接地图 
            
            # # 加载地图
            # self.__load_map(map_name)

            # # 转换为nav2可用于导航的坐标
            # MapManagerUtils.location_to_nav2_xy(self.map_folder, map_name, target_point_location)

            # # 计算每张地图的边界出口点，使用两点连线计算来找
        
            

            # # 导航到目标点
            # self.nav_result = -1
            # if self.mode == "test":

            #     if self.count % 2 == 0:
            #         x, y, z, w = (0.0, -2.0, 0.0, 2.0)
            #     else:
            #         x, y, z, w = (0.0, 0.0, 0.0, 1.0)
            # else:
            #     x, y, z, w = (1.0, 2.0, 0.0, 1.0)

            # self.get_logger().info(f"导航目标点为:{(x,y,z,w)}")
            # self.send_nav2_goal((x,y,z,w)) 

            # # 测试用
            # self.count += 1
        
def main(args=None):
    rclpy.init(args=args)
    mapManager = MapManager('mapManager', "real")
    mapManager.map_manager()
    mapManager.destroy_node()
    rclpy.shutdown()
