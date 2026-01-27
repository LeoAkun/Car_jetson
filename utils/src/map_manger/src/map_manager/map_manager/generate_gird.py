import rclpy
import rclpy.node
from .utils import GeneratorGridJson, GeneratorFalseData, PublicUtils
from ament_index_python.packages import get_package_share_directory
import yaml

def main():
    # 导入参数
    share_path = get_package_share_directory("map_manager")
    print(share_path)
    with open(share_path + "/config/generate_gird_config.yaml", "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)['generate_gird_config']
    print(data)
    map_folder = data['map_folder']
    json_path = data['json_path']
    place_name = data['place_name']
    grid_size = data['grid_size']
    use_empty_pcd = data['use_empty_pcd_pgm']

    # 给地图划分网格，并生成对应json文件
    generator = GeneratorGridJson(
        place_name=place_name,
        grid_size=grid_size, # 间隔200米划分网格
        json_path=json_path
    )
    
    generator.load_campus()
    generator.generate_grid()

    # 生成json文件
    _ ,len = generator.build_json()

    # 生成pcd、pgm、地图原点GPS坐标假数据
    if(use_empty_pcd == True):
        gen_false_data = GeneratorFalseData(pcd_folder=map_folder + '/map_pcd/', 
                                            pgm_folder=map_folder + '/map_pgm/', 
                                            origin_location_folder=map_folder + '/map_origin_location')
        gen_false_data.generate_pcd_pgm(len)
        gen_false_data.generate_origin_location(len)

    PublicUtils.visualize_from_json(json_path)
