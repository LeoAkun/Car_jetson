#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "tf2/LinearMath/QuadWord.h" // 四元数转换
#include "tf2_ros/static_transform_broadcaster.h"  //静态tf发布器类
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // 消息类型转换函数

#include <sensor_msgs/msg/imu.hpp>

// 转换雷达，IMU时间，custom转换为pointcloud2
class Subscriber: public rclcpp::Node{
public:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscriber_laser;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_laser;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_brd;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu;
public:
    Subscriber(const std::string & node_name):Node(node_name){
        this->subscriber_laser = this->create_subscription<livox_ros_driver2::msg::CustomMsg>("/livox/lidar",
                10,
                std::bind(&Subscriber::callback_laser, this, std::placeholders::_1));
        this->publisher_laser = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_Pointcloud2",10);

        // this->subscriber_imu = this->create_subscription<sensor_msgs::msg::Imu>("/livox/imu",
        //         10,
        //         std::bind(&Subscriber::callback_imu, this, std::placeholders::_1));
        // this->publisher_imu = this->create_publisher<sensor_msgs::msg::Imu>("/livox_imu",10);

        // 创建静态tf发布器
        this->static_tf_brd = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // this->publish_static_tf();
    }

private:
    // 把Custom修改为PointCloud2D格式
    void callback_laser(livox_ros_driver2::msg::CustomMsg::SharedPtr custom)
    {
        //RCLCPP_INFO(this->get_logger(), "接收到数据");

        sensor_msgs::msg::PointCloud2 cloud2;
        cloud2.header=custom->header;
        cloud2.height = 1;
        cloud2.width = custom->point_num;
        cloud2.is_bigendian = false;
        cloud2.is_dense = true;
        // 设置字段 (x, y, z, intensity, ring, time)
        cloud2.fields.resize(6);
        
         // x字段
        cloud2.fields[0].name = "x";
        cloud2.fields[0].offset = 0;
        cloud2.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud2.fields[0].count = 1;
        
        // y字段
        cloud2.fields[1].name = "y";
        cloud2.fields[1].offset = 4;
        cloud2.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud2.fields[1].count = 1;
        
        // z字段
        cloud2.fields[2].name = "z";
        cloud2.fields[2].offset = 8;
        cloud2.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud2.fields[2].count = 1;
        
        // intensity字段
        cloud2.fields[3].name = "intensity";
        cloud2.fields[3].offset = 12;
        cloud2.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud2.fields[3].count = 1;
        
        // ring字段 (激光线号)
        cloud2.fields[4].name = "ring";
        cloud2.fields[4].offset = 16;
        cloud2.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
        cloud2.fields[4].count = 1;
        
        // time字段 (相对时间)
        cloud2.fields[5].name = "time";
        cloud2.fields[5].offset = 20;
        cloud2.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud2.fields[5].count = 1;

        // 设置点云步长 (x,y,z,intensity,ring,time = 24 bytes)
        cloud2.point_step = 24;
        cloud2.row_step = cloud2.point_step * cloud2.width;

        // 分配数据空间
        cloud2.data.resize(cloud2.row_step * cloud2.height);

        // 使用迭代器填充数据
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud2, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud2, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud2, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud2, "intensity");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_ring(cloud2, "ring");
        sensor_msgs::PointCloud2Iterator<float> iter_time(cloud2, "time");

        for (const auto& point : custom->points) {
            // 跳过无效点 (x=y=z=0)
            // if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f) {
            //     continue;
            // }
            
            *iter_x = point.x;
            *iter_y = point.y;
            *iter_z = point.z;
            *iter_intensity = static_cast<float>(point.reflectivity) / 255.0f; // 归一化到[0,1]
            *iter_ring = point.line;
            *iter_time = static_cast<float>(point.offset_time) * 1e-9f; // 转换为秒
            
            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_intensity;
            ++iter_ring;
            ++iter_time;
        }

        // 发布转换后的点云
        publisher_laser->publish(cloud2);
        
        //RCLCPP_INFO(this->get_logger(), "转换完成: %u 个有效点 (原始: %u 个点)", cloud2.width, custom->point_num);
    }


    // // 修改imu坐标系名称
    // void callback_imu(sensor_msgs::msg::Imu::SharedPtr imus)
    // {
    //     // RCLCPP_INFO(this->get_logger(), "imu\n");
    //     imus->header.frame_id="imu_link";
    //     imus->linear_acceleration.x*=9.80511;
    //     imus->linear_acceleration.y*=9.80511;
    //     imus->linear_acceleration.z*=9.80511;
    //     publisher_imu->publish(*imus);
    // }

};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto sub = std::make_shared<class Subscriber>("convert");
    rclcpp::spin(sub);
    rclcpp::shutdown();
    return 0;
    
}