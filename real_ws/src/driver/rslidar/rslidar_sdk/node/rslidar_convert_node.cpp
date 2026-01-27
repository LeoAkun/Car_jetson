#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// 转换rslidar点云格式为livox格式
class RslidarConverter: public rclcpp::Node{
public:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_rslidar;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_converted;

public:
    RslidarConverter(const std::string & node_name):Node(node_name){
        this->subscriber_rslidar = this->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points",
                10,
                std::bind(&RslidarConverter::callback_rslidar, this, std::placeholders::_1));
        this->publisher_converted = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rslidar_Pointcloud2",10);

        RCLCPP_INFO(this->get_logger(), "Rslidar to Livox format converter node started");
    }

private:
    // 把rslidar的PointCloud2格式转换为livox格式的PointCloud2
    void callback_rslidar(sensor_msgs::msg::PointCloud2::SharedPtr rslidar_cloud)
    {
        // 创建输出点云
        sensor_msgs::msg::PointCloud2 livox_cloud;
        livox_cloud.header = rslidar_cloud->header;
        livox_cloud.header.frame_id = "livox_frame";  // 修改frame_id为livox_frame
        livox_cloud.height = 1;  // livox格式为无序点云，height=1
        livox_cloud.width = rslidar_cloud->height * rslidar_cloud->width;  // 总点数
        livox_cloud.is_bigendian = false;
        livox_cloud.is_dense = true;

        // 设置字段 (x, y, z, intensity, ring, time)
        livox_cloud.fields.resize(6);

        // x字段
        livox_cloud.fields[0].name = "x";
        livox_cloud.fields[0].offset = 0;
        livox_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        livox_cloud.fields[0].count = 1;

        // y字段
        livox_cloud.fields[1].name = "y";
        livox_cloud.fields[1].offset = 4;
        livox_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        livox_cloud.fields[1].count = 1;

        // z字段
        livox_cloud.fields[2].name = "z";
        livox_cloud.fields[2].offset = 8;
        livox_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        livox_cloud.fields[2].count = 1;

        // intensity字段
        livox_cloud.fields[3].name = "intensity";
        livox_cloud.fields[3].offset = 12;
        livox_cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        livox_cloud.fields[3].count = 1;

        // ring字段 (激光线号)
        livox_cloud.fields[4].name = "ring";
        livox_cloud.fields[4].offset = 16;
        livox_cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
        livox_cloud.fields[4].count = 1;

        // time字段 (相对时间)
        livox_cloud.fields[5].name = "time";
        livox_cloud.fields[5].offset = 20;
        livox_cloud.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
        livox_cloud.fields[5].count = 1;

        // 设置点云步长 (x,y,z,intensity,ring,time = 24 bytes)
        livox_cloud.point_step = 24;
        livox_cloud.row_step = livox_cloud.point_step * livox_cloud.width;

        // 分配数据空间
        livox_cloud.data.resize(livox_cloud.row_step * livox_cloud.height);

        // 使用迭代器填充数据
        sensor_msgs::PointCloud2Iterator<float> iter_x_out(livox_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y_out(livox_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z_out(livox_cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity_out(livox_cloud, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_out(livox_cloud, "ring");
        sensor_msgs::PointCloud2Iterator<float> iter_time_out(livox_cloud, "time");

        // 从rslidar点云读取数据
        sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*rslidar_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*rslidar_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*rslidar_cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity_in(*rslidar_cloud, "intensity");
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring_in(*rslidar_cloud, "ring");
        sensor_msgs::PointCloud2ConstIterator<double> iter_timestamp_in(*rslidar_cloud, "timestamp");

        size_t valid_points = 0;
        for (size_t i = 0; i < rslidar_cloud->height * rslidar_cloud->width; ++i) {
            // 复制点云数据
            *iter_x_out = *iter_x_in;
            *iter_y_out = *iter_y_in;
            *iter_z_out = *iter_z_in;
            *iter_intensity_out = *iter_intensity_in;
            *iter_ring_out = *iter_ring_in;

            // 将timestamp转换为相对时间(秒)
            // rslidar的timestamp是double类型的绝对时间戳
            // 转换为相对于帧开始的时间
            *iter_time_out = static_cast<float>(*iter_timestamp_in);

            ++iter_x_in;
            ++iter_y_in;
            ++iter_z_in;
            ++iter_intensity_in;
            ++iter_ring_in;
            ++iter_timestamp_in;

            ++iter_x_out;
            ++iter_y_out;
            ++iter_z_out;
            ++iter_intensity_out;
            ++iter_ring_out;
            ++iter_time_out;

            valid_points++;
        }

        // 发布转换后的点云
        publisher_converted->publish(livox_cloud);

        // RCLCPP_INFO(this->get_logger(), "转换完成: %zu 个点", valid_points);
    }
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto converter = std::make_shared<RslidarConverter>("rslidar_converter");
    rclcpp::spin(converter);
    rclcpp::shutdown();
    return 0;
}
