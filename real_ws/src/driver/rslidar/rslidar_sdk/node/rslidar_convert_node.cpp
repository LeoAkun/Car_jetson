#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// 转换rslidar点云格式为LIO-SAM兼容格式
class RslidarConverter: public rclcpp::Node{
public:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_rslidar;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_converted;

public:
    RslidarConverter(const std::string & node_name):Node(node_name){
        // 使用BEST_EFFORT QoS以匹配LIO-SAM的要求
        auto qos = rclcpp::QoS(rclcpp::KeepLast(5))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);

        this->subscriber_rslidar = this->create_subscription<sensor_msgs::msg::PointCloud2>("/rslidar_points",
                10,
                std::bind(&RslidarConverter::callback_rslidar, this, std::placeholders::_1));
        this->publisher_converted = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rslidar_Pointcloud2", qos);

        RCLCPP_INFO(this->get_logger(), "Rslidar to LIO-SAM compatible format converter node started");
    }

private:
    // 把rslidar的PointCloud2格式转换为LIO-SAM兼容格式的PointCloud2
    // LIO-SAM需要: x, y, z, intensity, ring, time (XYZIRT格式)
    void callback_rslidar(sensor_msgs::msg::PointCloud2::SharedPtr rslidar_cloud)
    {
        // 创建输出点云
        sensor_msgs::msg::PointCloud2 output_cloud;
        output_cloud.header = rslidar_cloud->header;
        output_cloud.height = 1;  // 无序点云，height=1
        output_cloud.width = rslidar_cloud->height * rslidar_cloud->width;  // 总点数
        output_cloud.is_bigendian = false;
        output_cloud.is_dense = rslidar_cloud->is_dense;

        // 设置字段 (x, y, z, intensity, ring, time)
        // 注意：LIO-SAM期望的字段顺序和类型
        output_cloud.fields.resize(6);

        // x字段
        output_cloud.fields[0].name = "x";
        output_cloud.fields[0].offset = 0;
        output_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_cloud.fields[0].count = 1;

        // y字段
        output_cloud.fields[1].name = "y";
        output_cloud.fields[1].offset = 4;
        output_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_cloud.fields[1].count = 1;

        // z字段
        output_cloud.fields[2].name = "z";
        output_cloud.fields[2].offset = 8;
        output_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_cloud.fields[2].count = 1;

        // intensity字段
        output_cloud.fields[3].name = "intensity";
        output_cloud.fields[3].offset = 12;
        output_cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_cloud.fields[3].count = 1;

        // ring字段 (激光线号) - LIO-SAM使用uint16_t
        output_cloud.fields[4].name = "ring";
        output_cloud.fields[4].offset = 16;
        output_cloud.fields[4].datatype = sensor_msgs::msg::PointField::UINT16;
        output_cloud.fields[4].count = 1;

        // time字段 (相对时间) - LIO-SAM使用float
        output_cloud.fields[5].name = "time";
        output_cloud.fields[5].offset = 18;
        output_cloud.fields[5].datatype = sensor_msgs::msg::PointField::FLOAT32;
        output_cloud.fields[5].count = 1;

        // 设置点云步长 (x,y,z,intensity = 16 bytes, ring = 2 bytes, time = 4 bytes = 22 bytes total)
        output_cloud.point_step = 22;
        output_cloud.row_step = output_cloud.point_step * output_cloud.width;

        // 分配数据空间
        output_cloud.data.resize(output_cloud.row_step * output_cloud.height);

        // 使用迭代器填充数据
        sensor_msgs::PointCloud2Iterator<float> iter_x_out(output_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y_out(output_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z_out(output_cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity_out(output_cloud, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_out(output_cloud, "ring");
        sensor_msgs::PointCloud2Iterator<float> iter_time_out(output_cloud, "time");

        // 从rslidar点云读取数据
        sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*rslidar_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y_in(*rslidar_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z_in(*rslidar_cloud, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity_in(*rslidar_cloud, "intensity");
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring_in(*rslidar_cloud, "ring");
        sensor_msgs::PointCloud2ConstIterator<double> iter_timestamp_in(*rslidar_cloud, "timestamp");

        // 计算第一个点的时间戳作为基准
        double first_timestamp = 0.0;
        bool first_point = true;

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
            if (first_point) {
                first_timestamp = *iter_timestamp_in;
                first_point = false;
            }
            *iter_time_out = static_cast<float>(*iter_timestamp_in - first_timestamp);

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
        }

        // 发布转换后的点云
        publisher_converted->publish(output_cloud);
    }
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto converter = std::make_shared<RslidarConverter>("rslidar_converter");
    rclcpp::spin(converter);
    rclcpp::shutdown();
    return 0;
}
