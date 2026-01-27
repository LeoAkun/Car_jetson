#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// IMU QoS转换节点：将/imu话题从RELIABLE改为BEST_EFFORT
class ImuQosConverter : public rclcpp::Node
{
public:
    ImuQosConverter(const std::string & node_name) : Node(node_name)
    {
        // 订阅/imu话题，使用RELIABLE QoS（默认）
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile);

        subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            sub_qos,
            std::bind(&ImuQosConverter::imu_callback, this, std::placeholders::_1));

        // 发布到/imu_best_effort话题，使用BEST_EFFORT QoS
        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(rclcpp::ReliabilityPolicy::BestEffort)
            .durability(rclcpp::DurabilityPolicy::Volatile);

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu_best_effort",
            pub_qos);

        RCLCPP_INFO(this->get_logger(), "IMU QoS Converter node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /imu with RELIABLE QoS");
        RCLCPP_INFO(this->get_logger(), "Publishing to /imu_best_effort with BEST_EFFORT QoS");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 直接转发IMU消息，不做任何修改
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto converter = std::make_shared<ImuQosConverter>("imu_qos_converter");
    rclcpp::spin(converter);
    rclcpp::shutdown();
    return 0;
}
