#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "tf2/LinearMath/QuadWord.h" // 四元数转换
#include "tf2_ros/static_transform_broadcaster.h"  //静态tf发布器类
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // 消息类型转换函数

// TODO转换雷达，IMU时间，将仿真时间转换为真实时间
class Publisher: public rclcpp::Node{
public:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_brd;
public:
    Publisher(const std::string & node_name):Node(node_name){
        // 创建发布者
        this->publisher_imu = this->create_publisher<sensor_msgs::msg::Imu>("/my_imu",10);
        
        // 创建定时器
        timer = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&Publisher::timer_callback, this));
        
        // 创建静态tf发布器
        this->static_tf_brd = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->publish_static_tf();
    }

private:
    void timer_callback(){
        auto msg = sensor_msgs::msg::Imu();  
        auto now = std::chrono::system_clock::now();  
        auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        uint64_t ns_since_epoch = now_ns.time_since_epoch().count();       
        rclcpp::Time real_time(ns_since_epoch, RCL_SYSTEM_TIME);
        msg.header.stamp = real_time;
        msg.header.frame_id = "imu_link";
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;
        msg.angular_velocity.x = 0.0;
        msg.angular_velocity.y = 0.0;
        msg.angular_velocity.z = 0.0;
        msg.linear_acceleration.x = 0.0;
        msg.linear_acceleration.y = 0.0;
        msg.linear_acceleration.z = -9.81;  // 标准重力加速度
        // 发布消息
        publisher_imu->publish(msg);
    }

    void publish_static_tf(void)
    {
        // 构建消息接口，设置静态变换的参数
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "base_link";
        tf.child_frame_id = "imu_link";
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;
        
        // 弧度制转四元数
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        this->static_tf_brd->sendTransform(tf);
        RCLCPP_INFO(get_logger(),"Cpp:发布静态tf");
    }
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto sub = std::make_shared<class Publisher>("publisher");
    rclcpp::spin(sub);
    rclcpp::shutdown();
    return 0;
    
}