#include "usb2can_core.h"
#include <iostream>
// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>



int main(int argc, char **argv)
{
    // ros::init(argc, argv, "can_module");
    
    // USB2CAN::CAN_app app;
    // app.run();

    rclcpp::init(argc, argv);
    auto node = std::make_shared<USB2CAN::CAN_app>();
    node->run();
    // rclcpp::spin(std::make_shared<USB2CAN::CAN_app>());
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
