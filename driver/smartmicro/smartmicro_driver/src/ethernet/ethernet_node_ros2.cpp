#include <rclcpp/rclcpp.hpp>

#include "ethernet/ros2/node.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<smartmicro::ros2::Node>());
    rclcpp::shutdown();
    return 0;
}
