#include <rclcpp/rclcpp.hpp>

#include "can/ros2/node.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<smartmicro::ros2::CanNode>());
    rclcpp::shutdown();
    return 0;
}
