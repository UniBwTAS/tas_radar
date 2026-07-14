#include <rclcpp/rclcpp.hpp>

#include "ros2/node.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radar_marker::ros2::Node>());
    rclcpp::shutdown();
    return 0;
}
