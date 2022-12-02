#include <rclcpp/rclcpp.hpp>

#include "node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radar_marker::Converter>("radar_marker_absolute"));
    rclcpp::shutdown();
    return 0;
}
