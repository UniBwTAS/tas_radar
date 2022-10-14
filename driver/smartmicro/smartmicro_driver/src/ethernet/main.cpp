#include <rclcpp/rclcpp.hpp>

#include "node.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<smartmicro_driver::Driver>("smartmicro_driver_ethernet"));
    rclcpp::shutdown();
    return 0;
}
