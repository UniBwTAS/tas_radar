#include <ros/ros.h>

#include "ethernet/ros1/node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smartmicro_driver_ethernet");
    ros::NodeHandle n("~");

    smartmicro::ros1::Node node(n);

    ros::spin();

    return 0;
}
