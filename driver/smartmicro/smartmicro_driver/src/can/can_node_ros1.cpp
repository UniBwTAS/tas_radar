#include <ros/ros.h>

#include "can/ros1/node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_driver_umrr_can");
    ros::NodeHandle n("~");

    smartmicro::ros1::CanNode node(n);

    ros::spin();

    return 0;
}
