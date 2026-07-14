#include <ros/ros.h>

#include "ros1/node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_marker_absolute");
    ros::NodeHandle n("~");

    radar_marker::ros1::Node node(n);

    ros::spin();

    return 0;
}
