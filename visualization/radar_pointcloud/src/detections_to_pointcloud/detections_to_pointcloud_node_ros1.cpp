#include <ros/ros.h>

#include "ros1/node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detections_to_pointcloud");
    ros::NodeHandle n("~");

    radar_pointcloud::ros1::Node node(n);

    ros::spin();

    return 0;
}
