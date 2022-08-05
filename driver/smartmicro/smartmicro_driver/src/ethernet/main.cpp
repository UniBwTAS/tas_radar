#include <ros/ros.h>

#include "node.h"

int main(int argc, char **argv)
{
    /// Initialization of ROS
    ros::init(argc, argv, "smartmicro_driver_ethernet");

    /// Initialization of the node
    ros::NodeHandle n("~");

    /// Start node
    Node node(n);

    /// Run node
    ros::spin();

    return 0;
}
