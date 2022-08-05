// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#include <ros/ros.h>

#include "node.h"

int main(int argc, char **argv)
{
    /// Initialisierung und Vorbereitungen
    ros::init(argc, argv, "radar_driver_umrr_can");

    /// Aufbau
    ros::NodeHandle n("~");

    /// Node starten
    Node Node(n);

    /// Abfahrt
    ros::spin();

    return 0;
}
