// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#include <ros/ros.h>

#include "node.h"

int main(int argc, char **argv)
{
    /// Initialisierung und Vorbereitungen
    ros::init(argc, argv, "detection_accumulation");

    /// Aufbau
    ros::NodeHandle n("~");

    /// Node starten
    Node node(n);

    /// Abfahrt
    ros::spin();

    return 0;
}
