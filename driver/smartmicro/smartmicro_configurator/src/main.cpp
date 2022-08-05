// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#include <ros/ros.h>
#include <QApplication>
#include <librosqt/QRosCallBackQueue.h>

#include "node.h"

int main(int argc, char **argv)
{
    /// Initialisierung und Vorbereitungen
    QApplication app(argc, argv);
    QRosCallBackQueue::replaceGlobalQueue();
    ros::init(argc, argv, "smartmicro_configurator", ros::init_options::NoSigintHandler);

    /// Aufbau
    ros::NodeHandle n("");

    /// Node starten
    Node node(n);
    node.show();

    /// Abfahrt
    return app.exec();
}
