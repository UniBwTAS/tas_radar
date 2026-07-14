#pragma once

#include "ethernet/core/sms.h"

#include <ros/ros.h>
#include <ethernet_msgs/Packet.h>
#include <smartmicro_driver/Instructions.h>
#include <smartmicro_driver/Sensors.h>

#include <memory>
#include <string>

namespace smartmicro
{
namespace ros1
{

// ROS 1 wrapper for the SMS ethernet driver: maps ethernet_msgs / radar_msgs /
// smartmicro_driver messages onto the ROS-agnostic SmsDriver core.
class Node : public SmsSink
{
public:
    explicit Node(ros::NodeHandle& node_handle);

    // SmsSink
    void onDetections(const DetectionRecord& detections) override;
    void onInstructions(const Instructions& instructions) override;
    void onSensors(const Sensors& sensors) override;
    void onEthernetPacket(const EthernetPacket& packet) override;

private:
    void rosCallback_ethernetMeasurements(const ethernet_msgs::Packet::ConstPtr& msg);
    void rosCallback_ethernetAlive(const ethernet_msgs::Packet::ConstPtr& msg);
    void rosCallback_instructions(const smartmicro_driver::Instructions::ConstPtr& msg);

    ros::NodeHandle& ros_handle_;

    ros::Subscriber subscriber_ethernet_measurements_;
    ros::Subscriber subscriber_ethernet_alive_;
    ros::Publisher  publisher_ethernet_;
    ros::Publisher  publisher_radarDetections_;
    ros::Subscriber subscriber_instructions_;
    ros::Publisher  publisher_instructions_;
    ros::Publisher  publisher_sensors_;

    struct
    {
        std::string topic_ethernetMeasurementsInput;
        std::string topic_ethernetAliveInput;
        std::string topic_ethernetOutput;
        std::string topic_detectionsOutput;
        std::string topic_sensorsOutput;
        std::string topic_instructionsRequest;
        std::string topic_instructionsResponse;
        std::string frame_sensor;
        double      timestamp_correction;
    }   configuration_;

    std::unique_ptr<SmsDriver> driver_;
};

} // namespace ros1
} // namespace smartmicro
