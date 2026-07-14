#pragma once

#include "ethernet/core/sms.h"

#include <rclcpp/rclcpp.hpp>
#include <ethernet_msgs/msg/packet.hpp>
#include <radar_msgs/msg/detection_record.hpp>
#include <smartmicro_driver/msg/instructions.hpp>
#include <smartmicro_driver/msg/sensors.hpp>

#include <memory>
#include <string>

namespace smartmicro
{
namespace ros2
{

// ROS 2 wrapper for the SMS ethernet driver: maps ethernet_msgs / radar_msgs /
// smartmicro_driver messages onto the ROS-agnostic SmsDriver core.
class Node : public rclcpp::Node, public SmsSink
{
public:
    Node();

    // SmsSink
    void onDetections(const DetectionRecord& detections) override;
    void onInstructions(const Instructions& instructions) override;
    void onSensors(const Sensors& sensors) override;
    void onEthernetPacket(const EthernetPacket& packet) override;

private:
    void onMeasurements(const ethernet_msgs::msg::Packet::SharedPtr msg);
    void onAlive(const ethernet_msgs::msg::Packet::SharedPtr msg);
    void onInstructionsRequest(const smartmicro_driver::msg::Instructions::SharedPtr msg);

    rclcpp::Subscription<ethernet_msgs::msg::Packet>::SharedPtr sub_measurements_;
    rclcpp::Subscription<ethernet_msgs::msg::Packet>::SharedPtr sub_alive_;
    rclcpp::Subscription<smartmicro_driver::msg::Instructions>::SharedPtr sub_instructions_;
    rclcpp::Publisher<ethernet_msgs::msg::Packet>::SharedPtr pub_ethernet_;
    rclcpp::Publisher<radar_msgs::msg::DetectionRecord>::SharedPtr pub_detections_;
    rclcpp::Publisher<smartmicro_driver::msg::Instructions>::SharedPtr pub_instructions_;
    rclcpp::Publisher<smartmicro_driver::msg::Sensors>::SharedPtr pub_sensors_;

    std::unique_ptr<SmsDriver> driver_;
};

} // namespace ros2
} // namespace smartmicro
