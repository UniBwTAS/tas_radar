#pragma once

#include "can/core/can.h"

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <radar_msgs/msg/detection_record.hpp>

#include <memory>
#include <string>

namespace smartmicro
{
namespace ros2
{

// ROS 2 wrapper for the UMRR CAN driver: maps can_msgs / radar_msgs onto the
// ROS-agnostic CanDriver core.
class CanNode : public rclcpp::Node, public CanSink
{
public:
    CanNode();

    void onDetections(const DetectionRecord& detections) override;

private:
    void onFrame(const can_msgs::msg::Frame::SharedPtr msg);

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
    rclcpp::Publisher<radar_msgs::msg::DetectionRecord>::SharedPtr pub_detections_;

    std::unique_ptr<CanDriver> driver_;
};

} // namespace ros2
} // namespace smartmicro
