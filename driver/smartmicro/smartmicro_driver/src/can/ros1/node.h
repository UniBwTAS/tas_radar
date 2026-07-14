#pragma once

#include "can/core/can.h"

#include <ros/ros.h>
#include <can_msgs/Frame.h>

#include <memory>
#include <string>

namespace smartmicro
{
namespace ros1
{

// ROS 1 wrapper for the UMRR CAN driver: maps can_msgs / radar_msgs onto the
// ROS-agnostic CanDriver core.
class CanNode : public CanSink
{
public:
    explicit CanNode(ros::NodeHandle& node_handle);

    void onDetections(const DetectionRecord& detections) override;

private:
    void rosCallback_can(const can_msgs::Frame::ConstPtr& msg);

    ros::NodeHandle& ros_handle_;
    ros::Subscriber  subscriber_can_;
    ros::Publisher   publisher_radarDetections_;

    struct
    {
        std::string topic_canInput;
        std::string topic_detectionsOutput;
        std::string frame_radar;
    }   configuration_;

    std::unique_ptr<CanDriver> driver_;
};

} // namespace ros1
} // namespace smartmicro
