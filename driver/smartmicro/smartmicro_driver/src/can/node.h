// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#pragma once

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include "dbcc/umrr.h"

class Node
{
public:
    Node(ros::NodeHandle& node_handle);
    ~Node();


private:
    // Reference to ROS Handle
    ros::NodeHandle& ros_handle_;

    // ROS Interfaces
    ros::Subscriber subscriber_can_;
    ros::Publisher 	publisher_radarDetections_;

private:
    // ROS data reception callbacks
    void rosCallback_can(const can_msgs::Frame::ConstPtr &msg);

private:
    // configuration
    struct
    {
        // ROS topics
        std::string topic_canInput;
        std::string topic_detectionsOutput;
        std::string frame_radar;
    }   configuration_;

    // data buffer
    struct DetectionBuffer
    {
        bool available_mode0{false};
        bool available_mode1{false};
        can_obj_umrr_target_h_t data;
    };

    struct SetupBuffer
    {
        bool available{false};
        ros::Time timestamp;
        can_obj_umrr_header_h_t data;
        uint8_t nDetections; // number of detections, cached
    };

    struct
    {
        SetupBuffer setup;
        std::array<DetectionBuffer, 256> detection;

    }   buffer_;

private:
    void flush();
    void transmitMeasurementIfAvailable();
};
