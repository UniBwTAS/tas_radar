// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/detection_record.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <DataFitting/Service.h>

namespace radar_marker {

class Converter : public rclcpp::Node
{
    enum DynamicsIndex
    {
        speed,
        yawrate
    };
    typedef DataFitting::Type<DynamicsIndex,2> DynamicsType;

public:
    explicit Converter(const std::string& name);


private:

    // ROS Interfaces
    rclcpp::Subscription<radar_msgs::msg::DetectionRecord>::SharedPtr subscriber_radarDetections_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odometry_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_marker_;

private:
    // ROS data reception callbacks
    void rosCallback_radarDetections(const radar_msgs::msg::DetectionRecord::ConstSharedPtr &msg);
    void rosCallback_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);

private:
    // configuration
    struct
    {
        bool config_showMetaInformation{false};
        bool config_showAbsoluteDoppler{true};
    }   configuration_;

    // helper
    void HSVtoRGB(float &fR, float &fG, float &fB, float &fH, float &fS, float &fV);

    // Data interpolation and buffering
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    DataFitting::Service<DynamicsType> dynamics_service_;
};
}
