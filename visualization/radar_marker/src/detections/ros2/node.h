#pragma once

#include "core/marker_core.h"

#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/detection_record.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <DataFitting/Service.h>

#include <memory>
#include <string>

namespace radar_marker
{
namespace ros2
{

class Node : public rclcpp::Node
{
    enum DynamicsIndex { speed, yawrate };
    typedef DataFitting::Type<DynamicsIndex, 2> DynamicsType;

public:
    Node();

private:
    void onDetections(const radar_msgs::msg::DetectionRecord::SharedPtr msg);
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<radar_msgs::msg::DetectionRecord>::SharedPtr sub_detections_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

    MarkerConfig config_;
    LogFn        log_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    DataFitting::Service<DynamicsType> dynamics_service_;
};

} // namespace ros2
} // namespace radar_marker
