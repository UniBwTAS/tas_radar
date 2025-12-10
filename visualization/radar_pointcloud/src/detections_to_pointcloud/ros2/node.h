#pragma once

#include "core/pointcloud_core.h"

#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/detection_record.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <DataFitting/Service.h>

#include <map>
#include <memory>
#include <string>

namespace radar_pointcloud
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
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pub_pointcloud_;

    std::string topic_pointcloudOutput_;
    LogFn log_;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    DataFitting::Service<DynamicsType> dynamics_service_;
    std::unique_ptr<Converter> converter_;
};

} // namespace ros2
} // namespace radar_pointcloud
