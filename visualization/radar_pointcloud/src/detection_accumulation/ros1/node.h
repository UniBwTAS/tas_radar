#pragma once

#include "core/accumulate.h"

#include <ros/ros.h>
#include <radar_msgs/DetectionRecord.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <DataFitting/Service.h>

#include <memory>
#include <string>

namespace radar_pointcloud
{
namespace ros1
{

class Node
{
    enum DynamicsIndex { speed, yawrate };
    typedef DataFitting::Type<DynamicsIndex, 2> DynamicsType;

public:
    explicit Node(ros::NodeHandle& node_handle);

private:
    void rosCallback_radarDetections(const radar_msgs::DetectionRecord::ConstPtr& msg);
    void rosCallback_odometry(const nav_msgs::Odometry::ConstPtr& msg);

    ros::NodeHandle& ros_handle_;
    ros::Subscriber  subscriber_radarDetections_;
    ros::Subscriber  subscriber_odometry_;
    ros::Publisher   publisher_pointcloud_;

    std::string topic_detectionsInput_;
    std::string topic_odometryInput_;
    std::string topic_accumulationOutput_;
    Config      config_;

    tf2_ros::Buffer            tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    DataFitting::Service<DynamicsType> dynamics_service_;
    std::unique_ptr<Accumulator> accumulator_;
};

} // namespace ros1
} // namespace radar_pointcloud
