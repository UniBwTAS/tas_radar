#pragma once

#include "core/marker_core.h"

#include <ros/ros.h>
#include <radar_msgs/DetectionRecord.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <DataFitting/Service.h>

#include <string>

namespace radar_marker
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
    ros::Publisher   publisher_marker_;

    MarkerConfig config_;
    std::string  topic_detectionsInput_;
    std::string  topic_odometryInput_;
    std::string  topic_markerOutput_;

    tf2_ros::Buffer            tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    DataFitting::Service<DynamicsType> dynamics_service_;
};

} // namespace ros1
} // namespace radar_marker
