// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#pragma once

#include <ros/ros.h>
#include <radar_msgs/DetectionRecord.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <DataFitting/Service.h>

class Node
{
    enum DynamicsIndex
    {
        speed,
        yawrate
    };
    typedef DataFitting::Type<DynamicsIndex,2> DynamicsType;

public:
    Node(ros::NodeHandle& node_handle);
    ~Node();


private:
    // Reference to ROS Handle
    ros::NodeHandle& ros_handle_;

    // ROS Interfaces
    ros::Subscriber subscriber_radarDetections_;
    ros::Subscriber subscriber_odometry_;
    ros::Publisher 	publisher_marker_;

private:
    // ROS data reception callbacks
    void rosCallback_radarDetections(const radar_msgs::DetectionRecord::ConstPtr &msg);
    void rosCallback_odometry(const nav_msgs::Odometry::ConstPtr &msg);

private:
    // configuration
    struct
    {
        // ROS topics
        std::string topic_detectionsInput;
        std::string topic_odometryInput;
        std::string topic_markerOutput;
        std::string frame_base;
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
