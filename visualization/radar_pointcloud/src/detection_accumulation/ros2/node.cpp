#include "ros2/node.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>

#include <functional>

namespace radar_pointcloud
{
namespace ros2
{

namespace
{

LogFn makeLogger(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
{
    return [logger, clock](LogLevel level, const std::string& msg)
    {
        switch (level)
        {
            case LogLevel::Info:  RCLCPP_INFO (logger, "%s", msg.c_str()); break;
            case LogLevel::Warn:  RCLCPP_WARN_THROTTLE(logger, *clock, 1000000, "%s", msg.c_str()); break;
            case LogLevel::Error: RCLCPP_ERROR(logger, "%s", msg.c_str()); break;
        }
    };
}

std::string statusToString(DataFitting::ComputationStatus status)
{
    switch (status)
    {
        case DataFitting::ComputationStatus::none:          return "none";
        case DataFitting::ComputationStatus::approximated:  return "approximated";
        case DataFitting::ComputationStatus::interpolation: return "interpolation";
        case DataFitting::ComputationStatus::extrapolation: return "extrapolation";
        default:                                            return "invalid";
    }
}

Input toInput(const radar_msgs::msg::DetectionRecord& msg)
{
    Input in;
    in.stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
    in.frame_id = msg.header.frame_id;
    in.detections.reserve(msg.detections.size());
    for (const auto& d : msg.detections)
    {
        Detection nd;
        nd.range = d.range.value;
        nd.azimuth = d.azimuth.value;
        nd.elevation = d.elevation.value;
        nd.radial_speed = d.radial_speed.value;
        in.detections.push_back(nd);
    }
    return in;
}

Transform toTransform(const geometry_msgs::msg::TransformStamped& t)
{
    Transform x;
    x.tx = t.transform.translation.x;
    x.ty = t.transform.translation.y;
    x.tz = t.transform.translation.z;
    x.qw = t.transform.rotation.w;
    x.qx = t.transform.rotation.x;
    x.qy = t.transform.rotation.y;
    x.qz = t.transform.rotation.z;
    return x;
}

sensor_msgs::msg::PointCloud2 toPointCloud2(const PointCloudData& data)
{
    sensor_msgs::msg::PointCloud2 pcl_msg;
    sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
    modifier.setPointCloud2Fields(5,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "absolute_radial_speed", 1, sensor_msgs::msg::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);

    pcl_msg.header.stamp = rclcpp::Time(data.stamp_ns);
    pcl_msg.header.frame_id = data.frame_id;
    pcl_msg.height = 1;
    pcl_msg.width = data.points.size();
    pcl_msg.is_dense = true;
    pcl_msg.point_step = 20;
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.row_step);

    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "absolute_radial_speed");
    sensor_msgs::PointCloud2Iterator<float> iterRgb(pcl_msg, "rgb");

    for (const PclPoint& p : data.points)
    {
        *iterX = p.x;
        *iterY = p.y;
        *iterZ = p.z;
        *iterIntensity = p.absolute_radial_speed;
        *iterRgb = p.rgb;
        ++iterX;
        ++iterY;
        ++iterZ;
        ++iterIntensity;
        ++iterRgb;
    }
    return pcl_msg;
}

} // namespace

Node::Node() : rclcpp::Node("radar_detection_accumulation")
{
    const auto topic_detections   = declare_parameter<std::string>("topic_detectionsInput", "detections");
    const auto topic_odometry     = declare_parameter<std::string>("topic_odometryInput", "odometry");
    const auto topic_accumulation = declare_parameter<std::string>("topic_accumulationOutput", "accumulation");
    config_.frame_base = declare_parameter<std::string>("frame_base", "odom");
    config_.accumulationTime = declare_parameter<double>("config_accumulationTime", config_.accumulationTime);

    log_ = makeLogger(get_logger(), get_clock());
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    accumulator_ = std::make_unique<Accumulator>(config_);
    dynamics_service_.setConfiguration(200);

    // High-rate sensor inputs use best_effort (loss-tolerant, no reliable back-pressure on weak CPUs).
    sub_detections_ = create_subscription<radar_msgs::msg::DetectionRecord>(
        topic_detections, rclcpp::QoS(100).best_effort(), std::bind(&Node::onDetections, this, std::placeholders::_1));
    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
        topic_odometry, rclcpp::QoS(100).best_effort(), std::bind(&Node::onOdometry, this, std::placeholders::_1));
    pub_pointcloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_accumulation, rclcpp::QoS(100));
}

void Node::onDetections(const radar_msgs::msg::DetectionRecord::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer_->lookupTransform("vehicle/rear_axis", msg->header.frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
        return;
    }

    geometry_msgs::msg::TransformStamped transformOdomStamped;
    try
    {
        transformOdomStamped = tfBuffer_->lookupTransform(config_.frame_base, msg->header.frame_id, tf2::TimePointZero);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
        return;
    }

    DataFitting::ComputationStatus status;
    const double meas_time = rclcpp::Time(msg->header.stamp).seconds();
    DynamicsType dynamics = dynamics_service_.getValueAtTime(meas_time, &status);
    if (status != DataFitting::ComputationStatus::interpolation)
    {
        bool ok;
        auto newestSample = dynamics_service_.getNewestSample(&ok);
        if (ok)
            RCLCPP_WARN(get_logger(), "Odom interpolation issue. Odom available: %s, latest odom sample time: %lf, current measurement time: %lf", statusToString(status).data(), newestSample.time, meas_time);
        else
        {
            RCLCPP_WARN(get_logger(), "Odom interpolation issue. Odom available: %s, latest odom sample time: n/a, current measurement time: %lf. Discarding measurement.", statusToString(status).data(), meas_time);
            return;
        }
    }

    Dynamics dyn;
    dyn.speed = dynamics.get(DynamicsIndex::speed);
    dyn.yawrate = dynamics.get(DynamicsIndex::yawrate);

    PointCloudData cloud = accumulator_->ingest(toInput(*msg), toTransform(transformStamped),
                                                toTransform(transformOdomStamped), dyn, log_);
    pub_pointcloud_->publish(toPointCloud2(cloud));
}

void Node::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    dynamics_service_.registerValue(DynamicsType({msg->twist.twist.linear.x, msg->twist.twist.angular.z}),
                                    rclcpp::Time(msg->header.stamp).seconds());
}

} // namespace ros2
} // namespace radar_pointcloud
