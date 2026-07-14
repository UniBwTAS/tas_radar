#include "ros2/node.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/time.h>

#include <functional>

namespace radar_marker
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

MarkerMeas cvtMeas(const radar_msgs::msg::Measurement& m)
{
    MarkerMeas out;
    out.available = m.available;
    out.value = m.value;
    return out;
}

MarkerInput toInput(const radar_msgs::msg::DetectionRecord& msg)
{
    MarkerInput in;
    in.stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
    in.frame_id = msg.header.frame_id;
    in.detections.reserve(msg.detections.size());
    for (const auto& d : msg.detections)
    {
        MarkerDetection md;
        md.range = cvtMeas(d.range);
        md.azimuth = cvtMeas(d.azimuth);
        md.elevation = cvtMeas(d.elevation);
        md.radial_speed = cvtMeas(d.radial_speed);
        md.rcs = cvtMeas(d.rcs);
        md.power = cvtMeas(d.power);
        md.noise = cvtMeas(d.noise);
        in.detections.push_back(md);
    }
    return in;
}

visualization_msgs::msg::Marker toMarker(const MarkerPrimitive& p)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = p.frame_id;
    m.header.stamp = rclcpp::Time(p.stamp_ns);
    m.ns = p.ns;
    m.id = p.id;
    m.type = p.type;
    m.action = p.action;
    m.frame_locked = p.frame_locked;
    m.scale.x = p.scale.x;
    m.scale.y = p.scale.y;
    m.scale.z = p.scale.z;
    m.pose.position.x = p.pose_position.x;
    m.pose.position.y = p.pose_position.y;
    m.pose.position.z = p.pose_position.z;
    m.pose.orientation.w = p.pose_orientation_w;
    m.pose.orientation.x = p.pose_orientation_x;
    m.pose.orientation.y = p.pose_orientation_y;
    m.pose.orientation.z = p.pose_orientation_z;
    m.color.r = p.color.r;
    m.color.g = p.color.g;
    m.color.b = p.color.b;
    m.color.a = p.color.a;
    m.text = p.text;
    m.points.reserve(p.points.size());
    for (const auto& pt : p.points)
    {
        geometry_msgs::msg::Point gp;
        gp.x = pt.x;
        gp.y = pt.y;
        gp.z = pt.z;
        m.points.push_back(gp);
    }
    m.colors.reserve(p.colors.size());
    for (const auto& c : p.colors)
    {
        std_msgs::msg::ColorRGBA gc;
        gc.r = c.r;
        gc.g = c.g;
        gc.b = c.b;
        gc.a = c.a;
        m.colors.push_back(gc);
    }
    return m;
}

} // namespace

Node::Node() : rclcpp::Node("radar_marker_absolute")
{
    const auto topic_detections = declare_parameter<std::string>("topic_detectionsInput", "detections");
    const auto topic_odometry   = declare_parameter<std::string>("topic_odometryInput", "odometry");
    const auto topic_marker     = declare_parameter<std::string>("topic_markerOutput", "marker");
    config_.frame_base = declare_parameter<std::string>("frame_base", "base_link");
    config_.showMetaInformation = declare_parameter<bool>("config_showMetaInformation", false);
    config_.showAbsoluteDoppler = declare_parameter<bool>("config_showAbsoluteDoppler", false);

    log_ = makeLogger(get_logger(), get_clock());

    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    dynamics_service_.setConfiguration(200);

    // High-rate sensor inputs use best_effort (loss-tolerant, no reliable back-pressure on weak CPUs).
    sub_detections_ = create_subscription<radar_msgs::msg::DetectionRecord>(
        topic_detections, rclcpp::QoS(100).best_effort(), std::bind(&Node::onDetections, this, std::placeholders::_1));
    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
        topic_odometry, rclcpp::QoS(100).best_effort(), std::bind(&Node::onOdometry, this, std::placeholders::_1));
    pub_marker_ = create_publisher<visualization_msgs::msg::Marker>(topic_marker, rclcpp::QoS(100));
}

void Node::onDetections(const radar_msgs::msg::DetectionRecord::SharedPtr msg)
{
    MarkerInput input = toInput(*msg);

    SensorTransform xform;
    Dynamics dyn;
    const SensorTransform* xform_ptr = nullptr;
    const Dynamics* dyn_ptr = nullptr;

    if (config_.showAbsoluteDoppler)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer_->lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
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

        xform.tx = transformStamped.transform.translation.x;
        xform.ty = transformStamped.transform.translation.y;
        xform.tz = transformStamped.transform.translation.z;
        xform.qw = transformStamped.transform.rotation.w;
        xform.qx = transformStamped.transform.rotation.x;
        xform.qy = transformStamped.transform.rotation.y;
        xform.qz = transformStamped.transform.rotation.z;
        dyn.speed = dynamics.get(DynamicsIndex::speed);
        dyn.yawrate = dynamics.get(DynamicsIndex::yawrate);
        xform_ptr = &xform;
        dyn_ptr = &dyn;
    }

    for (const MarkerPrimitive& prim : buildMarkers(input, config_, xform_ptr, dyn_ptr, log_))
        pub_marker_->publish(toMarker(prim));
}

void Node::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    dynamics_service_.registerValue(DynamicsType({msg->twist.twist.linear.x, msg->twist.twist.angular.z}),
                                    rclcpp::Time(msg->header.stamp).seconds());
}

} // namespace ros2
} // namespace radar_marker
