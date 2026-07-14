#include "ros1/node.h"

#include <visualization_msgs/Marker.h>

namespace radar_marker
{
namespace ros1
{

namespace
{

LogFn makeLogger()
{
    return [](LogLevel level, const std::string& msg)
    {
        switch (level)
        {
            case LogLevel::Info:  ROS_INFO ("%s", msg.c_str()); break;
            case LogLevel::Warn:  ROS_WARN_THROTTLE(1000, "%s", msg.c_str()); break;
            case LogLevel::Error: ROS_ERROR("%s", msg.c_str()); break;
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

MarkerMeas cvtMeas(const radar_msgs::Measurement& m)
{
    MarkerMeas out;
    out.available = m.available;
    out.value = m.value;
    return out;
}

MarkerInput toInput(const radar_msgs::DetectionRecord& msg)
{
    MarkerInput in;
    in.stamp_ns = msg.header.stamp.toNSec();
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

visualization_msgs::Marker toMarker(const MarkerPrimitive& p)
{
    visualization_msgs::Marker m;
    m.header.frame_id = p.frame_id;
    m.header.stamp = ros::Time().fromNSec(static_cast<uint64_t>(p.stamp_ns));
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
        geometry_msgs::Point gp;
        gp.x = pt.x;
        gp.y = pt.y;
        gp.z = pt.z;
        m.points.push_back(gp);
    }
    m.colors.reserve(p.colors.size());
    for (const auto& c : p.colors)
    {
        std_msgs::ColorRGBA gc;
        gc.r = c.r;
        gc.g = c.g;
        gc.b = c.b;
        gc.a = c.a;
        m.colors.push_back(gc);
    }
    return m;
}

} // namespace

Node::Node(ros::NodeHandle& node_handle) : ros_handle_(node_handle), tfListener_(tfBuffer_)
{
    node_handle.param<std::string>("topic_detectionsInput", topic_detectionsInput_, "detections");
    node_handle.param<std::string>("topic_odometryInput", topic_odometryInput_, "odometry");
    node_handle.param<std::string>("topic_markerOutput", topic_markerOutput_, "marker");
    node_handle.param<std::string>("frame_base", config_.frame_base, "base_link");
    node_handle.param<bool>("config_showMetaInformation", config_.showMetaInformation, false);
    node_handle.param<bool>("config_showAbsoluteDoppler", config_.showAbsoluteDoppler, false);

    subscriber_radarDetections_ = ros_handle_.subscribe(topic_detectionsInput_, 100, &Node::rosCallback_radarDetections, this);
    subscriber_odometry_ = ros_handle_.subscribe(topic_odometryInput_, 100, &Node::rosCallback_odometry, this);
    publisher_marker_ = ros_handle_.advertise<visualization_msgs::Marker>(topic_markerOutput_, 100);

    dynamics_service_.setConfiguration(200);
}

void Node::rosCallback_radarDetections(const radar_msgs::DetectionRecord::ConstPtr& msg)
{
    MarkerInput input = toInput(*msg);

    SensorTransform xform;
    Dynamics dyn;
    const SensorTransform* xform_ptr = nullptr;
    const Dynamics* dyn_ptr = nullptr;

    if (config_.showAbsoluteDoppler)
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer_.lookupTransform("base_link", msg->header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException& ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        DataFitting::ComputationStatus status;
        DynamicsType dynamics = dynamics_service_.getValueAtTime(msg->header.stamp.toSec(), &status);
        if (status != DataFitting::ComputationStatus::interpolation)
        {
            bool ok;
            auto newestSample = dynamics_service_.getNewestSample(&ok);
            if (ok)
                ROS_WARN("Odom interpolation issue. Odom available: %s, latest odom sample time: %lf, current measurement time: %lf", statusToString(status).data(), newestSample.time, msg->header.stamp.toSec());
            else
            {
                ROS_WARN("Odom interpolation issue. Odom available: %s, latest odom sample time: n/a, current measurement time: %lf. Discarding measurement.", statusToString(status).data(), msg->header.stamp.toSec());
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

    const LogFn log = makeLogger();
    for (const MarkerPrimitive& prim : buildMarkers(input, config_, xform_ptr, dyn_ptr, log))
        publisher_marker_.publish(toMarker(prim));
}

void Node::rosCallback_odometry(const nav_msgs::Odometry::ConstPtr& msg)
{
    dynamics_service_.registerValue(DynamicsType({msg->twist.twist.linear.x, msg->twist.twist.angular.z}), msg->header.stamp.toSec());
}

} // namespace ros1
} // namespace radar_marker
