#include "ros1/node.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace radar_pointcloud
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

Input toInput(const radar_msgs::DetectionRecord& msg)
{
    Input in;
    in.stamp_ns = msg.header.stamp.toNSec();
    in.frame_id = msg.header.frame_id;
    in.detections.reserve(msg.detections.size());
    for (const auto& d : msg.detections)
    {
        Detection nd;
        nd.range = d.range.value;
        nd.azimuth = d.azimuth.value;
        nd.elevation = d.elevation.value;
        nd.radial_speed = d.radial_speed.value;
        nd.rcs = d.rcs.value;
        nd.power = d.power.value;
        nd.noise = d.noise.value;
        in.detections.push_back(nd);
    }
    return in;
}

Transform toTransform(const geometry_msgs::TransformStamped& t)
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

sensor_msgs::PointCloud2 toPointCloud2(const PointCloudData& data)
{
    sensor_msgs::PointCloud2 pcl_msg;
    sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
    modifier.setPointCloud2Fields(12,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "absolute_radial_speed", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32,
        "range", 1, sensor_msgs::PointField::FLOAT64,
        "azimuth", 1, sensor_msgs::PointField::FLOAT64,
        "elevation", 1, sensor_msgs::PointField::FLOAT64,
        "range_rate", 1, sensor_msgs::PointField::FLOAT64,
        "rcs", 1, sensor_msgs::PointField::FLOAT32,
        "power", 1, sensor_msgs::PointField::FLOAT32,
        "noise", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(data.points.size());

    pcl_msg.header.stamp = ros::Time().fromNSec(static_cast<uint64_t>(data.stamp_ns));
    pcl_msg.header.frame_id = data.frame_id;
    pcl_msg.height = 1;
    pcl_msg.width = data.points.size();
    pcl_msg.is_dense = true;

    sensor_msgs::PointCloud2Iterator<float>  iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float>  iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float>  iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float>  iterIntensity(pcl_msg, "absolute_radial_speed");
    sensor_msgs::PointCloud2Iterator<float>  iterRgb(pcl_msg, "rgb");
    sensor_msgs::PointCloud2Iterator<double> iterRange(pcl_msg, "range");
    sensor_msgs::PointCloud2Iterator<double> iterAzimuth(pcl_msg, "azimuth");
    sensor_msgs::PointCloud2Iterator<double> iterElevation(pcl_msg, "elevation");
    sensor_msgs::PointCloud2Iterator<double> iterRangeRate(pcl_msg, "range_rate");
    sensor_msgs::PointCloud2Iterator<float>  iterRCS(pcl_msg, "rcs");
    sensor_msgs::PointCloud2Iterator<float>  iterPower(pcl_msg, "power");
    sensor_msgs::PointCloud2Iterator<float>  iterNoise(pcl_msg, "noise");

    for (const PclPoint& p : data.points)
    {
        *iterX = p.x;
        *iterY = p.y;
        *iterZ = p.z;
        *iterIntensity = p.absolute_radial_speed;
        *iterRgb = p.rgb;
        *iterRange = p.range;
        *iterAzimuth = p.azimuth;
        *iterElevation = p.elevation;
        *iterRangeRate = p.range_rate;
        *iterRCS = p.rcs;
        *iterPower = p.power;
        *iterNoise = p.noise;
        ++iterX;
        ++iterY;
        ++iterZ;
        ++iterIntensity;
        ++iterRgb;
        ++iterRange;
        ++iterAzimuth;
        ++iterElevation;
        ++iterRangeRate;
        ++iterRCS;
        ++iterPower;
        ++iterNoise;
    }
    return pcl_msg;
}

// Parse the sensor IP from the trailing "/<ip>" of the frame_id.
std::string sensorIp(const std::string& frame_id)
{
    size_t idx = frame_id.find_last_of("/");
    return idx != std::string::npos ? frame_id.substr(idx + 1) : frame_id;
}

} // namespace

Node::Node(ros::NodeHandle& node_handle) : ros_handle_(node_handle), tfListener_(tfBuffer_)
{
    node_handle.param<std::string>("topic_detectionsInput", topic_detectionsInput_, "detections");
    node_handle.param<std::string>("topic_odometryInput", topic_odometryInput_, "odometry");
    node_handle.param<std::string>("topic_pointcloudOutput", topic_pointcloudOutput_, "pointcloud");

    converter_.reset(new Converter());

    subscriber_radarDetections_ = ros_handle_.subscribe(topic_detectionsInput_, 100, &Node::rosCallback_radarDetections, this);
    subscriber_odometry_ = ros_handle_.subscribe(topic_odometryInput_, 100, &Node::rosCallback_odometry, this);

    dynamics_service_.setConfiguration(200);
}

void Node::rosCallback_radarDetections(const radar_msgs::DetectionRecord::ConstPtr& msg)
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer_.lookupTransform("vehicle/rear_axis", msg->header.frame_id, ros::Time(0));
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

    Dynamics dyn;
    dyn.speed = dynamics.get(DynamicsIndex::speed);
    dyn.yawrate = dynamics.get(DynamicsIndex::yawrate);

    PointCloudData cloud = converter_->build(toInput(*msg), toTransform(transformStamped), dyn, makeLogger());

    if (!publisher_pointcloud_.count(msg->header.frame_id))
        publisher_pointcloud_[msg->header.frame_id] =
            ros_handle_.advertise<sensor_msgs::PointCloud2>(topic_pointcloudOutput_ + "/" + sensorIp(msg->header.frame_id), 100);
    publisher_pointcloud_[msg->header.frame_id].publish(toPointCloud2(cloud));
}

void Node::rosCallback_odometry(const nav_msgs::Odometry::ConstPtr& msg)
{
    dynamics_service_.registerValue(DynamicsType({msg->twist.twist.linear.x, msg->twist.twist.angular.z}), msg->header.stamp.toSec());
}

} // namespace ros1
} // namespace radar_pointcloud
