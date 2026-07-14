#include "can/ros1/node.h"

#include <radar_msgs/DetectionRecord.h>

namespace smartmicro
{
namespace ros1
{

namespace
{

LogFn makeCanLogger()
{
    return [](LogLevel level, const std::string& msg)
    {
        switch (level)
        {
            case LogLevel::Info:  ROS_INFO ("%s", msg.c_str()); break;
            case LogLevel::Warn:  ROS_WARN ("%s", msg.c_str()); break;
            case LogLevel::Error: ROS_ERROR("%s", msg.c_str()); break;
        }
    };
}

radar_msgs::Measurement toMsg(const Measurement& m)
{
    radar_msgs::Measurement out;
    out.available = m.available;
    out.value = m.value;
    out.uncertainty = m.uncertainty;
    return out;
}

} // namespace

CanNode::CanNode(ros::NodeHandle& node_handle) : ros_handle_(node_handle)
{
    node_handle.param<std::string>("topic_canInput", configuration_.topic_canInput, "/can/sms/deviceToHost");
    node_handle.param<std::string>("topic_detectionsOutput", configuration_.topic_detectionsOutput, "/sensor/radar/umrr96/detections");
    node_handle.param<std::string>("frame_radar", configuration_.frame_radar, "sensor/radar/umrr96");

    CanConfig config;
    config.frame_radar = configuration_.frame_radar;
    driver_.reset(new CanDriver(config, *this, makeCanLogger()));

    subscriber_can_ = ros_handle_.subscribe(configuration_.topic_canInput, 100, &CanNode::rosCallback_can, this, ros::TransportHints().tcpNoDelay());
    publisher_radarDetections_ = ros_handle_.advertise<radar_msgs::DetectionRecord>(configuration_.topic_detectionsOutput, 100);
}

void CanNode::rosCallback_can(const can_msgs::Frame::ConstPtr& msg)
{
    driver_->ingestFrame(msg->header.stamp.toNSec(), msg->id, msg->dlc, msg->data.data(), msg->data.size());
}

void CanNode::onDetections(const DetectionRecord& rec)
{
    radar_msgs::DetectionRecord out;
    out.header.stamp = ros::Time().fromNSec(static_cast<uint64_t>(rec.stamp_ns));
    out.header.seq = rec.seq;
    out.header.frame_id = rec.frame_id;
    out.detections.reserve(rec.detections.size());
    for (const auto& d : rec.detections)
    {
        radar_msgs::Detection od;
        od.range = toMsg(d.range);
        od.radial_speed = toMsg(d.radial_speed);
        od.azimuth = toMsg(d.azimuth);
        od.elevation = toMsg(d.elevation);
        od.rcs = toMsg(d.rcs);
        od.power = toMsg(d.power);
        od.noise = toMsg(d.noise);
        od.coverage.value = d.coverage;
        od.existance_probability = toMsg(d.existance_probability);
        out.detections.push_back(od);
    }
    publisher_radarDetections_.publish(out);
}

} // namespace ros1
} // namespace smartmicro
