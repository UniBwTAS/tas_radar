#include "can/ros2/node.h"

#include <functional>

namespace smartmicro
{
namespace ros2
{

namespace
{

LogFn makeCanLogger(rclcpp::Logger logger)
{
    return [logger](LogLevel level, const std::string& msg)
    {
        switch (level)
        {
            case LogLevel::Info:  RCLCPP_INFO (logger, "%s", msg.c_str()); break;
            case LogLevel::Warn:  RCLCPP_WARN (logger, "%s", msg.c_str()); break;
            case LogLevel::Error: RCLCPP_ERROR(logger, "%s", msg.c_str()); break;
        }
    };
}

radar_msgs::msg::Measurement toMsg(const Measurement& m)
{
    radar_msgs::msg::Measurement out;
    out.available = m.available;
    out.value = m.value;
    out.uncertainty = m.uncertainty;
    return out;
}

} // namespace

CanNode::CanNode() : rclcpp::Node("radar_driver_umrr_can")
{
    const auto topic_can        = declare_parameter<std::string>("topic_canInput", "/can/sms/deviceToHost");
    const auto topic_detections = declare_parameter<std::string>("topic_detectionsOutput", "/sensor/radar/umrr96/detections");

    CanConfig config;
    config.frame_radar = declare_parameter<std::string>("frame_radar", "sensor/radar/umrr96");
    driver_.reset(new CanDriver(config, *this, makeCanLogger(get_logger())));

    // Inbound CAN frames use best_effort: high-rate, loss-tolerant, no reliable back-pressure on weak CPUs.
    sub_can_ = create_subscription<can_msgs::msg::Frame>(
        topic_can, rclcpp::QoS(100).best_effort(), std::bind(&CanNode::onFrame, this, std::placeholders::_1));
    pub_detections_ = create_publisher<radar_msgs::msg::DetectionRecord>(topic_detections, rclcpp::QoS(100));
}

void CanNode::onFrame(const can_msgs::msg::Frame::SharedPtr msg)
{
    driver_->ingestFrame(rclcpp::Time(msg->header.stamp).nanoseconds(), msg->id, msg->dlc, msg->data.data(), msg->data.size());
}

void CanNode::onDetections(const DetectionRecord& rec)
{
    radar_msgs::msg::DetectionRecord out;
    out.header.stamp = rclcpp::Time(rec.stamp_ns);
    out.header.frame_id = rec.frame_id;
    // ROS 2 std_msgs/Header has no seq field; the CAN cycle counter (rec.seq) is dropped.
    out.detections.reserve(rec.detections.size());
    for (const auto& d : rec.detections)
    {
        radar_msgs::msg::Detection od;
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
    pub_detections_->publish(out);
}

} // namespace ros2
} // namespace smartmicro
