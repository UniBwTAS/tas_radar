#include "ethernet/ros1/node.h"

#include <radar_msgs/DetectionRecord.h>
#include <ethernet_msgs/utils.h>

namespace smartmicro
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

Node::Node(ros::NodeHandle& node_handle) : ros_handle_(node_handle)
{
    /// Parameter
    node_handle.param<std::string>("topic_ethernetMeasurementsInput", configuration_.topic_ethernetMeasurementsInput, "ethernet_measurements/bus_to_host");
    node_handle.param<std::string>("topic_ethernetAliveInput", configuration_.topic_ethernetAliveInput, "ethernet_alive/bus_to_host");
    node_handle.param<std::string>("topic_ethernetOutput", configuration_.topic_ethernetOutput, "ethernet/host_to_bus");
    node_handle.param<std::string>("topic_detectionsOutput", configuration_.topic_detectionsOutput, "detections");
    node_handle.param<std::string>("topic_sensorsOutput", configuration_.topic_sensorsOutput, "sensors");

    // legacy topic handling
    if (!node_handle.hasParam("topic_ethernetMeasurementsInput") && node_handle.hasParam("topic_ethernetInput"))
    {
        node_handle.getParam("topic_ethernetInput", configuration_.topic_ethernetMeasurementsInput);
        ROS_WARN("parameter 'topic_ethernetInput' is deprecated. Please use 'topic_ethernetMeasurementsInput'.");
    }

    node_handle.param<std::string>("topic_instructionsRequest", configuration_.topic_instructionsRequest, "instructions/request");
    node_handle.param<std::string>("topic_instructionsResponse", configuration_.topic_instructionsResponse, "instructions/response");
    node_handle.param<std::string>("frame_sensor", configuration_.frame_sensor, "sensor/radar/umrr");
    node_handle.param<double>("timestamp_correction", configuration_.timestamp_correction, 0);

    /// Core driver
    SmsConfig config;
    config.frame_sensor = configuration_.frame_sensor;
    config.timestamp_correction = configuration_.timestamp_correction;
    driver_.reset(new SmsDriver(config, *this, makeLogger()));

    /// Subscribing & Publishing
    subscriber_ethernet_measurements_ = ros_handle_.subscribe(configuration_.topic_ethernetMeasurementsInput, 100, &Node::rosCallback_ethernetMeasurements, this, ros::TransportHints().tcpNoDelay());
    subscriber_ethernet_alive_ = ros_handle_.subscribe(configuration_.topic_ethernetAliveInput, 100, &Node::rosCallback_ethernetAlive, this, ros::TransportHints().tcpNoDelay());
    publisher_ethernet_ = ros_handle_.advertise<ethernet_msgs::Packet>(configuration_.topic_ethernetOutput, 100);
    publisher_radarDetections_ = ros_handle_.advertise<radar_msgs::DetectionRecord>(configuration_.topic_detectionsOutput, 100);
    subscriber_instructions_ = ros_handle_.subscribe(configuration_.topic_instructionsRequest, 100, &Node::rosCallback_instructions, this);
    publisher_instructions_ = ros_handle_.advertise<smartmicro_driver::Instructions>(configuration_.topic_instructionsResponse, 100);
    publisher_sensors_ = ros_handle_.advertise<smartmicro_driver::Sensors>(configuration_.topic_sensorsOutput, 1, true);
}

void Node::rosCallback_ethernetMeasurements(const ethernet_msgs::Packet::ConstPtr& msg)
{
    driver_->ingestMeasurements(msg->header.stamp.toNSec(), ethernet_msgs::nativeIp4ByArray(msg->sender_ip), msg->payload);
}

void Node::rosCallback_ethernetAlive(const ethernet_msgs::Packet::ConstPtr& msg)
{
    driver_->ingestAlive(msg->header.stamp.toNSec(), ethernet_msgs::nativeIp4ByArray(msg->sender_ip));
}

void Node::rosCallback_instructions(const smartmicro_driver::Instructions::ConstPtr& msg)
{
    std::vector<Instruction> instructions;
    instructions.reserve(msg->instructions.size());
    for (const auto& in : msg->instructions)
    {
        Instruction i;
        i.request = in.request;
        i.response = in.response;
        i.section = in.section;
        i.id = in.id;
        i.datatype = in.datatype;
        i.dim_count = in.dim_count;
        i.signature = in.signature;
        i.value = in.value;
        instructions.push_back(i);
    }
    driver_->requestInstructions(msg->header.stamp.toNSec(), msg->destination, instructions);
}

void Node::onDetections(const DetectionRecord& rec)
{
    radar_msgs::DetectionRecord out;
    out.header.stamp = ros::Time().fromNSec(static_cast<uint64_t>(rec.stamp_ns));
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

void Node::onInstructions(const Instructions& ins)
{
    smartmicro_driver::Instructions out;
    out.header.stamp = ros::Time().fromNSec(static_cast<uint64_t>(ins.stamp_ns));
    out.header.frame_id = ins.frame_id;
    out.destination = ins.destination;
    out.instructions.reserve(ins.instructions.size());
    for (const auto& in : ins.instructions)
    {
        smartmicro_driver::Instruction oi;
        oi.request = in.request;
        oi.response = in.response;
        oi.section = in.section;
        oi.id = in.id;
        oi.datatype = in.datatype;
        oi.dim_count = in.dim_count;
        oi.signature = in.signature;
        oi.value = in.value;
        out.instructions.push_back(oi);
    }
    publisher_instructions_.publish(out);
}

void Node::onSensors(const Sensors& s)
{
    smartmicro_driver::Sensors out;
    out.header.stamp = ros::Time().fromNSec(static_cast<uint64_t>(s.stamp_ns));
    out.header.frame_id = s.frame_id;
    out.sensors.reserve(s.sensors.size());
    for (const auto& sen : s.sensors)
    {
        smartmicro_driver::Sensor os;
        os.ip = sen.ip;
        os.radar_type = sen.radar_type;
        out.sensors.push_back(os);
    }
    publisher_sensors_.publish(out);
}

void Node::onEthernetPacket(const EthernetPacket& p)
{
    ethernet_msgs::Packet out;
    out.header.stamp = ros::Time().fromNSec(static_cast<uint64_t>(p.stamp_ns));
    out.receiver_ip = ethernet_msgs::arrayByNativeIp4(p.receiver_ip);
    out.receiver_port = p.receiver_port;
    out.payload = p.payload;
    publisher_ethernet_.publish(out);
}

} // namespace ros1
} // namespace smartmicro
