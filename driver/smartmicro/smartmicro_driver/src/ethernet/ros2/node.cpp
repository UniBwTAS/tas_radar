#include "ethernet/ros2/node.h"

#include <ethernet_msgs/utils.h>

#include <functional>

namespace smartmicro
{
namespace ros2
{

namespace
{

LogFn makeLogger(rclcpp::Logger logger)
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

Node::Node() : rclcpp::Node("smartmicro_driver_ethernet")
{
    const auto topic_measurements = declare_parameter<std::string>("topic_ethernetMeasurementsInput", "ethernet_measurements/bus_to_host");
    const auto topic_alive        = declare_parameter<std::string>("topic_ethernetAliveInput", "ethernet_alive/bus_to_host");
    const auto topic_ethernet_out = declare_parameter<std::string>("topic_ethernetOutput", "ethernet/host_to_bus");
    const auto topic_detections   = declare_parameter<std::string>("topic_detectionsOutput", "detections");
    const auto topic_sensors      = declare_parameter<std::string>("topic_sensorsOutput", "sensors");
    const auto topic_instr_req    = declare_parameter<std::string>("topic_instructionsRequest", "instructions/request");
    const auto topic_instr_res    = declare_parameter<std::string>("topic_instructionsResponse", "instructions/response");

    SmsConfig config;
    config.frame_sensor = declare_parameter<std::string>("frame_sensor", "sensor/radar/umrr");
    config.timestamp_correction = declare_parameter<double>("timestamp_correction", 0.0);
    driver_.reset(new SmsDriver(config, *this, makeLogger(get_logger())));

    // Inbound sensor-bus data uses best_effort: high-rate and loss-tolerant (an incomplete
    // shot is dropped by the reassembly logic anyway), and it avoids reliable back-pressure
    // on weak CPUs. Instructions are low-rate commands and stay reliable.
    sub_measurements_ = create_subscription<ethernet_msgs::msg::Packet>(
        topic_measurements, rclcpp::QoS(100).best_effort(), std::bind(&Node::onMeasurements, this, std::placeholders::_1));
    sub_alive_ = create_subscription<ethernet_msgs::msg::Packet>(
        topic_alive, rclcpp::QoS(100).best_effort(), std::bind(&Node::onAlive, this, std::placeholders::_1));
    sub_instructions_ = create_subscription<smartmicro_driver::msg::Instructions>(
        topic_instr_req, rclcpp::QoS(100), std::bind(&Node::onInstructionsRequest, this, std::placeholders::_1));

    pub_ethernet_ = create_publisher<ethernet_msgs::msg::Packet>(topic_ethernet_out, rclcpp::QoS(100));
    pub_detections_ = create_publisher<radar_msgs::msg::DetectionRecord>(topic_detections, rclcpp::QoS(100));
    pub_instructions_ = create_publisher<smartmicro_driver::msg::Instructions>(topic_instr_res, rclcpp::QoS(100));
    pub_sensors_ = create_publisher<smartmicro_driver::msg::Sensors>(topic_sensors, rclcpp::QoS(1).transient_local());
}

void Node::onMeasurements(const ethernet_msgs::msg::Packet::SharedPtr msg)
{
    driver_->ingestMeasurements(rclcpp::Time(msg->header.stamp).nanoseconds(),
                                ethernet_msgs::nativeIp4ByArray(msg->sender_ip), msg->payload);
}

void Node::onAlive(const ethernet_msgs::msg::Packet::SharedPtr msg)
{
    driver_->ingestAlive(rclcpp::Time(msg->header.stamp).nanoseconds(),
                         ethernet_msgs::nativeIp4ByArray(msg->sender_ip));
}

void Node::onInstructionsRequest(const smartmicro_driver::msg::Instructions::SharedPtr msg)
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
    driver_->requestInstructions(rclcpp::Time(msg->header.stamp).nanoseconds(), msg->destination, instructions);
}

void Node::onDetections(const DetectionRecord& rec)
{
    radar_msgs::msg::DetectionRecord out;
    out.header.stamp = rclcpp::Time(rec.stamp_ns);
    out.header.frame_id = rec.frame_id;
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

void Node::onInstructions(const Instructions& ins)
{
    smartmicro_driver::msg::Instructions out;
    out.header.stamp = rclcpp::Time(ins.stamp_ns);
    out.header.frame_id = ins.frame_id;
    out.destination = ins.destination;
    out.instructions.reserve(ins.instructions.size());
    for (const auto& in : ins.instructions)
    {
        smartmicro_driver::msg::Instruction oi;
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
    pub_instructions_->publish(out);
}

void Node::onSensors(const Sensors& s)
{
    smartmicro_driver::msg::Sensors out;
    out.header.stamp = rclcpp::Time(s.stamp_ns);
    out.header.frame_id = s.frame_id;
    out.sensors.reserve(s.sensors.size());
    for (const auto& sen : s.sensors)
    {
        smartmicro_driver::msg::Sensor os;
        os.ip = sen.ip;
        os.radar_type = sen.radar_type;
        out.sensors.push_back(os);
    }
    pub_sensors_->publish(out);
}

void Node::onEthernetPacket(const EthernetPacket& p)
{
    ethernet_msgs::msg::Packet out;
    out.header.stamp = rclcpp::Time(p.stamp_ns);
    out.receiver_ip = ethernet_msgs::arrayByNativeIp4(p.receiver_ip);
    out.receiver_port = p.receiver_port;
    out.payload = p.payload;
    pub_ethernet_->publish(out);
}

} // namespace ros2
} // namespace smartmicro
