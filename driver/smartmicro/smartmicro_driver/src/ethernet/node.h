#pragma once

#include <ethernet_msgs/msg/packet.hpp>
#include <radar_msgs/msg/detection_record.hpp>
#include <smartmicro_driver/msg/instructions.hpp>
#include <rclcpp/rclcpp.hpp>

namespace smartmicro_driver
{
class Driver : public rclcpp::Node
{
  public:
    explicit Driver(const std::string& name);


private:
    // ROS Interfaces
    rclcpp::Subscription<ethernet_msgs::msg::Packet>::SharedPtr subscriber_ethernet_;
    rclcpp::Publisher<ethernet_msgs::msg::Packet>::SharedPtr publisher_ethernet_;
    rclcpp::Publisher<radar_msgs::msg::DetectionRecord>::SharedPtr publisher_radarDetections_;
    rclcpp::Subscription<smartmicro_driver::msg::Instructions>::SharedPtr subscriber_instructions_;
    rclcpp::Publisher<smartmicro_driver::msg::Instructions>::SharedPtr publisher_instructions_;

private:
    // ROS data reception callbacks
    void rosCallback_ethernet(const ethernet_msgs::msg::Packet::ConstSharedPtr &msg);
    void rosCallback_instructions(const smartmicro_driver::msg::Instructions::ConstSharedPtr &msg);

private:
    // configuration
    struct
    {
        // ROS topics
        std::string frame_sensor;
        double      timestamp_correction;
    }   configuration_;

private:
    // type definitions for SMS protocol entities
    struct PacketMeta
    {
        rclcpp::Time time;
        uint32_t ip;
    };

    // type definitions for "SMS Transport Packages"
    template<typename T>
    struct ConditionalData
    {
        bool available{false};
        T data;
    };

    struct SmsTransportHeader
    {
        uint8_t applicationProtocolType;
        ConditionalData<uint16_t> messageCounter;
        ConditionalData<uint32_t> sourceClientId;
        ConditionalData<uint16_t> dataIdentifier;
        ConditionalData<uint16_t> segmentation;
    };

    struct SmsTransport
    {
        PacketMeta meta;
        SmsTransportHeader header;
        std::vector<uint8_t> applicationData;
    };

    // type definitions for "SMS Ports"
    struct SmsPortHeader
    {
        uint32_t identifier;
        uint16_t version_portMajor;
        uint16_t version_portMinor;
        uint64_t timestamp;
        uint32_t size;
        uint8_t  endianess;
        uint8_t  index;
        uint8_t  version_headerMajor;
        uint8_t  version_headerMinor;
    };

    struct SmsPort
    {
        PacketMeta meta;
        SmsPortHeader header;
        std::vector<uint8_t> data;
    };

private:
    // "SMS Transport Packages" assembler functionality
    void flush();
    std::vector<SmsTransport> pool;

private:
    // processing functions for "SMS Transport Packages"
    void deserialize_smsTransport(PacketMeta const& meta, std::vector<uint8_t> const& data);
    void process_smsTransport(SmsTransport const& package);
    void serialize_smsTransport(PacketMeta const& meta, uint8_t applicationProtocolType, std::vector<uint8_t> const& data);

    // processing functions for "SMS Ports"
    void deserialize_smsPort(PacketMeta const& meta, std::vector<uint8_t> const& data);
    void process_smsPort(SmsPort const& port);
    void serialize_smsPort(PacketMeta const& meta, uint64_t timestamp, uint32_t identifier, std::vector<uint8_t> const& data);

    // processing functions for "Target Lists" in "SMS Ports"
    void deserialize_smsTargetList(PacketMeta const& meta, SmsPortHeader const& header, std::vector<uint8_t> const& data);

    // processing functions for "Instructions" in "SMS Ports"
    void deserialize_smsInstructions(PacketMeta const& meta, SmsPortHeader const& header, std::vector<uint8_t> const& data);
    void serialize_smsInstructions(PacketMeta const& meta, const std::vector<smartmicro_driver::msg::Instruction>& instructions);

private:
    // converters and helpers
    static uint8_t readUint8(std::vector<uint8_t> const& data, unsigned long offset);
    static uint16_t readUint16(std::vector<uint8_t> const& data, unsigned long offset);
    static uint32_t readUint32(std::vector<uint8_t> const& data, unsigned long offset);
    static uint64_t readUint64(std::vector<uint8_t> const& data, unsigned long offset);
    static float readFloat32(std::vector<uint8_t> const& data, unsigned long offset);
    static uint16_t crc16(std::vector<uint8_t> const& data, int start, int length);
    static void writeUint8(std::vector<uint8_t> & data, unsigned long offset, uint8_t value);
    static void writeUint16(std::vector<uint8_t> & data, unsigned long offset, uint16_t value);
    static void writeUint32(std::vector<uint8_t> & data, unsigned long offset, uint32_t value);
    static void writeUint64(std::vector<uint8_t> & data, unsigned long offset, uint64_t value);
};
} // namespace smartmicro_driver
