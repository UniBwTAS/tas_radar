#pragma once

#include <ros/ros.h>
#include <ethernet_msgs/Packet.h>
#include <smartmicro_driver/Instructions.h>
#include <smartmicro_driver/Sensors.h>
#include <map>

class Node
{
public:
    Node(ros::NodeHandle& node_handle);
    ~Node();


private:
    // Reference to ROS Handle
    ros::NodeHandle& ros_handle_;

    // ROS Interfaces
    ros::Subscriber subscriber_ethernet_measurements_;
    ros::Subscriber subscriber_ethernet_alive_;
    ros::Publisher  publisher_ethernet_;
    ros::Publisher 	publisher_radarDetections_;
    ros::Subscriber subscriber_instructions_;
    ros::Publisher  publisher_instructions_;
    ros::Publisher  publisher_sensors_;

private:
    // ROS data reception callbacks
    void rosCallback_ethernetMeasurements(const ethernet_msgs::Packet::ConstPtr &msg);
    void rosCallback_ethernetAlive(const ethernet_msgs::Packet::ConstPtr &msg);
    void rosCallback_instructions(const smartmicro_driver::Instructions::ConstPtr &msg);

private:
    // configuration
    struct
    {
        // ROS topics
        std::string topic_ethernetMeasurementsInput;
        std::string topic_ethernetAliveInput;
        std::string topic_ethernetOutput;
        std::string topic_detectionsOutput;
        std::string topic_sensorsOutput;
        std::string topic_instructionsRequest;
        std::string topic_instructionsResponse;
        std::string frame_sensor;
        double      timestamp_correction;
    }   configuration_;

private:
    // type definitions for SMS protocol entities
    struct PacketMeta
    {
        ros::Time time;
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

    struct SensorProfile
    {
        uint8_t radar_type{smartmicro_driver::Sensor::RADAR_TYPE_UNKNOWN};
        bool alive_seen{false};
        uint16_t target_list_port_version_major{0};
        uint16_t target_list_port_version_minor{0};
    };
    std::map<uint32_t, SensorProfile> sensor_profiles_;
    bool updateSensorProfileFromAlive(uint32_t ip);
    bool updateSensorProfileFromTargetList(uint32_t ip, uint16_t version_major, uint16_t version_minor);
    void publishSensorProfiles(ros::Time const& stamp) const;
    uint8_t getSensorRadarType(uint32_t ip) const;

private:
    // processing functions for "SMS Transport Packages"
    void deserialize_smsTransport(PacketMeta const& meta, std::vector<uint8_t> const& data);
    void process_smsTransport(SmsTransport const& package);
    void serialize_smsTransport(PacketMeta const& meta, uint8_t applicationProtocolType, std::vector<uint8_t> const& data);

    // processing functions for "SMS Ports"
    void deserialize_smsPort(PacketMeta const& meta, std::vector<uint8_t> const& data);
    void process_smsPort(SmsPort const& port);
    void serialize_smsPort(PacketMeta const& meta, uint64_t timestamp, uint32_t identifier, std::vector<uint8_t> const& data, bool use_drvegrd_port_format = false);

    // processing functions for "Target Lists" in "SMS Ports"
    void deserialize_smsTargetList(PacketMeta const& meta, SmsPortHeader const& header, std::vector<uint8_t> const& data);

    // processing functions for "Instructions" in "SMS Ports"
    void deserialize_smsInstructions(PacketMeta const& meta, SmsPortHeader const& header, std::vector<uint8_t> const& data);
    void serialize_smsInstructions(PacketMeta const& meta, const std::vector<smartmicro_driver::Instruction>& instructions);

private:
    // converters and helpers
    static uint8_t readUint8(std::vector<uint8_t> const& data, unsigned long offset);
    static uint16_t readUint16(std::vector<uint8_t> const& data, unsigned long offset);
    static uint32_t readUint32(std::vector<uint8_t> const& data, unsigned long offset);
    static uint64_t readUint64(std::vector<uint8_t> const& data, unsigned long offset);
    static float readFloat32(std::vector<uint8_t> const& data, unsigned long offset);
    static uint16_t readUint16Body(std::vector<uint8_t> const& data, unsigned long offset, uint8_t endianess);
    static uint32_t readUint32Body(std::vector<uint8_t> const& data, unsigned long offset, uint8_t endianess);
    static uint64_t readUint64Body(std::vector<uint8_t> const& data, unsigned long offset, uint8_t endianess);
    static float readFloat32Body(std::vector<uint8_t> const& data, unsigned long offset, uint8_t endianess);
    static uint16_t crc16(std::vector<uint8_t> const& data, int start, int length);
    static void writeUint8(std::vector<uint8_t> & data, unsigned long offset, uint8_t value);
    static void writeUint16(std::vector<uint8_t> & data, unsigned long offset, uint16_t value);
    static void writeUint32(std::vector<uint8_t> & data, unsigned long offset, uint32_t value);
    static void writeUint64(std::vector<uint8_t> & data, unsigned long offset, uint64_t value);
    static void writeUint16Body(std::vector<uint8_t> & data, unsigned long offset, uint16_t value, uint8_t endianess);
    static void writeUint32Body(std::vector<uint8_t> & data, unsigned long offset, uint32_t value, uint8_t endianess);
};
