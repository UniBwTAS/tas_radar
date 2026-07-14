#pragma once

#include "common/detections.h"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace smartmicro
{

// Parsed results are delivered to a sink implemented by the ROS 1 / ROS 2 wrapper.
class SmsSink
{
public:
    virtual ~SmsSink() = default;
    virtual void onDetections(const DetectionRecord& detections) = 0;
    virtual void onInstructions(const Instructions& instructions) = 0;
    virtual void onSensors(const Sensors& sensors) = 0;
    virtual void onEthernetPacket(const EthernetPacket& packet) = 0;  // outbound to sensor
};

struct SmsConfig
{
    std::string frame_sensor{"sensor/radar/umrr"};
    double      timestamp_correction{0.0};   // [s], added to sensor timestamps
};

// ROS-agnostic Smartmicro Systems (SMS) ethernet protocol driver: raw datagram
// payloads in, neutral detection / instruction / sensor structs out via the sink;
// outbound sensor requests are emitted as EthernetPacket via the sink.
class SmsDriver
{
public:
    SmsDriver(SmsConfig config, SmsSink& sink, LogFn log);

    // inbound (bus_to_host)
    void ingestMeasurements(int64_t stamp_ns, uint32_t sender_ip, const std::vector<uint8_t>& payload);
    void ingestAlive(int64_t stamp_ns, uint32_t sender_ip);

    // outbound request (from instructions/request)
    void requestInstructions(int64_t stamp_ns, uint8_t destination, const std::vector<Instruction>& instructions);

private:
    // --- SMS protocol entities ---
    struct PacketMeta
    {
        int64_t  time_ns{0};
        uint32_t ip{0};
    };

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

    // --- "SMS Transport Packages" assembler state ---
    void flush();
    std::vector<SmsTransport> pool;

    struct SensorProfile
    {
        uint8_t  radar_type{RADAR_TYPE_UNKNOWN};
        bool     alive_seen{false};
        uint16_t target_list_port_version_major{0};
        uint16_t target_list_port_version_minor{0};
    };
    std::map<uint32_t, SensorProfile> sensor_profiles_;
    bool updateSensorProfileFromAlive(uint32_t ip);
    bool updateSensorProfileFromTargetList(uint32_t ip, uint16_t version_major, uint16_t version_minor);
    void publishSensorProfiles(int64_t stamp_ns) const;
    uint8_t getSensorRadarType(uint32_t ip) const;

    // --- processing functions for "SMS Transport Packages" ---
    void deserialize_smsTransport(PacketMeta const& meta, std::vector<uint8_t> const& data);
    void process_smsTransport(SmsTransport const& package);
    void serialize_smsTransport(PacketMeta const& meta, uint8_t applicationProtocolType, std::vector<uint8_t> const& data);

    // --- processing functions for "SMS Ports" ---
    void deserialize_smsPort(PacketMeta const& meta, std::vector<uint8_t> const& data);
    void process_smsPort(SmsPort const& port);
    void serialize_smsPort(PacketMeta const& meta, uint64_t timestamp, uint32_t identifier, std::vector<uint8_t> const& data, bool use_drvegrd_port_format = false);

    // --- "Target Lists" in "SMS Ports" ---
    void deserialize_smsTargetList(PacketMeta const& meta, SmsPortHeader const& header, std::vector<uint8_t> const& data);

    // --- "Instructions" in "SMS Ports" ---
    void deserialize_smsInstructions(PacketMeta const& meta, SmsPortHeader const& header, std::vector<uint8_t> const& data);
    void serialize_smsInstructions(PacketMeta const& meta, const std::vector<Instruction>& instructions);

    // --- config / io ---
    SmsConfig config_;
    SmsSink&  sink_;
    LogFn     log_;
    void logf(LogLevel level, const char* fmt, ...) const;
};

} // namespace smartmicro
