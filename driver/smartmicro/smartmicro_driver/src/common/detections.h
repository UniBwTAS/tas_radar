#pragma once

// ROS-agnostic mirror of the radar_msgs / smartmicro_driver message payloads and
// their enum constants. The parser core produces these plain structs; the ROS 1
// and ROS 2 wrappers map them 1:1 onto the generated message types (the wrappers
// static_assert that these values match the .msg constants).

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace smartmicro
{

// --- shared logging plumbing (used by the SMS and CAN driver cores) ---
enum class LogLevel { Info, Warn, Error };
using LogFn = std::function<void(LogLevel, const std::string&)>;

// radar_msgs/Measurement.available
enum : uint8_t
{
    AVAILABLE_NOTHING = 0,
    AVAILABLE_VALUE = 1,
    AVAILABLE_VALUE_AND_UNCERTAINTY = 2,
};

// radar_msgs/Coverage.value
enum : uint8_t
{
    COVERAGE_UNKNOWN = 0,
    COVERAGE_SHORTRANGE = 1,
    COVERAGE_MIDRANGE = 2,
    COVERAGE_LONGRANGE = 3,
};

// smartmicro_driver/Sensor.radar_type
enum : uint8_t
{
    RADAR_TYPE_UNKNOWN = 0,
    RADAR_TYPE_UMRR = 1,
    RADAR_TYPE_DRVEGRD = 2,
};

// smartmicro_driver/Instruction.datatype
enum : uint8_t
{
    DATATYPE_I8 = 1,
    DATATYPE_U8 = 2,
    DATATYPE_I16 = 3,
    DATATYPE_U16 = 4,
    DATATYPE_I32 = 5,
    DATATYPE_U32 = 6,
    DATATYPE_F32 = 7,
};

struct Measurement
{
    uint8_t available{AVAILABLE_NOTHING};
    double  value{0.0};
    float   uncertainty{0.0f};
};

struct Detection
{
    Measurement range;
    Measurement radial_speed;
    Measurement azimuth;
    Measurement elevation;
    Measurement rcs;
    Measurement power;
    Measurement noise;
    uint8_t     coverage{COVERAGE_UNKNOWN};
    Measurement existance_probability;
};

struct DetectionRecord
{
    int64_t                stamp_ns{0};
    uint32_t               seq{0};   // ROS 1 Header.seq (CAN cycle counter); no-op under ROS 2
    std::string            frame_id;
    std::vector<Detection> detections;
};

struct Instruction
{
    uint8_t  request{0};
    uint8_t  response{0};
    uint16_t section{0};
    uint16_t id{0};
    uint8_t  datatype{0};
    uint8_t  dim_count{0};
    uint32_t signature{0};
    double   value{0.0};
};

struct Instructions
{
    int64_t                  stamp_ns{0};
    std::string              frame_id;
    uint8_t                  destination{0};
    std::vector<Instruction> instructions;
};

struct Sensor
{
    uint8_t ip{0};
    uint8_t radar_type{RADAR_TYPE_UNKNOWN};
};

struct Sensors
{
    int64_t             stamp_ns{0};
    std::string         frame_id;
    std::vector<Sensor> sensors;
};

// outbound datagram to the sensor (host_to_bus)
struct EthernetPacket
{
    int64_t              stamp_ns{0};
    uint32_t             receiver_ip{0};
    uint16_t             receiver_port{0};
    std::vector<uint8_t> payload;
};

} // namespace smartmicro
