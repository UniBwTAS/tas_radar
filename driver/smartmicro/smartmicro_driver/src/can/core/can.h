#pragma once

#include "common/detections.h"
#include "can/core/dbcc/umrr.h"

#include <array>
#include <cstdint>
#include <string>

namespace smartmicro
{

// Parsed CAN detection records are delivered to a sink implemented by the wrapper.
class CanSink
{
public:
    virtual ~CanSink() = default;
    virtual void onDetections(const DetectionRecord& detections) = 0;
};

struct CanConfig
{
    std::string frame_radar{"sensor/radar/umrr96"};
};

// ROS-agnostic Smartmicro UMRR CAN driver: raw CAN frames in (0x400 setup +
// 0x401..0x4FF targets), neutral DetectionRecords out via the sink.
class CanDriver
{
public:
    CanDriver(CanConfig config, CanSink& sink, LogFn log);

    // one CAN frame (id, dlc, up to 8 data bytes at data[0..len-1])
    void ingestFrame(int64_t stamp_ns, uint32_t id, uint8_t dlc, const uint8_t* data, size_t len);

private:
    void flush();
    void transmitMeasurementIfAvailable();

    struct DetectionBuffer
    {
        bool available_mode0{false};
        bool available_mode1{false};
        can_obj_umrr_target_h_t data;
    };

    struct SetupBuffer
    {
        bool                    available{false};
        int64_t                 timestamp_ns{0};
        can_obj_umrr_header_h_t data;
        uint8_t                 nDetections{0};
    };

    struct
    {
        SetupBuffer setup;
        std::array<DetectionBuffer, 256> detection;
    }   buffer_;

    CanConfig config_;
    CanSink&  sink_;
    LogFn     log_;
    void logf(LogLevel level, const char* fmt, ...) const;
};

} // namespace smartmicro
