#include "can/core/can.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>

namespace smartmicro
{

namespace
{
template<typename... Ts>
inline void debug_noop(Ts&&...) {}
} // namespace

#define DEBUG(...) do { if (false) debug_noop(__VA_ARGS__); } while (0)

CanDriver::CanDriver(CanConfig config, CanSink& sink, LogFn log)
    : config_(std::move(config)), sink_(sink), log_(std::move(log))
{
    flush();
}

void CanDriver::logf(LogLevel level, const char* fmt, ...) const
{
    if (!log_)
        return;

    char buffer[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    log_(level, std::string(buffer));
}

void CanDriver::ingestFrame(int64_t stamp_ns, uint32_t id, uint8_t dlc, const uint8_t* data, size_t len)
{
    DEBUG("CAN: %i", id);

    /// setup frame 0x400
    if (id == 0x400)
    {
        flush();    // discard buffer; a valid measurement would already have been sent

        if (len != 8)
        {
            logf(LogLevel::Error, "invalid can_msgs format. data array size != 8");
            return;
        }

        uint64_t payload = u64_from_can_msg(data);

        if ((unpack_message_header(&buffer_.setup.data, 0x400, payload, dlc, static_cast<dbcc_time_stamp_t>(stamp_ns / 1000)) >= 0)
            && (decode_can_0x400_NUMBER_OF_OBJECTS(&buffer_.setup.data, &buffer_.setup.nDetections) >= 0))
        {
            DEBUG("detections %i", buffer_.setup.nDetections);
            buffer_.setup.timestamp_ns = stamp_ns;
            buffer_.setup.available = true;
        }
        else
        {
            logf(LogLevel::Error, "could not parse 0x400");
            flush();
            return;
        }
    }
    /// detection frame 0x401-0x4FF
    else if ((id >= 0x400) && (id <= 0x4FF))
    {
        unsigned int iDetection = id - 0x401;

        uint64_t payload = u64_from_can_msg(data);
        uint8_t modeSignal;

        if ((unpack_message_target(&buffer_.detection[iDetection].data, 0x401, payload, dlc, static_cast<dbcc_time_stamp_t>(stamp_ns / 1000)) >= 0)
            && (decode_can_0x401_MODE_SIGNAL(&buffer_.detection[iDetection].data, &modeSignal) >= 0))
        {
            if (modeSignal == 0)
                buffer_.detection[iDetection].available_mode0 = true;

            if (modeSignal == 1)
                buffer_.detection[iDetection].available_mode1 = true;
        }
        else
        {
            logf(LogLevel::Error, "could not parse mode_signal");
            flush();
            return;
        }
    }

    transmitMeasurementIfAvailable();
}

void CanDriver::flush()
{
    buffer_.setup.available = false;
    for (unsigned int iDetection = 0; iDetection <= 255; iDetection += 1)
    {
        buffer_.detection[iDetection].available_mode0 = false;
        buffer_.detection[iDetection].available_mode1 = false;
    }
}

void CanDriver::transmitMeasurementIfAvailable()
{
    // setup frame received?
    if (!buffer_.setup.available)
        return;

    // all announced detection frames received? Targets are sent in increasing index order, so
    // the highest index arrives last; looping downwards bails out earliest on incomplete shots.
    for (int iDetection = buffer_.setup.nDetections - 1; iDetection >= 0; iDetection -= 1)
    {
        if (!buffer_.detection[iDetection].available_mode0)
            return;

        if (!buffer_.detection[iDetection].available_mode1)
            return;
    }

    // all announced detections are received - assemble and send
    DetectionRecord record;
    record.stamp_ns = buffer_.setup.timestamp_ns;
    record.frame_id = config_.frame_radar;

    bool decode_success = true;
    decode_success &= decode_can_0x400_CYCLE_COUNTER(&buffer_.setup.data, &record.seq) >= 0;

    record.detections.reserve(buffer_.setup.nDetections);
    for (unsigned int iDetection = 0; iDetection < buffer_.setup.nDetections; iDetection += 1)
    {
        Detection detection;
        decode_success &= decode_can_0x401_RANGE(&buffer_.detection[iDetection].data, &detection.range.value) >= 0;
        decode_success &= decode_can_0x401_AZIMUTH(&buffer_.detection[iDetection].data, &detection.azimuth.value) >= 0;
        decode_success &= decode_can_0x401_ELEVATION(&buffer_.detection[iDetection].data, &detection.elevation.value) >= 0;
        decode_success &= decode_can_0x401_SPEED_RADIAL(&buffer_.detection[iDetection].data, &detection.radial_speed.value) >= 0;
        decode_success &= decode_can_0x401_RCS(&buffer_.detection[iDetection].data, &detection.rcs.value) >= 0;
        detection.coverage = COVERAGE_UNKNOWN;

        // conversion of [deg] to [rad]
        detection.azimuth.value *= M_PI / 180.0;
        detection.elevation.value *= M_PI / 180.0;

        detection.range.available = AVAILABLE_VALUE;
        detection.azimuth.available = AVAILABLE_VALUE;
        detection.elevation.available = AVAILABLE_VALUE;
        detection.radial_speed.available = AVAILABLE_VALUE;
        detection.rcs.available = AVAILABLE_VALUE;

        record.detections.push_back(detection);
    }

    if (decode_success)
    {
        DEBUG("sending...");
        sink_.onDetections(record);
    }
    else
        logf(LogLevel::Error, "not sending");

    flush();
}

} // namespace smartmicro
