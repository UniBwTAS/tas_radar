#include "ethernet/core/sms.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <string>

namespace smartmicro
{

namespace
{

// --- byte read/write helpers (pure, ROS-agnostic) ---

uint8_t readUint8(std::vector<uint8_t> const& data, unsigned long offset)
{
    return data.at(offset);
}

uint16_t readUint16(std::vector<uint8_t> const& data, unsigned long offset)
{
    return ( static_cast<uint16_t>(data.at(offset + 0)) <<  8) |
           ( static_cast<uint16_t>(data.at(offset + 1)) <<  0);
}

uint32_t readUint32(std::vector<uint8_t> const& data, unsigned long offset)
{
    return ( static_cast<uint32_t>(data.at(offset + 0)) << 24) |
           ( static_cast<uint32_t>(data.at(offset + 1)) << 16) |
           ( static_cast<uint32_t>(data.at(offset + 2)) <<  8) |
           ( static_cast<uint32_t>(data.at(offset + 3)) <<  0);
}

uint64_t readUint64(std::vector<uint8_t> const& data, unsigned long offset)
{
    return ( static_cast<uint64_t>(data.at(offset + 0)) << 56) |
           ( static_cast<uint64_t>(data.at(offset + 1)) << 48) |
           ( static_cast<uint64_t>(data.at(offset + 2)) << 40) |
           ( static_cast<uint64_t>(data.at(offset + 3)) << 32) |
           ( static_cast<uint64_t>(data.at(offset + 4)) << 24) |
           ( static_cast<uint64_t>(data.at(offset + 5)) << 16) |
           ( static_cast<uint64_t>(data.at(offset + 6)) <<  8) |
           ( static_cast<uint64_t>(data.at(offset + 7)) <<  0);
}

uint16_t readUint16Body(std::vector<uint8_t> const& data, unsigned long offset, uint8_t endianess)
{
    if (endianess == 2)
        return ( static_cast<uint16_t>(data.at(offset + 0)) <<  0) |
               ( static_cast<uint16_t>(data.at(offset + 1)) <<  8);
    return readUint16(data, offset);
}

uint32_t readUint32Body(std::vector<uint8_t> const& data, unsigned long offset, uint8_t endianess)
{
    if (endianess == 2)
        return ( static_cast<uint32_t>(data.at(offset + 0)) <<  0) |
               ( static_cast<uint32_t>(data.at(offset + 1)) <<  8) |
               ( static_cast<uint32_t>(data.at(offset + 2)) << 16) |
               ( static_cast<uint32_t>(data.at(offset + 3)) << 24);
    return readUint32(data, offset);
}

uint64_t readUint64Body(std::vector<uint8_t> const& data, unsigned long offset, uint8_t endianess)
{
    if (endianess == 2)
        return ( static_cast<uint64_t>(data.at(offset + 0)) <<  0) |
               ( static_cast<uint64_t>(data.at(offset + 1)) <<  8) |
               ( static_cast<uint64_t>(data.at(offset + 2)) << 16) |
               ( static_cast<uint64_t>(data.at(offset + 3)) << 24) |
               ( static_cast<uint64_t>(data.at(offset + 4)) << 32) |
               ( static_cast<uint64_t>(data.at(offset + 5)) << 40) |
               ( static_cast<uint64_t>(data.at(offset + 6)) << 48) |
               ( static_cast<uint64_t>(data.at(offset + 7)) << 56);
    return readUint64(data, offset);
}

float readFloat32Body(std::vector<uint8_t> const& data, unsigned long offset, uint8_t endianess)
{
    uint32_t raw = readUint32Body(data, offset, endianess);
    return *((float*) &raw);
}

uint16_t crc16(std::vector<uint8_t> const& data, int start, int length)
{
    uint8_t i;
    uint16_t wCrc = 0xffff;
    while (length--)
    {
        wCrc ^= static_cast<uint16_t>(data.at(start++)) << 8;
        for (i = 0; i < 8; i++)
            wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
    }
    return wCrc & 0xffff;
}

void writeUint8(std::vector<uint8_t>& data, unsigned long offset, uint8_t value)
{
    data[offset] = value;
}

void writeUint16(std::vector<uint8_t>& data, unsigned long offset, uint16_t value)
{
    data[offset]     = (value >>  8) & 0xFF;
    data[offset + 1] = (value >>  0) & 0xFF;
}

void writeUint32(std::vector<uint8_t>& data, unsigned long offset, uint32_t value)
{
    data[offset]     = (value >> 24) & 0xFF;
    data[offset + 1] = (value >> 16) & 0xFF;
    data[offset + 2] = (value >>  8) & 0xFF;
    data[offset + 3] = (value >>  0) & 0xFF;
}

void writeUint64(std::vector<uint8_t>& data, unsigned long offset, uint64_t value)
{
    data[offset]     = (value >> 56) & 0xFF;
    data[offset + 1] = (value >> 48) & 0xFF;
    data[offset + 2] = (value >> 40) & 0xFF;
    data[offset + 3] = (value >> 32) & 0xFF;
    data[offset + 4] = (value >> 24) & 0xFF;
    data[offset + 5] = (value >> 16) & 0xFF;
    data[offset + 6] = (value >>  8) & 0xFF;
    data[offset + 7] = (value >>  0) & 0xFF;
}

void writeUint16Body(std::vector<uint8_t>& data, unsigned long offset, uint16_t value, uint8_t endianess)
{
    if (endianess == 2)
    {
        data[offset]     = (value >>  0) & 0xFF;
        data[offset + 1] = (value >>  8) & 0xFF;
        return;
    }
    writeUint16(data, offset, value);
}

void writeUint32Body(std::vector<uint8_t>& data, unsigned long offset, uint32_t value, uint8_t endianess)
{
    if (endianess == 2)
    {
        data[offset]     = (value >>  0) & 0xFF;
        data[offset + 1] = (value >>  8) & 0xFF;
        data[offset + 2] = (value >> 16) & 0xFF;
        data[offset + 3] = (value >> 24) & 0xFF;
        return;
    }
    writeUint32(data, offset, value);
}

// no-op that still references its arguments, reproducing the original
// "#define DEBUG if(0) ROS_INFO" (compiled, never executed, no unused warnings).
template<typename... Ts>
inline void debug_noop(Ts&&...) {}

} // namespace

#define DEBUG(...) do { if (false) debug_noop(__VA_ARGS__); } while (0)

SmsDriver::SmsDriver(SmsConfig config, SmsSink& sink, LogFn log)
    : config_(std::move(config)), sink_(sink), log_(std::move(log))
{
    flush();
}

void SmsDriver::logf(LogLevel level, const char* fmt, ...) const
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

void SmsDriver::ingestMeasurements(int64_t stamp_ns, uint32_t sender_ip, const std::vector<uint8_t>& payload)
{
    PacketMeta meta;
    meta.time_ns = stamp_ns;
    meta.ip = sender_ip;

    deserialize_smsTransport(meta, payload);
}

void SmsDriver::ingestAlive(int64_t stamp_ns, uint32_t sender_ip)
{
    PacketMeta meta;
    meta.time_ns = stamp_ns;
    meta.ip = sender_ip;

    if (updateSensorProfileFromAlive(meta.ip))
        publishSensorProfiles(meta.time_ns);
}

void SmsDriver::requestInstructions(int64_t stamp_ns, uint8_t destination, const std::vector<Instruction>& instructions)
{
    PacketMeta meta;
    meta.time_ns = stamp_ns;
    meta.ip = 3232238353 - 17 + destination;   // prepend "192.168.11."

    serialize_smsInstructions(meta, instructions);
}

void SmsDriver::flush()
{
    pool.clear();
}

bool SmsDriver::updateSensorProfileFromAlive(uint32_t ip)
{
    auto& profile = sensor_profiles_[ip];
    bool changed{false};

    if (!profile.alive_seen)
    {
        profile.alive_seen = true;
        changed = true;
    }

    if (profile.radar_type != RADAR_TYPE_DRVEGRD)
    {
        if (profile.radar_type != RADAR_TYPE_UNKNOWN)
            logf(LogLevel::Warn, "sensor profile update for .%u: alive service indicates DRVEGRD, replacing previous type %u.", ip & 0xFF, profile.radar_type);
        profile.radar_type = RADAR_TYPE_DRVEGRD;
        changed = true;
    }

    return changed;
}

bool SmsDriver::updateSensorProfileFromTargetList(uint32_t ip, uint16_t version_major, uint16_t version_minor)
{
    auto& profile = sensor_profiles_[ip];
    bool changed{false};
    uint8_t const inferred_type = (version_major >= 4) ? RADAR_TYPE_DRVEGRD : RADAR_TYPE_UMRR;

    if (profile.target_list_port_version_major != version_major)
    {
        profile.target_list_port_version_major = version_major;
        changed = true;
    }

    if (profile.target_list_port_version_minor != version_minor)
    {
        profile.target_list_port_version_minor = version_minor;
        changed = true;
    }

    if (profile.radar_type != inferred_type)
    {
        if (profile.radar_type != RADAR_TYPE_UNKNOWN)
            logf(LogLevel::Warn, "sensor profile update for .%u: target list version %u.%u indicates type %u, replacing previous type %u.",
                 ip & 0xFF, version_major, version_minor, inferred_type, profile.radar_type);
        profile.radar_type = inferred_type;
        changed = true;
    }

    return changed;
}

void SmsDriver::publishSensorProfiles(int64_t stamp_ns) const
{
    Sensors sensors;
    sensors.stamp_ns = stamp_ns;
    sensors.frame_id = config_.frame_sensor;
    sensors.sensors.reserve(sensor_profiles_.size());

    for (auto const& entry : sensor_profiles_)
    {
        Sensor sensor;
        sensor.ip = static_cast<uint8_t>(entry.first & 0xFF);
        sensor.radar_type = entry.second.radar_type;
        sensors.sensors.push_back(sensor);
    }

    sink_.onSensors(sensors);
}

uint8_t SmsDriver::getSensorRadarType(uint32_t ip) const
{
    auto it = sensor_profiles_.find(ip);
    if (it == sensor_profiles_.end())
        return RADAR_TYPE_UNKNOWN;

    return it->second.radar_type;
}

void SmsDriver::deserialize_smsTransport(PacketMeta const& meta, const std::vector<uint8_t>& data)
{
    DEBUG("-------");

    // absolute minimum packet size given?
    if (data.size() < 12)
    {
        logf(LogLevel::Error, "packet size < 12. dropping packet.");
        return;
    }

    // correct start pattern?
    if (data.at(0) != 0x7E)
    {
        logf(LogLevel::Error, "invalid packet start pattern. dropping packet.");
        return;
    }

    // correct protocol version?
    if (data.at(1) != 1)
    {
        logf(LogLevel::Error, "invalid protocol version. dropping packet.");
        return;
    }

    int header_length = readUint8(data, 2);
    DEBUG("header length: %i", header_length);

    int payload_length = readUint16(data, 3);
    DEBUG("payload length: %i", payload_length);

    int packet_length = data.size();
    DEBUG("packet length: %i", packet_length);

    uint8_t application_protocol = readUint8(data, 5);
    DEBUG("application prot: %i", application_protocol);

    uint32_t flags = readUint32(data, 6);
    DEBUG("flags: %i", flags);

    bool flag_messageCounter = flags & (1 << 0);
    bool flag_skipPayloadCRC = flags & (1 << 2);
    bool flag_sourceClientId = flags & (1 << 3);
    bool flag_dataIdentifier = flags & (1 << 5);
    bool flag_segmentation = flags & (1 << 6);

    DEBUG("flag.messageCounter = %i", flag_messageCounter);
    DEBUG("flag.skipPayloadCRC = %i", flag_skipPayloadCRC);
    DEBUG("flag.sourceClientId = %i", flag_sourceClientId);
    DEBUG("flag.dataIdentifier = %i", flag_dataIdentifier);
    DEBUG("flag.segmentation = %i", flag_segmentation);

    int header_flags_length = 2 * flag_messageCounter + 4 * flag_sourceClientId + 2 * flag_dataIdentifier + 2 * flag_segmentation;
    DEBUG("header flags length = %i", header_flags_length);

    // header size ok?
    if (header_length != 12 + header_flags_length)
    {
        logf(LogLevel::Error, "header length inconsistent. dropping packet.");
        return;
    }

    // packet size ok?
    if (packet_length != header_length + payload_length + 2 * (!flag_skipPayloadCRC))
    {
        logf(LogLevel::Error, "packet size does not match header information. dropping packet.");
        return;
    }

    // it is now safe to access more than the first 12 bytes; check CRC
    uint16_t crc = readUint16(data, header_length - 2);
    uint16_t crc_computed = crc16(data, 0, header_length - 2);
    DEBUG("crc header: %i, computed: %i", crc, crc_computed);

    if (crc != crc_computed)
    {
        logf(LogLevel::Error, "checksum wrong. dropping packet.");
        return;
    }

    if (flag_segmentation && (!flag_dataIdentifier))
    {
        logf(LogLevel::Error, "segmented packet without data_identifier. dropping packet.");
        return;
    }

    // All checks completed, collect data
    SmsTransport package;
    package.meta = meta;
    package.header.applicationProtocolType = application_protocol;
    package.header.messageCounter.available = flag_messageCounter;
    package.header.sourceClientId.available = flag_sourceClientId;
    package.header.dataIdentifier.available = flag_dataIdentifier;
    package.header.segmentation.available = flag_segmentation;
    unsigned long offset = 10;

    if (package.header.messageCounter.available)
    {
        package.header.messageCounter.data = readUint16(data, offset);
        offset += 2;
        DEBUG("messageCounter = %i", package.header.messageCounter.data);
    }

    if (package.header.sourceClientId.available)
    {
        package.header.sourceClientId.data = readUint32(data, offset);
        offset += 4;
        DEBUG("sourceClientId = %i", package.header.sourceClientId.data);
    }

    if (package.header.dataIdentifier.available)
    {
        package.header.dataIdentifier.data = readUint16(data, offset);
        offset += 2;
        DEBUG("dataIdentifier = %i", package.header.dataIdentifier.data);
    }

    if (package.header.segmentation.available)
    {
        package.header.segmentation.data = readUint16(data, offset);
        offset += 2;
        DEBUG("segmentation = %i", package.header.segmentation.data);
    }
    package.applicationData = std::vector<uint8_t>(data.begin() + header_length, data.begin() + header_length + payload_length);

    bool isSegmented = package.header.segmentation.available && package.header.dataIdentifier.available && package.header.messageCounter.available;
    bool isSingle = !package.header.segmentation.available;
    if ((!isSegmented) && (!isSingle))
    {
        logf(LogLevel::Error, "segmented packet with missing segmentation meta data. dropping packet.");
        return;
    }

    if (isSingle)
    {
        // process single, unfragmented "SMS Transport Package"
        process_smsTransport(package);
    }
    else if (isSegmented)
    {
        DEBUG("pool size: %zu", pool.size());

        // remove outdated packets of pool (1 s in the future / 0.2 s in the past)
        for (int i = pool.size() - 1; i >= 0; i -= 1)
            if ((package.meta.time_ns - pool.at(i).meta.time_ns > 1000000000LL) || (pool.at(i).meta.time_ns - package.meta.time_ns > 200000000LL))
            {
                logf(LogLevel::Warn, "removing outdated packet from pool. Current packet timestamp = %f, Pool packet timestamp = %f, IP = .%u", package.meta.time_ns / 1e9, pool.at(i).meta.time_ns / 1e9, package.meta.ip & 0xFF);
                pool.erase(pool.begin() + i);
            }

        // associate single packages by IP if no unique "Source Client ID" is provided
        if (!package.header.sourceClientId.available)
        {
            package.header.sourceClientId.data = package.meta.ip;
            package.header.sourceClientId.available = true;
        }

        // add package to pool
        pool.push_back(package);

        // find packages from same source and index them by message counter
        std::vector<int> idx(package.header.segmentation.data, -1);
        for (int i = pool.size() - 1; i >= 0; i -= 1)
        {
            if (
                    (pool.at(i).header.sourceClientId.data == package.header.sourceClientId.data) &&
                    (pool.at(i).header.segmentation.data == package.header.segmentation.data) &&
                    (pool.at(i).header.dataIdentifier.data == package.header.dataIdentifier.data) &&
                    (pool.at(i).header.messageCounter.data < package.header.segmentation.data)
               )
            {
                idx[pool.at(i).header.messageCounter.data] = i;
            }
        }

        // all fragments received?
        if (*std::min_element(idx.begin(), idx.end()) < 0)
            return; // not all packets available

        // assemble the fragmented package
        package.applicationData.clear();
        for (int i = 0; i < idx.size(); i += 1)
            package.applicationData.insert(package.applicationData.end(), pool.at(idx[i]).applicationData.begin(), pool.at(idx[i]).applicationData.end());
        package.header.segmentation.available = false;
        package.header.messageCounter.available = false;

        // sort indices and take timestamp from first received packet
        std::sort(idx.begin(), idx.end());
        package.meta.time_ns = pool.at(idx.at(0)).meta.time_ns;

        // remove used packets (decreasing index order)
        for (int i = idx.size() - 1; i >= 0; i -= 1)
            pool.erase(pool.begin() + idx.at(i));

        // process completely assembled package
        process_smsTransport(package);
    }
}

void SmsDriver::process_smsTransport(const SmsTransport& package)
{
    switch (package.header.applicationProtocolType)
    {
        case 6:
            DEBUG("got undocumented debug message!");
        break;

        case 8:
            DEBUG("got port!");
            deserialize_smsPort(package.meta, package.applicationData);
        break;

    default:
        break;
    }
}

void SmsDriver::deserialize_smsPort(PacketMeta const& meta, std::vector<uint8_t> const& data)
{
    if (data.size() < 24)
    {
        logf(LogLevel::Error, "port size < 24. dropping port.");
        return;
    }

    SmsPortHeader header;
    header.identifier = readUint32(data, 0);
    header.version_portMajor = readUint16(data, 4);
    header.version_portMinor = readUint16(data, 6);
    header.timestamp = readUint64(data, 8);
    header.size = readUint32(data, 16);
    header.endianess = readUint8(data, 20);
    header.index = readUint8(data, 21);
    header.version_headerMajor = readUint8(data, 22);
    header.version_headerMinor = readUint8(data, 23);

    DEBUG("port_identifier %u", header.identifier);
    DEBUG("port_version_major %u", header.version_portMajor);
    DEBUG("port_version_minor %u", header.version_portMinor);
    DEBUG("port_timestamp %lu", header.timestamp);
    DEBUG("port_size %u", header.size);
    DEBUG("port_endianess %u", header.endianess);
    DEBUG("port_index %u", header.index);
    DEBUG("port_header_version_major %u", header.version_headerMajor);
    DEBUG("port_header_version_minor %u", header.version_headerMinor);

    if (data.size() < header.size)
    {
        // no strict "size !=" comparison: some firmware versions append filling bytes
        logf(LogLevel::Error, "actual port size (%zu) less than announced size (%i). dropping port.", data.size(), header.size);
        return;
    }

    const int version_headerMajor = 2;
    const int version_headerMinor = 2;
    const int endianess_big = 1;
    const int endianess_little = 2;

    if (header.version_headerMajor != version_headerMajor || header.version_headerMinor > version_headerMinor)
    {
        logf(LogLevel::Error, "port header version incompatible. Datastream: Major = %i, Minor = %i. Parser: Major = %i, Minor = %i. dropping port.", header.version_headerMajor, header.version_headerMinor, version_headerMajor, version_headerMinor);
        return;
    }

    if (header.endianess != endianess_big && header.endianess != endianess_little)
    {
        logf(LogLevel::Error, "port identifier %u with port version %u.%u and header version %u.%u uses unsupported endianess %u. dropping port.",
             header.identifier, header.version_portMajor, header.version_portMinor, header.version_headerMajor, header.version_headerMinor, header.endianess);
        return;
    }

    SmsPort port;
    port.meta = meta;
    port.header = header;
    port.data = std::vector<uint8_t>(data.begin() + 24, data.end());
    process_smsPort(port);
}

void SmsDriver::process_smsPort(const SmsPort& port)
{
    switch (port.header.identifier)
    {
        case 66:
            DEBUG("got target list!");
            deserialize_smsTargetList(port.meta, port.header, port.data);
        break;

        case 46:
            DEBUG("got instruction list!");
            deserialize_smsInstructions(port.meta, port.header, port.data);
        break;

    default:
        break;
    }
}

void SmsDriver::deserialize_smsTargetList(PacketMeta const& meta, SmsPortHeader const& header, std::vector<uint8_t> const& data)
{
    enum class ParserVersion
    {
        unknown,
        v1_0,
        v2_1,
        v4_1
    }   parserVersion;

    if (header.version_portMajor == 1 && header.version_portMinor <= 0)
        parserVersion = ParserVersion::v1_0;
    else if (header.version_portMajor == 2 && header.version_portMinor <= 1)
        parserVersion = ParserVersion::v2_1;
    else if (header.version_portMajor == 4 && header.version_portMinor <= 1)
        parserVersion = ParserVersion::v4_1;
    else
        parserVersion = ParserVersion::unknown;

    if (parserVersion == ParserVersion::unknown)
    {
        logf(LogLevel::Error, "Target List port version incompatible. Datastream: Major = %i, Minor = %i. Parser: Major = %i, Minor = %i. dropping port.", header.version_portMajor, header.version_portMinor, 2, 1);
        return;
    }

    if (updateSensorProfileFromTargetList(meta.ip, header.version_portMajor, header.version_portMinor))
        publishSensorProfiles(meta.time_ns);

    if ((data.size() < 8) || ((parserVersion == ParserVersion::v4_1) && (data.size() < 24)))
    {
        logf(LogLevel::Error, "target list header not received. dropping target list.");
        return;
    }

    float cycleTime = readFloat32Body(data, 0, header.endianess);
    uint16_t nrOfTargets = readUint16Body(data, 4, header.endianess);
    uint8_t coverage_value = COVERAGE_UNKNOWN;
    unsigned int sizePortHeader = 8;  // depends on interface version

    if (parserVersion == ParserVersion::v1_0)
    {
        DEBUG("got %u detections. cycle time = %f.", nrOfTargets, cycleTime);
        coverage_value = COVERAGE_UNKNOWN;
    }
    else if (parserVersion == ParserVersion::v2_1)
    {
        uint16_t acquisitionSetup = readUint16Body(data, 6, header.endianess);
        DEBUG("got %u detections. cycle time = %f. acq = %u.", nrOfTargets, cycleTime, acquisitionSetup);

        if (acquisitionSetup & 0b1)
        {
            unsigned int txAntennaIdx = (acquisitionSetup & 0b110) >> 1;
            unsigned int sweepIdx = (acquisitionSetup & 0b11000) >> 3;
            unsigned int centerFreqIdx = (acquisitionSetup & 0b1100000) >> 5;
            DEBUG("txAntennaIdx = %u. sweepIdx = %u. centerFreqIdx = %u.", txAntennaIdx, sweepIdx, centerFreqIdx);

            switch (sweepIdx)
            {
                case 0:  coverage_value = COVERAGE_LONGRANGE;  break;
                case 1:  coverage_value = COVERAGE_MIDRANGE;   break;
                case 2:  coverage_value = COVERAGE_SHORTRANGE; break;
                default: coverage_value = COVERAGE_LONGRANGE;  break;
            }
        }
        else
            coverage_value = COVERAGE_UNKNOWN;
    }
    else if (parserVersion == ParserVersion::v4_1)
    {
        uint8_t acqSweep = readUint8(data, 7);
        uint8_t acqCF = readUint8(data, 8);
        uint64_t acquisitionStart = readUint64Body(data, 16, header.endianess); // NTP time in [2^-32 seconds]
        DEBUG("got %u detections. acqSweep = %u. acqCF = %u. acquisitionStart = %lu.", nrOfTargets, acqSweep, acqCF, acquisitionStart);

        switch (acqSweep)
        {
            // warning: documentation version 2026-02-19 inconsistent, p. 27 vs p. 20
            case 0:  coverage_value = COVERAGE_LONGRANGE;  break;
            case 1:  coverage_value = COVERAGE_MIDRANGE;   break;
            case 2:  coverage_value = COVERAGE_SHORTRANGE; break;
            default: coverage_value = COVERAGE_LONGRANGE;  break;
        }

        sizePortHeader = 24;
    }

    if (nrOfTargets > 110)
    {
        logf(LogLevel::Error, "too high number of targets (%i). dropping target list.", nrOfTargets);
        return;
    }

    if (data.size() < sizePortHeader + 56 * nrOfTargets)
    {
        logf(LogLevel::Error, "portData size %lu smaller than required data size (8 + 56 * %u). dropping target list.", data.size(), nrOfTargets);
        return;
    }

    if (header.size != 24 + sizePortHeader + 56 * nrOfTargets)
    {
        logf(LogLevel::Error, "port size %i mismatches announced size (24 + 8 + 56 * %i). dropping target list.", header.size, nrOfTargets);
        return;
    }

    DetectionRecord record;
    record.stamp_ns = meta.time_ns + static_cast<int64_t>(config_.timestamp_correction * 1e9);
    record.frame_id = config_.frame_sensor + "/" + std::to_string(meta.ip & 0xFF);

    record.detections.reserve(nrOfTargets);
    for (unsigned int i = 0; i < nrOfTargets; i += 1)
    {
        Detection detection;
        detection.range.available = AVAILABLE_VALUE;
        detection.range.value = readFloat32Body(data, sizePortHeader + i * 56 + 0, header.endianess);
        detection.radial_speed.available = AVAILABLE_VALUE;
        detection.radial_speed.value = readFloat32Body(data, sizePortHeader + i * 56 + 4, header.endianess);
        detection.azimuth.available = AVAILABLE_VALUE;
        detection.azimuth.value = readFloat32Body(data, sizePortHeader + i * 56 + 8, header.endianess);
        detection.elevation.available = AVAILABLE_VALUE;
        detection.elevation.value = readFloat32Body(data, sizePortHeader + i * 56 + 12, header.endianess);
        detection.rcs.available = AVAILABLE_VALUE;
        detection.rcs.value = readFloat32Body(data, sizePortHeader + i * 56 + 32, header.endianess);
        detection.power.available = AVAILABLE_VALUE;
        detection.power.value = readFloat32Body(data, sizePortHeader + i * 56 + 44, header.endianess);
        detection.noise.available = AVAILABLE_VALUE;
        detection.noise.value = readFloat32Body(data, sizePortHeader + i * 56 + 48, header.endianess);
        detection.coverage = coverage_value;

        record.detections.push_back(detection);
    }

    sink_.onDetections(record);
}

void SmsDriver::deserialize_smsInstructions(const PacketMeta& meta, const SmsPortHeader& header, const std::vector<uint8_t>& data)
{
    const int version_portMajor = 2;
    const int version_portMinor = 2;

    if (header.version_portMajor != version_portMajor || header.version_portMinor < version_portMinor)
    {
        logf(LogLevel::Error, "Instruction port version incompatible. Datastream: Major = %i, Minor = %i. Parser: Major = %i, Minor = %i. dropping port.", header.version_portMajor, header.version_portMinor, version_portMajor, version_portMinor);
        return;
    }

    if (data.size() < 8)
    {
        logf(LogLevel::Error, "instructions header not received. dropping instructions.");
        return;
    }

    uint8_t nrOfInstructions = readUint8(data, 0);
    DEBUG("got %u instructions", nrOfInstructions);

    if (nrOfInstructions > 10)
    {
        logf(LogLevel::Error, "too high number of instructions (%i). dropping instructions.", nrOfInstructions);
        return;
    }

    if (data.size() != 8 + 24 * nrOfInstructions)
    {
        logf(LogLevel::Error, "port payload size %lu mismatches expected size (8 + 24 * %u). dropping instructions.", data.size(), nrOfInstructions);
        return;
    }

    Instructions instructions;
    instructions.stamp_ns = meta.time_ns;
    instructions.frame_id = config_.frame_sensor + "/" + std::to_string(meta.ip & 0xFF);

    instructions.instructions.reserve(nrOfInstructions);
    for (unsigned int i = 0; i < nrOfInstructions; i += 1)
    {
        Instruction instruction;
        const unsigned int offset = 8 + i * 24;

        instruction.request   = readUint8(data, offset + 0);
        instruction.response  = readUint8(data, offset + 1);
        instruction.section   = readUint16Body(data, offset + 2, header.endianess);
        instruction.id        = readUint16Body(data, offset + 4, header.endianess);
        instruction.datatype  = readUint8(data, offset + 6);
        instruction.dim_count = readUint8(data, offset + 7);
        instruction.signature = readUint32Body(data, offset + 12, header.endianess);
        double value_conv;

        switch (instruction.datatype)
        {
            case DATATYPE_U8:
            case DATATYPE_I8:
                value_conv = readUint8(data, offset + 16);
            break;

            case DATATYPE_U16:
            case DATATYPE_I16:
                value_conv = readUint16Body(data, offset + 16, header.endianess);
            break;

            case DATATYPE_U32:
            case DATATYPE_I32:
                value_conv = readUint32Body(data, offset + 16, header.endianess);
            break;

            case DATATYPE_F32:
            {
                uint32_t val = readUint32Body(data, offset + 16, header.endianess);
                value_conv = *((float*) &val);
            }
            break;

            default:
                logf(LogLevel::Error, "invalid instruction response: datatype invalid (%u). dropping instructions.", instruction.datatype);
                return;
            break;
        }
        instruction.value = value_conv;

        instructions.instructions.push_back(instruction);
    }

    instructions.destination = meta.ip & 0xFF;
    sink_.onInstructions(instructions);
}

void SmsDriver::serialize_smsInstructions(const PacketMeta& meta, const std::vector<Instruction>& instructions)
{
    if (instructions.size() > 10)
        return;

    static uint32_t sequenceCounter{0};
    bool const useDrvegrdRequestFormat = (getSensorRadarType(meta.ip) == RADAR_TYPE_DRVEGRD);
    const uint8_t bodyEndianess = useDrvegrdRequestFormat ? 2 : 1;
    std::vector<uint8_t> instruction_data(8 + 24 * instructions.size(), 0);

    writeUint8(instruction_data, 0, static_cast<uint8_t>(instructions.size()));
    writeUint32Body(instruction_data, 4, ++sequenceCounter, bodyEndianess);

    for (size_t i = 0; i < instructions.size(); i += 1)
    {
        const unsigned int offset = 8 + i * 24;

        writeUint8(instruction_data, offset + 0, instructions.at(i).request);
        writeUint8(instruction_data, offset + 1, instructions.at(i).response);
        writeUint16Body(instruction_data, offset + 2, instructions.at(i).section, bodyEndianess);
        writeUint16Body(instruction_data, offset + 4, instructions.at(i).id, bodyEndianess);
        writeUint8(instruction_data, offset + 6, instructions.at(i).datatype);
        writeUint8(instruction_data, offset + 7, instructions.at(i).dim_count);
        writeUint32Body(instruction_data, offset + 12, instructions.at(i).signature, bodyEndianess);

        switch (instructions.at(i).datatype)
        {
            case DATATYPE_U8:
            case DATATYPE_I8:
                writeUint8(instruction_data, offset + 16, static_cast<uint8_t>(instructions.at(i).value));
            break;

            case DATATYPE_U16:
            case DATATYPE_I16:
                writeUint16Body(instruction_data, offset + 16, static_cast<uint16_t>(instructions.at(i).value), bodyEndianess);
            break;

            case DATATYPE_U32:
            case DATATYPE_I32:
                writeUint32Body(instruction_data, offset + 16, static_cast<uint32_t>(instructions.at(i).value), bodyEndianess);
            break;

            case DATATYPE_F32:
            {
                float val = static_cast<float>(instructions.at(i).value);
                writeUint32Body(instruction_data, offset + 16, *((uint32_t*) &val), bodyEndianess);
            }
            break;

            default:
                logf(LogLevel::Warn, "invalid instruction request: datatype invalid (%u). dropping instructions.", instructions.at(i).datatype);
                return;
            break;
        }
    }

    serialize_smsPort(meta, 0, 46, instruction_data, useDrvegrdRequestFormat);
}

void SmsDriver::serialize_smsPort(PacketMeta const& meta, uint64_t timestamp, uint32_t identifier, std::vector<uint8_t> const& data, bool useDrvegrdPortFormat)
{
    SmsPortHeader header;
    header.identifier = identifier;
    header.version_portMajor = 2;   // depends on Port type
    header.version_portMinor = (identifier == 46 && useDrvegrdPortFormat) ? 5 : 2;
    header.timestamp = timestamp;
    header.size = 24 + data.size();
    header.endianess = (identifier == 46 && useDrvegrdPortFormat) ? 2 : 1;
    header.index = (identifier == 46 && useDrvegrdPortFormat) ? 0 : 1;
    header.version_headerMajor = 2;
    header.version_headerMinor = 0;

    std::vector<uint8_t> port_binary(24);
    writeUint32(port_binary, 0, header.identifier);
    writeUint16(port_binary, 4, header.version_portMajor);
    writeUint16(port_binary, 6, header.version_portMinor);
    writeUint64(port_binary, 8, header.timestamp);
    writeUint32(port_binary, 16, 24 + data.size());
    writeUint8(port_binary, 20, header.endianess);
    writeUint8(port_binary, 21, header.index);
    writeUint8(port_binary, 22, header.version_headerMajor);
    writeUint8(port_binary, 23, header.version_headerMinor);
    port_binary.reserve(24 + data.size());
    port_binary.insert(std::end(port_binary), std::begin(data), std::end(data));

    serialize_smsTransport(meta, 8, port_binary);
}

void SmsDriver::serialize_smsTransport(PacketMeta const& meta, uint8_t applicationProtocolType, std::vector<uint8_t> const& data)
{
    // safety check: unfragmented package (instructions guaranteed to be < MTP)
    if (data.size() > 1100)
        return;

    std::vector<uint8_t> transport_binary(18);
    writeUint8(transport_binary, 0, 0x7E);
    writeUint8(transport_binary, 1, 1);
    writeUint8(transport_binary, 2, 18);
    writeUint16(transport_binary, 3, data.size());
    writeUint8(transport_binary, 5, applicationProtocolType);
    writeUint32(transport_binary, 6, 0xD);
    writeUint16(transport_binary, 10, 0);
    writeUint32(transport_binary, 12, 1);
    writeUint16(transport_binary, 16, crc16(transport_binary, 0, 16));
    transport_binary.reserve(18 + data.size());
    transport_binary.insert(std::end(transport_binary), std::begin(data), std::end(data));

    EthernetPacket packet;
    packet.stamp_ns = meta.time_ns;
    packet.receiver_ip = meta.ip;
    packet.receiver_port = 55555;
    packet.payload = transport_binary;
    sink_.onEthernetPacket(packet);
}

} // namespace smartmicro
