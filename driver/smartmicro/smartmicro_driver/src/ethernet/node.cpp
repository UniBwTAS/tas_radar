#include "node.h"
#include <radar_msgs/DetectionRecord.h>
#include <ethernet_msgs/utils.h>

#define DEBUG if(0) ROS_INFO


Node::Node(ros::NodeHandle& node_handle) : ros_handle_(node_handle)
{
    /// Parameter
    // Topics
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

    // instructions/request, instructions/response
    node_handle.param<std::string>("topic_instructionsRequest", configuration_.topic_instructionsRequest, "instructions/request");
    node_handle.param<std::string>("topic_instructionsResponse", configuration_.topic_instructionsResponse, "instructions/response");

    // frame prefix (LSB of sensor IP is added)
    node_handle.param<std::string>("frame_sensor", configuration_.frame_sensor, "sensor/radar/umrr");

    // timestamp offset to correct bus latencies (if desired and not otherwise synchronized)
    node_handle.param<double>("timestamp_correction", configuration_.timestamp_correction, 0);

    /// Subscribing & Publishing
    subscriber_ethernet_measurements_ = ros_handle_.subscribe(configuration_.topic_ethernetMeasurementsInput, 100, &Node::rosCallback_ethernetMeasurements, this, ros::TransportHints().tcpNoDelay());
    subscriber_ethernet_alive_ = ros_handle_.subscribe(configuration_.topic_ethernetAliveInput, 100, &Node::rosCallback_ethernetAlive, this, ros::TransportHints().tcpNoDelay());
    publisher_ethernet_ = ros_handle_.advertise<ethernet_msgs::Packet>(configuration_.topic_ethernetOutput, 100);
    publisher_radarDetections_ = ros_handle_.advertise<radar_msgs::DetectionRecord>(configuration_.topic_detectionsOutput, 100);
    subscriber_instructions_ = ros_handle_.subscribe(configuration_.topic_instructionsRequest, 100, &Node::rosCallback_instructions, this);
    publisher_instructions_ = ros_handle_.advertise<smartmicro_driver::Instructions>(configuration_.topic_instructionsResponse, 100);
    publisher_sensors_ = ros_handle_.advertise<smartmicro_driver::Sensors>(configuration_.topic_sensorsOutput, 1, true);

    /// Initialize buffer
    flush();
}

Node::~Node()
{

}

void Node::rosCallback_ethernetMeasurements(const ethernet_msgs::Packet::ConstPtr& msg)
{
    PacketMeta meta;
    meta.time = msg->header.stamp;
    meta.ip = ethernet_msgs::nativeIp4ByArray(msg->sender_ip);

    deserialize_smsTransport(meta, msg->payload);
}

void Node::rosCallback_ethernetAlive(const ethernet_msgs::Packet::ConstPtr& msg)
{
    PacketMeta meta;
    meta.time = msg->header.stamp;
    meta.ip = ethernet_msgs::nativeIp4ByArray(msg->sender_ip);

    if (updateSensorProfileFromAlive(meta.ip))
        publishSensorProfiles(meta.time);
}

void Node::rosCallback_instructions(const smartmicro_driver::Instructions::ConstPtr &msg)
{
    PacketMeta meta;
    meta.time = msg->header.stamp;
    meta.ip = 3232238353 - 17 + msg->destination;   // prepend "192.168.11."

    serialize_smsInstructions(meta, msg->instructions);
}

void Node::flush()
{
    pool.clear();
}

bool Node::updateSensorProfileFromAlive(uint32_t ip)
{
    auto& profile = sensor_profiles_[ip];
    bool changed{false};

    if (!profile.alive_seen)
    {
        profile.alive_seen = true;
        changed = true;
    }

    if (profile.radar_type != smartmicro_driver::Sensor::RADAR_TYPE_DRVEGRD)
    {
        if (profile.radar_type != smartmicro_driver::Sensor::RADAR_TYPE_UNKNOWN)
            ROS_WARN("sensor profile update for .%u: alive service indicates DRVEGRD, replacing previous type %u.", ip & 0xFF, profile.radar_type);
        profile.radar_type = smartmicro_driver::Sensor::RADAR_TYPE_DRVEGRD;
        changed = true;
    }

    return changed;
}

bool Node::updateSensorProfileFromTargetList(uint32_t ip, uint16_t version_major, uint16_t version_minor)
{
    auto& profile = sensor_profiles_[ip];
    bool changed{false};
    uint8_t const inferred_type =
            (version_major >= 4) ? smartmicro_driver::Sensor::RADAR_TYPE_DRVEGRD : smartmicro_driver::Sensor::RADAR_TYPE_UMRR;

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
        if (profile.radar_type != smartmicro_driver::Sensor::RADAR_TYPE_UNKNOWN)
            ROS_WARN("sensor profile update for .%u: target list version %u.%u indicates type %u, replacing previous type %u.",
                     ip & 0xFF, version_major, version_minor, inferred_type, profile.radar_type);
        profile.radar_type = inferred_type;
        changed = true;
    }

    return changed;
}

void Node::publishSensorProfiles(ros::Time const& stamp) const
{
    smartmicro_driver::Sensors sensors;
    sensors.header.stamp = stamp;
    sensors.header.frame_id = configuration_.frame_sensor;
    sensors.sensors.reserve(sensor_profiles_.size());

    for (auto const& entry : sensor_profiles_)
    {
        smartmicro_driver::Sensor sensor;
        sensor.ip = static_cast<uint8_t>(entry.first & 0xFF);
        sensor.radar_type = entry.second.radar_type;
        sensors.sensors.push_back(sensor);
    }

    publisher_sensors_.publish(sensors);
}

uint8_t Node::getSensorRadarType(uint32_t ip) const
{
    auto it = sensor_profiles_.find(ip);
    if (it == sensor_profiles_.end())
        return smartmicro_driver::Sensor::RADAR_TYPE_UNKNOWN;

    return it->second.radar_type;
}

void Node::deserialize_smsTransport(Node::PacketMeta const& meta, const std::vector<uint8_t> &data)
{
    DEBUG("-------");

    // absolute minimum packet size given?
    if (data.size() < 12)
    {
        ROS_ERROR("packet size < 12. dropping packet.");
        return;
    }

    // correct start pattern?
    if (data.at(0) != 0x7E)
    {
        ROS_ERROR("invalid packet start pattern. dropping packet.");
        return;
    }

    // correct protocol version?
    if (data.at(1) != 1)
    {
        ROS_ERROR("invalid protocol version. dropping packet.");
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
        ROS_ERROR("header length inconsistent. dropping packet.");
        return;
    }

    // packet size ok?
    if (packet_length != header_length + payload_length + 2 * (!flag_skipPayloadCRC))
    {
        ROS_ERROR("packet size does not match header information. dropping packet.");
        return;
    }

    // it is now safe to access more than the first 12 bytes
    // check CRC
    uint16_t crc = readUint16(data, header_length - 2);
    uint16_t crc_computed = crc16(data, 0, header_length - 2);
    DEBUG("crc header: %i, computed: %i", crc, crc_computed);

    // checksum ok?
    if (crc != crc_computed)
    {
        ROS_ERROR("checksum wrong. dropping packet.");
        return;
    }

    if (flag_segmentation && (!flag_dataIdentifier))
    {
        ROS_ERROR("segmented packet without data_identifier. dropping packet.");
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
        ROS_ERROR("segmented packet with missing segmentation meta data. dropping packet.");
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

        // remove outdated packets of pool
        for (int i = pool.size() - 1; i >= 0; i -= 1)
            if ((package.meta.time - pool.at(i).meta.time > ros::Duration(1)) || (pool.at(i).meta.time - package.meta.time > ros::Duration(0.2)))
            {
                ROS_WARN("removing outdated packet from pool. Current packet timestamp = %f, Pool packet timestamp = %f, IP = .%u",package.meta.time.toSec(), pool.at(i).meta.time.toSec(), package.meta.ip & 0xFF);
                pool.erase(pool.begin() + i);
            }

        // associate single "SMS Transport Packages" by IP if no unique "Source Client ID" is provided (both u32)
        if (!package.header.sourceClientId.available)
        {
            package.header.sourceClientId.data = package.meta.ip;
            package.header.sourceClientId.available = true;
        }

        // add "SMS Transport Packages" to pool
        pool.push_back(package);

        // search "SMS Transport Packages" from same source and add their indices in the pool to the message counter association array "idx"
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

        // check if all "SMS Transport Packages" for a complete "SMS Application Package" are now received
        if (*min_element(idx.begin(), idx.end()) < 0)
            return; // not all packets available

        // Frankenstein the obtained fragmented "SMS Transport Package" to a fully assembled one
        package.applicationData.clear();
        for (int i = 0; i < idx.size(); i += 1)
            package.applicationData.insert(package.applicationData.end(), pool.at(idx[i]).applicationData.begin(), pool.at(idx[i]).applicationData.end());
        package.header.segmentation.available = false;
        package.header.messageCounter.available = false;

        // sort fragmented "SMS Transport Packages" by pool indices and assign timestamp
        std::sort(idx.begin(), idx.end());
        package.meta.time = pool.at(idx.at(0)).meta.time;  // take timestamp from first received packet

        // remove used packets (in decreasing index order to preserve indices in pool)
        for (int i = idx.size() - 1; i >= 0; i -= 1)
            pool.erase(pool.begin() + idx.at(i));

        // process completely assembled "SMS Transport Package" from fragmented "SMS Transport Packages"
        process_smsTransport(package);
    }
}

void Node::process_smsTransport(const SmsTransport &package)
{
    switch(package.header.applicationProtocolType)
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

void Node::deserialize_smsPort(PacketMeta const& meta, std::vector<uint8_t> const& data)
{
    // minimum length available?
    if (data.size() < 24)
    {
        ROS_ERROR("port size < 24. dropping port.");
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

    // check size
    if (data.size() < header.size)
    {
        ROS_ERROR("actual port size (%zu) less than announced size (%i). dropping port.", data.size(), header.size);
        // no "portData.size() != port.size" comparison as sometimes filling bytes are appended depending on the firmware version
        return;
    }

    // check port header version compatibility
    const int version_headerMajor = 2;
    const int version_headerMinor = 2;
    const int endianess_big = 1;
    const int endianess_little = 2;

    if (header.version_headerMajor != version_headerMajor || header.version_headerMinor > version_headerMinor)
    {
        ROS_ERROR("port header version incompatible. Datastream: Major = %i, Minor = %i. Parser: Major = %i, Minor = %i. dropping port.", header.version_headerMajor, header.version_headerMinor, version_headerMajor, version_headerMinor);
        return;
    }

    if (header.endianess != endianess_big && header.endianess != endianess_little)
    {
        ROS_ERROR("port identifier %u with port version %u.%u and header version %u.%u uses unsupported endianess %u. dropping port.",
                  header.identifier,
                  header.version_portMajor,
                  header.version_portMinor,
                  header.version_headerMajor,
                  header.version_headerMinor,
                  header.endianess);
        return;
    }

    // process "SMS Port"
    SmsPort port;
    port.meta = meta;
    port.header = header;
    port.data = std::vector<uint8_t>(data.begin() + 24, data.end());
    process_smsPort(port);
}

void Node::process_smsPort(const Node::SmsPort &port)
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

void Node::deserialize_smsTargetList(PacketMeta const& meta, SmsPortHeader const& header, std::vector<uint8_t> const& data)
{
    // check port version compatibility
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
        ROS_ERROR("Target List port version incompatible. Datastream: Major = %i, Minor = %i. Parser: Major = %i, Minor = %i. dropping port.", header.version_portMajor, header.version_portMinor, 2, 1);
        return;
    }

    if (updateSensorProfileFromTargetList(meta.ip, header.version_portMajor, header.version_portMinor))
        publishSensorProfiles(meta.time);

    // minimum length available?
    if ((data.size() < 8) || ((parserVersion == ParserVersion::v4_1) && (data.size() < 24)))
    {
        ROS_ERROR("target list header not received. dropping target list.");
        return;
    }

    // read target list header
    float cycleTime = readFloat32Body(data, 0, header.endianess);
    uint16_t nrOfTargets = readUint16Body(data, 4, header.endianess);
    radar_msgs::Coverage coverage;
    unsigned int sizePortHeader = 8;  // depends on interface version

    if (parserVersion == ParserVersion::v1_0)
    {
        DEBUG("got %u detections. cycle time = %f.", nrOfTargets, cycleTime);
        coverage.value = radar_msgs::Coverage::UNKNOWN;
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

            switch(sweepIdx)
            {
                case 0:
                    coverage.value = radar_msgs::Coverage::LONGRANGE;
                break;

                case 1:
                    coverage.value = radar_msgs::Coverage::MIDRANGE;
                break;

                case 2:
                    coverage.value = radar_msgs::Coverage::SHORTRANGE;
                break;

                default:
                    coverage.value = radar_msgs::Coverage::LONGRANGE;
                break;
            }
        }
        else
            coverage.value = radar_msgs::Coverage::UNKNOWN;
    }
    else if (parserVersion == ParserVersion::v4_1)
    {
        uint8_t acqSweep = readUint8(data, 7);
        uint8_t acqCF = readUint8(data, 8);
        uint64_t acquisitionStart = readUint64Body(data, 16, header.endianess); // NTP time in [2^-32 seconds]
        DEBUG("got %u detections. cycle time = %f. acqSweep = %u. acqCF = %u. acquisitionStart = %lu.", nrOfTargets, acqSweep, acqCF, acquisitionStart);

        switch(acqSweep)
        {
            // warning: documentation version 2026-02-19 inconsistent, p. 27 vs p. 20
            case 0:
                coverage.value = radar_msgs::Coverage::LONGRANGE;
                break;

            case 1:
                coverage.value = radar_msgs::Coverage::MIDRANGE;
                break;

            case 2:
                coverage.value = radar_msgs::Coverage::SHORTRANGE;
                break;

            default:
                coverage.value = radar_msgs::Coverage::LONGRANGE;
                break;
        }

        sizePortHeader = 24;
    }

    // check number of targets
    if (nrOfTargets > 110)
    {
        ROS_ERROR("too high number of targets (%i). dropping target list.", nrOfTargets);
        return;
    }

    // check port body data size (port brutto)
    if (data.size() < sizePortHeader + 56*nrOfTargets)
    {
        ROS_ERROR("portData size %lu smaller than required data size (8 + 56 * %u). dropping target list.", data.size(), nrOfTargets);
        return;
    }

    // check actual port size (port netto)
    if (header.size != 24 + sizePortHeader + 56*nrOfTargets)
    {
        ROS_ERROR("port size %i mismatches announced size (24 + 8 + 56 * %i). dropping target list.", header.size, nrOfTargets);
        return;
    }

    // read detections
    radar_msgs::DetectionRecord record;
    record.header.stamp = meta.time + ros::Duration().fromSec(configuration_.timestamp_correction);
    record.header.frame_id = configuration_.frame_sensor + "/" + std::to_string(meta.ip & 0xFF);

    record.detections.reserve(nrOfTargets);
    for (unsigned int i = 0; i < nrOfTargets; i += 1)
    {
        radar_msgs::Detection detection;
        detection.range.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.range.value = readFloat32Body(data, sizePortHeader + i*56 + 0, header.endianess);
        detection.radial_speed.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.radial_speed.value = readFloat32Body(data, sizePortHeader + i*56 + 4, header.endianess);
        detection.azimuth.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.azimuth.value = readFloat32Body(data, sizePortHeader + i*56 + 8, header.endianess);
        detection.elevation.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.elevation.value = readFloat32Body(data, sizePortHeader + i*56 + 12, header.endianess);
        detection.rcs.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.rcs.value = readFloat32Body(data, sizePortHeader + i*56 + 32, header.endianess);
        detection.power.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.power.value = readFloat32Body(data, sizePortHeader + i*56 + 44, header.endianess);
        detection.noise.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.noise.value = readFloat32Body(data, sizePortHeader + i*56 + 48, header.endianess);
        detection.coverage.value = coverage.value;

        record.detections.push_back(detection);
    }

    // process "SMS Target List" (in ROS world)
    publisher_radarDetections_.publish(record);
}

void Node::deserialize_smsInstructions(const PacketMeta &meta, const SmsPortHeader &header, const std::vector<uint8_t> &data)
{
    // check port version compatibility
    const int version_portMajor = 2;
    const int version_portMinor = 2;

    if (header.version_portMajor != version_portMajor || header.version_portMinor < version_portMinor)
    {
        ROS_ERROR("Instruction port version incompatible. Datastream: Major = %i, Minor = %i. Parser: Major = %i, Minor = %i. dropping port.", header.version_portMajor, header.version_portMinor, version_portMajor, version_portMinor);
        return;
    }

    // minimum length available?
    if (data.size() < 8)
    {
        ROS_ERROR("instructions header not received. dropping instructions.");
        return;
    }

    // read target list header
    uint8_t nrOfInstructions = readUint8(data, 0);
    DEBUG("got %u instructions", nrOfInstructions);

    // check number of targets
    if (nrOfInstructions > 10)
    {
        ROS_ERROR("too high number of instructions (%i). dropping instructions.", nrOfInstructions);
        return;
    }

    // check size
    if (data.size() != 8 + 24*nrOfInstructions)
    {
        ROS_ERROR("port payload size %lu mismatches expected size (8 + 24 * %u). dropping instructions.", data.size(), nrOfInstructions);
        return;
    }

    // read instructions
    smartmicro_driver::Instructions instructions;
    instructions.header.stamp = meta.time;
    instructions.header.frame_id = configuration_.frame_sensor + "/" + std::to_string(meta.ip & 0xFF);

    instructions.instructions.reserve(nrOfInstructions);
    for (unsigned int i = 0; i < nrOfInstructions; i += 1)
    {
        smartmicro_driver::Instruction instruction;
        const unsigned int offset = 8 + i*24;

        instruction.request     = readUint8 (data, offset +  0);
        instruction.response    = readUint8 (data, offset +  1);
        instruction.section     = readUint16Body(data, offset +  2, header.endianess);
        instruction.id          = readUint16Body(data, offset +  4, header.endianess);
        instruction.datatype    = readUint8 (data, offset +  6);
        instruction.dim_count   = readUint8 (data, offset +  7);
        instruction.signature   = readUint32Body(data, offset + 12, header.endianess);
        double value_conv;

        switch (instruction.datatype)
        {
            case smartmicro_driver::Instruction::DATATYPE_U8:
            case smartmicro_driver::Instruction::DATATYPE_I8:
                value_conv = readUint8(data, offset + 16);
            break;

            case smartmicro_driver::Instruction::DATATYPE_U16:
            case smartmicro_driver::Instruction::DATATYPE_I16:
                value_conv = readUint16Body(data, offset + 16, header.endianess);
            break;

            case smartmicro_driver::Instruction::DATATYPE_U32:
            case smartmicro_driver::Instruction::DATATYPE_I32:
                value_conv = readUint32Body(data, offset + 16, header.endianess);
            break;

            case smartmicro_driver::Instruction::DATATYPE_F32:
            {
                uint32_t val = readUint32Body(data, offset + 16, header.endianess);
                value_conv = *((float*) &val);
            }
            break;

            default:
                ROS_ERROR("invalid instruction response: datatype invalid (%u). dropping instructions.", instruction.datatype);
                return;
            break;
        }
        instruction.value = value_conv;

        instructions.instructions.push_back(instruction);
    }

    // process "SMS Instructions" (in ROS world)
    instructions.destination = meta.ip & 0xFF;
    publisher_instructions_.publish(instructions);
}

void Node::serialize_smsInstructions(const PacketMeta &meta, const std::vector<smartmicro_driver::Instruction> &instructions)
{
    if (instructions.size() > 10)
        return;

    static uint32_t sequenceCounter{0};
    bool const useDrvegrdRequestFormat = (getSensorRadarType(meta.ip) == smartmicro_driver::Sensor::RADAR_TYPE_DRVEGRD);
    const uint8_t bodyEndianess = useDrvegrdRequestFormat ? 2 : 1;
    std::vector<uint8_t> instruction_data(8 + 24 * instructions.size(), 0);

    writeUint8(instruction_data, 0, static_cast<uint8_t>(instructions.size()));
    writeUint32Body(instruction_data, 4, ++sequenceCounter, bodyEndianess);

    for (size_t i = 0; i < instructions.size(); i += 1)
    {
        const unsigned int offset = 8 + i*24;

        writeUint8 (instruction_data, offset +  0, instructions.at(i).request);
        writeUint8 (instruction_data, offset +  1, instructions.at(i).response);
        writeUint16Body(instruction_data, offset +  2, instructions.at(i).section, bodyEndianess);
        writeUint16Body(instruction_data, offset +  4, instructions.at(i).id, bodyEndianess);
        writeUint8 (instruction_data, offset +  6, instructions.at(i).datatype);
        writeUint8 (instruction_data, offset +  7, instructions.at(i).dim_count);
        writeUint32Body(instruction_data, offset + 12, instructions.at(i).signature, bodyEndianess);

        switch (instructions.at(i).datatype)
        {
            case smartmicro_driver::Instruction::DATATYPE_U8:
            case smartmicro_driver::Instruction::DATATYPE_I8:
                writeUint8(instruction_data, offset + 16, static_cast<uint8_t>(instructions.at(i).value));
            break;

            case smartmicro_driver::Instruction::DATATYPE_U16:
            case smartmicro_driver::Instruction::DATATYPE_I16:
                writeUint16Body(instruction_data, offset + 16, static_cast<uint16_t>(instructions.at(i).value), bodyEndianess);
            break;

            case smartmicro_driver::Instruction::DATATYPE_U32:
            case smartmicro_driver::Instruction::DATATYPE_I32:
                writeUint32Body(instruction_data, offset + 16, static_cast<uint32_t>(instructions.at(i).value), bodyEndianess);
            break;

            case smartmicro_driver::Instruction::DATATYPE_F32:
            {
                float val = static_cast<float>(instructions.at(i).value);
                writeUint32Body(instruction_data, offset + 16, *((uint32_t*) &val), bodyEndianess);
            }
            break;

            default:
                ROS_WARN("invalid instruction request: datatype invalid (%u). dropping instructions.", instructions.at(i).datatype);
                return;
            break;
        }
    }

    serialize_smsPort(meta, 0, 46, instruction_data, useDrvegrdRequestFormat);
}

void Node::serialize_smsPort(PacketMeta const& meta, uint64_t timestamp, uint32_t identifier, std::vector<uint8_t> const& data, bool useDrvegrdPortFormat)
{
    // assemble header
    SmsPortHeader header;
    header.identifier = identifier;
    header.version_portMajor = 2;   // depends on Port type
    header.version_portMinor = (identifier == 46 && useDrvegrdPortFormat) ? 5 : 2;   // depends on Port type
    header.timestamp = timestamp;
    header.size = 24 + data.size();
    header.endianess = (identifier == 46 && useDrvegrdPortFormat) ? 2 : 1;
    header.index = (identifier == 46 && useDrvegrdPortFormat) ? 0 : 1;
    header.version_headerMajor = 2;
    header.version_headerMinor = 0;

    // serialize port
    std::vector<uint8_t> port_binary(24);
    writeUint32(port_binary, 0, header.identifier);
    writeUint16(port_binary, 4, header.version_portMajor);
    writeUint16(port_binary, 6, header.version_portMinor);
    writeUint64(port_binary, 8, header.timestamp);
    writeUint32(port_binary, 16, 24 + data.size());
    writeUint8 (port_binary, 20, header.endianess);
    writeUint8 (port_binary, 21, header.index);
    writeUint8 (port_binary, 22, header.version_headerMajor);
    writeUint8 (port_binary, 23, header.version_headerMinor);
    port_binary.reserve(24 + data.size());
    port_binary.insert(std::end(port_binary), std::begin(data), std::end(data));

    // serialize transport
    serialize_smsTransport(meta, 8, port_binary);
}

void Node::serialize_smsTransport(PacketMeta const& meta, uint8_t applicationProtocolType, std::vector<uint8_t> const& data)
{
    // safety check: unfragmented package (instructions guaranted to be < MTP)
    if (data.size() > 1100)
        return;

    // serialize transport
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

    // send to sensor
    ethernet_msgs::Packet packet;
    packet.header.stamp = meta.time;
    packet.receiver_ip = ethernet_msgs::arrayByNativeIp4(meta.ip);
    packet.receiver_port = 55555;
    packet.payload = transport_binary;
    publisher_ethernet_.publish(packet);
}


uint8_t Node::readUint8(const std::vector<uint8_t> &data, unsigned long offset)
{
    return data.at(offset);
}

uint16_t Node::readUint16(const std::vector<uint8_t> &data, unsigned long offset)
{
    return
            ( static_cast<uint16_t>(data.at(offset + 0)) <<  8) |
            ( static_cast<uint16_t>(data.at(offset + 1)) <<  0) ;
}

uint32_t Node::readUint32(const std::vector<uint8_t> &data, unsigned long offset)
{
    return
            ( static_cast<uint32_t>(data.at(offset + 0)) << 24 ) |
            ( static_cast<uint32_t>(data.at(offset + 1)) << 16 ) |
            ( static_cast<uint32_t>(data.at(offset + 2)) <<  8 ) |
            ( static_cast<uint32_t>(data.at(offset + 3)) <<  0 ) ;
}

uint64_t Node::readUint64(const std::vector<uint8_t> &data, unsigned long offset)
{
    return
            ( static_cast<uint64_t>(data.at(offset + 0)) << 56 ) |
            ( static_cast<uint64_t>(data.at(offset + 1)) << 48 ) |
            ( static_cast<uint64_t>(data.at(offset + 2)) << 40 ) |
            ( static_cast<uint64_t>(data.at(offset + 3)) << 32 ) |
            ( static_cast<uint64_t>(data.at(offset + 4)) << 24 ) |
            ( static_cast<uint64_t>(data.at(offset + 5)) << 16 ) |
            ( static_cast<uint64_t>(data.at(offset + 6)) <<  8 ) |
            ( static_cast<uint64_t>(data.at(offset + 7)) <<  0 ) ;
}

float Node::readFloat32(const std::vector<uint8_t> &data, unsigned long offset)
{
    uint32_t raw =
            ( static_cast<uint32_t>(data.at(offset + 0)) << 24 ) |
            ( static_cast<uint32_t>(data.at(offset + 1)) << 16 ) |
            ( static_cast<uint32_t>(data.at(offset + 2)) <<  8 ) |
            ( static_cast<uint32_t>(data.at(offset + 3)) <<  0 ) ;

    return *((float*) &raw);
}

uint16_t Node::readUint16Body(const std::vector<uint8_t> &data, unsigned long offset, uint8_t endianess)
{
    if (endianess == 2)
        return
                ( static_cast<uint16_t>(data.at(offset + 0)) <<  0 ) |
                ( static_cast<uint16_t>(data.at(offset + 1)) <<  8 ) ;

    return readUint16(data, offset);
}

uint32_t Node::readUint32Body(const std::vector<uint8_t> &data, unsigned long offset, uint8_t endianess)
{
    if (endianess == 2)
        return
                ( static_cast<uint32_t>(data.at(offset + 0)) <<  0 ) |
                ( static_cast<uint32_t>(data.at(offset + 1)) <<  8 ) |
                ( static_cast<uint32_t>(data.at(offset + 2)) << 16 ) |
                ( static_cast<uint32_t>(data.at(offset + 3)) << 24 ) ;

    return readUint32(data, offset);
}

uint64_t Node::readUint64Body(const std::vector<uint8_t> &data, unsigned long offset, uint8_t endianess)
{
    if (endianess == 2)
        return
                ( static_cast<uint64_t>(data.at(offset + 0)) <<  0 ) |
                ( static_cast<uint64_t>(data.at(offset + 1)) <<  8 ) |
                ( static_cast<uint64_t>(data.at(offset + 2)) << 16 ) |
                ( static_cast<uint64_t>(data.at(offset + 3)) << 24 ) |
                ( static_cast<uint64_t>(data.at(offset + 4)) << 32 ) |
                ( static_cast<uint64_t>(data.at(offset + 5)) << 40 ) |
                ( static_cast<uint64_t>(data.at(offset + 6)) << 48 ) |
                ( static_cast<uint64_t>(data.at(offset + 7)) << 56 ) ;

    return readUint64(data, offset);
}

float Node::readFloat32Body(const std::vector<uint8_t> &data, unsigned long offset, uint8_t endianess)
{
    uint32_t raw = readUint32Body(data, offset, endianess);
    return *((float*) &raw);
}

uint16_t Node::crc16(const std::vector<uint8_t> &data, int start, int length)
{
    uint8_t i;
    uint16_t wCrc = 0xffff;
    while (length--) {
        wCrc ^= static_cast<uint16_t>(data.at(start++)) << 8;
        for (i=0; i < 8; i++)
            wCrc = wCrc & 0x8000 ? (wCrc << 1) ^ 0x1021 : wCrc << 1;
    }
    return wCrc & 0xffff;
}

void Node::writeUint8(std::vector<uint8_t> &data, unsigned long offset, uint8_t value)
{
    data[offset] = value;
}

void Node::writeUint16(std::vector<uint8_t> &data, unsigned long offset, uint16_t value)
{
    data[offset]    = (value >>  8) & 0xFF;
    data[offset+1]  = (value >>  0) & 0xFF;
}

void Node::writeUint32(std::vector<uint8_t> &data, unsigned long offset, uint32_t value)
{
    data[offset]    = (value >> 24) & 0xFF;
    data[offset+1]  = (value >> 16) & 0xFF;
    data[offset+2]  = (value >>  8) & 0xFF;
    data[offset+3]  = (value >>  0) & 0xFF;
}

void Node::writeUint64(std::vector<uint8_t> &data, unsigned long offset, uint64_t value)
{
    data[offset]    = (value >> 56) & 0xFF;
    data[offset+1]  = (value >> 48) & 0xFF;
    data[offset+2]  = (value >> 40) & 0xFF;
    data[offset+3]  = (value >> 32) & 0xFF;
    data[offset+4]  = (value >> 24) & 0xFF;
    data[offset+5]  = (value >> 16) & 0xFF;
    data[offset+6]  = (value >>  8) & 0xFF;
    data[offset+7]  = (value >>  0) & 0xFF;
}

void Node::writeUint16Body(std::vector<uint8_t> &data, unsigned long offset, uint16_t value, uint8_t endianess)
{
    if (endianess == 2)
    {
        data[offset]    = (value >>  0) & 0xFF;
        data[offset+1]  = (value >>  8) & 0xFF;
        return;
    }

    writeUint16(data, offset, value);
}

void Node::writeUint32Body(std::vector<uint8_t> &data, unsigned long offset, uint32_t value, uint8_t endianess)
{
    if (endianess == 2)
    {
        data[offset]    = (value >>  0) & 0xFF;
        data[offset+1]  = (value >>  8) & 0xFF;
        data[offset+2]  = (value >> 16) & 0xFF;
        data[offset+3]  = (value >> 24) & 0xFF;
        return;
    }

    writeUint32(data, offset, value);
}
