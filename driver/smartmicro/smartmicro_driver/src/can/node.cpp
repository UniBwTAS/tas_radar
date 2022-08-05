// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#include "node.h"
#include <radar_msgs/DetectionRecord.h>

#define DEBUG if(0) ROS_INFO


Node::Node(ros::NodeHandle& node_handle) : ros_handle_(node_handle)
{
    /// Parameter
    // Topics
    node_handle.param<std::string>("topic_canInput", configuration_.topic_canInput, "/can/sms/deviceToHost");
    node_handle.param<std::string>("topic_detectionsOutput", configuration_.topic_detectionsOutput, "/sensor/radar/umrr96/detections");
    node_handle.param<std::string>("frame_radar", configuration_.frame_radar, "sensor/radar/umrr96");

    /// Subscribing & Publishing
    subscriber_can_ = ros_handle_.subscribe(configuration_.topic_canInput, 100, &Node::rosCallback_can, this, ros::TransportHints().tcpNoDelay());
    publisher_radarDetections_ = ros_handle_.advertise<radar_msgs::DetectionRecord>(configuration_.topic_detectionsOutput, 100);

    /// Initialize buffer
    flush();
}

Node::~Node()
{

}

void Node::rosCallback_can(const can_msgs::Frame::ConstPtr& msg)
{
    DEBUG("CAN: %i", msg->id);

    /// Check and parse setup frame 0x400
    if (msg->id == 0x400)
    {
        flush();    // discard buffer. if there was a valid measurement inside, it should have been sent out already.

        if (msg->data.size() != 8)
        {
            ROS_ERROR("invalid can_msgs format. data array size != 8");
            return;
        }

        uint64_t payload = u64_from_can_msg(msg->data.data());

        if ((unpack_message_header(&buffer_.setup.data, 0x400, payload, msg->dlc, msg->header.stamp.toNSec() / 1000) >= 0)
            && (decode_can_0x400_NUMBER_OF_OBJECTS(&buffer_.setup.data, &buffer_.setup.nDetections) >= 0))
        {
            DEBUG("detections %i", buffer_.setup.nDetections);
            buffer_.setup.available = true;
        }
        else
        {
            ROS_ERROR("could not parse 0x400");
            flush();
            return;
        }
    }

    /// Check and parse detection frame 0x401-0x4FF
    else if ((msg->id >= 0x400) && (msg->id <= 0x4FF))
    {
        unsigned int iDetection = msg->id - 0x401;

        uint64_t payload = u64_from_can_msg(msg->data.data());
        uint8_t modeSignal;

        if ((unpack_message_target(&buffer_.detection[iDetection].data, 0x401, payload, msg->dlc, msg->header.stamp.toNSec() / 1000) >= 0)
            && (decode_can_0x401_MODE_SIGNAL(&buffer_.detection[iDetection].data, &modeSignal) >= 0))
        {
            if (modeSignal == 0)
                buffer_.detection[iDetection].available_mode0 = true;

            if (modeSignal == 1)
                buffer_.detection[iDetection].available_mode1 = true;
        }
        else
        {
            ROS_ERROR("could not parse mode_signal");
            flush();
            return;
        }

    }

    transmitMeasurementIfAvailable();
}

void Node::flush()
{
    buffer_.setup.available = false;
    for (unsigned int iDetection = 0; iDetection <= 255; iDetection += 1)
    {
        buffer_.detection[iDetection].available_mode0 = false;
        buffer_.detection[iDetection].available_mode1 = false;
    }
}

void Node::transmitMeasurementIfAvailable()
{
    // setup frame received?
    if (!buffer_.setup.available)
        return;

    // all detections frames received? (assuming increasing send order, thus looping downwards)
    for (unsigned int iDetection = buffer_.setup.nDetections - 1; iDetection >= 0; iDetection -= 1)
    {
        if(!buffer_.detection[iDetection].available_mode0)
            return;

        if(!buffer_.detection[iDetection].available_mode1)
            return;
    }

    // all announced detections are received. send them out!
    radar_msgs::DetectionRecord record;
    record.header.stamp = buffer_.setup.timestamp;
    record.header.frame_id = configuration_.frame_radar;

    bool decode_success = true;

    decode_success &= decode_can_0x400_CYCLE_COUNTER(&buffer_.setup.data, &record.header.seq) >= 0;

    record.detections.reserve(buffer_.setup.nDetections);
    for (unsigned int iDetection = 0; iDetection < buffer_.setup.nDetections; iDetection += 1)
    {
        radar_msgs::Detection detection;
        decode_success &= decode_can_0x401_RANGE(&buffer_.detection[iDetection].data, &detection.range.value) >= 0;
        decode_success &= decode_can_0x401_AZIMUTH(&buffer_.detection[iDetection].data, &detection.azimuth.value) >= 0;
        decode_success &= decode_can_0x401_ELEVATION(&buffer_.detection[iDetection].data, &detection.elevation.value) >= 0;
        decode_success &= decode_can_0x401_SPEED_RADIAL(&buffer_.detection[iDetection].data, &detection.radial_speed.value) >= 0;
        decode_success &= decode_can_0x401_RCS(&buffer_.detection[iDetection].data, &detection.rcs.value) >= 0;
        detection.coverage.value = radar_msgs::Coverage::UNKNOWN;

        // Conversion of [°] to [rad]
        detection.azimuth.value *= M_PI/180.0;
        detection.elevation.value *= M_PI/180.0;

        detection.range.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.azimuth.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.elevation.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.radial_speed.available = radar_msgs::Measurement::AVAILABLE_VALUE;
        detection.rcs.available = radar_msgs::Measurement::AVAILABLE_VALUE;

        record.detections.push_back(detection);
    }

    if (decode_success)
    {
        DEBUG("sending...");
        publisher_radarDetections_.publish(record);
    }
    else
        ROS_ERROR("not sending");

    flush();
}
