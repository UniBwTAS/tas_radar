// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#include "node.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/convert.h>

#define DEBUG if(0) ROS_INFO


Node::Node(ros::NodeHandle& node_handle) : ros_handle_(node_handle), tfListener(tfBuffer)
{
    /// Parameter
    // Topics
    node_handle.param<std::string>("topic_detectionsInput", configuration_.topic_detectionsInput, "detections");
    node_handle.param<std::string>("topic_odometryInput", configuration_.topic_odometryInput, "odometry");      // optional, required by config_showAbsoluteDoppler
    node_handle.param<std::string>("topic_accumulationOutput", configuration_.topic_accumulationOutput, "accumulation");

    // Frames
    node_handle.param<std::string>("frame_base", configuration_.frame_base, "odom");

    // Configuration
    node_handle.param<double>("config_accumulationTime", configuration_.config_accumulationTime, configuration_.config_accumulationTime);

    /// Subscribing & Publishing
    subscriber_radarDetections_ = ros_handle_.subscribe(configuration_.topic_detectionsInput, 100, &Node::rosCallback_radarDetections, this);
    subscriber_odometry_ = ros_handle_.subscribe(configuration_.topic_odometryInput, 100, &Node::rosCallback_odometry, this);
    publisher_pointcloud_ = ros_handle_.advertise<sensor_msgs::PointCloud2>(configuration_.topic_accumulationOutput, 100);

    /// Initialization
    dynamics_service_.setConfiguration(200);
}

Node::~Node()
{

}

std::string computationStatusToString(DataFitting::ComputationStatus status)
{
    switch (status)
    {
        case DataFitting::ComputationStatus::none:
            return "none";
            break;

        case DataFitting::ComputationStatus::approximated:
            return "approximated";
            break;

        case DataFitting::ComputationStatus::interpolation:
            return "interpolation";
            break;

        case DataFitting::ComputationStatus::extrapolation:
            return "extrapolation";
            break;

        default:
            return "invalid";
            break;
    }
}

void Node::rosCallback_radarDetections(const radar_msgs::DetectionRecord::ConstPtr &msg)
{
    /// Get IP/ID
    int ip = -1;
    size_t idx = msg->header.frame_id.find_last_of("/");
    if (idx != std::string::npos)
    {
        try
        {
            ip = std::stoi(msg->header.frame_id.substr(idx+1));
        }
        catch (const std::invalid_argument& ia) {
            //std::cerr << "Invalid argument: " << ia.what() << std::endl;
            ip = -2;
        }

        catch (const std::out_of_range& oor) {
            //std::cerr << "Out of Range error: " << oor.what() << std::endl;
            ip = -3;
        }

        catch (const std::exception& e)
        {
            //std::cerr << "Undefined error: " << e.what() << std::endl;
            ip = -4;
        }
    }
    if (ip < 0)
        ROS_WARN_THROTTLE(1000, "Could not parse sensor IP in frame_id %s", msg->header.frame_id.c_str());

    /// Compute map sensor_ip <-> marker color
    float hue = 60.0f * (ip % 5);    // in degree
    float saturation = 1.0;
    float lightness;
    if (ip < 10)
        lightness = 0.25;
    else if (ip < 20)
        lightness = 0.5;
    else if (ip < 30)
        lightness = 0.75;
    else
        lightness = 1.0;
    if (ip < 0)
    {
        lightness = 0.0;
    }
    std_msgs::ColorRGBA color;
    HSVtoRGB(color.r, color.g, color.b, hue, saturation, lightness);
    int color_binary = int(round(color.r * 255.0)) << 16 | int(round(color.g * 255.0)) << 8 | int(round(color.b * 255.0)) << 0;

    /// Get mounting location
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("vehicle/rear_axis", msg->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }

    /// Get reference frame location
    geometry_msgs::TransformStamped transformOdomStamped;
    try
    {
        transformOdomStamped = tfBuffer.lookupTransform(configuration_.frame_base, msg->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }

    /// Dynamics evaluation preparation
    double ow = transformStamped.transform.rotation.w;
    double ox = transformStamped.transform.rotation.x;
    double oy = transformStamped.transform.rotation.y;
    double oz = transformStamped.transform.rotation.z;

    double roll; double pitch; double yaw;

    tf::Quaternion q(ox, oy, oz, ow);
    tf::Matrix3x3 p(q);
    p.getRPY(roll, pitch, yaw);

    // Get dynamics
    DataFitting::ComputationStatus status;
    DynamicsType dynamics = dynamics_service_.getValueAtTime(msg->header.stamp.toSec(), &status);
    if (status != DataFitting::ComputationStatus::interpolation)
    {
        bool ok;
        auto newestSample = dynamics_service_.getNewestSample(&ok);

        if (ok)
            ROS_WARN("Odom interpolation issue. Odom available: %s, latest odom sample time: %lf, current measurement time: %lf", computationStatusToString(status).data(), newestSample.time, msg->header.stamp.toSec());
        else
        {
            ROS_WARN("Odom interpolation issue. Odom available: %s, latest odom sample time: n/a, current measurement time: %lf. Discarding measurement.", computationStatusToString(status).data(), msg->header.stamp.toSec());
            return;
        }
    }

    double speed = dynamics.get(DynamicsIndex::speed);
    double yawrate = dynamics.get(DynamicsIndex::yawrate);

    // Doppler: calculate velocity at sensor position
    tf::Vector3 sensor_velocity = tf::Vector3(speed - yawrate * transformStamped.transform.translation.y, yawrate * transformStamped.transform.translation.x, 0);
    double sign = fabs(roll) > 0.2 ? -1 : +1;  // workaround 3D meas. for 2D odometries

    /// Clean Queue
    if (!accumulation.empty() && (accumulation.back().timestamp - msg->header.stamp) > ros::Duration(0.3))
        accumulation = std::deque<MetaDetectionShot>();

    while (!accumulation.empty() && (msg->header.stamp - accumulation.front().timestamp) > ros::Duration(configuration_.config_accumulationTime))
        accumulation.pop_front();

    /// Push Detection Shot
    MetaDetectionShot shot;
    shot.timestamp = msg->header.stamp;
    shot.detections.reserve(msg->detections.size());
    for (int i = 0; i < msg->detections.size(); i += 1)
    {
        MetaDetection metaDetection;

        // Get Cartesian coordinates
        geometry_msgs::PointStamped detection;
        detection.point.x = msg->detections.at(i).range.value * cos(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
        detection.point.y = msg->detections.at(i).range.value * sin(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
        detection.point.z = msg->detections.at(i).range.value * sin(msg->detections.at(i).elevation.value);

        // Cartestian reference target coordinates
        tf2::doTransform(detection, metaDetection.point, transformOdomStamped);

        // Doppler: rotate velocity vector to global azimuth angle
        tf::Vector3 velocity_compensation = sensor_velocity.rotate(tf::Vector3(0, 0, 1), -(sign*msg->detections.at(i).azimuth.value + yaw));
        // - compensation is the radial component (x-direction)
        double radial_velocity_compensation = velocity_compensation.x();
        // - ego compensated range rate
        double range_rate_absolute = msg->detections.at(i).radial_speed.value + radial_velocity_compensation;

        // Collect values
        metaDetection.absolute_radial_speed = range_rate_absolute;
        metaDetection.color_hex_as_float = *reinterpret_cast<float*>(&color_binary);
        shot.detections.push_back(std::move(metaDetection));
    }
    accumulation.push_back(shot);

    /// Fill PointCloud meta (Source: https://medium.com/@tonyjacob_/pointcloud2-message-explained-853bd9907743)
    sensor_msgs::PointCloud2 pcl_msg;
    //Modifier to describe what the fields are.
    sensor_msgs::PointCloud2Modifier modifier(pcl_msg);

    modifier.setPointCloud2Fields(5,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "absolute_radial_speed", 1, sensor_msgs::PointField::FLOAT32,
        "rgb", 1, sensor_msgs::PointField::FLOAT32);

    // Point count
    int point_count = 0;
    for (int iShot = 0; iShot < accumulation.size(); iShot += 1)
        point_count += accumulation[iShot].detections.size();

    //Msg header
    pcl_msg.header = msg->header;
    pcl_msg.header.frame_id = configuration_.frame_base;

    pcl_msg.height = 1;
    pcl_msg.width = point_count;
    pcl_msg.is_dense = true;

    //Total number of bytes per point
    pcl_msg.point_step = 20;
    pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;
    pcl_msg.data.resize(pcl_msg.row_step);

    //Iterators for PointCloud msg
    sensor_msgs::PointCloud2Iterator<float> iterX(pcl_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iterY(pcl_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iterZ(pcl_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iterIntensity(pcl_msg, "absolute_radial_speed");
    sensor_msgs::PointCloud2Iterator<float> iterRgb(pcl_msg, "rgb");


    for (int iShot = 0; iShot < accumulation.size(); iShot += 1)
        for (int iDetection = 0; iDetection < accumulation[iShot].detections.size(); iDetection += 1)
        {
            auto& this_detection = accumulation[iShot].detections.at(iDetection);

            *iterX = this_detection.point.point.x;
            *iterY = this_detection.point.point.y;
            *iterZ = this_detection.point.point.z;
            *iterIntensity = this_detection.absolute_radial_speed;
            *iterRgb = this_detection.color_hex_as_float;

            ++iterX;
            ++iterY;
            ++iterZ;
            ++iterIntensity;
            ++iterRgb;
        }

    publisher_pointcloud_.publish(pcl_msg);
}

void Node::rosCallback_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{
    dynamics_service_.registerValue(DynamicsType({msg->twist.twist.linear.x,msg->twist.twist.angular.z}), msg->header.stamp.toSec());
}

/*! \brief Convert HSV to RGB color space

  Converts a given set of HSV values `h', `s', `v' into RGB
  coordinates. The output RGB values are in the range [0, 1], and
  the input HSV values are in the ranges h = [0, 360], and s, v =
  [0, 1], respectively.

  \param fR Red component, used as output, range: [0, 1]
  \param fG Green component, used as output, range: [0, 1]
  \param fB Blue component, used as output, range: [0, 1]
  \param fH Hue component, used as input, range: [0, 360]
  \param fS Hue component, used as input, range: [0, 1]
  \param fV Hue component, used as input, range: [0, 1]

  Source: google
*/
void Node::HSVtoRGB(float& fR, float& fG, float& fB, float& fH, float& fS, float& fV) {
  float fC = fV * fS; // Chroma
  float fHPrime = fmod(fH / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = fV - fC;

  if(0 <= fHPrime && fHPrime < 1) {
    fR = fC;
    fG = fX;
    fB = 0;
  } else if(1 <= fHPrime && fHPrime < 2) {
    fR = fX;
    fG = fC;
    fB = 0;
  } else if(2 <= fHPrime && fHPrime < 3) {
    fR = 0;
    fG = fC;
    fB = fX;
  } else if(3 <= fHPrime && fHPrime < 4) {
    fR = 0;
    fG = fX;
    fB = fC;
  } else if(4 <= fHPrime && fHPrime < 5) {
    fR = fX;
    fG = 0;
    fB = fC;
  } else if(5 <= fHPrime && fHPrime < 6) {
    fR = fC;
    fG = 0;
    fB = fX;
  } else {
    fR = 0;
    fG = 0;
    fB = 0;
  }

  fR += fM;
  fG += fM;
  fB += fM;
}
