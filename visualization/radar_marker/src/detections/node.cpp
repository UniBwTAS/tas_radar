// HSDF, Philipp Berthold, philipp.berthold@unibw.de

#include "node.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

#define DEBUG if(0) ROS_INFO


Node::Node(ros::NodeHandle& node_handle) : ros_handle_(node_handle), tfListener(tfBuffer)
{
    /// Parameter
    // Topics
    node_handle.param<std::string>("topic_detectionsInput", configuration_.topic_detectionsInput, "detections");
    node_handle.param<std::string>("topic_odometryInput", configuration_.topic_odometryInput, "odometry");      // optional, required by config_showAbsoluteDoppler
    node_handle.param<std::string>("topic_markerOutput", configuration_.topic_markerOutput, "marker");

    // Frames
    node_handle.param<std::string>("frame_base", configuration_.frame_base, "base_link");

    // Config
    node_handle.param<bool>("config_showMetaInformation", configuration_.config_showMetaInformation, false);    // warning: high marker load! only for debugging
    node_handle.param<bool>("config_showAbsoluteDoppler", configuration_.config_showAbsoluteDoppler, false);     // requires odometry with synchronized timestamps

    /// Subscribing & Publishing
    subscriber_radarDetections_ = ros_handle_.subscribe(configuration_.topic_detectionsInput, 100, &Node::rosCallback_radarDetections, this);
    subscriber_odometry_ = ros_handle_.subscribe(configuration_.topic_odometryInput, 100, &Node::rosCallback_odometry, this);
    publisher_marker_ = ros_handle_.advertise<visualization_msgs::Marker>(configuration_.topic_markerOutput, 100);

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

    /// Generate Markers
    visualization_msgs::Marker marker;
    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = msg->header.stamp;
    std::string ns = ip >= 0 ? (std::string("Radar .") + std::to_string(ip)) : "unknown";
    marker.ns = ns;

    /// - Detections
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.frame_locked = true;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.pose.orientation.w = 1.0;

    marker.points.reserve(msg->detections.size());

    /// Render detections color
    if (configuration_.config_showAbsoluteDoppler)
    {
        // Get mounting location
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("base_link", msg->header.frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }

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
            ROS_WARN("data service %s, odom %lf, radar %lf", computationStatusToString(status).data(), dynamics_service_.getNewestSample().time, msg->header.stamp.toSec());

        double speed = dynamics.get(DynamicsIndex::speed);
        double yawrate = dynamics.get(DynamicsIndex::yawrate);

        // Doppler: calculate velocity at sensor position
        tf::Vector3 sensor_velocity = tf::Vector3(speed - yawrate * transformStamped.transform.translation.y, yawrate * transformStamped.transform.translation.x, 0);
        double sign = fabs(roll) > 0.2 ? -1 : +1;  // workaround 3D meas. for 2D odometries

        for (int i = 0; i < msg->detections.size(); i += 1)
        {
            // TODO: check if range, azimuth and __elevation__ available!
            geometry_msgs::Point detection;
            detection.x = msg->detections.at(i).range.value * cos(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
            detection.y = msg->detections.at(i).range.value * sin(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
            detection.z = msg->detections.at(i).range.value * sin(msg->detections.at(i).elevation.value);

            // Doppler: rotate velocity vector to global azimuth angle
            tf::Vector3 velocity_compensation = sensor_velocity.rotate(tf::Vector3(0, 0, 1), -(sign*msg->detections.at(i).azimuth.value + yaw));
            // - compensation is the radial component (x-direction)
            double radial_velocity_compensation = velocity_compensation.x();
            // - ego compensated range rate
            double range_rate_absolute = msg->detections.at(i).radial_speed.value + radial_velocity_compensation;

            if (fabs(msg->detections.at(i).radial_speed.value) > 20)
                continue;   // SMS default radial filter

            std_msgs::ColorRGBA color;
            color.a = 1;
            if (fabs(range_rate_absolute) < (0.25 + speed * 0.2))
            {
                color.b = 0.9;
                color.r = 0.9;
                color.g = 0.9;
                continue;
            }
            else
            {
                marker.scale.x = 1;
                marker.scale.y = 1;
                marker.scale.z = 1;

                float range = msg->detections.at(i).radial_speed.value / 20;
                if (range > 1)
                    range = 1;
                if (range < -1)
                    range = -1;

                color.b = 0;
                color.r = 1;
                color.g = 1;

                if (range > 0)
                    color.r = 1 - range;
                else
                    color.g = 1 + range;
            }

            marker.points.push_back(detection);
            marker.colors.push_back(color);
        }
    }
    else    // static colors by IP map
    {
        marker.color.a = 1.0; // Don't forget to set the alpha!
        HSVtoRGB(marker.color.r, marker.color.g, marker.color.b, hue, saturation, lightness);

        for (int i = 0; i < msg->detections.size(); i += 1)
        {
            // TODO: check if range, azimuth and __elevation__ available!
            geometry_msgs::Point detection;
            detection.x = msg->detections.at(i).range.value * cos(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
            detection.y = msg->detections.at(i).range.value * sin(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
            detection.z = msg->detections.at(i).range.value * sin(msg->detections.at(i).elevation.value);

            marker.points.push_back(detection);
        }
    }
    publisher_marker_.publish( marker );

    /// - Mounting location
    marker.id = 0;
    marker.ns = ns + "/S";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.1;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.color.a = 1.0;
    HSVtoRGB(marker.color.r, marker.color.g, marker.color.b, hue, saturation, lightness);
    marker.points.clear();
    publisher_marker_.publish(marker);

    /// Radial speed (raw) measurement
    marker.id = 2;
    marker.ns = ns + "/D";
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.frame_locked = true;
    marker.scale.x = 0.02;
    marker.scale.y = 0;
    marker.scale.z = 0;
    marker.color.a = 0.4;
    marker.points.clear();
    marker.points.reserve(msg->detections.size());
    for (unsigned int i = 0; i < msg->detections.size(); i += 1)
    {
        // TODO: check if range, azimuth and __elevation__ available!
        geometry_msgs::Point start;
        start.x = msg->detections.at(i).range.value * cos(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
        start.y = msg->detections.at(i).range.value * sin(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
        start.z = msg->detections.at(i).range.value * sin(msg->detections.at(i).elevation.value);
        marker.points.push_back(start);

        geometry_msgs::Point end;
        double range_inOneSecond = msg->detections.at(i).range.value + msg->detections.at(i).radial_speed.value;
        end.x = range_inOneSecond * cos(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
        end.y = range_inOneSecond * sin(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
        end.z = range_inOneSecond * sin(msg->detections.at(i).elevation.value);
        marker.points.push_back(end);
    }
    publisher_marker_.publish( marker );

    /// - Meta measurement. Only use for debug purposes: high bandwidth.
    if (configuration_.config_showMetaInformation)
    {
        marker.id = 0;
        marker.ns = ns + "/I";
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.frame_locked = true;
        marker.scale.x = 0;
        marker.scale.y = 0;
        marker.scale.z = 0.2;
        marker.color.a = 0.9;
        marker.points.clear();

        for (unsigned int i = 0; i < msg->detections.size(); i += 1)
        {
            marker.id += 1;
            std::stringstream wtf;
            if (msg->detections.at(i).rcs.available)
                wtf << "rcs " << std::fixed << std::setprecision(3) << msg->detections.at(i).rcs.value << std::endl;
            if (msg->detections.at(i).power.available)
                wtf << "pow " << std::fixed << std::setprecision(1) << msg->detections.at(i).power.value << std::endl;
            if (msg->detections.at(i).noise.available)
                wtf << "nse " << std::fixed << std::setprecision(1) << msg->detections.at(i).noise.value << std::endl;
            marker.text = wtf.str();

            marker.pose.position.x = msg->detections.at(i).range.value * cos(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
            marker.pose.position.y = msg->detections.at(i).range.value * sin(msg->detections.at(i).azimuth.value) * cos(msg->detections.at(i).elevation.value);
            marker.pose.position.z = msg->detections.at(i).range.value * sin(msg->detections.at(i).elevation.value);
            publisher_marker_.publish( marker );
        }

        // isn't there a better solution without using a memory from the last cycle of this sensor?
        for (unsigned int i = msg->detections.size(); i < msg->detections.size() + 10; i += 1)
        {
            marker.id += 1;
            marker.action = visualization_msgs::Marker::DELETE;
            publisher_marker_.publish( marker );
        }
    }
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
