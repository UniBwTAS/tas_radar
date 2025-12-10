#pragma once

// ROS-agnostic core for the radar detections-to-pointcloud node: converts a
// single radar DetectionRecord into one rich, ego-motion-compensated
// PointCloud2-shaped cloud in the SENSOR frame. Points are NOT accumulated and
// NOT transformed into odom; each detection keeps its raw measurement values.
// The ROS 1 / ROS 2 wrappers resolve the tf2 rear-axis transform and odometry
// and serialize the result into sensor_msgs::PointCloud2.

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace radar_pointcloud
{

enum class LogLevel { Info, Warn, Error };
using LogFn = std::function<void(LogLevel, const std::string&)>;

struct Transform { double tx{0.0}, ty{0.0}, tz{0.0}; double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0}; };
struct Dynamics  { double speed{0.0}, yawrate{0.0}; };

struct Detection
{
    double range{0.0}, azimuth{0.0}, elevation{0.0}, radial_speed{0.0};
    double rcs{0.0}, power{0.0}, noise{0.0};
};
struct Input { int64_t stamp_ns{0}; std::string frame_id; std::vector<Detection> detections; };

// One rich output point (PointCloud2 layout: x,y,z,absolute_radial_speed,rgb as
// float32; range,azimuth,elevation,range_rate as float64; rcs,power,noise as float32).
struct PclPoint
{
    float  x{0}, y{0}, z{0};
    float  absolute_radial_speed{0};
    float  rgb{0};
    double range{0}, azimuth{0}, elevation{0}, range_rate{0};
    float  rcs{0}, power{0}, noise{0};
};
struct PointCloudData { int64_t stamp_ns{0}; std::string frame_id; std::vector<PclPoint> points; };

class Converter
{
public:
    Converter() = default;

    // rearAxis: tf2 transform vehicle/rear_axis <- sensor (for the ego velocity
    // lever arm and mounting orientation). Points stay in the sensor frame.
    PointCloudData build(const Input& in, const Transform& rearAxis, const Dynamics& dyn, const LogFn& log);
};

} // namespace radar_pointcloud
