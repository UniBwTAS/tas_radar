#pragma once

// ROS-agnostic core for the radar detection-accumulation node: keeps a
// time-windowed deque of ego-motion-compensated, odom-frame detection points and
// returns the full accumulated cloud. The ROS 1 / ROS 2 wrappers resolve the tf2
// transforms and odometry and serialize the result into sensor_msgs::PointCloud2.

#include <cstdint>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace radar_pointcloud
{

enum class LogLevel { Info, Warn, Error };
using LogFn = std::function<void(LogLevel, const std::string&)>;

struct Transform { double tx{0.0}, ty{0.0}, tz{0.0}; double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0}; };
struct Dynamics  { double speed{0.0}, yawrate{0.0}; };

struct Detection { double range{0.0}, azimuth{0.0}, elevation{0.0}, radial_speed{0.0}; };
struct Input     { int64_t stamp_ns{0}; std::string frame_id; std::vector<Detection> detections; };

// One accumulated point (PointCloud2 layout: x,y,z,absolute_radial_speed,rgb, all float32)
struct PclPoint { float x{0}, y{0}, z{0}, absolute_radial_speed{0}, rgb{0}; };
struct PointCloudData { int64_t stamp_ns{0}; std::string frame_id; std::vector<PclPoint> points; };

struct Config { std::string frame_base{"odom"}; double accumulationTime{2.0}; };

class Accumulator
{
public:
    explicit Accumulator(Config config);

    // rearAxis: tf2 transform vehicle/rear_axis <- sensor (for the ego velocity);
    // odom:     tf2 transform frame_base <- sensor (to place points in the odom frame).
    PointCloudData ingest(const Input& in, const Transform& rearAxis, const Transform& odom,
                          const Dynamics& dyn, const LogFn& log);

private:
    struct MetaDetection { float x, y, z, absolute_radial_speed, color_hex_as_float; };
    struct Shot { int64_t timestamp_ns; std::vector<MetaDetection> detections; };

    std::deque<Shot> accumulation_;
    Config           config_;
};

} // namespace radar_pointcloud
