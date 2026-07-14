#pragma once

// ROS-agnostic core for the radar detection marker node: turns a radar detection
// record (+ optional ego-motion for absolute-Doppler colouring) into neutral
// marker primitives. The ROS 1 / ROS 2 wrappers resolve the tf2 transform and
// odometry and map the primitives onto visualization_msgs::Marker.

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace radar_marker
{

enum class LogLevel { Info, Warn, Error };
using LogFn = std::function<void(LogLevel, const std::string&)>;

// visualization_msgs/Marker constants (identical values in ROS 1 and ROS 2)
enum : int32_t { MARKER_CUBE = 1, MARKER_LINE_LIST = 5, MARKER_SPHERE_LIST = 7, MARKER_TEXT_VIEW_FACING = 9 };
enum : int32_t { MARKER_ADD = 0, MARKER_DELETE = 2 };

struct Color
{
    double r, g, b, a;
    Color(double r = 0.0, double g = 0.0, double b = 0.0, double a = 0.0) : r(r), g(g), b(b), a(a) {}
};

struct Vec3
{
    double x, y, z;
    Vec3(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}
};

struct MarkerPrimitive
{
    int32_t     type{0};
    int32_t     action{MARKER_ADD};
    std::string ns;
    int32_t     id{0};
    std::string frame_id;
    int64_t     stamp_ns{0};
    bool        frame_locked{false};
    Vec3        scale;
    Vec3        pose_position;
    double      pose_orientation_w{1.0};
    double      pose_orientation_x{0.0};
    double      pose_orientation_y{0.0};
    double      pose_orientation_z{0.0};
    Color       color;
    std::vector<Vec3>  points;
    std::vector<Color> colors;
    std::string text;
};

struct MarkerConfig
{
    std::string frame_base{"base_link"};
    bool showMetaInformation{false};
    bool showAbsoluteDoppler{false};
};

struct MarkerMeas { bool available{false}; double value{0.0}; };
struct MarkerDetection
{
    MarkerMeas range, azimuth, elevation, radial_speed, rcs, power, noise;
};
struct MarkerInput
{
    int64_t     stamp_ns{0};
    std::string frame_id;
    std::vector<MarkerDetection> detections;
};

// Resolved by the wrapper (tf2 lookup base_link<-sensor, odometry) for the
// absolute-Doppler path.
struct SensorTransform { double tx{0.0}, ty{0.0}, tz{0.0}; double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0}; };
struct Dynamics        { double speed{0.0}, yawrate{0.0}; };

// Build all marker layers for one detection record. When cfg.showAbsoluteDoppler
// and xform+dyn are non-null the ego-motion-compensated colouring is used,
// otherwise the static per-IP colouring. The wrapper resolves the transform and
// odometry (and handles the "discard measurement" cases) before calling.
std::vector<MarkerPrimitive> buildMarkers(const MarkerInput& in, const MarkerConfig& cfg,
                                          const SensorTransform* xform, const Dynamics* dyn,
                                          const LogFn& log);

} // namespace radar_marker
