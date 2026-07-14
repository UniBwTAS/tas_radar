#include "core/accumulate.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <stdexcept>

namespace radar_pointcloud
{

namespace
{

void HSVtoRGB(float& fR, float& fG, float& fB, float fH, float fS, float fV)
{
    float fC = fV * fS;
    float fHPrime = std::fmod(fH / 60.0f, 6.0f);
    float fX = fC * (1 - std::fabs(std::fmod(fHPrime, 2.0f) - 1));
    float fM = fV - fC;

    if      (0 <= fHPrime && fHPrime < 1) { fR = fC; fG = fX; fB = 0; }
    else if (1 <= fHPrime && fHPrime < 2) { fR = fX; fG = fC; fB = 0; }
    else if (2 <= fHPrime && fHPrime < 3) { fR = 0;  fG = fC; fB = fX; }
    else if (3 <= fHPrime && fHPrime < 4) { fR = 0;  fG = fX; fB = fC; }
    else if (4 <= fHPrime && fHPrime < 5) { fR = fX; fG = 0;  fB = fC; }
    else if (5 <= fHPrime && fHPrime < 6) { fR = fC; fG = 0;  fB = fX; }
    else                                  { fR = 0;  fG = 0;  fB = 0; }

    fR += fM;
    fG += fM;
    fB += fM;
}

} // namespace

Accumulator::Accumulator(Config config) : config_(std::move(config)) {}

PointCloudData Accumulator::ingest(const Input& in, const Transform& rearAxis, const Transform& odom,
                                   const Dynamics& dyn, const LogFn& log)
{
    /// Get IP/ID and colour
    int ip = -1;
    size_t idx = in.frame_id.find_last_of("/");
    if (idx != std::string::npos)
    {
        try { ip = std::stoi(in.frame_id.substr(idx + 1)); }
        catch (const std::invalid_argument&) { ip = -2; }
        catch (const std::out_of_range&)     { ip = -3; }
        catch (const std::exception&)        { ip = -4; }
    }
    if (ip < 0 && log)
        log(LogLevel::Warn, "Could not parse sensor IP in frame_id " + in.frame_id);

    float hue = 60.0f * (ip % 5);
    float saturation = 1.0;
    float lightness;
    if (ip < 10)      lightness = 0.25;
    else if (ip < 20) lightness = 0.5;
    else if (ip < 30) lightness = 0.75;
    else              lightness = 1.0;
    if (ip < 0)       lightness = 0.0;

    float cr = 0, cg = 0, cb = 0;
    HSVtoRGB(cr, cg, cb, hue, saturation, lightness);
    int color_binary = int(std::round(cr * 255.0)) << 16 | int(std::round(cg * 255.0)) << 8 | int(std::round(cb * 255.0)) << 0;
    const float color_hex_as_float = *reinterpret_cast<float*>(&color_binary);

    /// Ego-motion preparation (rear-axis transform)
    tf2::Quaternion q(rearAxis.qx, rearAxis.qy, rearAxis.qz, rearAxis.qw);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    const double speed = dyn.speed;
    const double yawrate = dyn.yawrate;
    tf2::Vector3 sensor_velocity(speed - yawrate * rearAxis.ty, yawrate * rearAxis.tx, 0);
    double sign = std::fabs(roll) > 0.2 ? -1 : +1;  // workaround 3D meas. for 2D odometries

    /// odom-frame transform for the detection points
    tf2::Transform odomTf(tf2::Quaternion(odom.qx, odom.qy, odom.qz, odom.qw),
                          tf2::Vector3(odom.tx, odom.ty, odom.tz));

    /// Clean queue
    if (!accumulation_.empty() && (accumulation_.back().timestamp_ns - in.stamp_ns) > 300000000LL)
        accumulation_ = std::deque<Shot>();

    const int64_t window_ns = static_cast<int64_t>(config_.accumulationTime * 1e9);
    while (!accumulation_.empty() && (in.stamp_ns - accumulation_.front().timestamp_ns) > window_ns)
        accumulation_.pop_front();

    /// Push detection shot
    Shot shot;
    shot.timestamp_ns = in.stamp_ns;
    shot.detections.reserve(in.detections.size());
    for (const Detection& d : in.detections)
    {
        MetaDetection md;

        // Cartesian coordinates in the sensor frame, transformed into frame_base
        tf2::Vector3 point(d.range * std::cos(d.azimuth) * std::cos(d.elevation),
                           d.range * std::sin(d.azimuth) * std::cos(d.elevation),
                           d.range * std::sin(d.elevation));
        tf2::Vector3 transformed = odomTf * point;
        md.x = transformed.x();
        md.y = transformed.y();
        md.z = transformed.z();

        // Doppler: rotate velocity vector to global azimuth angle
        tf2::Vector3 velocity_compensation = sensor_velocity.rotate(tf2::Vector3(0, 0, 1), -(sign * d.azimuth + yaw));
        double range_rate_absolute = d.radial_speed + velocity_compensation.x();

        md.absolute_radial_speed = range_rate_absolute;
        md.color_hex_as_float = color_hex_as_float;
        shot.detections.push_back(md);
    }
    accumulation_.push_back(shot);

    /// Assemble the accumulated cloud
    PointCloudData out;
    out.stamp_ns = in.stamp_ns;
    out.frame_id = config_.frame_base;
    for (const Shot& s : accumulation_)
        for (const MetaDetection& m : s.detections)
        {
            PclPoint p;
            p.x = m.x;
            p.y = m.y;
            p.z = m.z;
            p.absolute_radial_speed = m.absolute_radial_speed;
            p.rgb = m.color_hex_as_float;
            out.points.push_back(p);
        }
    return out;
}

} // namespace radar_pointcloud
