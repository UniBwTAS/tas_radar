#include "core/marker_core.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace radar_marker
{

namespace
{

// Convert HSV -> RGB (h in [0,360], s/v in [0,1], outputs in [0,1]).
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

Color hsvColor(float hue, float saturation, float lightness)
{
    float r = 0, g = 0, b = 0;
    HSVtoRGB(r, g, b, hue, saturation, lightness);
    Color c;
    c.r = r;
    c.g = g;
    c.b = b;
    c.a = 1.0;
    return c;
}

Vec3 project(const MarkerDetection& d)
{
    Vec3 p;
    p.x = d.range.value * std::cos(d.azimuth.value) * std::cos(d.elevation.value);
    p.y = d.range.value * std::sin(d.azimuth.value) * std::cos(d.elevation.value);
    p.z = d.range.value * std::sin(d.elevation.value);
    return p;
}

} // namespace

std::vector<MarkerPrimitive> buildMarkers(const MarkerInput& in, const MarkerConfig& cfg,
                                          const SensorTransform* xform, const Dynamics* dyn,
                                          const LogFn& log)
{
    std::vector<MarkerPrimitive> out;

    /// Get IP/ID from frame_id
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

    /// map sensor_ip <-> marker color
    float hue = 60.0f * (ip % 5);
    float saturation = 1.0;
    float lightness;
    if (ip < 10)      lightness = 0.25;
    else if (ip < 20) lightness = 0.5;
    else if (ip < 30) lightness = 0.75;
    else              lightness = 1.0;
    if (ip < 0)       lightness = 0.0;

    const Color hsv = hsvColor(hue, saturation, lightness);
    const std::string ns = ip >= 0 ? (std::string("Radar .") + std::to_string(ip)) : "unknown";

    /// --- SPHERE_LIST: detections ---
    MarkerPrimitive sphere;
    sphere.frame_id = in.frame_id;
    sphere.stamp_ns = in.stamp_ns;
    sphere.ns = ns;
    sphere.id = 1;
    sphere.type = MARKER_SPHERE_LIST;
    sphere.action = MARKER_ADD;
    sphere.frame_locked = true;
    sphere.scale = {0.3, 0.3, 0.3};
    sphere.pose_orientation_w = 1.0;
    sphere.points.reserve(in.detections.size());

    if (cfg.showAbsoluteDoppler && xform && dyn)
    {
        tf2::Quaternion q(xform->qx, xform->qy, xform->qz, xform->qw);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        const double speed = dyn->speed;
        const double yawrate = dyn->yawrate;

        // velocity at the sensor position
        tf2::Vector3 sensor_velocity(speed - yawrate * xform->ty, yawrate * xform->tx, 0);
        double sign = std::fabs(roll) > 0.2 ? -1 : +1;  // workaround 3D meas. for 2D odometries

        for (size_t i = 0; i < in.detections.size(); i += 1)
        {
            const MarkerDetection& d = in.detections.at(i);
            Vec3 point = project(d);

            // Doppler: rotate velocity vector to global azimuth angle
            tf2::Vector3 velocity_compensation = sensor_velocity.rotate(tf2::Vector3(0, 0, 1), -(sign * d.azimuth.value + yaw));
            double radial_velocity_compensation = velocity_compensation.x();
            double range_rate_absolute = d.radial_speed.value + radial_velocity_compensation;

            if (std::fabs(d.radial_speed.value) > 20)
                continue;   // SMS default radial filter

            if (std::fabs(range_rate_absolute) < (0.25 + speed * 0.2))
                continue;   // static-world detection, not shown

            sphere.scale = {1, 1, 1};

            float range = d.radial_speed.value / 20;
            if (range > 1)  range = 1;
            if (range < -1) range = -1;

            Color color;
            color.a = 1;
            color.b = 0;
            color.r = 1;
            color.g = 1;
            if (range > 0) color.r = 1 - range;
            else           color.g = 1 + range;

            sphere.points.push_back(point);
            sphere.colors.push_back(color);
        }
    }
    else    // static colors by IP map
    {
        sphere.color = hsv;
        for (size_t i = 0; i < in.detections.size(); i += 1)
            sphere.points.push_back(project(in.detections.at(i)));
    }
    out.push_back(sphere);

    /// --- CUBE: mounting location ---
    MarkerPrimitive cube;
    cube.frame_id = in.frame_id;
    cube.stamp_ns = in.stamp_ns;
    cube.ns = ns + "/S";
    cube.id = 0;
    cube.type = MARKER_CUBE;
    cube.action = MARKER_ADD;
    cube.frame_locked = true;
    cube.scale = {0.1, 0.2, 0.1};
    cube.pose_position = {0, 0, 0};
    cube.pose_orientation_w = 1.0;
    cube.color = hsv;
    out.push_back(cube);

    /// --- LINE_LIST: raw radial speed ---
    MarkerPrimitive lines;
    lines.frame_id = in.frame_id;
    lines.stamp_ns = in.stamp_ns;
    lines.ns = ns + "/D";
    lines.id = 2;
    lines.type = MARKER_LINE_LIST;
    lines.action = MARKER_ADD;
    lines.frame_locked = true;
    lines.scale = {0.02, 0, 0};
    lines.pose_position = {0, 0, 0};
    lines.pose_orientation_w = 1.0;
    lines.color = hsv;
    lines.color.a = 0.4;
    lines.points.reserve(in.detections.size() * 2);
    for (size_t i = 0; i < in.detections.size(); i += 1)
    {
        const MarkerDetection& d = in.detections.at(i);
        lines.points.push_back(project(d));

        double range_inOneSecond = d.range.value + d.radial_speed.value;
        Vec3 end;
        end.x = range_inOneSecond * std::cos(d.azimuth.value) * std::cos(d.elevation.value);
        end.y = range_inOneSecond * std::sin(d.azimuth.value) * std::cos(d.elevation.value);
        end.z = range_inOneSecond * std::sin(d.elevation.value);
        lines.points.push_back(end);
    }
    out.push_back(lines);

    /// --- TEXT meta information (debug only, high bandwidth) ---
    if (cfg.showMetaInformation)
    {
        int32_t id = 0;
        for (size_t i = 0; i < in.detections.size(); i += 1)
        {
            const MarkerDetection& d = in.detections.at(i);
            id += 1;

            std::stringstream text;
            if (d.rcs.available)   text << "rcs " << std::fixed << std::setprecision(3) << d.rcs.value << std::endl;
            if (d.power.available) text << "pow " << std::fixed << std::setprecision(1) << d.power.value << std::endl;
            if (d.noise.available) text << "nse " << std::fixed << std::setprecision(1) << d.noise.value << std::endl;

            MarkerPrimitive meta;
            meta.frame_id = in.frame_id;
            meta.stamp_ns = in.stamp_ns;
            meta.ns = ns + "/I";
            meta.id = id;
            meta.type = MARKER_TEXT_VIEW_FACING;
            meta.action = MARKER_ADD;
            meta.frame_locked = true;
            meta.scale = {0, 0, 0.2};
            meta.color.a = 0.9;
            meta.pose_orientation_w = 1.0;
            meta.text = text.str();
            meta.pose_position = project(d);
            out.push_back(meta);
        }

        // clear stale text markers from the previous cycle
        for (size_t i = in.detections.size(); i < in.detections.size() + 10; i += 1)
        {
            id += 1;
            MarkerPrimitive del;
            del.frame_id = in.frame_id;
            del.stamp_ns = in.stamp_ns;
            del.ns = ns + "/I";
            del.id = id;
            del.type = MARKER_TEXT_VIEW_FACING;
            del.action = MARKER_DELETE;
            del.frame_locked = true;
            del.pose_orientation_w = 1.0;
            out.push_back(del);
        }
    }

    return out;
}

} // namespace radar_marker
