#include "mars_yard_metrics/metrics_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mars_yard_metrics
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
}  // namespace

double wrap_angle(double angle)
{
  if (!std::isfinite(angle))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::remainder(angle, kTwoPi);
}

double yaw_from_quat(const Quat& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

Vec3 rotate_world_to_body(const Quat& q_world_body, const Vec3& v_world)
{
  const double norm = std::sqrt(q_world_body.w * q_world_body.w +
                                q_world_body.x * q_world_body.x +
                                q_world_body.y * q_world_body.y +
                                q_world_body.z * q_world_body.z);
  if (!(norm > 0.0))
  {
    return v_world;
  }

  const double inv = 1.0 / norm;
  const double w = q_world_body.w * inv;
  const double x = q_world_body.x * inv;
  const double y = q_world_body.y * inv;
  const double z = q_world_body.z * inv;

  // Apply conjugate quaternion q* = (w, -u) to v_world: v_body = v + w*t + cross(t, u),
  // where t = 2 * cross(v, u).
  const double tx = 2.0 * (v_world.y * z - v_world.z * y);
  const double ty = 2.0 * (v_world.z * x - v_world.x * z);
  const double tz = 2.0 * (v_world.x * y - v_world.y * x);

  return Vec3{
    v_world.x + w * tx + (ty * z - tz * y),
    v_world.y + w * ty + (tz * x - tx * z),
    v_world.z + w * tz + (tx * y - ty * x),
  };
}

Pose2D relative_pose(const Pose2D& origin, const Pose2D& pose)
{
  const double dx = pose.x - origin.x;
  const double dy = pose.y - origin.y;
  const double c = std::cos(origin.yaw);
  const double s = std::sin(origin.yaw);
  return Pose2D{
    c * dx + s * dy,
    -s * dx + c * dy,
    wrap_angle(pose.yaw - origin.yaw),
  };
}

DriftResult drift(const Pose2D& est, const Pose2D& truth)
{
  return DriftResult{
    std::hypot(est.x - truth.x, est.y - truth.y),
    wrap_angle(est.yaw - truth.yaw),
  };
}

double cross_track_error(double px, double py, const std::vector<Point2D>& polyline)
{
  if (polyline.empty())
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (polyline.size() == 1)
  {
    return std::hypot(px - polyline[0].x, py - polyline[0].y);
  }

  double min_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i + 1 < polyline.size(); ++i)
  {
    const double ax = polyline[i].x;
    const double ay = polyline[i].y;
    const double vx = polyline[i + 1].x - ax;
    const double vy = polyline[i + 1].y - ay;
    const double len2 = vx * vx + vy * vy;

    double dist = 0.0;
    if (len2 <= 0.0)
    {
      dist = std::hypot(px - ax, py - ay);
    }
    else
    {
      const double t = std::clamp(((px - ax) * vx + (py - ay) * vy) / len2, 0.0, 1.0);
      dist = std::hypot(px - (ax + t * vx), py - (ay + t * vy));
    }
    if (dist < min_dist)
    {
      min_dist = dist;
    }
  }
  return min_dist;
}

double wheel_linear_velocity(const std::array<double, 4>& wheel_omegas, double wheel_radius)
{
  return wheel_radius * 0.25 * (wheel_omegas[0] + wheel_omegas[1] + wheel_omegas[2] + wheel_omegas[3]);
}

double instantaneous_slip(double v_wheel, double v_true_x, double min_wheel_speed)
{
  if (!std::isfinite(v_wheel) || !std::isfinite(v_true_x) || std::abs(v_wheel) < min_wheel_speed)
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return 1.0 - (v_true_x / v_wheel);
}

SlipAccumulator::SlipAccumulator(double min_wheel_speed)
  : min_wheel_speed_(min_wheel_speed)
{
}

void SlipAccumulator::update(double dt, double v_wheel, double v_true_x, double v_true_y)
{
  if (!std::isfinite(dt) || dt <= 0.0)
  {
    return;
  }

  // Always accumulate lateral slide so downhill sliding during in-place turns (v_wheel ~ 0) is visible.
  if (std::isfinite(v_true_y))
  {
    lateral_slide_ += std::abs(v_true_y) * dt;
  }

  if (std::isfinite(v_wheel) && std::isfinite(v_true_x) && std::abs(v_wheel) > min_wheel_speed_)
  {
    wheel_distance_ += std::abs(v_wheel) * dt;
    true_long_distance_ += std::abs(v_true_x) * dt;
  }
}

void SlipAccumulator::reset()
{
  wheel_distance_ = 0.0;
  true_long_distance_ = 0.0;
  lateral_slide_ = 0.0;
}

double SlipAccumulator::slip_ratio() const
{
  if (wheel_distance_ <= 0.0)
  {
    return 0.0;
  }
  return 1.0 - (true_long_distance_ / wheel_distance_);
}

void EnergyAccumulator::update(double dt, const std::vector<double>& torques, const std::vector<double>& omegas)
{
  if (!std::isfinite(dt) || dt <= 0.0)
  {
    return;
  }
  const size_t n = std::min(torques.size(), omegas.size());
  double power = 0.0;
  for (size_t i = 0; i < n; ++i)
  {
    if (std::isfinite(torques[i]) && std::isfinite(omegas[i]))
    {
      power += std::abs(torques[i] * omegas[i]);
    }
  }
  energy_ += power * dt;
}

void EnergyAccumulator::reset()
{
  energy_ = 0.0;
}

}  // namespace mars_yard_metrics
