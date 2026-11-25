#pragma once

#include <array>
#include <vector>

namespace mars_yard_metrics
{

inline constexpr double kMinSlipWheelSpeed = 0.02;

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct DriftResult
{
  double pos_err{0.0};
  double yaw_err{0.0};
};

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quat
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

double wrap_angle(double angle);
double yaw_from_quat(const Quat& q);
Vec3 rotate_world_to_body(const Quat& q_world_body, const Vec3& v_world);

Pose2D relative_pose(const Pose2D& origin, const Pose2D& pose);
DriftResult drift(const Pose2D& est, const Pose2D& truth);
double cross_track_error(double px, double py, const std::vector<Point2D>& polyline);

double wheel_linear_velocity(const std::array<double, 4>& wheel_omegas, double wheel_radius);
double instantaneous_slip(double v_wheel, double v_true_x, double min_wheel_speed = kMinSlipWheelSpeed);

class SlipAccumulator
{
public:
  explicit SlipAccumulator(double min_wheel_speed = kMinSlipWheelSpeed);

  void update(double dt, double v_wheel, double v_true_x, double v_true_y);
  void reset();

  double wheel_distance() const { return wheel_distance_; }
  double true_long_distance() const { return true_long_distance_; }
  double lateral_slide() const { return lateral_slide_; }
  double slip_ratio() const;

private:
  double min_wheel_speed_{kMinSlipWheelSpeed};
  double wheel_distance_{0.0};
  double true_long_distance_{0.0};
  double lateral_slide_{0.0};
};

class EnergyAccumulator
{
public:
  void update(double dt, const std::vector<double>& torques, const std::vector<double>& omegas);
  void reset();

  double energy() const { return energy_; }

private:
  double energy_{0.0};
};

}  // namespace mars_yard_metrics
