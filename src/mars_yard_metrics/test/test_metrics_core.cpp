#include "mars_yard_metrics/metrics_core.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <vector>

namespace mars_yard_metrics
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(MetricsCoreTest, WrapAngleBoundariesAndMultiples)
{
  EXPECT_NEAR(wrap_angle(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrap_angle(kPi), kPi, 1e-12);
  EXPECT_NEAR(wrap_angle(-kPi), -kPi, 1e-12);
  EXPECT_NEAR(wrap_angle(kPi + 0.1), -kPi + 0.1, 1e-12);
  EXPECT_NEAR(wrap_angle(-kPi - 0.1), kPi - 0.1, 1e-12);
  EXPECT_NEAR(wrap_angle(2.0 * kPi), 0.0, 1e-12);
  EXPECT_NEAR(wrap_angle(-4.0 * kPi + 0.25), 0.25, 1e-12);
  EXPECT_TRUE(std::isnan(wrap_angle(std::numeric_limits<double>::quiet_NaN())));
}

TEST(MetricsCoreTest, QuatYawAndWorldToBodyRotation)
{
  // +90 deg yaw about +z: body +x aligns with world +y, body +y aligns with world -x.
  const double c = std::cos(0.25 * kPi);
  const double s = std::sin(0.25 * kPi);
  const Quat q_yaw90{c, 0.0, 0.0, s};

  EXPECT_NEAR(yaw_from_quat(q_yaw90), 0.5 * kPi, 1e-12);

  const Vec3 v_body = rotate_world_to_body(q_yaw90, Vec3{0.0, 2.0, -0.5});
  EXPECT_NEAR(v_body.x, 2.0, 1e-12);
  EXPECT_NEAR(v_body.y, 0.0, 1e-12);
  EXPECT_NEAR(v_body.z, -0.5, 1e-12);

  // Pitch nose-down +90 deg about +y: body +x aligns with world -z.
  const Quat q_pitch90{c, 0.0, s, 0.0};
  const Vec3 v_pitched = rotate_world_to_body(q_pitch90, Vec3{0.0, 0.0, -3.0});
  EXPECT_NEAR(v_pitched.x, 3.0, 1e-12);
  EXPECT_NEAR(v_pitched.y, 0.0, 1e-12);
  EXPECT_NEAR(v_pitched.z, 0.0, 1e-12);
}

TEST(MetricsCoreTest, RelativePosePlanarTransform)
{
  const Pose2D origin{1.0, 2.0, 0.5 * kPi};
  // 3 m forward along origin +x (world +y) and 1 m left along origin +y (world -x).
  const Pose2D pose{0.0, 5.0, -0.75 * kPi};

  const Pose2D rel = relative_pose(origin, pose);
  EXPECT_NEAR(rel.x, 3.0, 1e-12);
  EXPECT_NEAR(rel.y, 1.0, 1e-12);
  // -135 deg - 90 deg = -225 deg -> wraps to +135 deg.
  EXPECT_NEAR(rel.yaw, 0.75 * kPi, 1e-12);
}

TEST(MetricsCoreTest, DriftPositionAndWrappedYaw)
{
  const Pose2D est{4.0, -3.0, -kPi + 0.05};
  const Pose2D truth{1.0, 1.0, kPi - 0.05};

  const DriftResult d = drift(est, truth);
  EXPECT_NEAR(d.pos_err, 5.0, 1e-12);
  // est is 0.1 rad ahead of truth across the +/-pi seam.
  EXPECT_NEAR(d.yaw_err, 0.1, 1e-12);
}

TEST(MetricsCoreTest, CrossTrackErrorEdgeCases)
{
  // Empty path -> NaN
  EXPECT_TRUE(std::isnan(cross_track_error(1.0, 2.0, {})));

  // Single-point path -> Euclidean distance to point
  EXPECT_NEAR(cross_track_error(4.0, 6.0, {Point2D{1.0, 2.0}}), 5.0, 1e-12);

  // Two-segment L-shaped polyline: (0,0) -> (10,0) -> (10,10)
  const std::vector<Point2D> path{{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}};
  // Interior projection on first segment
  EXPECT_NEAR(cross_track_error(4.0, 0.3, path), 0.3, 1e-12);
  // Beyond start of first segment
  EXPECT_NEAR(cross_track_error(-3.0, 4.0, path), 5.0, 1e-12);
  // Beyond end of last segment
  EXPECT_NEAR(cross_track_error(13.0, 14.0, path), 5.0, 1e-12);
  // Closer to second segment than first
  EXPECT_NEAR(cross_track_error(9.5, 6.0, path), 0.5, 1e-12);

  // Degenerate zero-length segment
  const std::vector<Point2D> degenerate{{2.0, 3.0}, {2.0, 3.0}};
  EXPECT_NEAR(cross_track_error(5.0, 7.0, degenerate), 5.0, 1e-12);
}

TEST(MetricsCoreTest, SlipForwardZeroReverseAndInPlaceTurn)
{
  // Mean of 4 wheels * radius
  EXPECT_NEAR(wheel_linear_velocity({4.0, 4.0, 4.0, 4.0}, 0.12), 0.48, 1e-12);
  EXPECT_NEAR(wheel_linear_velocity({-4.0, 4.0, -4.0, 4.0}, 0.12), 0.0, 1e-12);

  // Instantaneous slip
  EXPECT_TRUE(std::isnan(instantaneous_slip(0.0, 0.0)));
  EXPECT_TRUE(std::isnan(instantaneous_slip(0.019, 0.015)));
  EXPECT_NEAR(instantaneous_slip(0.4, 0.36), 0.1, 1e-12);
  // Reverse driving: both negative -> positive slip ratio
  EXPECT_NEAR(instantaneous_slip(-0.4, -0.36), 0.1, 1e-12);

  // Empty accumulator -> 0 slip ratio
  SlipAccumulator acc;
  EXPECT_DOUBLE_EQ(acc.slip_ratio(), 0.0);
  EXPECT_DOUBLE_EQ(acc.lateral_slide(), 0.0);

  // Below threshold (in-place turn with lateral downhill slide): ratio unaffected, lateral_slide accumulates
  acc.update(2.0, 0.0, 0.05, -0.25);
  EXPECT_DOUBLE_EQ(acc.wheel_distance(), 0.0);
  EXPECT_DOUBLE_EQ(acc.slip_ratio(), 0.0);
  EXPECT_NEAR(acc.lateral_slide(), 0.5, 1e-12);

  // Forward driving: 1 s at 0.4 m/s wheel, 0.38 m/s true -> 5% slip
  acc.update(1.0, 0.4, 0.38, 0.02);
  EXPECT_NEAR(acc.wheel_distance(), 0.4, 1e-12);
  EXPECT_NEAR(acc.true_long_distance(), 0.38, 1e-12);
  EXPECT_NEAR(acc.slip_ratio(), 0.05, 1e-12);
  EXPECT_NEAR(acc.lateral_slide(), 0.52, 1e-12);

  // Reverse driving: 1 s at -0.4 m/s wheel, -0.34 m/s true -> total d_w=0.8, d_t=0.72 -> 10% slip
  acc.update(1.0, -0.4, -0.34, -0.01);
  EXPECT_NEAR(acc.wheel_distance(), 0.8, 1e-12);
  EXPECT_NEAR(acc.true_long_distance(), 0.72, 1e-12);
  EXPECT_NEAR(acc.slip_ratio(), 0.1, 1e-12);
  EXPECT_NEAR(acc.lateral_slide(), 0.53, 1e-12);

  // Non-positive dt ignored
  acc.update(0.0, 0.4, 0.4, 0.4);
  acc.update(-0.1, 0.4, 0.4, 0.4);
  EXPECT_NEAR(acc.wheel_distance(), 0.8, 1e-12);

  acc.reset();
  EXPECT_DOUBLE_EQ(acc.wheel_distance(), 0.0);
  EXPECT_DOUBLE_EQ(acc.lateral_slide(), 0.0);
  EXPECT_DOUBLE_EQ(acc.slip_ratio(), 0.0);
}

TEST(MetricsCoreTest, EnergyAccumulatorSignsAndMonotonicity)
{
  EnergyAccumulator acc;
  EXPECT_DOUBLE_EQ(acc.energy(), 0.0);

  // Forward drive: positive torque and speed
  acc.update(0.5, {2.0, 2.0, 2.0, 2.0}, {3.0, 3.0, 3.0, 3.0});
  EXPECT_NEAR(acc.energy(), 12.0, 1e-12);

  // Mixed signs (reverse & braking): uses |tau * omega|, so strictly additive
  acc.update(0.25, {-4.0, 4.0, -2.0, 0.0}, {-2.0, -1.0, 3.0, 5.0});
  // Power = 8 + 4 + 6 + 0 = 18 W; * 0.25 s = 4.5 J -> total 16.5 J
  EXPECT_NEAR(acc.energy(), 16.5, 1e-12);

  // Non-positive dt ignored
  acc.update(0.0, {10.0}, {10.0});
  acc.update(-1.0, {10.0}, {10.0});
  EXPECT_NEAR(acc.energy(), 16.5, 1e-12);

  acc.reset();
  EXPECT_DOUBLE_EQ(acc.energy(), 0.0);
}

TEST(MetricsCoreTest, TrackerResetZeroesAccumulatorsAndReanchorsDriftReferences)
{
  MetricsTracker tracker;
  tracker.update_gt(Pose2D{0.0, 0.0, 0.0}, Vec3{0.0, 0.0, 0.0});
  tracker.update_ekf(Pose2D{0.0, 0.0, 0.0});
  tracker.update_wheel(Pose2D{0.0, 0.0, 0.0});
  tracker.set_plan({Point2D{0.0, 0.0}, Point2D{10.0, 0.0}});

  // Simulate driving and accumulating drift, distance, slip, lateral slide, and energy.
  tracker.update_gt(Pose2D{4.0, 0.5, 0.1}, Vec3{0.36, 0.05, 0.0});
  tracker.update_ekf(Pose2D{3.8, 0.2, 0.15});
  tracker.update_wheel(Pose2D{3.5, 0.0, 0.5});
  tracker.update_joints(1.0, {3.333, 3.333, 3.333, 3.333}, true, 0.12,
                        {4.0, 4.0, 4.0, 4.0}, {3.333, 3.333, 3.333, 3.333});

  const MetricsSnapshot before = tracker.snapshot();
  EXPECT_GT(before.true_distance, 4.0);
  EXPECT_GT(before.ekf_pos_drift, 0.3);
  EXPECT_GT(before.wheel_pos_drift, 0.5);
  EXPECT_GT(before.energy, 50.0);
  EXPECT_GT(before.lateral_slide, 0.04);
  EXPECT_GT(before.slip_ratio, 0.05);
  EXPECT_NEAR(before.cross_track_error, 0.5, 1e-12);

  // Teleport rover to (5, -3, pi/2), EKF reset to (5, -3, pi/2), wheel odom un-reset at (3.5, 0.0, 0.5).
  tracker.update_gt(Pose2D{5.0, -3.0, 0.5 * kPi}, Vec3{0.0, 0.0, 0.0});
  tracker.update_ekf(Pose2D{5.0, -3.0, 0.5 * kPi});
  tracker.reset(/*reanchor_to_latest=*/true);

  const MetricsSnapshot after_reset = tracker.snapshot();
  EXPECT_DOUBLE_EQ(after_reset.true_distance, 0.0);
  EXPECT_DOUBLE_EQ(after_reset.energy, 0.0);
  EXPECT_DOUBLE_EQ(after_reset.lateral_slide, 0.0);
  EXPECT_DOUBLE_EQ(after_reset.slip_ratio, 0.0);
  EXPECT_TRUE(std::isnan(after_reset.slip_instant));
  EXPECT_TRUE(std::isnan(after_reset.cross_track_error));
  EXPECT_NEAR(after_reset.ekf_pos_drift, 0.0, 1e-12);
  EXPECT_NEAR(after_reset.ekf_yaw_drift, 0.0, 1e-12);
  EXPECT_NEAR(after_reset.wheel_pos_drift, 0.0, 1e-12);
  EXPECT_NEAR(after_reset.wheel_yaw_drift, 0.0, 1e-12);

  // Drive 2 m along new heading (+y in world from (5, -3)) and verify relative drift + map-frame CTE.
  tracker.set_plan({Point2D{5.0, -3.0}, Point2D{5.0, 5.0}});
  tracker.update_gt(Pose2D{5.1, -1.0, 0.5 * kPi}, Vec3{0.4, 0.0, 0.0});
  tracker.update_ekf(Pose2D{5.0, -1.0, 0.5 * kPi});
  // Wheel odom advances 2.0 m along its own local +x (heading 0.5 rad).
  tracker.update_wheel(Pose2D{3.5 + 2.0 * std::cos(0.5), 2.0 * std::sin(0.5), 0.5});

  const MetricsSnapshot after_drive = tracker.snapshot();
  EXPECT_NEAR(after_drive.true_distance, std::hypot(0.1, 2.0), 1e-12);
  EXPECT_NEAR(after_drive.ekf_pos_drift, 0.1, 1e-12);
  EXPECT_NEAR(after_drive.ekf_yaw_drift, 0.0, 1e-12);
  EXPECT_NEAR(after_drive.wheel_pos_drift, 0.1, 1e-12);
  EXPECT_NEAR(after_drive.cross_track_error, 0.1, 1e-12);
}

}  // namespace
}  // namespace mars_yard_metrics
