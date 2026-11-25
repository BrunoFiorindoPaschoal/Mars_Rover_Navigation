#include "mars_yard_metrics/metrics_core.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace mars_yard_metrics
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

Pose2D pose2d_from_odom(const nav_msgs::msg::Odometry& msg)
{
  const auto& p = msg.pose.pose.position;
  const auto& q = msg.pose.pose.orientation;
  return Pose2D{p.x, p.y, yaw_from_quat(Quat{q.w, q.x, q.y, q.z})};
}

double stamp_to_sec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

}  // namespace

class MetricsNode : public rclcpp::Node
{
public:
  MetricsNode()
    : Node("metrics_node")
  {
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.12);
    const double publish_rate = this->declare_parameter<double>("publish_rate", 10.0);
    wheel_names_ = this->declare_parameter<std::vector<std::string>>(
      "wheel_joint_names",
      {"wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"});

    if (wheel_names_.size() != 4)
    {
      RCLCPP_WARN(get_logger(), "Expected 4 wheel_joint_names, got %zu; using defaults.", wheel_names_.size());
      wheel_names_ = {"wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"};
    }

    gt_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", 50);

    ekf_pos_drift_pub_ = this->create_publisher<std_msgs::msg::Float64>("/metrics/ekf_pos_drift", 10);
    ekf_yaw_drift_pub_ = this->create_publisher<std_msgs::msg::Float64>("/metrics/ekf_yaw_drift", 10);
    wheel_pos_drift_pub_ = this->create_publisher<std_msgs::msg::Float64>("/metrics/wheel_pos_drift", 10);
    wheel_yaw_drift_pub_ = this->create_publisher<std_msgs::msg::Float64>("/metrics/wheel_yaw_drift", 10);
    slip_ratio_pub_ = this->create_publisher<std_msgs::msg::Float64>("/metrics/slip_ratio", 10);
    slip_instant_pub_ = this->create_publisher<std_msgs::msg::Float64>("/metrics/slip_instant", 10);
    lateral_slide_pub_ = this->create_publisher<std_msgs::msg::Float64>("/metrics/lateral_slide", 10);
    true_distance_pub_ = this->create_publisher<std_msgs::msg::Float64>("/metrics/true_distance", 10);
    energy_pub_ = this->create_publisher<std_msgs::msg::Float64>("/metrics/energy", 10);
    cross_track_pub_ = this->create_publisher<std_msgs::msg::Float64>("/metrics/cross_track_error", 10);

    gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/simulator/floating_base_state", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_gt(msg); });
    ekf_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_ekf(msg); });
    wheel_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/diff_drive_controller/odom", 50,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_wheel(msg); });
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 100,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_states(msg); });
    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", 10,
      [this](const nav_msgs::msg::Path::SharedPtr msg) { on_plan(msg); });

    const double rate = (publish_rate > 0.0) ? publish_rate : 10.0;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate));
    timer_ = rclcpp::create_timer(this, this->get_clock(), period, [this]() { publish_metrics(); });

    RCLCPP_INFO(get_logger(), "metrics_node started (wheel_radius=%.3f m, publish_rate=%.1f Hz)",
                wheel_radius_, rate);
  }

private:
  void maybe_capture_origin()
  {
    if (!has_gt_origin_ && has_gt_ && (has_ekf_ || has_wheel_))
    {
      gt_origin_ = latest_gt_;
      prev_gt_for_dist_ = latest_gt_;
      has_gt_origin_ = true;
    }
  }

  void on_gt(const nav_msgs::msg::Odometry::SharedPtr& msg)
  {
    const auto& q_msg = msg->pose.pose.orientation;
    const Quat q{q_msg.w, q_msg.x, q_msg.y, q_msg.z};
    const auto& v_msg = msg->twist.twist.linear;
    const Vec3 v_body = rotate_world_to_body(q, Vec3{v_msg.x, v_msg.y, v_msg.z});

    // shortcut: assumes map == MuJoCo world and EKF starts at spawn; revisit when SLAM/AMCL moves map->odom (Phase 5)
    nav_msgs::msg::Odometry gt_odom = *msg;
    gt_odom.header.frame_id = "map";
    gt_odom.child_frame_id = "base_link";
    gt_odom.twist.twist.linear.x = v_body.x;
    gt_odom.twist.twist.linear.y = v_body.y;
    gt_odom.twist.twist.linear.z = v_body.z;
    // MujocoSystemInterface 0.1.4 copies free-joint qvel[3..5] directly; MuJoCo stores it in the body-local frame.
    gt_odom_pub_->publish(gt_odom);

    latest_gt_ = pose2d_from_odom(*msg);
    latest_v_body_ = v_body;
    has_gt_ = true;

    maybe_capture_origin();

    if (has_gt_origin_)
    {
      true_distance_ += std::hypot(latest_gt_.x - prev_gt_for_dist_.x,
                                   latest_gt_.y - prev_gt_for_dist_.y);
      prev_gt_for_dist_ = latest_gt_;
    }
  }

  void on_ekf(const nav_msgs::msg::Odometry::SharedPtr& msg)
  {
    latest_ekf_ = pose2d_from_odom(*msg);
    has_ekf_ = true;
    maybe_capture_origin();
  }

  void on_wheel(const nav_msgs::msg::Odometry::SharedPtr& msg)
  {
    latest_wheel_ = pose2d_from_odom(*msg);
    has_wheel_ = true;
    maybe_capture_origin();
  }

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr& msg)
  {
    const double t = stamp_to_sec(msg->header.stamp);
    if (!has_prev_js_time_)
    {
      prev_js_time_ = t;
      has_prev_js_time_ = true;
      return;
    }

    const double dt = t - prev_js_time_;
    prev_js_time_ = t;
    if (dt <= 0.0 || dt > 1.0)
    {
      return;
    }

    std::array<double, 4> omegas{0.0, 0.0, 0.0, 0.0};
    std::vector<double> torques;
    std::vector<double> effort_omegas;
    torques.reserve(4);
    effort_omegas.reserve(4);

    int matched_vel = 0;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      for (size_t w = 0; w < 4; ++w)
      {
        if (msg->name[i] == wheel_names_[w])
        {
          if (i < msg->velocity.size())
          {
            omegas[w] = msg->velocity[i];
            ++matched_vel;
            if (i < msg->effort.size() && std::isfinite(msg->effort[i]))
            {
              torques.push_back(msg->effort[i]);
              effort_omegas.push_back(msg->velocity[i]);
            }
          }
          break;
        }
      }
    }

    if (matched_vel == 4 && has_gt_)
    {
      const double v_wheel = wheel_linear_velocity(omegas, wheel_radius_);
      slip_acc_.update(dt, v_wheel, latest_v_body_.x, latest_v_body_.y);
      slip_instant_ = instantaneous_slip(v_wheel, latest_v_body_.x);
    }

    if (!torques.empty())
    {
      energy_acc_.update(dt, torques, effort_omegas);
    }
  }

  void on_plan(const nav_msgs::msg::Path::SharedPtr& msg)
  {
    plan_points_.clear();
    plan_points_.reserve(msg->poses.size());
    for (const auto& ps : msg->poses)
    {
      plan_points_.push_back(Point2D{ps.pose.position.x, ps.pose.position.y});
    }
  }

  static void publish_scalar(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& pub, double val)
  {
    std_msgs::msg::Float64 msg;
    msg.data = val;
    pub->publish(msg);
  }

  void publish_metrics()
  {
    double ekf_pos = kNaN;
    double ekf_yaw = kNaN;
    double wheel_pos = kNaN;
    double wheel_yaw = kNaN;
    double cte = kNaN;

    if (has_gt_origin_)
    {
      const Pose2D gt_rel = relative_pose(gt_origin_, latest_gt_);
      if (has_ekf_)
      {
        const DriftResult d = drift(latest_ekf_, gt_rel);
        ekf_pos = d.pos_err;
        ekf_yaw = d.yaw_err;
      }
      if (has_wheel_)
      {
        const DriftResult d = drift(latest_wheel_, gt_rel);
        wheel_pos = d.pos_err;
        wheel_yaw = d.yaw_err;
      }
      if (!plan_points_.empty())
      {
        cte = cross_track_error(gt_rel.x, gt_rel.y, plan_points_);
      }
    }

    publish_scalar(ekf_pos_drift_pub_, ekf_pos);
    publish_scalar(ekf_yaw_drift_pub_, ekf_yaw);
    publish_scalar(wheel_pos_drift_pub_, wheel_pos);
    publish_scalar(wheel_yaw_drift_pub_, wheel_yaw);
    publish_scalar(slip_ratio_pub_, slip_acc_.slip_ratio());
    publish_scalar(slip_instant_pub_, slip_instant_);
    publish_scalar(lateral_slide_pub_, slip_acc_.lateral_slide());
    publish_scalar(true_distance_pub_, true_distance_);
    publish_scalar(energy_pub_, energy_acc_.energy());
    publish_scalar(cross_track_pub_, cte);
  }

  double wheel_radius_{0.12};
  std::vector<std::string> wheel_names_;

  bool has_gt_{false};
  bool has_gt_origin_{false};
  bool has_ekf_{false};
  bool has_wheel_{false};
  bool has_prev_js_time_{false};

  Pose2D gt_origin_{};
  Pose2D prev_gt_for_dist_{};
  Pose2D latest_gt_{};
  Pose2D latest_ekf_{};
  Pose2D latest_wheel_{};
  Vec3 latest_v_body_{};
  double prev_js_time_{0.0};

  double true_distance_{0.0};
  double slip_instant_{kNaN};
  SlipAccumulator slip_acc_{};
  EnergyAccumulator energy_acc_{};
  std::vector<Point2D> plan_points_{};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ekf_pos_drift_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ekf_yaw_drift_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheel_pos_drift_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheel_yaw_drift_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slip_ratio_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slip_instant_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lateral_slide_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr true_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr energy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cross_track_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mars_yard_metrics

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mars_yard_metrics::MetricsNode>());
  rclcpp::shutdown();
  return 0;
}
