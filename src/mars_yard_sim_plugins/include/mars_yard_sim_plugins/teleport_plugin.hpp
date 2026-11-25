#ifndef MARS_YARD_SIM_PLUGINS__TELEPORT_PLUGIN_HPP_
#define MARS_YARD_SIM_PLUGINS__TELEPORT_PLUGIN_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/header.hpp>

namespace mars_yard_sim_plugins
{

/**
 * @brief In-simulation rover teleport plugin for fast benchmark resets.
 *
 * Subscribes to `/sim/teleport` (`geometry_msgs/msg/Pose2D`: x, y, theta in world frame),
 * latches the request under a mutex, and applies it atomically on the physics thread in
 * `pre_step()` before `mj_step()`. Ground height is sampled via downward `mj_ray` casts
 * against the terrain geomgroups (excluding rover bodies), placing `base_free_joint` at
 * `ground + spawn_clearance + extra_clearance` with yaw-only orientation, zeroed velocities,
 * zeroed `qacc_warmstart`, and `mj_forward()`. Publishes `std_msgs/msg/Header` on
 * `/sim/teleport_done` stamped with the post-teleport simulation time.
 */
class TeleportPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void pre_step(mjData* data) override;
  void update_write(const mjModel* model, mjData* data) override;
  void on_reset(mjData* data) override;
  void cleanup() override;

private:
  void on_teleport_msg(const geometry_msgs::msg::Pose2D::SharedPtr msg);
  bool try_publish_done(const std_msgs::msg::Header& msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("TeleportPlugin") };
  const mjModel* model_{ nullptr };

  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr teleport_sub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Header>> done_pub_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Header>> done_pub_;

  int free_joint_id_{ -1 };
  int qpos_adr_{ -1 };
  int qvel_adr_{ -1 };
  int base_body_id_{ -1 };
  int front_axle_joint_id_{ -1 };
  int front_axle_qpos_adr_{ -1 };

  double spawn_clearance_{ 0.275 };
  double extra_clearance_{ 0.03 };
  double ray_start_z_{ 100.0 };
  double wheel_half_length_{ 0.25 };
  double wheel_half_track_{ 0.32 };

  // Include collision groups 0, 1, and 3 (terrain collision tiles); exclude visual-only group 2.
  std::array<mjtByte, mjNGROUP> geomgroup_{ 1, 1, 0, 1, 0, 0 };

  std::mutex req_mutex_;
  bool has_pending_{ false };
  double req_x_{ 0.0 };
  double req_y_{ 0.0 };
  double req_theta_{ 0.0 };

  std::mutex done_mutex_;
  bool done_pending_{ false };
  std_msgs::msg::Header pending_done_msg_;
};

}  // namespace mars_yard_sim_plugins

#endif  // MARS_YARD_SIM_PLUGINS__TELEPORT_PLUGIN_HPP_
