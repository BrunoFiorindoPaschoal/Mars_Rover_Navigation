#ifndef MARS_YARD_SIM_PLUGINS__RAY_SCAN_PLUGIN_HPP_
#define MARS_YARD_SIM_PLUGINS__RAY_SCAN_PLUGIN_HPP_

#include <array>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace mars_yard_sim_plugins
{

/**
 * @brief 2D horizontal lidar plugin using MuJoCo's mj_multiRay at a fixed sim-time rate.
 *
 * Casts `num_rays` rays counter-clockwise about the configured MJCF site's +z axis,
 * starting at `min_angle` measured from the site's +x axis, and publishes a REP-117
 * compliant sensor_msgs/msg/LaserScan message.
 */
class RayScanPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update_write(const mjModel* model, mjData* data) override;
  void on_reset(mjData* data) override;
  void cleanup() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("RayScanPlugin") };

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> scan_pub_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>> scan_pub_;
  sensor_msgs::msg::LaserScan scan_msg_;

  int site_id_{ -1 };
  int body_exclude_{ -1 };
  int num_rays_{ 360 };
  double min_angle_{ -M_PI };
  double max_angle_{ M_PI - M_PI / 180.0 };
  double range_min_{ 0.05 };
  double range_max_{ 10.0 };
  double publish_rate_{ 10.0 };
  double scan_period_{ 0.1 };

  bool has_published_{ false };
  double last_scan_time_{ 0.0 };
  std::atomic<bool> reset_pending_{ false };

  // Include collision groups 0, 1, and 3 (terrain collision tiles); exclude visual-only group 2.
  std::array<mjtByte, mjNGROUP> geomgroup_{ 1, 1, 0, 1, 0, 0 };

  // Precomputed unit ray directions in site frame, plus scratch buffers for mj_multiRay.
  std::vector<mjtNum> local_dirs_;
  std::vector<mjtNum> world_dirs_;
  std::vector<int> geomids_;
  std::vector<mjtNum> dists_;
};

}  // namespace mars_yard_sim_plugins

#endif  // MARS_YARD_SIM_PLUGINS__RAY_SCAN_PLUGIN_HPP_
