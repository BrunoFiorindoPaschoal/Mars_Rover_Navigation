#include "mars_yard_sim_plugins/teleport_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace mars_yard_sim_plugins
{

namespace
{

std::string namespacedParamName(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
  const std::string sub_ns = node->get_sub_namespace();
  return sub_ns.empty() ? name : ("mujoco_plugins." + sub_ns + "." + name);
}

template <typename T>
T declareOrGetParameter(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value)
{
  const std::string full_name = namespacedParamName(node, name);
  if (!node->has_parameter(full_name))
  {
    node->declare_parameter(full_name, default_value);
  }
  return node->get_parameter(full_name).get_value<T>();
}

}  // namespace

bool TeleportPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  model_ = model;
  const std::string sub_ns = node_->get_sub_namespace();
  logger_ = sub_ns.empty() ? node_->get_logger() : node_->get_logger().get_child(sub_ns);

  if (model_ == nullptr)
  {
    RCLCPP_ERROR(logger_, "TeleportPlugin received null mjModel.");
    return false;
  }

  const std::string topic = declareOrGetParameter<std::string>(node_, "topic", "/sim/teleport");
  const std::string done_topic = declareOrGetParameter<std::string>(node_, "done_topic", "/sim/teleport_done");
  const std::string free_joint_name = declareOrGetParameter<std::string>(node_, "free_joint", "base_free_joint");
  const std::string base_body_name = declareOrGetParameter<std::string>(node_, "base_body", "base_link");
  const std::string front_axle_joint_name =
      declareOrGetParameter<std::string>(node_, "front_axle_joint", "front_axle_joint");
  spawn_clearance_ = declareOrGetParameter<double>(node_, "spawn_clearance", 0.275);
  extra_clearance_ = declareOrGetParameter<double>(node_, "extra_clearance", 0.03);
  ray_start_z_ = declareOrGetParameter<double>(node_, "ray_start_z", 100.0);
  wheel_half_length_ = declareOrGetParameter<double>(node_, "wheel_half_length", 0.25);
  wheel_half_track_ = declareOrGetParameter<double>(node_, "wheel_half_track", 0.32);

  const std::vector<int64_t> default_geomgroups{ 1, 1, 0, 1, 0, 0 };
  const auto geomgroups_param =
      declareOrGetParameter<std::vector<int64_t>>(node_, "geomgroups", default_geomgroups);
  if (geomgroups_param.size() != static_cast<size_t>(mjNGROUP))
  {
    RCLCPP_ERROR(logger_, "Parameter 'geomgroups' must have length %d (got %zu).",
                 mjNGROUP, geomgroups_param.size());
    return false;
  }
  for (size_t i = 0; i < static_cast<size_t>(mjNGROUP); ++i)
  {
    if (geomgroups_param[i] != 0 && geomgroups_param[i] != 1)
    {
      RCLCPP_ERROR(logger_, "Parameter 'geomgroups[%zu]' must be 0 or 1 (got %ld).",
                   i, geomgroups_param[i]);
      return false;
    }
    geomgroup_[i] = static_cast<mjtByte>(geomgroups_param[i]);
  }

  free_joint_id_ = mj_name2id(model_, mjOBJ_JOINT, free_joint_name.c_str());
  if (free_joint_id_ < 0)
  {
    RCLCPP_ERROR(logger_, "Free joint '%s' not found in model.", free_joint_name.c_str());
    return false;
  }
  if (model_->jnt_type[free_joint_id_] != mjJNT_FREE)
  {
    RCLCPP_ERROR(logger_, "Joint '%s' is not a free joint.", free_joint_name.c_str());
    return false;
  }
  qpos_adr_ = model_->jnt_qposadr[free_joint_id_];
  qvel_adr_ = model_->jnt_dofadr[free_joint_id_];

  base_body_id_ = mj_name2id(model_, mjOBJ_BODY, base_body_name.c_str());
  if (base_body_id_ < 0)
  {
    base_body_id_ = model_->jnt_bodyid[free_joint_id_];
  }

  front_axle_joint_id_ = mj_name2id(model_, mjOBJ_JOINT, front_axle_joint_name.c_str());
  front_axle_qpos_adr_ =
      (front_axle_joint_id_ >= 0) ? model_->jnt_qposadr[front_axle_joint_id_] : -1;

  done_pub_raw_ = node_->create_publisher<std_msgs::msg::Header>(
      done_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
  done_pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Header>>(done_pub_raw_);

  teleport_sub_ = node_->create_subscription<geometry_msgs::msg::Pose2D>(
      topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](const geometry_msgs::msg::Pose2D::SharedPtr msg) { on_teleport_msg(msg); });

  RCLCPP_INFO(logger_,
              "TeleportPlugin initialized: sub='%s', done_pub='%s', free_joint='%s' (qpos=%d, qvel=%d), "
              "base_body=%d, clearance=%.3f+%.3f m, geomgroups=[%d,%d,%d,%d,%d,%d]",
              topic.c_str(), done_topic.c_str(), free_joint_name.c_str(), qpos_adr_, qvel_adr_,
              base_body_id_, spawn_clearance_, extra_clearance_,
              static_cast<int>(geomgroup_[0]), static_cast<int>(geomgroup_[1]), static_cast<int>(geomgroup_[2]),
              static_cast<int>(geomgroup_[3]), static_cast<int>(geomgroup_[4]), static_cast<int>(geomgroup_[5]));

  return true;
}

void TeleportPlugin::on_teleport_msg(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
  if (!std::isfinite(msg->x) || !std::isfinite(msg->y) || !std::isfinite(msg->theta))
  {
    RCLCPP_WARN(logger_, "Ignoring non-finite teleport request (x=%f, y=%f, theta=%f).",
                msg->x, msg->y, msg->theta);
    return;
  }
  std::lock_guard<std::mutex> lock(req_mutex_);
  req_x_ = msg->x;
  req_y_ = msg->y;
  req_theta_ = msg->theta;
  has_pending_ = true;
}

bool TeleportPlugin::try_publish_done(const std_msgs::msg::Header& msg)
{
  if (!done_pub_)
  {
    return false;
  }
#if REALTIME_TOOLS_VERSION_MAJOR > 2
  return done_pub_->try_publish(msg);
#else
  return done_pub_->tryPublish(msg);
#endif
}

void TeleportPlugin::pre_step(mjData* data)
{
  if (model_ == nullptr || data == nullptr || qpos_adr_ < 0)
  {
    return;
  }

  bool do_teleport = false;
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  {
    std::lock_guard<std::mutex> lock(req_mutex_);
    if (has_pending_)
    {
      do_teleport = true;
      x = req_x_;
      y = req_y_;
      theta = req_theta_;
      has_pending_ = false;
    }
  }

  if (!do_teleport)
  {
    return;
  }

  // Park the rover far below the terrain before raycasting so downward rays can never hit
  // child wheel geoms even when teleporting onto or near the rover's previous position.
  data->qpos[qpos_adr_ + 2] = -1000.0;
  mj_kinematics(model_, data);

  // Sample ground height at centre and all 4 wheel contact points rotated by theta,
  // taking the highest hit so no uphill wheel starts penetrating sloped terrain.
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  const std::array<std::array<double, 2>, 5> offsets{{
      { 0.0, 0.0 },
      { wheel_half_length_, wheel_half_track_ },
      { wheel_half_length_, -wheel_half_track_ },
      { -wheel_half_length_, wheel_half_track_ },
      { -wheel_half_length_, -wheel_half_track_ },
  }};

  const mjtNum ray_dir[3] = { 0.0, 0.0, -1.0 };
  double max_ground_z = -std::numeric_limits<double>::infinity();
  bool any_hit = false;

  for (const auto& off : offsets)
  {
    const mjtNum ray_pnt[3] = {
      static_cast<mjtNum>(x + c * off[0] - s * off[1]),
      static_cast<mjtNum>(y + s * off[0] + c * off[1]),
      static_cast<mjtNum>(ray_start_z_),
    };
    int geomid = -1;
    const mjtNum dist = mj_ray(model_, data, ray_pnt, ray_dir, geomgroup_.data(),
                               /*flg_static=*/1, base_body_id_, &geomid, /*normal=*/nullptr);
    if (dist >= 0.0)
    {
      const double hit_z = ray_start_z_ - static_cast<double>(dist);
      if (!any_hit || hit_z > max_ground_z)
      {
        max_ground_z = hit_z;
        any_hit = true;
      }
    }
  }

  if (!any_hit)
  {
    RCLCPP_WARN(logger_, "Teleport downward ray missed all terrain at (%.3f, %.3f); using z=0.", x, y);
    max_ground_z = 0.0;
  }

  const double target_z = max_ground_z + spawn_clearance_ + extra_clearance_;

  data->qpos[qpos_adr_ + 0] = static_cast<mjtNum>(x);
  data->qpos[qpos_adr_ + 1] = static_cast<mjtNum>(y);
  data->qpos[qpos_adr_ + 2] = static_cast<mjtNum>(target_z);
  data->qpos[qpos_adr_ + 3] = static_cast<mjtNum>(std::cos(0.5 * theta));
  data->qpos[qpos_adr_ + 4] = 0.0;
  data->qpos[qpos_adr_ + 5] = 0.0;
  data->qpos[qpos_adr_ + 6] = static_cast<mjtNum>(std::sin(0.5 * theta));

  // Level the passive front axle; keep wheel qpos untouched so diff_drive_controller
  // position feedback sees zero step discontinuity.
  if (front_axle_qpos_adr_ >= 0)
  {
    data->qpos[front_axle_qpos_adr_] = 0.0;
  }

  // Zero free-joint, axle, and all wheel velocities, accelerations, warmstart, and actuator commands.
  mju_zero(data->qvel, model_->nv);
  mju_zero(data->qacc, model_->nv);
  mju_zero(data->qacc_warmstart, model_->nv);
  mju_zero(data->ctrl, model_->nu);

  mj_forward(model_, data);

  const double sim_time = static_cast<double>(data->time);
  const int64_t sim_ns = std::max<int64_t>(0, static_cast<int64_t>(std::llround(sim_time * 1e9)));
  std_msgs::msg::Header done_msg;
  done_msg.stamp.sec = static_cast<int32_t>(sim_ns / 1000000000LL);
  done_msg.stamp.nanosec = static_cast<uint32_t>(sim_ns % 1000000000LL);
  done_msg.frame_id = "map";

  if (!try_publish_done(done_msg))
  {
    std::lock_guard<std::mutex> lock(done_mutex_);
    pending_done_msg_ = done_msg;
    done_pending_ = true;
  }
}

void TeleportPlugin::update_write(const mjModel* /*model*/, mjData* /*data*/)
{
  std_msgs::msg::Header msg;
  {
    std::lock_guard<std::mutex> lock(done_mutex_);
    if (!done_pending_)
    {
      return;
    }
    msg = pending_done_msg_;
  }
  if (try_publish_done(msg))
  {
    std::lock_guard<std::mutex> lock(done_mutex_);
    done_pending_ = false;
  }
}

void TeleportPlugin::on_reset(mjData* /*data*/)
{
  {
    std::lock_guard<std::mutex> lock(req_mutex_);
    has_pending_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(done_mutex_);
    done_pending_ = false;
  }
}

void TeleportPlugin::cleanup()
{
  teleport_sub_.reset();
  done_pub_.reset();
  done_pub_raw_.reset();
  node_.reset();
  model_ = nullptr;
}

}  // namespace mars_yard_sim_plugins

PLUGINLIB_EXPORT_CLASS(mars_yard_sim_plugins::TeleportPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
