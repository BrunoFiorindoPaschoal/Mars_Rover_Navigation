#include "mars_yard_sim_plugins/ray_scan_plugin.hpp"

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

bool RayScanPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  const std::string sub_ns = node_->get_sub_namespace();
  logger_ = sub_ns.empty() ? node_->get_logger() : node_->get_logger().get_child(sub_ns);

  if (model == nullptr)
  {
    RCLCPP_ERROR(logger_, "RayScanPlugin received null mjModel.");
    return false;
  }

  const std::string site_name = declareOrGetParameter<std::string>(node_, "site", "lidar");
  const std::string frame_name = declareOrGetParameter<std::string>(node_, "frame_name", "lidar_link");
  const std::string topic = declareOrGetParameter<std::string>(node_, "topic", "/scan");
  const int64_t num_rays_param = declareOrGetParameter<int64_t>(node_, "num_rays", 360);
  min_angle_ = declareOrGetParameter<double>(node_, "min_angle", -M_PI);
  max_angle_ = declareOrGetParameter<double>(node_, "max_angle", M_PI - M_PI / 180.0);
  range_min_ = declareOrGetParameter<double>(node_, "range_min", 0.05);
  range_max_ = declareOrGetParameter<double>(node_, "range_max", 10.0);
  publish_rate_ = declareOrGetParameter<double>(node_, "publish_rate", 10.0);
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

  if (site_name.empty())
  {
    RCLCPP_ERROR(logger_, "Parameter 'site' must be a non-empty MJCF site name.");
    return false;
  }

  site_id_ = mj_name2id(model, mjOBJ_SITE, site_name.c_str());
  if (site_id_ < 0)
  {
    RCLCPP_ERROR(logger_, "MJCF site '%s' not found in model.", site_name.c_str());
    return false;
  }
  body_exclude_ = model->site_bodyid[site_id_];

  if (num_rays_param < 1)
  {
    RCLCPP_ERROR(logger_, "Parameter 'num_rays' must be >= 1 (got %ld).", num_rays_param);
    return false;
  }
  if (publish_rate_ <= 0.0)
  {
    RCLCPP_ERROR(logger_, "Parameter 'publish_rate' must be > 0 (got %f).", publish_rate_);
    return false;
  }
  if (range_min_ < 0.0 || range_max_ <= range_min_)
  {
    RCLCPP_ERROR(logger_, "Invalid range limits: range_min=%f, range_max=%f.", range_min_, range_max_);
    return false;
  }
  if (num_rays_param > 1 && max_angle_ < min_angle_)
  {
    RCLCPP_ERROR(logger_, "Invalid angle limits: min_angle=%f > max_angle=%f.", min_angle_, max_angle_);
    return false;
  }

  num_rays_ = static_cast<int>(num_rays_param);
  scan_period_ = 1.0 / publish_rate_;
  has_published_ = false;
  last_scan_time_ = 0.0;
  reset_pending_.store(false, std::memory_order_relaxed);

  const double angle_increment =
      (num_rays_ > 1) ? ((max_angle_ - min_angle_) / static_cast<double>(num_rays_ - 1)) : 0.0;

  // Precompute unit ray directions in the site frame (horizontal plane z=0, CCW about +z from +x).
  local_dirs_.resize(static_cast<size_t>(num_rays_) * 3);
  world_dirs_.resize(static_cast<size_t>(num_rays_) * 3);
  geomids_.resize(static_cast<size_t>(num_rays_));
  dists_.resize(static_cast<size_t>(num_rays_));

  for (int i = 0; i < num_rays_; ++i)
  {
    const double theta = min_angle_ + static_cast<double>(i) * angle_increment;
    local_dirs_[3 * i + 0] = std::cos(theta);
    local_dirs_[3 * i + 1] = std::sin(theta);
    local_dirs_[3 * i + 2] = 0.0;
  }

  // Static LaserScan fields.
  scan_msg_.header.frame_id = frame_name;
  scan_msg_.angle_min = static_cast<float>(min_angle_);
  scan_msg_.angle_max = static_cast<float>(max_angle_);
  scan_msg_.angle_increment = static_cast<float>(angle_increment);
  scan_msg_.time_increment = 0.0f;
  scan_msg_.scan_time = static_cast<float>(scan_period_);
  scan_msg_.range_min = static_cast<float>(range_min_);
  scan_msg_.range_max = static_cast<float>(range_max_);
  scan_msg_.ranges.resize(static_cast<size_t>(num_rays_));
  scan_msg_.intensities.clear();

  scan_pub_raw_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
  scan_pub_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>>(scan_pub_raw_);

  RCLCPP_INFO(logger_,
              "RayScanPlugin initialized: site='%s' (id=%d, body_exclude=%d), frame='%s', topic='%s', "
              "num_rays=%d, angles=[%.4f, %.4f] rad (inc=%.5f), range=[%.2f, %.2f] m, rate=%.2f Hz, "
              "geomgroups=[%d,%d,%d,%d,%d,%d]",
              site_name.c_str(), site_id_, body_exclude_, frame_name.c_str(), topic.c_str(),
              num_rays_, min_angle_, max_angle_, angle_increment, range_min_, range_max_, publish_rate_,
              static_cast<int>(geomgroup_[0]), static_cast<int>(geomgroup_[1]), static_cast<int>(geomgroup_[2]),
              static_cast<int>(geomgroup_[3]), static_cast<int>(geomgroup_[4]), static_cast<int>(geomgroup_[5]));

  return true;
}

void RayScanPlugin::on_reset(mjData* /*data*/)
{
  reset_pending_.store(true, std::memory_order_release);
}

void RayScanPlugin::update_write(const mjModel* model, mjData* data)
{
  if (!scan_pub_ || model == nullptr || data == nullptr || site_id_ < 0)
  {
    return;
  }

  if (reset_pending_.exchange(false, std::memory_order_acq_rel))
  {
    has_published_ = false;
    last_scan_time_ = 0.0;
  }

  // Rate-limit strictly in simulation time using the snapshot's own timestamp.
  const double sim_time = static_cast<double>(data->time);
  if (has_published_ && sim_time >= last_scan_time_ &&
      (sim_time - last_scan_time_) < (scan_period_ - 1e-9))
  {
    return;
  }
  last_scan_time_ = sim_time;
  has_published_ = true;

  // Transform precomputed site-frame horizontal ray directions into world frame.
  const mjtNum* site_pos = data->site_xpos + 3 * site_id_;
  const mjtNum* site_mat = data->site_xmat + 9 * site_id_;
  for (int i = 0; i < num_rays_; ++i)
  {
    const mjtNum lx = local_dirs_[3 * i + 0];
    const mjtNum ly = local_dirs_[3 * i + 1];
    world_dirs_[3 * i + 0] = site_mat[0] * lx + site_mat[1] * ly;
    world_dirs_[3 * i + 1] = site_mat[3] * lx + site_mat[4] * ly;
    world_dirs_[3 * i + 2] = site_mat[6] * lx + site_mat[7] * ly;
  }

  // Cast all rays in a single mj_multiRay call on the borrowed post-step snapshot (no sim lock).
  // geomgroup_ selects collision groups (e.g. 0, 1, and 3 for terrain tiles) and skips visual-only group 2.
  const mjtNum cutoff = static_cast<mjtNum>(range_max_) + 1e-3;
  mj_multiRay(model, data, site_pos, world_dirs_.data(), geomgroup_.data(),
              /*flg_static=*/1, body_exclude_, geomids_.data(), dists_.data(),
              /*normal=*/nullptr, num_rays_, cutoff);

  // REP 117 encoding: miss or > range_max -> +inf; < range_min -> -inf.
  const float pos_inf = std::numeric_limits<float>::infinity();
  const float neg_inf = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < num_rays_; ++i)
  {
    const mjtNum d = dists_[i];
    if (d < 0.0 || d > range_max_)
    {
      scan_msg_.ranges[static_cast<size_t>(i)] = pos_inf;
    }
    else if (d < range_min_)
    {
      scan_msg_.ranges[static_cast<size_t>(i)] = neg_inf;
    }
    else
    {
      scan_msg_.ranges[static_cast<size_t>(i)] = static_cast<float>(d);
    }
  }

  const int64_t sim_ns = std::max<int64_t>(0, static_cast<int64_t>(std::llround(sim_time * 1e9)));
  scan_msg_.header.stamp.sec = static_cast<int32_t>(sim_ns / 1000000000LL);
  scan_msg_.header.stamp.nanosec = static_cast<uint32_t>(sim_ns % 1000000000LL);

#if REALTIME_TOOLS_VERSION_MAJOR > 2
  scan_pub_->try_publish(scan_msg_);
#else
  scan_pub_->tryPublish(scan_msg_);
#endif
}

void RayScanPlugin::cleanup()
{
  scan_pub_.reset();
  scan_pub_raw_.reset();
  node_.reset();
}

}  // namespace mars_yard_sim_plugins

PLUGINLIB_EXPORT_CLASS(mars_yard_sim_plugins::RayScanPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
