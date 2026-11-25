#include "mars_yard_sim_plugins/ray_scan_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/point_field.hpp>

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
  const std::string cloud_topic = declareOrGetParameter<std::string>(node_, "cloud_topic", "/points");
  const int64_t num_rays_param = declareOrGetParameter<int64_t>(node_, "num_rays", 360);
  const int64_t num_rings_param = declareOrGetParameter<int64_t>(node_, "num_rings", 1);
  min_angle_ = declareOrGetParameter<double>(node_, "min_angle", -M_PI);
  max_angle_ = declareOrGetParameter<double>(node_, "max_angle", M_PI - M_PI / 180.0);
  min_elevation_ = declareOrGetParameter<double>(node_, "min_elevation", 0.0);
  max_elevation_ = declareOrGetParameter<double>(node_, "max_elevation", 0.0);
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
  if (num_rings_param < 1)
  {
    RCLCPP_ERROR(logger_, "Parameter 'num_rings' must be >= 1 (got %ld).", num_rings_param);
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
  if (num_rings_param > 1 && max_elevation_ < min_elevation_)
  {
    RCLCPP_ERROR(logger_, "Invalid elevation limits: min_elevation=%f > max_elevation=%f.",
                 min_elevation_, max_elevation_);
    return false;
  }

  num_rays_ = static_cast<int>(num_rays_param);
  num_rings_ = static_cast<int>(num_rings_param);
  total_rays_ = num_rings_ * num_rays_;
  publish_cloud_ = (num_rings_ > 1);
  publish_scan_ = (num_rings_ == 1) || !topic.empty();

  if (!publish_scan_ && !publish_cloud_)
  {
    RCLCPP_ERROR(logger_, "Neither LaserScan nor PointCloud2 output is enabled.");
    return false;
  }
  if (publish_scan_ && topic.empty())
  {
    RCLCPP_ERROR(logger_, "Parameter 'topic' must be non-empty when num_rings == 1.");
    return false;
  }
  if (publish_cloud_ && cloud_topic.empty())
  {
    RCLCPP_ERROR(logger_, "Parameter 'cloud_topic' must be non-empty when num_rings > 1.");
    return false;
  }

  scan_period_ = 1.0 / publish_rate_;
  has_published_ = false;
  last_scan_time_ = 0.0;
  reset_pending_.store(false, std::memory_order_relaxed);

  const double angle_increment =
      (num_rays_ > 1) ? ((max_angle_ - min_angle_) / static_cast<double>(num_rays_ - 1)) : 0.0;
  const double elev_increment =
      (num_rings_ > 1) ? ((max_elevation_ - min_elevation_) / static_cast<double>(num_rings_ - 1)) : 0.0;

  // Precompute unit ray directions in site frame (ring-major: idx = ring * num_rays + az_idx).
  local_dirs_.resize(static_cast<size_t>(total_rays_) * 3);
  world_dirs_.resize(static_cast<size_t>(total_rays_) * 3);
  geomids_.resize(static_cast<size_t>(total_rays_));
  dists_.resize(static_cast<size_t>(total_rays_));

  horizontal_ring_ = 0;
  double best_abs_el = std::numeric_limits<double>::infinity();
  for (int r = 0; r < num_rings_; ++r)
  {
    const double el = (num_rings_ > 1) ? (min_elevation_ + static_cast<double>(r) * elev_increment) : 0.0;
    if (std::abs(el) < best_abs_el)
    {
      best_abs_el = std::abs(el);
      horizontal_ring_ = r;
    }
    const double cos_el = std::cos(el);
    const double sin_el = std::sin(el);
    for (int a = 0; a < num_rays_; ++a)
    {
      const double theta = min_angle_ + static_cast<double>(a) * angle_increment;
      const size_t idx = static_cast<size_t>(r * num_rays_ + a);
      local_dirs_[3 * idx + 0] = cos_el * std::cos(theta);
      local_dirs_[3 * idx + 1] = cos_el * std::sin(theta);
      local_dirs_[3 * idx + 2] = sin_el;
    }
  }

  if (publish_scan_)
  {
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
  }

  if (publish_cloud_)
  {
    cloud_msg_.header.frame_id = frame_name;
    cloud_msg_.height = 1;
    cloud_msg_.width = 0;
    cloud_msg_.is_bigendian = false;
    cloud_msg_.point_step = 12;
    cloud_msg_.row_step = 0;
    cloud_msg_.is_dense = true;
    cloud_msg_.fields.resize(3);
    const char* field_names[3] = { "x", "y", "z" };
    for (size_t f = 0; f < 3; ++f)
    {
      cloud_msg_.fields[f].name = field_names[f];
      cloud_msg_.fields[f].offset = static_cast<uint32_t>(f * 4);
      cloud_msg_.fields[f].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg_.fields[f].count = 1;
    }
    cloud_msg_.data.reserve(static_cast<size_t>(total_rays_) * 12);

    cloud_pub_raw_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    cloud_pub_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::PointCloud2>>(cloud_pub_raw_);
  }

  RCLCPP_INFO(logger_,
              "RayScanPlugin initialized: site='%s' (id=%d, body_exclude=%d), frame='%s', "
              "scan_topic='%s', cloud_topic='%s', rings=%d x rays=%d (total=%d), "
              "az=[%.4f, %.4f] rad, el=[%.4f, %.4f] rad, range=[%.2f, %.2f] m, rate=%.2f Hz, "
              "geomgroups=[%d,%d,%d,%d,%d,%d]",
              site_name.c_str(), site_id_, body_exclude_, frame_name.c_str(),
              publish_scan_ ? topic.c_str() : "(disabled)",
              publish_cloud_ ? cloud_topic.c_str() : "(disabled)",
              num_rings_, num_rays_, total_rays_,
              min_angle_, max_angle_, min_elevation_, max_elevation_,
              range_min_, range_max_, publish_rate_,
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
  if ((!scan_pub_ && !cloud_pub_) || model == nullptr || data == nullptr || site_id_ < 0)
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

  // Transform precomputed site-frame unit ray directions into world frame.
  const mjtNum* site_pos = data->site_xpos + 3 * site_id_;
  const mjtNum* site_mat = data->site_xmat + 9 * site_id_;
  for (int i = 0; i < total_rays_; ++i)
  {
    const mjtNum lx = local_dirs_[3 * i + 0];
    const mjtNum ly = local_dirs_[3 * i + 1];
    const mjtNum lz = local_dirs_[3 * i + 2];
    world_dirs_[3 * i + 0] = site_mat[0] * lx + site_mat[1] * ly + site_mat[2] * lz;
    world_dirs_[3 * i + 1] = site_mat[3] * lx + site_mat[4] * ly + site_mat[5] * lz;
    world_dirs_[3 * i + 2] = site_mat[6] * lx + site_mat[7] * ly + site_mat[8] * lz;
  }

  // Cast all rings x rays in ONE mj_multiRay call on the borrowed post-step snapshot.
  const mjtNum cutoff = static_cast<mjtNum>(range_max_) + 1e-3;
  mj_multiRay(model, data, site_pos, world_dirs_.data(), geomgroup_.data(),
              /*flg_static=*/1, body_exclude_, geomids_.data(), dists_.data(),
              /*normal=*/nullptr, total_rays_, cutoff);

  const int64_t sim_ns = std::max<int64_t>(0, static_cast<int64_t>(std::llround(sim_time * 1e9)));
  const int32_t stamp_sec = static_cast<int32_t>(sim_ns / 1000000000LL);
  const uint32_t stamp_nsec = static_cast<uint32_t>(sim_ns % 1000000000LL);

  if (publish_scan_ && scan_pub_)
  {
    // REP 117 encoding: miss or > range_max -> +inf; < range_min -> -inf.
    const float pos_inf = std::numeric_limits<float>::infinity();
    const float neg_inf = -std::numeric_limits<float>::infinity();
    const int ring_offset = horizontal_ring_ * num_rays_;
    for (int a = 0; a < num_rays_; ++a)
    {
      const mjtNum d = dists_[ring_offset + a];
      if (d < 0.0 || d > range_max_)
      {
        scan_msg_.ranges[static_cast<size_t>(a)] = pos_inf;
      }
      else if (d < range_min_)
      {
        scan_msg_.ranges[static_cast<size_t>(a)] = neg_inf;
      }
      else
      {
        scan_msg_.ranges[static_cast<size_t>(a)] = static_cast<float>(d);
      }
    }
    scan_msg_.header.stamp.sec = stamp_sec;
    scan_msg_.header.stamp.nanosec = stamp_nsec;

#if REALTIME_TOOLS_VERSION_MAJOR > 2
    scan_pub_->try_publish(scan_msg_);
#else
    scan_pub_->tryPublish(scan_msg_);
#endif
  }

  if (publish_cloud_ && cloud_pub_)
  {
    // Pack hits only (float32 x, y, z in lidar_link site frame) without heap reallocations.
    cloud_msg_.data.clear();
    uint32_t hit_count = 0;
    for (int i = 0; i < total_rays_; ++i)
    {
      const mjtNum d = dists_[i];
      if (d >= 0.0 && d >= range_min_ && d <= range_max_)
      {
        const float xyz[3] = {
          static_cast<float>(d * local_dirs_[3 * i + 0]),
          static_cast<float>(d * local_dirs_[3 * i + 1]),
          static_cast<float>(d * local_dirs_[3 * i + 2]),
        };
        const size_t prev_size = cloud_msg_.data.size();
        cloud_msg_.data.resize(prev_size + sizeof(xyz));
        std::memcpy(cloud_msg_.data.data() + prev_size, xyz, sizeof(xyz));
        ++hit_count;
      }
    }
    cloud_msg_.width = hit_count;
    cloud_msg_.row_step = hit_count * 12;
    cloud_msg_.header.stamp.sec = stamp_sec;
    cloud_msg_.header.stamp.nanosec = stamp_nsec;

#if REALTIME_TOOLS_VERSION_MAJOR > 2
    cloud_pub_->try_publish(cloud_msg_);
#else
    cloud_pub_->tryPublish(cloud_msg_);
#endif
  }
}

void RayScanPlugin::cleanup()
{
  cloud_pub_.reset();
  cloud_pub_raw_.reset();
  scan_pub_.reset();
  scan_pub_raw_.reset();
  node_.reset();
}

}  // namespace mars_yard_sim_plugins

PLUGINLIB_EXPORT_CLASS(mars_yard_sim_plugins::RayScanPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
