#include "mapping.h"

#include <algorithm>
#include <jsk_rviz_plugins/OverlayText.h>
#include <octomap_msgs/conversions.h>
#include <stdexcept>

namespace mapping {

Mapping::Mapping(ros::NodeHandle &nh, const Options &options)
    : nh_(nh), options_(options), lidar_aft_mapped_(new PointCloudType),
      radar_aft_mapped_(new RadarPointCloudType) {

  if (options_.lidar_filter_num <= 0)
    throw std::invalid_argument("lidar_filter_num must be positive");
  
  synchronizer_ = std::make_unique<Synchronizer>(
      options_.sync_options,
      [this](const SyncDiagnostics &info) { LogInvalidSync(info); });
  LoadPoseGT(options_.pose_gt_file);

  // Initialize path message
  path_.header.frame_id = "world";
  path_.poses.clear();

  // Initialize publishers
  pub_lidar_aft_mapped_ =
      nh_.advertise<sensor_msgs::PointCloud2>("lidar_aft_mapped", 10);
  pub_radar_aft_mapped_ =
      nh_.advertise<sensor_msgs::PointCloud2>("radar_aft_mapped", 10);
  pub_octomap_ =
      nh_.advertise<octomap_msgs::Octomap>("octomap_binary", 1, true);
  pub_update_stats_ =
      nh_.advertise<jsk_rviz_plugins::OverlayText>("mapper_update_stats", 1);

  pub_path_ = nh_.advertise<nav_msgs::Path>("/path", 10);
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

  // Initialize the mapper based on the specified type
  if (options_.mapper_type == 0) {
    mapper_ = std::make_unique<OctoMapper>(options_.octomap_options);
  } else {
    ROS_ERROR("Unsupported mapper type: %d", options_.mapper_type);
    throw std::runtime_error("Unsupported mapper type");
  }

  last_process_cpu_time_ms_ = GetProcessCpuTimeMs();
  last_process_wall_time_ms_ = GetSteadyTimeMs();

  try {
    run_thread_ = std::thread(&Mapping::Run, this);
    sub_lidar_ =
        nh_.subscribe(options_.lidar_topic, 10, &Mapping::LidarCBK, this);
    sub_radar_ =
        nh_.subscribe(options_.radar_topic, 10, &Mapping::RadarCBK, this);
  } catch (...) {
    Stop();
    throw;
  }
}

Mapping::~Mapping() { Stop(); }

void Mapping::Stop() {
  ROS_INFO("Stopping Mapping...");

  sub_lidar_.shutdown();
  sub_radar_.shutdown();
  if (synchronizer_)
    synchronizer_->Stop();
  
    if (run_thread_.joinable())
    run_thread_.join();
}

void Mapping::LoadPoseGT(const std::string &pose_gt_file) {
  std::ifstream ifs(pose_gt_file);
  if (!ifs.is_open()) {
    throw std::runtime_error("Failed to open GT file: " + pose_gt_file);
  }

  std::size_t loaded_poses = 0;
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty() || line[0] == '#')
      continue; // Skip empty lines and comments

    std::istringstream iss(line);
    double timestamp, x, y, z, qx, qy, qz, qw;
    if (!(iss >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw)) {
      ROS_WARN("Malformed GT line: %s", line.c_str());
      continue;
    }

    V3D t_flu_body(x, y, z);
    QD q_flu_body = QD(qw, qx, qy, qz);
    if (!q_flu_body.coeffs().allFinite() || q_flu_body.norm() < 1e-12) {
      ROS_WARN("Invalid GT quaternion: %s", line.c_str());
      continue;
    }
    q_flu_body.normalize();
    M3D R_flu_body = q_flu_body.toRotationMatrix();
    Pose6D pose_flu_body(timestamp, V3D::Zero(), V3D::Zero(), V3D::Zero(),
                         t_flu_body, R_flu_body);

    if (synchronizer_->AddPoseData(pose_flu_body)) {
      ++loaded_poses;
    } else {
      ROS_WARN("Rejected invalid or non-increasing GT pose: %s", line.c_str());
    }
  }

  ifs.close();
  if (loaded_poses == 0)
    throw std::runtime_error("GT trajectory contains no valid poses: " +
                             pose_gt_file);
  ROS_INFO("Loaded GT trajectory with %zu poses from %s", loaded_poses,
           pose_gt_file.c_str());
}

void Mapping::LidarCBK(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // Handle LIDAR point cloud message
  CloudPtr cloud(new PointCloudType);
  Preprocess::RSAiry2PCL(msg, cloud, options_.lidar_filter_num,
                         options_.lidar_blind);
  if (cloud->empty()) {
    ROS_WARN_THROTTLE(1.0, "LiDAR cloud is empty after preprocessing.");
    return;
  }

  if (std::any_of(cloud->begin(), cloud->end(), [](const PointType &point) {
        return !std::isfinite(point.curvature);
      })) {
    ROS_WARN_THROTTLE(1.0, "LiDAR cloud contains invalid point timestamps.");
    return;
  }

  std::sort(cloud->points.begin(), cloud->points.end(),
            [](const PointType &a, const PointType &b) {
              return a.curvature < b.curvature;
            });

  const double timestamp = msg->header.stamp.toSec();
  if (!synchronizer_->AddLidarData(timestamp, std::move(cloud))) {
    ROS_WARN_THROTTLE(1.0, "Synchronizer rejected LiDAR input at %.6f",
                      timestamp);
  }
}

void Mapping::RadarCBK(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // Handle Radar point cloud message
  double timestamp = msg->header.stamp.toSec();
  RadarCloudPtr cloud(new RadarPointCloudType);
  Preprocess::Radar2PCL(msg, cloud);

  if (!synchronizer_->AddRadarData(timestamp, std::move(cloud))) {
    ROS_WARN_THROTTLE(1.0, "Synchronizer rejected Radar input at %.6f",
                      timestamp);
  }
}

void Mapping::Run() {
  ROS_INFO("Mapping thread is running...");
  MeasureGroup group;
  while (synchronizer_->WaitPop(group)) {
    if (options_.verbose) {
      if (group.last_radar_time >= 0.0) {
        ROS_INFO("Sync window (last Radar, current Radar]: (%.9f, %.9f]",
                 group.last_radar_time, group.radar_time);
      } else {
        ROS_INFO("Sync window: first slice, no previous Radar; current Radar %.9f",
                 group.radar_time);
      }
      ROS_INFO("  LiDAR: %zu points, time [%.9f, %.9f]",
               group.lidar_cloud->size(),
               group.lidar_points_stamped.front().timestamp,
               group.lidar_points_stamped.back().timestamp);
      ROS_INFO("  Radar: %zu points, timestamp %.9f",
               group.radar_cloud->size(), group.radar_time);
      ROS_INFO("  Pose : %zu samples, time [%.9f, %.9f]",
               group.pose_data.size(), group.pose_data.front().offset,
               group.pose_data.back().offset);
    }
    const MapperInput input = Process(group);
    UpdateMap(input);
    PublishResults(group);
  }
}

void Mapping::LogInvalidSync(const SyncDiagnostics &info) const {
  std::ostringstream text;
  text << std::fixed << std::setprecision(9)
       << "Discarding sync window: "
       << (info.pose_count < info.required_poses ? "insufficient poses; " : "")
       << (!info.has_start_pose ? "missing start-boundary pose; " : "")
       << "required poses " << info.required_poses
       << "\n  Last Radar: ";
  if (info.last_radar_time >= 0.0)
    text << info.last_radar_time;
  else
    text << "none (first slice)";
  text << "\n  Window reference: " << info.window_begin_time
       << "\n  LiDAR: " << info.lidar_points << " points, time ["
       << info.lidar_begin_time << ", " << info.lidar_end_time << "]"
       << "\n  Radar: " << info.radar_points << " points, timestamp " << info.radar_time
       << "\n  Pose : " << info.pose_count << " samples";
  if (info.pose_count > 0)
    text << ", time [" << info.pose_begin_time << ", " << info.pose_end_time << "]";
  else
    text << ", no pose at/before Radar; first available " << info.first_available_pose_time;
  ROS_WARN_STREAM(text.str());
}

void Mapping::PublishResults(const MeasureGroup &group) {
  // Publish processed data
  sensor_msgs::PointCloud2 lidar_aft_mapped_msg;
  pcl::toROSMsg(*lidar_aft_mapped_, lidar_aft_mapped_msg);
  lidar_aft_mapped_msg.header.stamp = ros::Time(group.radar_time);
  lidar_aft_mapped_msg.header.frame_id = "world";
  pub_lidar_aft_mapped_.publish(lidar_aft_mapped_msg);

  sensor_msgs::PointCloud2 radar_aft_mapped_msg;
  pcl::toROSMsg(*radar_aft_mapped_, radar_aft_mapped_msg);
  radar_aft_mapped_msg.header.stamp = ros::Time(group.radar_time);
  radar_aft_mapped_msg.header.frame_id = "world";
  pub_radar_aft_mapped_.publish(radar_aft_mapped_msg);

  // Publish path
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time(group.radar_time);
  pose_msg.header.frame_id = "world";
  pose_msg.pose.position.x = group.pose_data.back().trans.x();
  pose_msg.pose.position.y = group.pose_data.back().trans.y();
  pose_msg.pose.position.z = group.pose_data.back().trans.z();
  QD q = QD(group.pose_data.back().rot);
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  pose_msg.pose.orientation.w = q.w();
  path_.header.stamp = ros::Time(group.radar_time);
  path_.poses.push_back(pose_msg);
  pub_path_.publish(path_);

  // Publish synchronized odometry
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time(group.radar_time);
  odom_msg.header.frame_id = "world";
  odom_msg.pose.pose.position.x = group.pose_data.back().trans.x();
  odom_msg.pose.pose.position.y = group.pose_data.back().trans.y();
  odom_msg.pose.pose.position.z = group.pose_data.back().trans.z();
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  pub_odom_.publish(odom_msg);
}

MapperInput Mapping::Process(MeasureGroup &group) {
  Preprocess::UndistortPcl(group, options_.R_bl, options_.t_bl);

  M3D R_wb = group.pose_data.back().rot;
  V3D t_wb = group.pose_data.back().trans;

  // Transform LiDAR and Radar observations into the world frame.
  M3D R_wl = R_wb * options_.R_bl;
  V3D t_wl = R_wb * options_.t_bl + t_wb;

  // Radar-to-world transformation uses the composed body/radar extrinsics.
  M3D R_wr = R_wb * options_.R_br;
  V3D t_wr = R_wb * options_.t_br + t_wb;

  Preprocess::TransformPointCloud(group.lidar_cloud, R_wl, t_wl,
                                  lidar_aft_mapped_);

  Preprocess::TransformRadarPointCloud(group.radar_cloud, R_wr, t_wr,
                                       radar_aft_mapped_);

  MapperInput input;
  input.timestamp = group.radar_time;
  input.body_pose = group.pose_data.back();
  input.lidar_origin = t_wl;
  input.radar_origin = t_wr;
  input.lidar_cloud = lidar_aft_mapped_;
  input.radar_cloud = radar_aft_mapped_;
  return input;
}

void Mapping::UpdateMap(const MapperInput &input) {
  MapperUpdateStats stats;
  stats.lidar_points = input.lidar_cloud ? input.lidar_cloud->size() : 0;
  stats.radar_points = input.radar_cloud ? input.radar_cloud->size() : 0;

  const auto wall_before = std::chrono::steady_clock::now();
  const double cpu_before_ms = GetThreadCpuTimeMs();

  mapper_->Update(input);

  const double cpu_after_ms = GetThreadCpuTimeMs();
  const auto wall_after = std::chrono::steady_clock::now();
  const std::int64_t rss_after = GetProcessRssBytes();

  stats.wall_time_ms =
      std::chrono::duration<double, std::milli>(wall_after - wall_before)
          .count();
  stats.thread_cpu_time_ms = cpu_after_ms - cpu_before_ms;
  if (stats.wall_time_ms > 0.0) {
    stats.thread_cpu_utilization_percent =
        100.0 * stats.thread_cpu_time_ms / stats.wall_time_ms;
  }

  const double process_cpu_time_ms = GetProcessCpuTimeMs();
  const double process_wall_time_ms = GetSteadyTimeMs();
  const double process_wall_delta_ms =
      process_wall_time_ms - last_process_wall_time_ms_;
  if (process_wall_delta_ms > 0.0) {
    stats.process_cpu_utilization_percent =
        100.0 * (process_cpu_time_ms - last_process_cpu_time_ms_) /
        process_wall_delta_ms;
  }
  last_process_cpu_time_ms_ = process_cpu_time_ms;
  last_process_wall_time_ms_ = process_wall_time_ms;

  stats.rss_bytes = rss_after;

  PublishUpdateStats(stats);

  PublishOctomap(ros::Time(input.timestamp));
}

void Mapping::PublishUpdateStats(const MapperUpdateStats &stats) {
  constexpr double kBytesPerMiB = 1024.0 * 1024.0;

  std::ostringstream text;
  text << std::fixed << std::setprecision(3) << "Mapper Update\n"
       << "Update wall       : " << stats.wall_time_ms << " ms\n"
       << "Update thread CPU : " << stats.thread_cpu_time_ms << " ms\n"
       << std::setprecision(1)
       << "Update thread use : " << stats.thread_cpu_utilization_percent
       << " %\n"
       << "Process CPU avg   : " << stats.process_cpu_utilization_percent
       << " %\n";
  if (stats.rss_bytes >= 0) {
    text << "RSS       : "
         << static_cast<double>(stats.rss_bytes) / kBytesPerMiB << " MiB\n";
  } else {
    text << "RSS       : unavailable\n";
  }
  text << "Points    : LiDAR " << stats.lidar_points << " | Radar "
       << stats.radar_points << " | Total "
       << stats.lidar_points + stats.radar_points;

  jsk_rviz_plugins::OverlayText message;
  message.action = jsk_rviz_plugins::OverlayText::ADD;
  message.width = 600;
  message.height = 195;
  message.left = 10;
  message.top = 10;
  message.line_width = 1;
  message.text_size = 14.0;
  message.font = "DejaVu Sans Mono";
  message.bg_color.r = 0.0;
  message.bg_color.g = 0.0;
  message.bg_color.b = 0.0;
  message.bg_color.a = 0.65;
  message.fg_color.r = 0.3;
  message.fg_color.g = 1.0;
  message.fg_color.b = 0.3;
  message.fg_color.a = 1.0;
  message.text = text.str();
  pub_update_stats_.publish(message);
}

void Mapping::PublishOctomap(const ros::Time &stamp) {
  if (!mapper_ || !mapper_->GetOctree())
    return;

  if (options_.map_publish_period > 0.0 && has_published_octomap_ &&
      stamp >= last_map_publish_stamp_ &&
      (stamp - last_map_publish_stamp_).toSec() < options_.map_publish_period) {
    return;
  }

  octomap_msgs::Octomap message;
  if (!octomap_msgs::binaryMapToMsg(*mapper_->GetOctree(), message)) {
    ROS_WARN_THROTTLE(1.0, "Failed to serialize OctoMap for visualization.");
    return;
  }

  message.header.stamp = stamp;
  message.header.frame_id = "world";
  pub_octomap_.publish(message);
  last_map_publish_stamp_ = stamp;
  has_published_octomap_ = true;
}
} // namespace mapping
