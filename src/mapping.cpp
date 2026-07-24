#include "mapping.h"

namespace mapping {

Mapping::Mapping(ros::NodeHandle &nh, const Options &options)
    : nh_(nh), options_(options) {
  LoadPoseGT(options_.pose_gt_file);

  sub_lidar_ =
      nh_.subscribe(options_.lidar_topic, 10, &Mapping::LidarCBK, this);
  sub_radar_ =
      nh_.subscribe(options_.radar_topic, 10, &Mapping::RadarCBK, this);

  pub_lidar_aft_mapped_ =
      nh_.advertise<sensor_msgs::PointCloud2>("lidar_aft_mapped", 10);
  pub_radar_aft_mapped_ =
      nh_.advertise<sensor_msgs::PointCloud2>("radar_aft_mapped", 10);

  pub_path_ = nh_.advertise<nav_msgs::Path>("/path", 10);
  pub_odom_sync_ = nh_.advertise<nav_msgs::Odometry>("/odom_sync", 10);
  pub_lidar_sync_ = nh_.advertise<sensor_msgs::PointCloud2>("/lidar_sync", 10);
  pub_radar_sync_ = nh_.advertise<sensor_msgs::PointCloud2>("/radar_sync", 10);

  run_thread_ = std::make_shared<std::thread>(&Mapping::Run, this);
}

Mapping::~Mapping() {
  ROS_INFO("Mapping thread is stopping...");
  if (run_thread_ && run_thread_->joinable()) {
    run_thread_->join();
    run_thread_ == nullptr;
  }
}

void Mapping::LoadPoseGT(const std::string &pose_gt_file) {
  std::ifstream ifs(pose_gt_file);
  if (!ifs.is_open()) {
    ROS_ERROR("Failed to open GT file: %s", pose_gt_file.c_str());
    return;
  }

  data_.pose_buffer.clear();
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
    M3D R_flu_body = q_flu_body.toRotationMatrix();
    Pose6D pose_flu_body(timestamp, V3D::Zero(), V3D::Zero(), V3D::Zero(),
                         t_flu_body, R_flu_body);

    data_.pose_buffer.push_back(pose_flu_body);
  }

  ifs.close();
  ROS_INFO("Loaded GT trajectory with %lu poses from %s",
           data_.pose_buffer.size(), pose_gt_file.c_str());
}

void Mapping::LidarCBK(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // Handle LIDAR point cloud message
  CloudPtr cloud(new PointCloudType);
  Preprocess::RSAiry2PCL(msg, cloud, options_.lidar_filter_num,
                         options_.lidar_blind);
  std::sort(cloud->points.begin(), cloud->points.end(),
            [](const PointType &a, const PointType &b) {
              return a.curvature < b.curvature;
            });

  // Extract timestamps for each point
  double timestamp = msg->header.stamp.toSec();
  std::deque<PointStamped> lidar_points_stamped;
  int cloud_size = cloud->points.size();
  for (int i = 0; i < cloud_size; i++) {
    PointStamped pt_stamped;
    pt_stamped.x = cloud->points[i].x;
    pt_stamped.y = cloud->points[i].y;
    pt_stamped.z = cloud->points[i].z;
    pt_stamped.intensity = cloud->points[i].intensity;
    pt_stamped.timestamp =
        timestamp + cloud->points[i].curvature / 1000.0; // convert back to `s`
    lidar_points_stamped.push_back(pt_stamped);
  }
  ROS_INFO("Processed LiDAR points %lu, timestamp from %.4f to %.4f",
           lidar_points_stamped.size(), lidar_points_stamped.front().timestamp,
           lidar_points_stamped.back().timestamp);

  // change data buffer
  data_.mtx.lock();

  // check timestamp order
  if (timestamp < data_.last_lidar_time) {
    ROS_ERROR("LiDAR time Sync ERROR");
    data_.lidar_buffer.clear();
  }

  // update data buffer
  // note: first radar time init
  if (data_.last_radar_time < 0) {
    ROS_WARN("Radar time not initialized yet, initializing to lidar timestamp");
    data_.last_radar_time = timestamp;
  }

  data_.last_lidar_time = timestamp;
  data_.lidar_buffer.emplace_back(timestamp, cloud);
  data_.lidar_points_stamped_buffer.insert(
      data_.lidar_points_stamped_buffer.end(), lidar_points_stamped.begin(),
      lidar_points_stamped.end());

  data_.mtx.unlock();
}

void Mapping::RadarCBK(const sensor_msgs::PointCloud2ConstPtr &msg) {
  // Handle Radar point cloud message
  double timestamp = msg->header.stamp.toSec();
  RadarCloudPtr cloud(new RadarPointCloudType);
  Preprocess::Radar2PCL(msg, cloud);

  // change data buffer
  data_.mtx.lock();

  if (timestamp < data_.last_radar_time) {
    ROS_ERROR("Radar time Sync ERROR");
    data_.radar_buffer.clear();
  }

  data_.radar_buffer.emplace_back(timestamp, cloud);
  data_.mtx.unlock();
}

void Mapping::Run() {
  ROS_INFO("Mapping thread is running...");
  ros::Rate rate(20); // 100 Hz
  while (ros::ok()) {
    if (SyncGroup()) {
      // Process synchronized data group
      if (options_.verbose) {
        ROS_INFO(
            "---------------------------------------------------------------"
            "-----------------");
        ROS_INFO("Synchronized from %.4f to %.4f", sync_data_.lidar_beg_time,
                 sync_data_.radar_time);
        ROS_INFO("========> Radar points %lu from %.4f",
                 sync_data_.radar_cloud->points.size(), sync_data_.radar_time);
        ROS_INFO("========> Pose data %lu from %.4f to %.4f",
                 sync_data_.pose_data.size(),
                 sync_data_.pose_data.front().offset,
                 sync_data_.pose_data.back().offset);
        ROS_INFO("========> LiDAR data %lu from %.4f to %.4f",
                 sync_data_.lidar_cloud->points.size(),
                 sync_data_.lidar_points_stamped.front().timestamp,
                 sync_data_.lidar_points_stamped.back().timestamp);
        ROS_INFO(
            "---------------------------------------------------------------"
            "-----------------");
      }

      Process();
    }

    rate.sleep();
  }
}

bool Mapping::SyncGroup() {
  data_.mtx.lock();

  // last_radar_time          radar_front_time
  //    |<--- LiDAR points consumed --->|
  //    |<------ Pose data consumed --->|
  //    |                               |
  //    +-------------------------------+
  //          一个 sync 时间窗口

  // step: 1 check empty
  if (data_.radar_buffer.empty() || data_.lidar_points_stamped_buffer.empty()) {
    data_.mtx.unlock();
    return false;
  }

  // step: 2 check lidar and imu over radar time
  if (data_.lidar_points_stamped_buffer.back().timestamp <=
      data_.radar_buffer.front().first) {
    ROS_WARN("LiDAR points are all before the first radar timestamp.");
    data_.mtx.unlock();
    return false;
  }

  // step: 3 clear radar which is useless before
  while (data_.lidar_points_stamped_buffer.front().timestamp >=
         data_.radar_buffer.front().first) {
    data_.radar_buffer.pop_front();

    if (data_.radar_buffer.empty()) {
      ROS_WARN(
          "Radar buffer is empty after clearing. Waiting for more radar data.");
      data_.mtx.unlock();
      return false;
    }
  }

  // step: 4 sync poses
  sync_data_.pose_data.clear();
  while (!data_.pose_buffer.empty() &&
         data_.pose_buffer.front().offset <= data_.radar_buffer.front().first) {
    sync_data_.pose_data.push_back(data_.pose_buffer.front());
    data_.pose_buffer.pop_front();
  }

  // step: 5 sync lidar points
  CloudPtr lidar_cloud(new PointCloudType);
  sync_data_.lidar_points_stamped.clear();
  sync_data_.lidar_beg_time = data_.last_radar_time;
  while (!data_.lidar_points_stamped_buffer.empty() &&
         data_.lidar_points_stamped_buffer.front().timestamp <=
             data_.radar_buffer.front().first) {
    sync_data_.lidar_points_stamped.push_back(
        data_.lidar_points_stamped_buffer.front());
    sync_data_.lidar_end_time =
        data_.lidar_points_stamped_buffer.front().timestamp;

    // add to lidar cloud
    PointType pt;
    pt.x = data_.lidar_points_stamped_buffer.front().x;
    pt.y = data_.lidar_points_stamped_buffer.front().y;
    pt.z = data_.lidar_points_stamped_buffer.front().z;
    pt.intensity = data_.lidar_points_stamped_buffer.front().intensity;
    pt.normal_x = 0;
    pt.normal_y = 0;
    pt.normal_z = 0;
    pt.curvature = (data_.lidar_points_stamped_buffer.front().timestamp -
                    sync_data_.lidar_beg_time) *
                   1000.0; // ms
    lidar_cloud->points.push_back(pt);

    data_.lidar_points_stamped_buffer.pop_front();
  }
  lidar_cloud->width = sync_data_.lidar_points_stamped.size();
  lidar_cloud->height = 1;
  lidar_cloud->is_dense = true;
  sync_data_.lidar_cloud = lidar_cloud;

  // step: 6 sync radar cloud
  sync_data_.radar_time = data_.radar_buffer.front().first;
  sync_data_.radar_cloud = data_.radar_buffer.front().second;
  data_.last_radar_time =
      data_.radar_buffer.front().first; // update last radar time after sync
  data_.radar_buffer.pop_front();

  data_.mtx.unlock();
  return true; // Successfully synchronized
}

void Mapping::Process() {
  if (sync_data_.pose_data.size() < options_.pose_num_threshold) {
    ROS_WARN(
        "Not enough pose data to undistort LiDAR points. Skipping this group.");
    return;
  }

  Preprocess::Undistort(sync_data_);
}
} // namespace mapping