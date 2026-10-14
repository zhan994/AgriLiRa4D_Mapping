#include "sync.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mapping {

Synchronizer::Synchronizer() : Synchronizer(Options{}) {}

Synchronizer::Synchronizer(
    const Options &options, std::function<void(const SyncDiagnostics &)> on_invalid)
    : options_(options), on_invalid_(std::move(on_invalid)) {
  if (options_.pose_num_threshold == 0 || options_.output_queue_capacity == 0)
    throw std::invalid_argument("Synchronizer capacities must be positive");
  sync_thread_ = std::thread(&Synchronizer::Run, this);
}

Synchronizer::~Synchronizer() { Stop(); }

void Synchronizer::Stop() {
  std::cout << "Stopping Synchronizer..." << std::endl;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  input_cv_.notify_all();
  output_cv_.notify_all();

  // Joining must not hold mutex_: the worker needs it to finish.
  if (sync_thread_.joinable())
    sync_thread_.join();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.lidar_points_stamped_buffer.clear();
    data_.radar_buffer.clear();
    data_.pose_buffer.clear();
    output_closed_ = true;
  }
  output_cv_.notify_all();
}

bool Synchronizer::AddLidarData(double timestamp, CloudPtr cloud) {
  // Reject invalid input before acquiring the mutex to avoid blocking the
  // worker.
  if (!std::isfinite(timestamp) || timestamp < 0.0 || !cloud || cloud->empty())
    return false;

  // Conversion does not access shared state; keep it outside the critical
  // section.
  std::deque<PointStamped> points;
  for (const auto &point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.curvature))
      return false;

    PointStamped stamped{};
    stamped.x = point.x;
    stamped.y = point.y;
    stamped.z = point.z;
    stamped.intensity = point.intensity;
    stamped.timestamp =
        timestamp + static_cast<double>(point.curvature) / 1000.0;
    if (!std::isfinite(stamped.timestamp) || stamped.timestamp < 0.0)
      return false;
    points.push_back(stamped);
  }

  // Reject input with non-increasing timestamps or duplicates, and discard
  // points
  const auto time_less = [](const PointStamped &a, const PointStamped &b) {
    return a.timestamp < b.timestamp;
  };
  if (!std::is_sorted(points.begin(), points.end(), time_less))
    return false;

  // Add points to the buffer and update the last received timestamp.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || timestamp <= data_.last_lidar_received_time)
      return false;

    // Do not reintroduce points from a window already consumed by
    // synchronization.
    while (!points.empty() && points.front().timestamp <= data_.last_sync_time)
      points.pop_front();
    if (points.empty())
      return false;

    auto &buffer = data_.lidar_points_stamped_buffer;
    const auto previous_size = buffer.size();
    const bool overlaps =
        !buffer.empty() && points.front().timestamp < buffer.back().timestamp;
    buffer.insert(buffer.end(), points.begin(), points.end());
    if (overlaps) {
      // Individually sorted scans may still overlap in absolute point time.
      std::inplace_merge(buffer.begin(), buffer.begin() + previous_size,
                         buffer.end(), time_less);
    }
    data_.last_lidar_received_time = timestamp;
    input_updated_ = true;
  }
  input_cv_.notify_one();
  return true;
}

bool Synchronizer::AddRadarData(double timestamp, RadarCloudPtr cloud) {
  // Reject invalid input before acquiring the mutex to avoid blocking the
  // worker.
  if (!std::isfinite(timestamp) || timestamp < 0.0 || !cloud)
    return false;

  // Add the radar scan to the buffer and update the last received timestamp.
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || timestamp <= data_.last_radar_received_time ||
        timestamp <= data_.last_sync_time)
      return false;

    // An empty radar scan still supplies a valid synchronization timestamp.
    data_.radar_buffer.emplace_back(timestamp, std::move(cloud));
    data_.last_radar_received_time = timestamp;
    input_updated_ = true;
  }
  input_cv_.notify_one();
  return true;
}

bool Synchronizer::AddPoseData(const Pose6D &pose) {
  if (!std::isfinite(pose.offset) || pose.offset < 0.0 ||
      !pose.trans.allFinite() || !pose.rot.allFinite())
    return false;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || pose.offset <= data_.last_pose_received_time)
      return false;

    data_.pose_buffer.push_back(pose);
    data_.last_pose_received_time = pose.offset;
    input_updated_ = true;
  }
  input_cv_.notify_one();
  return true;
}

bool Synchronizer::WaitPop(MeasureGroup &group) {
  std::unique_lock<std::mutex> lock(mutex_);
  output_cv_.wait(lock,
                  [&] { return !output_queue_.empty() || output_closed_; });
  if (output_queue_.empty())
    return false;

  group = std::move(output_queue_.front());
  output_queue_.pop_front();
  lock.unlock();
  input_cv_.notify_one(); // Output queue now has space.
  return true;
}

void Synchronizer::Run() {
  std::unique_lock<std::mutex> lock(mutex_);
  while (!stopping_) {
    input_cv_.wait(lock, [&] { return input_updated_ || stopping_; });
    input_updated_ = false;

    while (!stopping_) {
      // Check capacity BEFORE consuming input. wait releases the shared mutex.
      input_cv_.wait(lock, [&] {
        return stopping_ ||
               output_queue_.size() < options_.output_queue_capacity;
      });
      if (stopping_)
        break;

      MeasureGroup group;
      SyncDiagnostics invalid;
      const bool ready = SyncLocked(group, invalid);
      if (invalid.discarded) {
        // External reporting must not block producers/consumers on our mutex.
        lock.unlock();
        if (on_invalid_)
          on_invalid_(invalid);
        lock.lock();
        continue;
      }
      
      if (!ready)
        break;

      output_queue_.push_back(std::move(group));
      lock.unlock();
      output_cv_.notify_one();
      lock.lock();
    }
  }
  output_closed_ = true;
  lock.unlock();
  output_cv_.notify_all();
}

bool Synchronizer::SyncLocked(MeasureGroup &group,
                              SyncDiagnostics &invalid) {
  auto &points = data_.lidar_points_stamped_buffer;
  auto &radars = data_.radar_buffer;
  auto &poses = data_.pose_buffer;

  while (!radars.empty() && !points.empty()) {
    const double radar_time = radars.front().first;
    // A radar preceding the earliest available point cannot form a slice.
    if (radar_time < points.front().timestamp) {
      radars.pop_front();
      if (data_.last_sync_time >= 0.0)
        data_.last_sync_time = radar_time;
      continue;
    }

    // Wait for temporal coverage instead of consuming an incomplete window.
    if (points.back().timestamp < radar_time || poses.empty() ||
        poses.back().offset < radar_time)
      return false;

    const double begin_time = data_.last_sync_time < 0.0
                                  ? points.front().timestamp
                                  : data_.last_sync_time;

    // Trim old GT, retaining the last pose at/before the window start.
    while (poses.size() > 1 && poses[1].offset <= begin_time)
      poses.pop_front();

    // The upper_bound is exclusive, so the last pose in the group is at/before
    // the radar timestamp. The front pose is at/before the window start.
    auto pose_end = std::upper_bound(
        poses.begin(), poses.end(), radar_time,
        [](double time, const Pose6D &pose) { return time < pose.offset; });
    const auto pose_count = static_cast<std::size_t>(pose_end - poses.begin());
    const bool has_start_pose = poses.front().offset <= begin_time;
    const bool valid = has_start_pose && pose_count >= options_.pose_num_threshold;

    if (valid) {
      group.last_radar_time = data_.last_sync_time;
      group.lidar_beg_time = begin_time;
      group.radar_time = radar_time;
      group.radar_cloud = std::move(radars.front().second);
      group.pose_data.assign(poses.begin(), pose_end);
    } else {
      const auto lidar_end = std::upper_bound(
          points.begin(), points.end(), radar_time,
          [](double time, const PointStamped &point) { return time < point.timestamp; });
      SyncDiagnostics info;
      info.discarded = true;
      info.last_radar_time = data_.last_sync_time;
      info.window_begin_time = begin_time;
      info.radar_time = radar_time;
      info.lidar_points = lidar_end - points.begin();
      info.lidar_begin_time = points.front().timestamp;
      info.lidar_end_time = (lidar_end - 1)->timestamp;
      info.radar_points = radars.front().second->size();
      info.pose_count = pose_count;
      info.required_poses = options_.pose_num_threshold;
      info.has_start_pose = has_start_pose;
      info.first_available_pose_time = poses.front().offset;
      if (pose_count > 0) {
        info.pose_begin_time = poses.front().offset;
        info.pose_end_time = (pose_end - 1)->offset;
      }
      invalid = info;
    }

    // Invalid slices are explicitly skipped, as in the previous pose-count
    // policy.
    while (!points.empty() && points.front().timestamp <= radar_time) {
      if (valid) {
        const auto &stamped = points.front();
        PointType point{};
        point.x = stamped.x;
        point.y = stamped.y;
        point.z = stamped.z;
        point.intensity = stamped.intensity;
        point.curvature = (stamped.timestamp - begin_time) * 1000.0;
        group.lidar_cloud->push_back(point);
        group.lidar_points_stamped.push_back(stamped);
        group.lidar_end_time = stamped.timestamp;
      }
      points.pop_front();
    }
    radars.pop_front();
    data_.last_sync_time = radar_time;
    return valid;
  }
  return false;
}

} // namespace mapping
