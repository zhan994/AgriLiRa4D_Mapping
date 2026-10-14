#include "sync.h"
#include "preprocess.h"

#include <cmath>
#include <future>
#include <iostream>
#include <optional>
#include <stdexcept>

using namespace mapping;
using namespace std::chrono_literals;

namespace {
void Check(bool condition) {
  if (!condition)
    throw std::runtime_error("Synchronizer test failed");
}

bool Near(double a, double b) { return std::abs(a - b) < 1e-9; }

CloudPtr Cloud(std::initializer_list<float> offsets) {
  CloudPtr cloud(new PointCloudType);
  for (float offset : offsets) {
    PointType point{};
    point.x = 3.0f;
    point.curvature = offset;
    cloud->push_back(point);
  }
  return cloud;
}

RadarCloudPtr Radar() { return RadarCloudPtr(new RadarPointCloudType); }

Pose6D Pose(double time) {
  return Pose6D(time, V3D::Zero(), V3D::Zero(), V3D::Zero(),
                V3D::Zero(), M3D::Identity());
}

auto PopAsync(Synchronizer &sync) {
  return std::async(std::launch::async, [&sync]() -> std::optional<MeasureGroup> {
    MeasureGroup group;
    if (!sync.WaitPop(group))
      return std::nullopt;
    return group;
  });
}

MeasureGroup Pop(Synchronizer &sync) {
  auto result = PopAsync(sync);
  if (result.wait_for(2s) != std::future_status::ready) {
    sync.Stop();
    throw std::runtime_error("Timed out waiting for synchronized output");
  }
  auto group = result.get();
  Check(group.has_value());
  return std::move(*group);
}

void TestRadarSlices() {
  Synchronizer sync({2, 1}); // Capacity one also exercises waiting for output space.
  for (double time : {1.0, 1.05, 1.1, 1.15, 1.2})
    Check(sync.AddPoseData(Pose(time)));
  Check(sync.AddRadarData(0.9, Radar())); // Before LiDAR: skip this radar.
  Check(sync.AddRadarData(1.1, Radar()));
  Check(sync.AddRadarData(1.2, Radar()));
  Check(sync.AddLidarData(1.0, Cloud({0, 25, 50, 75, 100, 125, 150, 175, 200})));

  auto first = Pop(sync);
  auto second = Pop(sync);
  Check(Near(first.radar_time, 1.1) && Near(second.radar_time, 1.2));
  Check(first.last_radar_time < 0.0 && Near(second.last_radar_time, 1.1));
  Check(second.lidar_points_stamped.front().timestamp > second.last_radar_time);
  Check(first.lidar_cloud->size() == 5 && second.lidar_cloud->size() == 4);
  Check(Near(first.lidar_beg_time, 1.0) && Near(second.lidar_beg_time, 1.1));
  Check(Near(first.lidar_end_time, 1.1) && Near(second.lidar_end_time, 1.2));
  Check(Near(second.pose_data.front().offset, 1.1));
  Check(Near(second.lidar_cloud->front().curvature, 25));
  for (const auto &point : second.lidar_points_stamped)
    Check(point.timestamp > 1.1 && point.timestamp <= 1.2);
  Check(first.lidar_cloud != second.lidar_cloud);
  sync.Stop();
  MeasureGroup group;
  Check(!sync.WaitPop(group));
}

void TestIncompleteCoverage() {
  Synchronizer sync({2, 8});
  Check(sync.AddPoseData(Pose(1.0)));
  Check(sync.AddRadarData(1.1, Radar()));
  Check(sync.AddLidarData(1.0, Cloud({0, 50})));
  auto pending = PopAsync(sync);
  Check(pending.wait_for(30ms) == std::future_status::timeout);
  Check(sync.AddLidarData(1.05, Cloud({0, 50})));
  Check(pending.wait_for(30ms) == std::future_status::timeout); // GT still missing.
  Check(sync.AddPoseData(Pose(1.1)));
  if (pending.wait_for(2s) != std::future_status::ready) {
    sync.Stop();
    throw std::runtime_error("GT arrival failed to wake synchronization");
  }
  auto group = pending.get();
  Check(group && group->lidar_cloud->size() == 4); // Neither scan was consumed early.
}

void TestInvalidSlice() {
  std::promise<SyncDiagnostics> diagnostic;
  auto reported = diagnostic.get_future();
  Synchronizer *instance = nullptr;
  Synchronizer sync({3, 8}, [&](const SyncDiagnostics &info) {
    // Reenter an input API to verify the callback is invoked outside mutex_.
    Check(instance->AddPoseData(Pose(1.25)));
    diagnostic.set_value(info);
  });
  instance = &sync;
  for (double time : {1.0, 1.05, 1.1, 1.15, 1.2})
    Check(sync.AddPoseData(Pose(time)));
  Check(sync.AddRadarData(1.05, Radar())); // Only two poses: skip this slice.
  Check(sync.AddRadarData(1.2, Radar()));
  Check(sync.AddLidarData(1.0, Cloud({0, 25, 50, 75, 100, 125, 150, 175, 200})));
  auto group = Pop(sync);
  Check(Near(group.radar_time, 1.2) && Near(group.lidar_beg_time, 1.05));
  Check(group.lidar_cloud->size() == 6);
  Check(reported.wait_for(2s) == std::future_status::ready);
  const auto info = reported.get();
  Check(info.pose_count == 2 && info.required_poses == 3 && info.has_start_pose);
  Check(info.last_radar_time < 0.0 && Near(info.radar_time, 1.05));
  Check(info.lidar_points == 3 && Near(info.lidar_begin_time, 1.0) &&
        Near(info.lidar_end_time, 1.05));
  Check(Near(info.pose_begin_time, 1.0) && Near(info.pose_end_time, 1.05));
}

void TestStop() {
  Synchronizer sync;
  auto pending = PopAsync(sync);
  sync.Stop();
  Check(pending.wait_for(2s) == std::future_status::ready);
  Check(!pending.get());
  sync.Stop();
  Check(!sync.AddLidarData(1.0, Cloud({0})));
  Check(!sync.AddRadarData(1.0, Radar()));
  Check(!sync.AddPoseData(Pose(1.0)));

  // Produce far more slices than capacity, with no consumer, then stop.
  Synchronizer full({1, 1});
  for (int i = 0; i <= 100; ++i)
    Check(full.AddPoseData(Pose(1.0 + i * 0.01)));
  Check(full.AddLidarData(1.0, Cloud({0, 1000})));
  for (int i = 1; i <= 100; ++i)
    Check(full.AddRadarData(1.0 + i * 0.01, Radar()));
  full.Stop(); // Must interrupt the worker even if it is waiting for queue space.
  MeasureGroup group;
  int drained = 0;
  while (full.WaitPop(group))
    ++drained;
  Check(drained <= 1);
}

void TestConcurrentProducers() {
  Synchronizer sync({2, 4});
  auto poses = std::async(std::launch::async, [&] {
    for (int i = 0; i <= 200; ++i)
      Check(sync.AddPoseData(Pose(1.0 + i * 0.01)));
  });
  auto lidar = std::async(std::launch::async, [&] {
    for (int i = 0; i < 20; ++i)
      Check(sync.AddLidarData(1.0 + i * 0.1, Cloud({0, 25, 50, 75, 100})));
  });
  auto radar = std::async(std::launch::async, [&] {
    for (int i = 0; i < 20; ++i)
      Check(sync.AddRadarData(1.05 + i * 0.1, Radar()));
  });
  for (int i = 0; i < 20; ++i) {
    auto group = Pop(sync);
    Check(Near(group.radar_time, 1.05 + i * 0.1));
    Check(!group.lidar_cloud->empty() && group.pose_data.size() >= 2);
  }
  poses.get();
  lidar.get();
  radar.get();
}

void TestFirstPointCompensation() {
  // A fixed world landmark at x=3, observed from body x=0, 1, 2.
  // All observations must become x=1 in the last body frame. An extra pose
  // before the first point's selected pose must not compensate it again.
  MeasureGroup group;
  group.lidar_beg_time = 1.05;
  group.lidar_cloud = Cloud({0, 100, 200});
  auto earlier_pose = Pose(0.9);
  earlier_pose.trans.x() = -1;
  group.pose_data.push_back(earlier_pose);
  for (int i = 0; i < 3; ++i) {
    auto pose = Pose(1.0 + i * 0.1);
    pose.trans.x() = i;
    group.pose_data.push_back(pose);
    PointStamped point{};
    point.timestamp = group.lidar_beg_time + i * 0.1;
    group.lidar_points_stamped.push_back(point);
    group.lidar_cloud->points[i].x = 3 - i;
  }
  Preprocess::UndistortPcl(group, M3D::Identity(), V3D::Zero());
  for (const auto &point : group.lidar_cloud->points)
    Check(Near(point.x, 1.0));

  RadarCloudPtr output(new RadarPointCloudType);
  RadarPointType old_point{};
  output->push_back(old_point);
  Preprocess::TransformRadarPointCloud(Radar(), M3D::Identity(), V3D::Zero(), output);
  Check(output->empty()); // Empty scan must not repeat the previous output.
}
} // namespace

int main() {
  try {
    TestRadarSlices();
    TestIncompleteCoverage();
    TestInvalidSlice();
    TestStop();
    TestConcurrentProducers();
    TestFirstPointCompensation();
    std::cout << "Synchronizer tests passed\n";
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
