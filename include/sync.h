#ifndef SYNC_H
#define SYNC_H

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

#include "comm.h"

namespace mapping {

// Owned by Synchronizer; every access is protected by mutex_.
// Receiving timestamps and the consumed window boundary have separate roles.
struct DataCache {
  double last_lidar_received_time = -1.0;
  double last_radar_received_time = -1.0;
  double last_pose_received_time = -1.0;
  double last_sync_time = -1.0;

  std::deque<PointStamped> lidar_points_stamped_buffer;
  std::deque<std::pair<double, RadarCloudPtr>> radar_buffer;
  // Keep the pose at/before the next window start until it is no longer needed.
  std::deque<Pose6D> pose_buffer;
};

// Plain diagnostic data; formatting and logging belong to the caller.
struct SyncDiagnostics {
  bool discarded = false; // True when this call discarded a window.
  double last_radar_time = -1.0;
  double window_begin_time = -1.0;
  double radar_time = -1.0;
  double lidar_begin_time = -1.0;
  double lidar_end_time = -1.0;
  double pose_begin_time = -1.0;
  double pose_end_time = -1.0;
  double first_available_pose_time = -1.0;
  std::size_t lidar_points = 0;
  std::size_t radar_points = 0;
  std::size_t pose_count = 0;
  std::size_t required_poses = 0;
  bool has_start_pose = false;
};

class Synchronizer {
public:
  struct Options {
    std::size_t pose_num_threshold = 7;
    std::size_t output_queue_capacity = 8; // Must be positive.
  };

  Synchronizer();
  // Optional diagnostics callback runs on the sync thread WITHOUT mutex_ held.
  // It must not throw or call Stop() (which would join its own thread).
  explicit Synchronizer(const Options &options,
                        std::function<void(const SyncDiagnostics &)> on_invalid = {});
  ~Synchronizer();

  // Constructor starts the worker. Owner calls Stop (idempotent) before destruction.
  // Interrupt waits and close output; queued groups remain available to WaitPop.
  // Unprocessed input is discarded. Lifecycle calls are not concurrent.
  void Stop();

  // Thread-safe producers. Return false for rejected input or after Stop().
  // Each input stream must have strictly increasing message timestamps.
  // Reject duplicates/time rollback without resetting the other streams or GT.
  // Update data_ and input_updated_ under mutex_; then notify input_cv_.
  // On success the caller must not modify the submitted cloud or its aliases.
  // LiDAR input contains sorted relative point times in curvature (milliseconds).
  bool AddLidarData(double timestamp, CloudPtr cloud);
  bool AddRadarData(double timestamp, RadarCloudPtr cloud);
  bool AddPoseData(const Pose6D &pose);

  // Single consumer. Move a queued group into group, waiting if necessary.
  // False means output is closed AND empty. Processing occurs after unlocking.
  // Wait on output_cv_ for a nonempty queue or output_closed_, using mutex_.
  // After popping, notify input_cv_ so the sync worker can resume if output was full.
  // Each output LiDAR cloud has independent storage; the worker retains no
  // mutable aliases to published clouds, which the consumer may undistort.
  bool WaitPop(MeasureGroup &group);

private:
  void Run();
  // Called with mutex_ held. Radar time slices LiDAR points; preserve boundary poses.
  // False with invalid.discarded means a slice was discarded; otherwise wait for input.
  bool SyncLocked(MeasureGroup &group, SyncDiagnostics &invalid);

  const Options options_;
  const std::function<void(const SyncDiagnostics &)> on_invalid_;

  // One mutex protects BOTH caches and every shared status flag below.
  // Never run motion compensation, mapping or publishing while holding it.
  std::mutex mutex_;
  // Sync worker: new input OR output queue space (different wait predicates).
  std::condition_variable input_cv_;
  // Mapping worker: output queue nonempty OR closed.
  std::condition_variable output_cv_;

  DataCache data_;
  std::deque<MeasureGroup> output_queue_;
  bool input_updated_ = false;  // Set by producers, cleared by the sync worker.
  bool stopping_ = false;       // Prevents further input/enqueue and interrupts sync waits.
  bool output_closed_ = false;  // No more output will be produced; queued groups can drain.

  std::thread sync_thread_; // Declared after the state used by the worker.
};

} // namespace mapping

#endif // SYNC_H
