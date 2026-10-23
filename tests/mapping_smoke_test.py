#!/usr/bin/env python3
"""Run a mapping_node binary against synthetic sensors on a private ROS master.

Usage: python3 tests/mapping_smoke_test.py /path/to/mapping_node [0|1] [publish_period] [deduplicate_rays]
Requires the ROS Noetic Python environment; no dataset or existing master needed.
"""

import csv
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client


def main():
    with tempfile.TemporaryDirectory(prefix="agrilira-mapping-") as directory:
        root = Path(directory)
        publish_period = float(sys.argv[3]) if len(sys.argv) > 3 else 0.15
        deduplicate_rays = len(sys.argv) > 4 and sys.argv[4].lower() == "true"
        csv_path = root / "timing" / "mapper.csv"
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            port = sock.getsockname()[1]
        os.environ.update(
            ROS_MASTER_URI=f"http://127.0.0.1:{port}",
            ROS_IP="127.0.0.1",
            ROS_HOME=str(root / "ros"),
            ROS_LOG_DIR=str(root / "logs"),
        )
        children = []
        with (root / "processes.log").open("w+") as log:
            try:
                children.append(subprocess.Popen(
                    ["roscore", "-p", str(port)], stdout=log, stderr=log))
                master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
                deadline = time.monotonic() + 10
                while True:
                    try:
                        if master.getPid("/mapping_smoke_test")[0] == 1:
                            break
                    except OSError:
                        pass
                    if time.monotonic() > deadline:
                        raise RuntimeError("ROS master failed to start")
                    time.sleep(0.05)

                import rospy
                from jsk_rviz_plugins.msg import OverlayText
                from nav_msgs.msg import Odometry, Path as RosPath
                from sensor_msgs.msg import PointCloud2, PointField
                from sensor_msgs import point_cloud2
                from std_msgs.msg import Header

                q = math.sqrt(0.5)  # Body has a 90-degree yaw in world.
                gt = root / "gt.txt"
                gt.write_text("".join(
                    f"{0.95 + i * 0.05:.6f} 10 5 2 0 0 {q} {q}\n"
                    for i in range(10)))
                config = root / "mapping.yaml"
                config.write_text(f"""pose_gt_file: '{gt}'
lidar_topic: /smoke/lidar
radar_topic: /smoke/radar
verbose: false
extrinsics:
  R_bl: [1, 0, 0, 0, 1, 0, 0, 0, 1]
  t_bl: [1, 0, 0]
  R_lr: [1, 0, 0, 0, 1, 0, 0, 0, 1]
  t_lr: [0, 1, 0]
preprocess:
  lidar_filter_num: 1
  lidar_blind: 2
sync:
  pose_num_threshold: 2
  output_queue_capacity: 8
mapper:
  type: {int(sys.argv[2]) if len(sys.argv) > 2 else 0}
  publish_period: {publish_period}
  timing_csv: "{csv_path}"
  deduplicate_rays: {str(deduplicate_rays).lower()}
  resolution: 0.5
  max_range: 50
  hit_prob: 0.7
  miss_prob: 0.4
  occupancy_threshold: 0.5
""")
                rospy.init_node("mapping_smoke_test", anonymous=True,
                                disable_signals=True)
                condition = threading.Condition()
                received = {}

                def capture(name):
                    def callback(message):
                        with condition:
                            received.setdefault(name, []).append(message)
                            condition.notify_all()
                    return callback

                subscribers = [rospy.Subscriber(topic, kind, capture(name))
                    for name, topic, kind in [
                        ("lidar", "/lidar_aft_mapped", PointCloud2),
                        ("radar", "/radar_aft_mapped", PointCloud2),
                        ("odom", "/odom", Odometry),
                        ("path", "/path", RosPath),
                        ("map", "/map_cloud", PointCloud2),
                        ("stats", "/mapper_update_stats", OverlayText),
                    ]]
                lidar = rospy.Publisher("/smoke/lidar", PointCloud2, queue_size=10)
                radar = rospy.Publisher("/smoke/radar", PointCloud2, queue_size=10)
                node = subprocess.Popen([sys.argv[1], str(config)],
                                        stdout=log, stderr=log)
                children.append(node)
                deadline = time.monotonic() + 10
                while not (lidar.get_num_connections() and radar.get_num_connections()
                           and all(sub.get_num_connections() for sub in subscribers)):
                    if node.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError("Mapping subscriptions did not connect")
                    time.sleep(0.05)

                lidar_fields = [
                    PointField("x", 0, PointField.FLOAT32, 1),
                    PointField("y", 4, PointField.FLOAT32, 1),
                    PointField("z", 8, PointField.FLOAT32, 1),
                    PointField("intensity", 12, PointField.FLOAT32, 1),
                    PointField("ring", 16, PointField.UINT16, 1),
                    PointField("timestamp", 18, PointField.FLOAT64, 1),
                ]
                radar_fields = [PointField(name, i * 4, PointField.FLOAT32, 1)
                    for i, name in enumerate(
                        ("x", "y", "z", "v_doppler_mps", "snr_db", "rcs"))]
                def ros_time(seconds):
                    # from_sec truncates some floats (e.g. 1.2) by one nanosecond.
                    nanoseconds = round(seconds * 1_000_000_000)
                    return rospy.Time(nanoseconds // 1_000_000_000,
                                      nanoseconds % 1_000_000_000)

                last_map_stamp = None
                expected_get = []
                for stamp in (1.0, 1.2):
                    extract_map = (last_map_stamp is None or publish_period <= 0 or
                                   stamp + 0.1 - last_map_stamp >= publish_period)
                    expected_get.append(extract_map)
                    if extract_map:
                        last_map_stamp = stamp + 0.1
                    outputs = ("lidar", "radar", "odom", "path")
                    if extract_map:
                        outputs += ("map",)
                    lidar.publish(point_cloud2.create_cloud(
                        Header(stamp=ros_time(stamp)), lidar_fields,
                        [(4, 0, 0, 5, 0, stamp + dt) for dt in (0, 0.05, 0.1)]))
                    radar.publish(point_cloud2.create_cloud(
                        Header(stamp=ros_time(stamp + 0.1)), radar_fields,
                        [(2, 0, 0, 0.5, 10, 1)]))
                    with condition:
                        ready = condition.wait_for(
                            lambda: all(received.get(name) and
                                abs(received[name][-1].header.stamp.to_sec() -
                                    (stamp + 0.1)) < 1e-6
                                for name in outputs),
                            timeout=10)
                    if not ready:
                        stamps = {name: [message.header.stamp.to_sec()
                            for message in messages] for name, messages in received.items()
                            if name != "stats"}
                        raise RuntimeError(f"Missing outputs for slice {stamp + 0.1}: {stamps}")

                def check(condition, message):
                    if not condition:
                        raise RuntimeError(message)

                for name, expected in (("lidar", (10, 10, 2)),
                                       ("radar", (9, 8, 2))):
                    cloud = received[name][-1]
                    check(cloud.header.frame_id == "world", "Wrong point cloud frame")
                    points = list(point_cloud2.read_points(cloud, ("x", "y", "z")))
                    check(len(points) == (3 if name == "lidar" else 1),
                          "Wrong mapped point count")
                    check(all(abs(a - b) < 1e-4 for point in points
                              for a, b in zip(point, expected)), "Wrong world coordinates")
                map_cloud = received["map"][-1]
                check(map_cloud.header.frame_id == "world", "Wrong map frame")
                centers = list(point_cloud2.read_points(map_cloud, ("x", "y", "z")))
                check(len(centers) == 1 and
                      all(abs(a - b) < 1e-4 for a, b in
                          zip(centers[0], (10.25, 10.25, 2.25))),
                      "Wrong occupied voxel center in published map")
                check(len(received["path"][-1].poses) == 2, "Wrong trajectory length")
                check(received["odom"][-1].pose.pose.position.x == 10,
                      "Wrong GT odometry")
                with condition:
                    check(condition.wait_for(lambda: bool(received.get("stats")), timeout=2),
                          "Missing mapper statistics")
                check("LiDAR 3 | Radar 1" in received["stats"][-1].text,
                      "Wrong mapper input statistics")
                late = rospy.wait_for_message("/map_cloud", PointCloud2, timeout=2)
                check(abs(late.header.stamp.to_sec() - last_map_stamp) < 1e-6,
                      "Map publisher did not latch latest map")
                node.send_signal(signal.SIGINT)
                check(node.wait(timeout=5) == 0, "Mapping failed to shut down cleanly")
                with csv_path.open(newline="") as stream:
                    reader = csv.DictReader(stream)
                    check(reader.fieldnames == ["timestamp", "data_size", "update_cost",
                                                "get_map_cost", "ray_mode", "valid_points",
                                                "candidate_rays", "ray_count", "touched_voxels"], "Wrong timing CSV schema")
                    rows = list(reader)
                check(len(rows) == 2, "Timing CSV must contain every update")
                for row, stamp, extracted in zip(rows, (1.1, 1.3), expected_get):
                    check(abs(float(row["timestamp"]) - stamp) < 1e-9,
                          "Wrong timing CSV timestamp")
                    check(int(row["data_size"]) == 3, "CSV must count input LiDAR points")
                    check(math.isfinite(float(row["update_cost"])) and
                          float(row["update_cost"]) >= 0, "Invalid update timing")
                    check(int(row["valid_points"]) == 3 and int(row["candidate_rays"]) == 3,
                          "Wrong valid point or candidate ray count")
                    check(int(row["ray_count"]) == (1 if deduplicate_rays else 3),
                          "Wrong deduplicated ray count")
                    check(row["ray_mode"] == ("endpoint_voxel" if deduplicate_rays else "per_point"),
                          "Wrong ray strategy in CSV")
                    if extracted:
                        check(math.isfinite(float(row["get_map_cost"])) and
                              float(row["get_map_cost"]) >= 0, "Missing extraction timing")
                    else:
                        check(row["get_map_cost"] == "", "Skipped extraction must be blank")
                print("Mapping ROS smoke test passed: callbacks -> sync -> mapper -> outputs")
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                for child in reversed(children):
                    if child.poll() is None:
                        child.send_signal(signal.SIGINT)
                        try:
                            child.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            child.kill()
                            child.wait()


if __name__ == "__main__":
    main()
