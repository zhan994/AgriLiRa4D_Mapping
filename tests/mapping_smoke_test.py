#!/usr/bin/env python3
"""Run a mapping_node binary against synthetic sensors on a private ROS master.

Usage: python3 tests/mapping_smoke_test.py /path/to/mapping_node
Requires the ROS Noetic Python environment; no dataset or existing master needed.
"""

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
                from octomap_msgs.msg import Octomap
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
  type: 0
  publish_period: 0.15
  octomap:
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
                        ("map", "/octomap_binary", Octomap),
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

                for stamp in (1.0, 1.2):
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
                                for name in ("lidar", "radar", "odom", "path", "map")),
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
                octomap = received["map"][-1]
                check(octomap.binary and octomap.id == "OcTree" and len(octomap.data) > 0,
                      "Mapper did not produce a binary occupancy map")
                check(octomap.header.frame_id == "world" and octomap.resolution == 0.5,
                      "Wrong map frame or resolution")
                check(len(received["path"][-1].poses) == 2, "Wrong trajectory length")
                check(received["odom"][-1].pose.pose.position.x == 10,
                      "Wrong GT odometry")
                with condition:
                    check(condition.wait_for(lambda: bool(received.get("stats")), timeout=2),
                          "Missing mapper statistics")
                check("LiDAR 3 | Radar 1" in received["stats"][-1].text,
                      "Wrong mapper input statistics")
                late = rospy.wait_for_message("/octomap_binary", Octomap, timeout=2)
                check(abs(late.header.stamp.to_sec() - 1.3) < 1e-6,
                      "Map publisher did not latch latest map")
                node.send_signal(signal.SIGINT)
                check(node.wait(timeout=5) == 0, "Mapping failed to shut down cleanly")
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
