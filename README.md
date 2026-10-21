# AgriLiRa4D_Mapping

***Mapping package for "AgriLiRa4D: A Multi-Sensor UAV Dataset for Robust SLAM in Challenging Agricultural Fields".***

<img src="imgs/Pipeline.png" style="zoom:70%;" />

**Supported Platforms**

- ROS-Noetic on Ubuntu20.04
<!-- - ROS-One on Ubuntu22.04 -->

## Third-party

- PCL
- Eigen3
- Octomap

## Run

```bash
# build
mkdir -p ws_agrilira4d/src
cd ws_agrilira4d/src
git clone git@github.com:zhan994/AgriLiRa4D_Mapping.git
cd ..
catkin_make

# run
source devel/setup.bash
roslaunch mapping mapping.launch
```


## Notes

**Ground-Truth Odometry for FRD (Body) in FLU**

![](imgs/FLU_ENU.png)

**Extrinsics for Robosense Airy LiDAR**

![](imgs/Airy_Extrinsics.png)




## Map output

The latched `/map_cloud` topic publishes occupied voxel centers as
`sensor_msgs/PointCloud2` in the `world` frame. Publication follows
`mapper.publish_period`. Merged OctoMap leaves are enumerated at
`mapper.octomap.resolution`, so every point represents one base-resolution voxel.
RViz's Map Cloud display uses 0.5 m boxes by default; adjust its Size (m) when
changing the configured resolution.

`Mapper::GetOccupiedCloud()` replaces its output with this occupied cloud.
`Mapper::Save(path)` writes the same points as a binary PCD file; use a `.pcd`
extension. This export stores occupied geometry, without free-space or probability
state.
