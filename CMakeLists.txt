cmake_minimum_required(VERSION 3.0.2)
project(mapping)

message(STATUS "Building mapping package...")
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PACKAGE_DEPENDENCIES
  geometry_msgs
  jsk_rviz_plugins
  nav_msgs
  octomap_msgs
  roscpp
  rosbag
  sensor_msgs
  tf2
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find PCL
find_package(PCL REQUIRED)

# Find YAML
find_package(yaml-cpp REQUIRED)

# Find Octomap
find_package(octomap REQUIRED)

catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})


###########
## Build ##
###########
set(MAPPING_SRC
  src/main.cpp
  src/mapping.cpp
  src/options.cpp
  src/preprocess.cpp
  src/mapper/mapper.cpp
  src/mapper/mapper_octomap.cpp
  src/mapper/mapper_ioctree.cpp
)

add_executable(mapping_node ${MAPPING_SRC})
target_include_directories(mapping_node PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
target_link_libraries(mapping_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
