/**
 * @file main.cpp
 * @author Zhihao Zhan (zhihazhan2-c@my.cityu.edu.hk)
 * @brief main entry for AgriLiRa4D Mapping
 * @version 0.1
 * @date 2026-07-18
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "comm.h"
#include "mapping.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapping_node");
  ros::NodeHandle nh;
  mapping::Options options = mapping::LoadOptionsFromFile("config.yaml");
  mapping::Mapping mapping(nh, options);

  ros::spin();
  return 0;
}