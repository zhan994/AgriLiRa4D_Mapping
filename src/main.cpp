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
  if (argc < 2) {
    std::cout << "usage: ./rlio_node <path-to-cfg>/cfg.yaml" << std::endl;
    return -1;
  }

  ros::init(argc, argv, "mapping_node");
  ros::NodeHandle nh;

  std::string cfg_fn = argv[1];
  mapping::Options options = mapping::LoadOptionsFromFile(cfg_fn);
  mapping::Mapping mapping(nh, options);

  ros::spin();
  return 0;
}