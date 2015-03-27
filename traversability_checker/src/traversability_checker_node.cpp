/*
 * traversability_checker_node.cpp
 *
 *  Created on: Mar 24, 2014
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "traversability_checker/TraversabilityChecker.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "traversability_checker");
  ros::NodeHandle nodeHandle("~");
  traversability_checker::TraversabilityChecker traversabilityChecker(nodeHandle);
  ros::spin();
  return 0;
}
