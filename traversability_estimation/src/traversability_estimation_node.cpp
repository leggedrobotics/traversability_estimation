/*
 * traversability_estimation_node.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "traversability_estimation/TraversabilityEstimation.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "traversability_estimation");
  ros::NodeHandle nodeHandle("~");
  traversability_estimation::TraversabilityEstimation traversabilityEstimation(nodeHandle);

  // Spin
  ros::AsyncSpinner spinner(0);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
