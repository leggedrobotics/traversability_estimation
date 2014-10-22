/*
 * elevation_obstacle_detection_node.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "elevation_obstacle_detection/ElevationObstacleDetection.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevation_obstacle_detection");
  ros::NodeHandle nodeHandle("~");
  elevation_obstacle_detection::ElevationObstacleDetection
    elevationObstacleDetection(nodeHandle);

  // Spin
  ros::AsyncSpinner spinner(1); // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
