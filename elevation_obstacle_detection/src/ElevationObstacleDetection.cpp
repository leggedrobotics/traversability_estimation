/*
 * ElevationObstacleDetection.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "elevation_obstacle_detection/ElevationObstacleDetection.hpp"

// Grid Map
#include <grid_map_msg/GridMap.h>

using namespace std;

namespace elevation_obstacle_detection {
  ElevationObstacleDetection::ElevationObstacleDetection(ros::NodeHandle&
      nodeHandle) :
    nodeHandle_(nodeHandle) {
    ROS_INFO("Elevation obstacle detection node started.");

    readParameters();
  }

  ElevationObstacleDetection::~ElevationObstacleDetection() {
    nodeHandle_.shutdown();
  }

  bool ElevationObstacleDetection::readParameters() {
    return true;
  }
} /* namespace */
