/*
 * ElevationObstacleDetection.hpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Grid Map
#include "grid_map_msg/GetGridMap.h"

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

namespace elevation_obstacle_detection {
  /*!
  * The elevation obstacle detection main class. Coordinates the ROS
  * interfaces, the timing, and the data handling between the other classes.
  */
  class ElevationObstacleDetection {
  public:
    /*!
    * Constructor.
    * @param nodeHandle the ROS node handle.
    */
    ElevationObstacleDetection(ros::NodeHandle& nodeHandle);

    /*!
    * Destructor.
    */
    virtual ~ElevationObstacleDetection();
  private:
    //! ROS nodehandle.
    ros::NodeHandle& nodeHandle_;

    /*!
    * Reads and verifies the ROS parameters.
    * @return true if successful.
    */
    bool readParameters();  
  };
} /* namespace */
