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
    
    //! ROS service clients.
    ros::ServiceClient submapClient_;

    //! ROS services for client calls.
    std::string submapService_;
  
    //! ROS publishers.
    ros::Publisher occupancyGridPublisher_;
  
    //! TF listener and broadcaster.
    tf::TransformListener transformListener_;
    
    //! Id of the frame at whose origin the map will be centered.
    std::string mapFrameId_;
  
    //! Timer for the map update.
    ros::Timer mapUpdateTimer_;
  
    //! Maximum duration between map updates.
    ros::Duration maxUpdateDuration_;
    
    //! Requested map cell type.
    std::string mapCellType_;
    
    //! Requested map center in x-direction in [m].
    double mapCenterX_;
    //! Requested map center in y-direction in [m].
    double mapCenterY_;
    
    //! Requested map width in x-direction in [m].
    double mapLengthX_;
    //! Requested map width in y-direction in [m].
    double mapLengthY_;
  
    /*!
    * Reads and verifies the ROS parameters.
    * @return true if successful.
    */
    bool readParameters();  
    
    /*!
    * Callback function for the update timer. Forces an update of the obstacle
    * map from a new elevation map requested from the grid map service.
    * @param timerEvent the timer event.
    */
    void mapUpdateTimerCallback(const ros::TimerEvent& timerEvent);
  };
} /* namespace */
