/*
 * ElevationObstacleDetection.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "elevation_obstacle_detection/ElevationObstacleDetection.hpp"

// Grid Map
#include <grid_map/GridMap.hpp>
#include <grid_map_msg/GetGridMap.h>

//STL
#include <deque>

using namespace std;
using namespace nav_msgs;
using namespace grid_map_msg;

namespace elevation_obstacle_detection {
  ElevationObstacleDetection::ElevationObstacleDetection(ros::NodeHandle&
      nodeHandle) :
    nodeHandle_(nodeHandle) {
    ROS_INFO("Elevation obstacle detection node started.");

    readParameters();
    submapClient_ = nodeHandle_.serviceClient<GetGridMap>(submapService_);
    occupancyGridPublisher_ = nodeHandle.advertise<OccupancyGrid>(
      "obstacle_map", 1, true);
    mapUpdateTimer_ = nodeHandle_.createTimer(maxUpdateDuration_,
      &ElevationObstacleDetection::mapUpdateTimerCallback, this);
  }

  ElevationObstacleDetection::~ElevationObstacleDetection() {
    mapUpdateTimer_.stop();
    nodeHandle_.shutdown();
  }

  bool ElevationObstacleDetection::readParameters() {
    nodeHandle_.param("submap_service", submapService_,
      string("/get_grid_map"));
    nodeHandle_.param("map_frame_id", mapFrameId_,
      string("/robot"));
    
    double maxUpdateRate;
    nodeHandle_.param("max_update_rate", maxUpdateRate, 2.0);
    maxUpdateDuration_.fromSec(1.0 / maxUpdateRate);
    
    nodeHandle_.param("map_cell_type", mapCellType_, string("elevation"));
    nodeHandle_.param("map_center_x", mapCenterX_, 0.0);
    nodeHandle_.param("map_center_y", mapCenterY_, 0.0);
    nodeHandle_.param("map_length_x", mapLengthX_, 5.0);
    nodeHandle_.param("map_length_y", mapLengthY_, 5.0);
    
    return true;
  }
  
  void ElevationObstacleDetection::mapUpdateTimerCallback(const
      ros::TimerEvent& timerEvent) {
    GetGridMap submapService;
    
    submapService.request.frameId.data = mapFrameId_;
    submapService.request.positionX = mapCenterX_;
    submapService.request.positionY = mapCenterY_;
    submapService.request.lengthX = mapLengthX_;
    submapService.request.lengthY = mapLengthY_;
    submapService.request.dataDefinition.resize(1);
    submapService.request.dataDefinition[0].data = mapCellType_;
    
    if (submapClient_.call(submapService)) {
      grid_map::GridMap elevationGrid(submapService.response.gridMap);
      OccupancyGrid occupancyGrid;
      
      elevationGrid.toOccupancyGrid(occupancyGrid, mapCellType_, -1.0, 0.0);
      occupancyGridPublisher_.publish(occupancyGrid);
    }
    else
      ROS_WARN("Failed to retrieve elevation grid map.");
  }
} /* namespace */
