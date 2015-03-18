/*
 * TraversabilityEstimation.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"

// Grid Map
#include <grid_map_msg/GetGridMap.h>

//STL
#include <deque>

using namespace std;
using namespace nav_msgs;
using namespace grid_map_msg;

namespace traversability_estimation {

TraversabilityEstimation::TraversabilityEstimation(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      traversabilityType_("traversability"),
      filter_chain_("grid_map::GridMap")
{
  ROS_INFO("Traversability estimation node started.");

  readParameters();
  submapClient_ = nodeHandle_.serviceClient<GetGridMap>(submapServiceName_);
  occupancyGridPublisher_ = nodeHandle.advertise<OccupancyGrid>("traversability_map", 1, true);
  updateTimer_ = nodeHandle_.createTimer(updateDuration_,
                                         &TraversabilityEstimation::updateTimerCallback, this);

  requestedMapTypes_.push_back("elevation");
  requestedMapTypes_.push_back("surface_normal_x");
  requestedMapTypes_.push_back("surface_normal_y");
  requestedMapTypes_.push_back("surface_normal_z");
}

TraversabilityEstimation::~TraversabilityEstimation()
{
  updateTimer_.stop();
  nodeHandle_.shutdown();
}

bool TraversabilityEstimation::readParameters()
{
  nodeHandle_.param("submap_service", submapServiceName_, string("/get_grid_map"));

  double updateRate;
  nodeHandle_.param("update_rate", updateRate, 1.0);
  updateDuration_.fromSec(1.0 / updateRate);

  nodeHandle_.param("map_frame_id", mapFrameId_, string("map"));
  string robotFrameId;
  nodeHandle_.param("robot_frame_id", robotFrameId, string("robot"));
  double mapCenterX, mapCenterY;
  nodeHandle_.param("map_center_x", mapCenterX, 0.0);
  nodeHandle_.param("map_center_y", mapCenterY, 0.0);
  submapPoint_.header.frame_id = robotFrameId;
  submapPoint_.point.x = mapCenterX;
  submapPoint_.point.y = mapCenterY;
  submapPoint_.point.z = 0.0;

  nodeHandle_.param("map_length_x", mapLength_.x(), 5.0);
  nodeHandle_.param("map_length_y", mapLength_.y(), 5.0);

  // Configure filter chain
  if (!filter_chain_.configure("traversability_map_filters", nodeHandle_)) {
    ROS_ERROR("Could not configure the filter chain!");
  }

  return true;
}

void TraversabilityEstimation::updateTimerCallback(const ros::TimerEvent& timerEvent)
{
  grid_map_msg::GridMap mapMessage;
  if (getGridMap(mapMessage)) {
    grid_map::GridMap map(mapMessage);
    computeTraversability(map);
    publishAsOccupancyGrid(map);
  } else {
    ROS_WARN("Failed to retrieve elevation grid map.");
  }
}

bool TraversabilityEstimation::getGridMap(grid_map_msg::GridMap& map)
{
  submapPoint_.header.stamp = ros::Time(0);
  geometry_msgs::PointStamped submapPointTransformed;

  try {
    transformListener_.transformPoint(mapFrameId_, submapPoint_, submapPointTransformed);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  GetGridMap submapService;
  submapService.request.positionX = submapPointTransformed.point.x;
  submapService.request.positionY = submapPointTransformed.point.y;
  submapService.request.lengthX = mapLength_.x();
  submapService.request.lengthY = mapLength_.y();
  submapService.request.dataDefinition.resize(requestedMapTypes_.size());

  for (unsigned int i = 0; i < requestedMapTypes_.size(); ++i) {
    submapService.request.dataDefinition[i] = requestedMapTypes_[i];
  }

  if (!submapClient_.call(submapService)) return false;
  map = submapService.response.gridMap;
  return true;
}

void TraversabilityEstimation::computeTraversability(grid_map::GridMap& elevationMap)
{
  // Run the filter chain
  ROS_INFO("Update Filter.");

  if (filter_chain_.update(elevationMap, traversabilityMap_)) {
    // Add to traversability map
    elevationMap.add(traversabilityType_, traversabilityMap_.get(traversabilityType_));
  }
  else {
    ROS_ERROR("Could not update the filter chain! No traversability computed!");
  }
}

void TraversabilityEstimation::publishAsOccupancyGrid(const grid_map::GridMap& map) const
{
  if (occupancyGridPublisher_.getNumSubscribers () < 1) return;
  OccupancyGrid occupancyGrid;
  // This flips data from traversability to occupancy.
  map.toOccupancyGrid(occupancyGrid, traversabilityType_, 1.0, 0.0);
  occupancyGridPublisher_.publish(occupancyGrid);
}

} /* namespace */
