/*
 * TraversabilityEstimation.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"

// Grid Map
#include <grid_map/Polygon.hpp>
#include <grid_map_lib/GridMap.hpp>
#include <grid_map_lib/iterators/CircleIterator.hpp>
#include <grid_map_lib/iterators/PolygonIterator.hpp>
#include <grid_map_msg/GetGridMap.h>

// Traversability estimation
#include "traversability_msgs/CheckFootprintPath.h"

//STL
#include <deque>

#include <Eigen/Geometry>

using namespace std;
using namespace nav_msgs;
using namespace grid_map_msg;

namespace traversability_estimation {

TraversabilityEstimation::TraversabilityEstimation(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      traversabilityType_("traversability"),
      slopeType_("traversability_slope"),
      stepType_("traversability_step"),
      roughnessType_("traversability_roughness"),
      filter_chain_("grid_map::GridMap")
{
  ROS_INFO("Traversability estimation node started.");

  readParameters();
  submapClient_ = nodeHandle_.serviceClient<GetGridMap>(submapServiceName_);
  traversabilityGridPublisher_ = nodeHandle_.advertise<OccupancyGrid>(
      "traversability_map", 1, true);
  slopeFilterGridPublisher_ = nodeHandle_.advertise<OccupancyGrid>("slope_map",
                                                                   1, true);
  stepFilterGridPublisher_ = nodeHandle_.advertise<OccupancyGrid>("step_map", 1,
                                                                  true);
  roughnessFilterGridPublisher_ = nodeHandle_.advertise<OccupancyGrid>(
      "roughness_map", 1, true);
  footprintPolygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("footprint_polygon", 1, true);

  updateTimer_ = nodeHandle_.createTimer(
      updateDuration_, &TraversabilityEstimation::updateTimerCallback, this);

  footprintPathService_ = nodeHandle_.advertiseService("check_footprint_path", &TraversabilityEstimation::checkFootprintPath, this);

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
  nodeHandle_.param("submap_service", submapServiceName_,
                    string("/get_grid_map"));

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
  ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
  submapClient_.waitForExistence();
  ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
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
    transformListener_.transformPoint(mapFrameId_, submapPoint_,
                                      submapPointTransformed);
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

  if (!submapClient_.call(submapService))
    return false;
  map = submapService.response.gridMap;
  return true;
}

void TraversabilityEstimation::computeTraversability(grid_map::GridMap& elevationMap)
{
  // Run the filter chain
  if (filter_chain_.update(elevationMap, traversabilityMap_)) {
    // Add to traversability map
    elevationMap.add(traversabilityType_,
                     traversabilityMap_.get(traversabilityType_));

    if (traversabilityMap_.exists(slopeType_)) {
      elevationMap.add(slopeType_, traversabilityMap_.get(slopeType_));
    }
    if (traversabilityMap_.exists(stepType_)) {
      elevationMap.add(stepType_, traversabilityMap_.get(stepType_));
    }
    if (traversabilityMap_.exists(roughnessType_)) {
      elevationMap.add(roughnessType_, traversabilityMap_.get(roughnessType_));
    }
  } else {
    ROS_ERROR("Could not update the filter chain! No traversability computed!");
  }
}

void TraversabilityEstimation::publishAsOccupancyGrid(
    const grid_map::GridMap& map) const
{
  if (traversabilityGridPublisher_.getNumSubscribers() >= 1) {
    OccupancyGrid traversabilityGrid;
    // This flips data from traversability to occupancy.
    map.toOccupancyGrid(traversabilityGrid, traversabilityType_, 1.0, 0.0);
    traversabilityGridPublisher_.publish(traversabilityGrid);
  }

  if (map.exists(slopeType_)) {
    if (slopeFilterGridPublisher_.getNumSubscribers() >= 1) {
      OccupancyGrid slopeGrid;
      map.toOccupancyGrid(slopeGrid, slopeType_, 1.0, 0.0);
      slopeFilterGridPublisher_.publish(slopeGrid);
    }
  }

  if (map.exists(stepType_)) {
    if (stepFilterGridPublisher_.getNumSubscribers() >= 1) {
      OccupancyGrid stepGrid;
      map.toOccupancyGrid(stepGrid, stepType_, 1.0, 0.0);
      stepFilterGridPublisher_.publish(stepGrid);
    }
  }

  if (map.exists(roughnessType_)) {
    if (roughnessFilterGridPublisher_.getNumSubscribers() >= 1) {
      OccupancyGrid roughnessGrid;
      map.toOccupancyGrid(roughnessGrid, roughnessType_, 1.0, 0.0);
      roughnessFilterGridPublisher_.publish(roughnessGrid);
    }
  }
}

bool TraversabilityEstimation::checkFootprintPath(
    traversability_msgs::CheckFootprintPath::Request& request,
    traversability_msgs::CheckFootprintPath::Response& response)
{
  if (!traversabilityMap_.exists(traversabilityType_)) {
    ROS_WARN("Failed to retrieve traversability map.");
    return false;
  }

  int arraySize = request.path.poses.poses.size();

  if (arraySize == 0) {
    ROS_WARN("No footprint path available to check!");
    return false;
  }

  double radius = request.path.radius;
  response.is_safe = true;
  response.traversability = 0.0;
  Eigen::Vector2d position;


  double traversability = 0.0;
  std::vector<std::string> validTypes;
  validTypes.push_back(traversabilityType_);

  if (arraySize == 1) {
    int nCells = 0;
    position.x() = request.path.poses.poses[0].position.x;
    position.y() = request.path.poses.poses[0].position.y;

    for (grid_map_lib::CircleIterator submapIterator(traversabilityMap_,
                                                     position, radius);
        !submapIterator.isPassedEnd(); ++submapIterator) {
      if (!traversabilityMap_.isValid(*submapIterator, validTypes))
        continue;

      if (traversabilityMap_.at(traversabilityType_, *submapIterator) == 0.0) {
        response.is_safe = false;
        traversability = 0.0;
        break;
      }
      traversability += traversabilityMap_.at(traversabilityType_,
                                              *submapIterator);
      nCells++;
    }
    response.traversability += traversability / nCells;
  }
  else {
    for (int i=0; i<(arraySize-1); i++) {
      const int nVertices = 6;

      Eigen::Vector2d centerStart, centerEnd;
      Eigen::Vector2d centerToVertex, centerToVertexTemp;

      centerStart.x() = request.path.poses.poses[i].position.x;
      centerStart.y() = request.path.poses.poses[i].position.y;
      centerEnd.x() = request.path.poses.poses[i+1].position.x;
      centerEnd.y() = request.path.poses.poses[i+1].position.y;
      centerToVertex = centerEnd - centerStart;
      centerToVertex.normalize();
      centerToVertex *= radius;
      ROS_DEBUG_STREAM(centerStart);
      ROS_DEBUG_STREAM(centerEnd);

      grid_map::Polygon polygon;
      for (int j=0; j<nVertices; j++) {
        double theta = M_PI_2 + j*M_PI/(nVertices-1);
        Eigen::Rotation2D<double> rot2d(theta);
        centerToVertexTemp = rot2d.toRotationMatrix()*centerToVertex;
        ROS_DEBUG_STREAM(centerStart+centerToVertexTemp);
        polygon.addVertex(centerStart+centerToVertexTemp);
      }
      for (int j=0; j<nVertices; j++) {
        double theta = 3*M_PI_2 + j*M_PI/(nVertices-1);
        Eigen::Rotation2D<double> rot2d(theta);
        centerToVertexTemp = rot2d.toRotationMatrix()*centerToVertex;
        ROS_DEBUG_STREAM(centerEnd+centerToVertexTemp);
        polygon.addVertex(centerEnd+centerToVertexTemp);
      }

      int nCells = 0;
      traversability = 0.0;
      for (grid_map_lib::PolygonIterator polygonIterator(traversabilityMap_, polygon); !polygonIterator.isPassedEnd(); ++polygonIterator) {
        if (!traversabilityMap_.isValid(*polygonIterator, validTypes))
          continue;

        if (traversabilityMap_.at(traversabilityType_, *polygonIterator) == 0.0) {
          response.is_safe = false;
          traversability = 0.0;
          break;
        }
        traversability += traversabilityMap_.at(traversabilityType_, *polygonIterator);
        nCells++;
      }
      response.traversability += traversability / (nCells * (arraySize-1));

      // Publish footprint polygon
      polygon.setFrameId(mapFrameId_);
      geometry_msgs::PolygonStamped polygonMsg;
      polygon.toMessage(polygonMsg);
      footprintPolygonPublisher_.publish(polygonMsg);
    }
  }
  return true;
}

} /* namespace */
