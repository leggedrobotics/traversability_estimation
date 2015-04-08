/*
 * TraversabilityEstimation.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"

// Grid Map
#include <grid_map_msgs/GetGridMap.h>

// Traversability estimation
#include "traversability_msgs/CheckFootprintPath.h"

//STL
#include <deque>

#include <Eigen/Geometry>

using namespace std;
using namespace nav_msgs;
using namespace grid_map_msgs;

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
  footprintPolygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("footprint_polygon", 1, true);
  traversabilityMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("traversability_map", 1);

  updateTimer_ = nodeHandle_.createTimer(
      updateDuration_, &TraversabilityEstimation::updateTimerCallback, this);

  footprintPathService_ = nodeHandle_.advertiseService("check_footprint_path", &TraversabilityEstimation::checkFootprintPath, this);

  requestedMapTypes_.push_back("elevation");
  requestedMapTypes_.push_back("variance");
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
  nodeHandle_.param("robot_frame_id", robotFrameId_, string("robot"));
  double mapCenterX, mapCenterY;
  nodeHandle_.param("map_center_x", mapCenterX, 0.0);
  nodeHandle_.param("map_center_y", mapCenterY, 0.0);
  submapPoint_.header.frame_id = robotFrameId_;
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

void TraversabilityEstimation::updateTimerCallback(
    const ros::TimerEvent& timerEvent)
{
  grid_map_msgs::GridMap mapMessage;
  ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
  submapClient_.waitForExistence();
  ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
  if (getGridMap(mapMessage)) {
    grid_map::GridMap elevationMap, traversabilityMap;
    grid_map::GridMapRosConverter::fromMessage(mapMessage, elevationMap);
    computeTraversability(elevationMap, traversabilityMap);
    grid_map::GridMapRosConverter::toMessage(traversabilityMap, mapMessage);
    if (!traversabilityMapPublisher_.getNumSubscribers() < 1)
      traversabilityMapPublisher_.publish(mapMessage);
  } else {
    ROS_WARN("Failed to retrieve elevation grid map.");
  }
}

bool TraversabilityEstimation::getGridMap(grid_map_msgs::GridMap& map)
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
  submapService.request.position_x = submapPointTransformed.point.x;
  submapService.request.position_y = submapPointTransformed.point.y;
  submapService.request.length_x = mapLength_.x();
  submapService.request.length_y = mapLength_.y();
  submapService.request.layers = requestedMapTypes_;

  if (!submapClient_.call(submapService))
    return false;
  map = submapService.response.map;

  return true;
}

bool TraversabilityEstimation::computeTraversability(const grid_map::GridMap& elevationMap, grid_map::GridMap& traversabilityMap)
{
  // Run the filter chain
  if (filter_chain_.update(elevationMap, traversabilityMap)) {
    traversabilityMap_ = traversabilityMap;
    return true;
  } else {
    ROS_ERROR("Could not update the filter chain! No traversability computed!");
    return false;
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

  const int arraySize = request.path.poses.poses.size();

  if (arraySize == 0) {
    ROS_WARN("No footprint path available to check!");
    return false;
  }

  double radius = request.path.radius;
  bool isSafe = true;
  response.traversability = 0.0;
  grid_map::Polygon polygon;
  Eigen::Vector2d position;

  double traversability = 0.0;

  if (request.path.footprint.polygon.points.size() == 0) {
    Eigen::Vector2d centerStart, centerEnd;
    for (int i = 0; i < arraySize; i++) {
      centerEnd = centerStart;
      centerStart.x() = request.path.poses.poses[i].position.x;
      centerStart.y() = request.path.poses.poses[i].position.y;

      if (arraySize == 1) {
        polygon = polygon.convexHullCircle(centerStart, radius);
        if (!isTraversable(polygon, traversability))
          isSafe = false;
        response.traversability = traversability;
      }

      if (arraySize > 1 && i > 0) {
        polygon = polygon.convexHullCircles(centerStart, centerEnd, radius);
        if (!isTraversable(polygon, traversability))
          isSafe = false;
        response.traversability += traversability / (arraySize - 1);
      }
    }
  } else {
    grid_map::Polygon polygon1, polygon2;
    polygon1.setFrameId(mapFrameId_);
    polygon2.setFrameId(mapFrameId_);
    for (int i = 0; i < arraySize; i++) {
      polygon2 = polygon1;
      Eigen::Vector3f footprintVertex, footprintVertexTransformed;
      Eigen::Translation<float, 3> toPosition;
      Eigen::Quaternionf orientation;

      toPosition.x() = request.path.poses.poses[i].position.x;
      toPosition.y() = request.path.poses.poses[i].position.y;
      toPosition.z() = request.path.poses.poses[i].position.z;
      orientation.x() = request.path.poses.poses[i].orientation.x;
      orientation.y() = request.path.poses.poses[i].orientation.y;
      orientation.z() = request.path.poses.poses[i].orientation.z;
      orientation.w() = request.path.poses.poses[i].orientation.w;

      for (const auto& point : request.path.footprint.polygon.points) {
        footprintVertex.x() = point.x;
        footprintVertex.y() = point.y;
        footprintVertex.z() = point.z;
        footprintVertexTransformed = toPosition * orientation * footprintVertex;

        Eigen::Vector2d vertex;
        vertex.x() = footprintVertexTransformed.x();
        vertex.y() = footprintVertexTransformed.y();
        polygon1.addVertex(vertex);
      }

      if (arraySize == 1) {
        polygon = polygon1;
        if (!isTraversable(polygon, traversability))
          isSafe = false;
        response.traversability = traversability;
      }

      if (arraySize > 1 && i > 0) {
        polygon = polygon.convexHull(polygon1, polygon2);

        if (!isTraversable(polygon, traversability))
          isSafe = false;
        response.traversability += traversability / (arraySize - 1);
      }
    }
  }
  polygon.setFrameId(mapFrameId_);
  polygon.setTimestamp(request.path.footprint.header.stamp.toNSec());
  geometry_msgs::PolygonStamped polygonMsg;
  grid_map::PolygonRosConverter::toMessage(polygon, polygonMsg);
  if (!footprintPolygonPublisher_.getNumSubscribers() < 1)
    footprintPolygonPublisher_.publish(polygonMsg);

  response.is_safe = isSafe;
  return true;
}

bool TraversabilityEstimation::isTraversable(const grid_map::Polygon& polygon, double& traversability)
{
  int nCells = 0;
  bool isSafe = true;
  traversability = 0.0;
  for (grid_map::PolygonIterator polygonIterator(traversabilityMap_,
                                                 polygon);
      !polygonIterator.isPassedEnd(); ++polygonIterator) {
    if (!traversabilityMap_.isValid(*polygonIterator,
                                    traversabilityType_))
      continue;

    nCells++;
    if (traversabilityMap_.at(traversabilityType_, *polygonIterator)
        == 0.0) {
      isSafe = false;
      traversability = 0.0;
      break;
    }
    traversability += traversabilityMap_.at(traversabilityType_, *polygonIterator);
  }
  traversability /= nCells;
  return isSafe;
}

} /* namespace */
