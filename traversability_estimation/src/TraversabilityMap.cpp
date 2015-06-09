/*
 * TraversabilityMap.cpp
 *
 *  Created on: Jun 09, 2014
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityMap.hpp"

// Grid Map
#include <grid_map_msgs/GetGridMap.h>

// ROS
#include <ros/package.h>
#include <geometry_msgs/Pose.h>

// kindr
#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/poses/PoseEigen.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

namespace traversability_estimation {

TraversabilityMap::TraversabilityMap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      traversabilityType_("traversability"),
      filter_chain_("grid_map::GridMap"),
      mapInitialized_(false),
      timerId_("check_footprint_timer"),
      timer_(timerId_, true)
{
  ROS_INFO("Traversability Map started.");

  readParameters();
  footprintPolygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("footprint_polygon", 1, true);
  traversabilityMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("traversability_map", 1);
}

TraversabilityMap::~TraversabilityMap()
{
  updateTimer_.stop();
  nodeHandle_.shutdown();
}

bool TraversabilityMap::readParameters()
{
  // Read footprint polygon.
  XmlRpc::XmlRpcValue footprint;
  nodeHandle_.getParam("footprint_polygon", footprint);
  if (footprint.size() < 2) {
    ROS_WARN("Footprint polygon must consist of at least 3 points. Only %i points found.", footprint.size());
    footprintPoints_.clear();
  } else {
    geometry_msgs::Point32 pt;
    pt.z = 0.0;
    for (int i = 0; i < footprint.size(); i++) {
      pt.x = (double) footprint[i][0];
      pt.y = (double) footprint[i][1];
      footprintPoints_.push_back(pt);
    }
  }

  nodeHandle_.param("map_frame_id", mapFrameId_, string("map"));
  nodeHandle_.param("robot_frame_id", robotFrameId_, string("robot"));
//  nodeHandle_.param("footprint_frame_id", robotFrameId_, string("robot"));
//  grid_map::Position mapCenter;
//  nodeHandle_.param("map_center_x", mapCenter.x(), 0.0);
//  nodeHandle_.param("map_center_y", mapCenter.y(), 0.0);
//  submapPoint_.header.frame_id = robotFrameId_;
//  submapPoint_.point.x = mapCenter.x();
//  submapPoint_.point.y = mapCenter.y();
//  submapPoint_.point.z = 0.0;
//  nodeHandle_.param("map_length_x", mapLength_.x(), 5.0);
//  nodeHandle_.param("map_length_y", mapLength_.y(), 5.0);
  nodeHandle_.param("traversability_default", traversabilityDefault_, 0.5);
//  nodeHandle_.param("footprint_yaw", footprintYaw_, M_PI_2);

  // Configure filter chain
  if (!filter_chain_.configure("traversability_map_filters", nodeHandle_)) {
    ROS_ERROR("Could not configure the filter chain!");
  }
  return true;
}

void TraversabilityMap::setElevationMap(const grid_map_msg::GridMap msg)
{

}

void TraversabilityMap::setTraversabilityMap(const grid_map_msg::GridMap msg)
{

}

void TraversabilityMap::computeTraversability()
{
  // Initialize timer.
  string timerId = "traversability_timer";
  sm::timing::Timer timer(timerId, true);
  if (timer.isTiming()) timer.stop();
  timer.start();

  if (mapInitialized_) {
    if (!filter_chain_.update(elevationMap_, traversabilityMap_)) ROS_ERROR("Traversability Estimation: Could not update the filter chain! No traversability computed!");
    grid_map_msgs::GridMap mapMessage;
    grid_map::GridMapRosConverter::toMessage(traversabilityMap_, mapMessage);
    if (!traversabilityMapPublisher_.getNumSubscribers() < 1) traversabilityMapPublisher_.publish(mapMessage);
  } else {
    ROS_ERROR("Traversability Estimation: Elevation map is not initialized!");
  }

//  grid_map_msgs::GridMap mapMessage;
//  if (!mapInitialized_) {
//    ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
//    submapClient_.waitForExistence();
//    ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
//    if (getGridMap(mapMessage)) {
//      grid_map::GridMapRosConverter::fromMessage(mapMessage, elevationMap_);
//      if (!filter_chain_.update(elevationMap_, traversabilityMap_)) ROS_ERROR("Could not update the filter chain! No traversability computed!");
//      grid_map::GridMapRosConverter::toMessage(traversabilityMap_, mapMessage);
//      if (!traversabilityMapPublisher_.getNumSubscribers() < 1)
//        traversabilityMapPublisher_.publish(mapMessage);
//    } else {
//      ROS_WARN("Failed to retrieve elevation grid map.");
//    }
//  } else {
//    if (!filter_chain_.update(elevationMap_, traversabilityMap_)) ROS_ERROR("Could not update the filter chain! No traversability computed!");
//    grid_map::GridMapRosConverter::toMessage(traversabilityMap_, mapMessage);
//    if (!traversabilityMapPublisher_.getNumSubscribers() < 1)
//      traversabilityMapPublisher_.publish(mapMessage);
//  }

  timer.stop();
  ROS_INFO("Traversability map has been updated in %f s.", sm::timing::Timing::getTotalSeconds(timerId));
  sm::timing::Timing::reset(timerId);
}

//void TraversabilityMap::updateTraversability()
//{
//  computeTraversability();
//  info.header.frame_id = mapFrameId_;
//  info.header.stamp = ros::Time::now();
//  info.resolution = traversabilityMap_.getResolution();
//  info.length_x = traversabilityMap_.getLength()[0];
//  info.length_y = traversabilityMap_.getLength()[1];
//  geometry_msgs::Pose pose;
//  grid_map::Position position = traversabilityMap_.getPosition();
//  pose.position.x = position[0];
//  pose.position.y = position[1];
//  pose.orientation.w = 1.0;
//  info.pose = pose;
//
//  sm::timing::Timing::reset(timerId_);
//}

void TraversabilityMap::checkFootprintPath(const traversability_msgs::FootprintPath& path, traversability_msgs::TraversabilityResult& result)
{
  if (timer_.isTiming()) timer_.stop();
    timer_.start();

  if (!traversabilityMap_.exists(traversabilityType_)) {
    ROS_WARN("Traversability Estimation: Failed to retrieve traversability map.");
    return;
  }

  const int arraySize = path.poses.poses.size();

  if (arraySize == 0) {
    ROS_WARN("Traversability Estimation: No footprint path available to check!");
    return;
  }

  double radius = path.radius;
  result.is_safe = false;
  result.traversability = 0.0;
  result.area = 0.0;
  grid_map::Polygon polygon;
  double traversability = 0.0;
  grid_map::Position start, end;

  if (path.footprint.polygon.points.size() == 0) {
    for (int i = 0; i < arraySize; i++) {
      start = end;
      end.x() = path.poses.poses[i].position.x;
      end.y() = path.poses.poses[i].position.y;

      if (arraySize == 1) {
        polygon = polygon.convexHullCircle(end, radius);
        if (!checkInclination(end, end))
          return;
        if (!isTraversable(polygon, traversability))
          return;
        result.traversability = traversability;
      }

      if (arraySize > 1 && i > 0) {
        polygon = polygon.convexHullCircles(start, end, radius);
        if (!checkInclination(start, end))
          return;
        if (!isTraversable(polygon, traversability))
          return;
        result.traversability += traversability / (arraySize - 1);
      }
      result.area = polygon.getArea();
    }
  } else {
    grid_map::Polygon polygon1, polygon2;
    polygon1.setFrameId(mapFrameId_);
    polygon2.setFrameId(mapFrameId_);
    for (int i = 0; i < arraySize; i++) {
      polygon1 = polygon2;
      start = end;
      polygon2.removeVertices();
      grid_map::Position3 positionToVertex, positionToVertexTransformed;
      Eigen::Translation<double, 3> toPosition;
      Eigen::Quaterniond orientation;

      toPosition.x() = path.poses.poses[i].position.x;
      toPosition.y() = path.poses.poses[i].position.y;
      toPosition.z() = path.poses.poses[i].position.z;
      orientation.x() = path.poses.poses[i].orientation.x;
      orientation.y() = path.poses.poses[i].orientation.y;
      orientation.z() = path.poses.poses[i].orientation.z;
      orientation.w() = path.poses.poses[i].orientation.w;
      end.x() = toPosition.x();
      end.y() = toPosition.y();

      for (const auto& point : path.footprint.polygon.points) {
        positionToVertex.x() = point.x;
        positionToVertex.y() = point.y;
        positionToVertex.z() = point.z;
        positionToVertexTransformed = toPosition * orientation
            * positionToVertex;

        grid_map::Position vertex;
        vertex.x() = positionToVertexTransformed.x();
        vertex.y() = positionToVertexTransformed.y();
        polygon2.addVertex(vertex);
      }

      if (path.conservative && i > 0) {
        grid_map::Vector startToEnd = end - start;
        vector<grid_map::Position> vertices1 = polygon1.getVertices();
        vector<grid_map::Position> vertices2 = polygon2.getVertices();
        for (const auto& vertex : vertices1) {
          polygon2.addVertex(vertex + startToEnd);
        }
        for (const auto& vertex : vertices2) {
          polygon1.addVertex(vertex - startToEnd);
        }
      }

      if (arraySize == 1) {
        polygon = polygon2;
        if (!checkInclination(end, end))
          return;
        if (!isTraversable(polygon, traversability))
          return;
        result.traversability = traversability;
        result.area = polygon.getArea();
      }

      if (arraySize > 1 && i > 0) {
        polygon = polygon.convexHull(polygon1, polygon2);
        if (!checkInclination(start, end))
          return;
        if (!isTraversable(polygon, traversability))
          return;
        result.traversability += traversability / (arraySize - 1);
        if (i > 1) {
          result.area += polygon.getArea() - polygon1.getArea();
        } else {
          result.area = polygon.getArea();
        }
      }
    }
  }

  polygon.setFrameId(mapFrameId_);
  polygon.setTimestamp(path.footprint.header.stamp.toNSec());
  geometry_msgs::PolygonStamped polygonMsg;
  grid_map::PolygonRosConverter::toMessage(polygon, polygonMsg);
  if (!footprintPolygonPublisher_.getNumSubscribers() < 1)
    footprintPolygonPublisher_.publish(polygonMsg);

  result.is_safe = true;
  ROS_DEBUG_STREAM("Traversability: " << result.traversability);
  timer_.stop();
  ROS_DEBUG("Mean: %f s, Min: %f s, Max: %f s.",
            sm::timing::Timing::getMeanSeconds(timerId_),
            sm::timing::Timing::getMinSeconds(timerId_),
            sm::timing::Timing::getMaxSeconds(timerId_));
}

bool TraversabilityMap::isTraversable(const grid_map::Polygon& polygon, double& traversability)
{
  int nCells = 0, nSteps = 0;
  traversability = 0.0;
  double windowRadius = 0.1; // TODO: read this as a parameter?
  double criticalLength = 0.1;
  int nSlopesCritical = floor(2 * windowRadius * criticalLength / pow(traversabilityMap_.getResolution(), 2));

  // Check for traversability.
  for (grid_map::PolygonIterator polygonIterator(traversabilityMap_, polygon);
      !polygonIterator.isPassedEnd(); ++polygonIterator) {

    // Check for steps
    if (traversabilityMap_.at(stepType_, *polygonIterator) == 0.0)
      nSteps++;
    if (nSteps > 3)
      return false;

    // Check for slopes
    if (traversabilityMap_.at(slopeType_, *polygonIterator) == 0.0) {
      // Requested position (center) of circle in map.
      grid_map::Position center;
      traversabilityMap_.getPosition(*polygonIterator, center);
      int nSlopes = 0;
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center,
                                                   windowRadius);
          !circleIterator.isPassedEnd(); ++circleIterator) {
        if (traversabilityMap_.at(slopeType_, *circleIterator) == 0.0)
          nSlopes++;
        if (nSlopes > nSlopesCritical)
          return false;
      }
    }

    // Check for roughness
    if (traversabilityMap_.at(roughnessType_, *polygonIterator) == 0.0) {
      // Requested position (center) of circle in map.
      grid_map::Position center;
      traversabilityMap_.getPosition(*polygonIterator, center);
      int nRoughness = 0;
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center,
                                                   windowRadius);
          !circleIterator.isPassedEnd(); ++circleIterator) {
        if (traversabilityMap_.at(roughnessType_, *circleIterator) == 0.0)
          nRoughness++;
        if (nRoughness > (nSlopesCritical * 0.75))
          return false;
      }
    }
  }

  for (grid_map::PolygonIterator polygonIterator(traversabilityMap_,
                                                 polygon);
      !polygonIterator.isPassedEnd(); ++polygonIterator) {
    nCells++;
    if (!traversabilityMap_.isValid(*polygonIterator,
                                    traversabilityType_)) {
      traversability += traversabilityDefault_;
    } else {
      traversability += traversabilityMap_.at(traversabilityType_, *polygonIterator);
    }
  }
  traversability /= nCells;
  return true;
}

bool TraversabilityMap::checkInclination(const grid_map::Position start, const grid_map::Position end)
{
  if (end == start) {
    if (traversabilityMap_.atPosition(robotSlopeType_, start) == 0.0) return false;
  } else {
    grid_map::Index startIndex, endIndex;
    traversabilityMap_.getIndex(start, startIndex);
    traversabilityMap_.getIndex(end, endIndex);
    for (grid_map::LineIterator lineIterator(traversabilityMap_, startIndex, endIndex); !lineIterator.isPassedEnd(); ++lineIterator) {
      if (!traversabilityMap_.isValid(*lineIterator, robotSlopeType_)) continue;
      if (traversabilityMap_.at(robotSlopeType_, *lineIterator) == 0.0) return false;
    }
  }
  return true;
}

} /* namespace */
