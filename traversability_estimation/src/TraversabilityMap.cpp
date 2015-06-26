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

#include <google/profiler.h>

using namespace std;

namespace traversability_estimation {

TraversabilityMap::TraversabilityMap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      traversabilityType_("traversability"),
      slopeType_("traversability_slope"),
      stepType_("traversability_step"),
      roughnessType_("traversability_roughness"),
      robotSlopeType_("robot_slope"),
      filter_chain_("grid_map::GridMap"),
      elevationMapInitialized_(false),
      traversabilityMapInitialized_(false)
//      timerId_("check_footprint_timer"),
//      timer_(timerId_, true)
{
  ROS_INFO("Traversability Map started.");
//  ProfilerStart("/home/martiwer/Documents/Profile/traversability_map.prof");

  readParameters();
  footprintPolygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("footprint_polygon", 1, true);
  traversabilityMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("traversability_map", 1);

  elevationMapLayers_.push_back("elevation");
  elevationMapLayers_.push_back("variance");
  // TODO: Adapt map layers to traversability filters.
  traversabilityMapLayers_.push_back(traversabilityType_);
  traversabilityMapLayers_.push_back(slopeType_);
  traversabilityMapLayers_.push_back(stepType_);
  traversabilityMapLayers_.push_back(roughnessType_);
  traversabilityMapLayers_.push_back(robotSlopeType_);
}

TraversabilityMap::~TraversabilityMap()
{
//  ProfilerStop();
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
  nodeHandle_.param("traversability_default", traversabilityDefault_, 0.5);

  // Configure filter chain
  if (!filter_chain_.configure("traversability_map_filters", nodeHandle_)) {
    ROS_ERROR("Could not configure the filter chain!");
  }
  return true;
}

bool TraversabilityMap::setElevationMap(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMap elevationMap;
  grid_map::GridMapRosConverter::fromMessage(msg, elevationMap);
  for (auto& layer : elevationMapLayers_) {
    if (!elevationMap.exists(layer)) {
      ROS_WARN("Traversability Map: Can't set elevation map because there is no layer %s.", layer.c_str());
      return false;
    }
  }
  elevationMap_ = elevationMap;
  elevationMapInitialized_ = true;
  return true;
}

bool TraversabilityMap::setTraversabilityMap(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMap traversabilityMap;
  grid_map::GridMapRosConverter::fromMessage(msg, traversabilityMap);
  for (auto& layer : traversabilityMapLayers_) {
    if (!traversabilityMap.exists(layer)) {
      ROS_WARN("Traversability Map: Can't set traversability map because there exists no layer %s.", layer.c_str());
      return false;
    }
  }
  traversabilityMap_ = traversabilityMap;
  traversabilityMapInitialized_ = true;
  return true;
}

grid_map::GridMap TraversabilityMap::getTraversabilityMap()
{
  return traversabilityMap_;
}

bool TraversabilityMap::computeTraversability()
{
  // reset check footprint timer.
//  sm::timing::Timing::reset(timerId_);
  // Initialize timer.
  string timerId = "traversability";
  sm::timing::Timer timer(timerId, true);
  if (timer.isTiming()) timer.stop();
  timer.start();

  if (elevationMapInitialized_) {
    if (!filter_chain_.update(elevationMap_, traversabilityMap_)) {
      ROS_ERROR("Traversability Estimation: Could not update the filter chain! No traversability computed!");
      traversabilityMapInitialized_ = false;
      return false;
    }
    grid_map_msgs::GridMap mapMessage;
    grid_map::GridMapRosConverter::toMessage(traversabilityMap_, mapMessage);
    if (!traversabilityMapPublisher_.getNumSubscribers() < 1) traversabilityMapPublisher_.publish(mapMessage);
  } else {
    ROS_ERROR("Traversability Estimation: Elevation map is not initialized!");
    traversabilityMapInitialized_ = false;
    return false;
  }
  traversabilityMapInitialized_ = true;
  traversabilityMap_.add("step_footprint");

  timer.stop();
  ROS_INFO("Traversability map has been updated in %f s.", sm::timing::Timing::getTotalSeconds(timerId));
  sm::timing::Timing::reset(timerId);
  return true;
}

bool TraversabilityMap::traversabilityFootprint(double footprintYaw)
{
  if (!traversabilityMapInitialized_) return false;

  // Initialize timer.
  string timerId = "traversability_footprint";
  sm::timing::Timer timer(timerId, true);
  if (timer.isTiming()) timer.stop();
  timer.start();

  traversabilityMap_.add("traversability_x");
  traversabilityMap_.add("traversability_rot");

  grid_map::Position position;
  grid_map::Polygon polygonX, polygonRot;

  ROS_DEBUG_STREAM("footprint yaw: " << footprintYaw);
  // Compute Orientation
  kindr::rotations::eigen_impl::RotationQuaternionPD xquat, rquat;
  kindr::rotations::eigen_impl::AngleAxisPD rotationAxis(footprintYaw, 0.0,
                                                         0.0, 1.0);
  rquat = rotationAxis * xquat;
  Eigen::Quaterniond orientationX = xquat.toImplementation();
  Eigen::Quaterniond orientationRot = rquat.toImplementation();

  for (grid_map::GridMapIterator iterator(traversabilityMap_); !iterator.isPassedEnd(); ++iterator) {
    polygonX.removeVertices();
    polygonRot.removeVertices();
    traversabilityMap_.getPosition(*iterator, position);

    grid_map::Position3 positionToVertex, positionToVertexTransformedX, positionToVertexTransformedRot;
    Eigen::Translation<double, 3> toPosition;
    Eigen::Quaterniond orientation;

    toPosition.x() = position.x();
    toPosition.y() = position.y();
    toPosition.z() = 0.0;

    for (const auto& point : footprintPoints_) {
      positionToVertex.x() = point.x;
      positionToVertex.y() = point.y;
      positionToVertex.z() = point.z;
      positionToVertexTransformedX = toPosition * orientationX * positionToVertex;
      positionToVertexTransformedRot = toPosition * orientationRot * positionToVertex;

      grid_map::Position vertexX, vertexRot;
      vertexX.x() = positionToVertexTransformedX.x();
      vertexRot.x() = positionToVertexTransformedRot.x();
      vertexX.y() = positionToVertexTransformedX.y();
      vertexRot.y() = positionToVertexTransformedRot.y();
      polygonX.addVertex(vertexX);
      polygonRot.addVertex(vertexRot);
    }

    double traversability;
    isTraversable(polygonX, traversability);
    traversabilityMap_.at("traversability_x", *iterator) = traversability;
    isTraversable(polygonRot, traversability);
    traversabilityMap_.at("traversability_rot", *iterator) = traversability;

//    if (!checkInclination(position, position)) {
//      traversabilityMap_.at("traversability_x", *iterator) = 0.0;
//    } else {
//      double traversability;
//      isTraversable(polygonX, traversability);
//      traversabilityMap_.at("traversability_x", *iterator) = traversability;
//    }
//
//    if (!checkInclination(position, position)) {
//      traversabilityMap_.at("traversability_rot", *iterator) = 0.0;
//    } else {
//      double traversability;
//      isTraversable(polygonRot, traversability);
//      traversabilityMap_.at("traversability_rot", *iterator) = traversability;
//    }
  }

  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(traversabilityMap_, mapMessage);
  if (!traversabilityMapPublisher_.getNumSubscribers() < 1)
    traversabilityMapPublisher_.publish(mapMessage);

  timer.stop();
  ROS_INFO("Traversability of footprint has been computed in %f s.", sm::timing::Timing::getTotalSeconds(timerId));
  sm::timing::Timing::reset(timerId);
  return true;
}

bool TraversabilityMap::checkFootprintPath(const traversability_msgs::FootprintPath& path, traversability_msgs::TraversabilityResult& result)
{
//  if (timer_.isTiming()) timer_.stop();
//  timer_.start();

  if (!traversabilityMap_.exists(traversabilityType_)) {
    ROS_WARN("Traversability Estimation: Failed to retrieve traversability map.");
    return false;
  }

  const int arraySize = path.poses.poses.size();
  if (arraySize == 0) {
    ROS_WARN("Traversability Estimation: This path has no poses to check!");
    return false;
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
//        if (!checkInclination(end, end))
//          return true;
        if (!isTraversable(polygon, traversability))
          return true;
        result.traversability = traversability;
      }

      if (arraySize > 1 && i > 0) {
        polygon = polygon.convexHullCircles(start, end, radius);
//        if (!checkInclination(start, end))
//          return true;
        if (!isTraversable(polygon, traversability))
          return true;
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
//        if (!checkInclination(end, end))
//          return true;
        if (!isTraversable(polygon, traversability))
          return true;
        result.traversability = traversability;
        result.area = polygon.getArea();
      }

      if (arraySize > 1 && i > 0) {
        polygon = polygon.convexHull(polygon1, polygon2);
//        if (!checkInclination(start, end))
//          return true;
        if (!isTraversable(polygon, traversability))
          return true;
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
//  timer_.stop();
//  ROS_DEBUG("Mean: %f s, Min: %f s, Max: %f s.",
//            sm::timing::Timing::getMeanSeconds(timerId_),
//            sm::timing::Timing::getMinSeconds(timerId_),
//            sm::timing::Timing::getMaxSeconds(timerId_));

  return true;
}

bool TraversabilityMap::isTraversable(const grid_map::Polygon& polygon, double& traversability)
{
  int nCells = 0, nSteps = 0;
  traversability = 0.0;
  double windowRadius = 0.1; // TODO: read this as a parameter?
  double criticalLength = 0.1;
  int nSlopesCritical = floor(2 * windowRadius * criticalLength / pow(traversabilityMap_.getResolution(), 2));

  // Check for steps
  if (!checkForStep(polygon)) {
    return false;
  }

  // Check for traversability.
  for (grid_map::PolygonIterator polygonIterator(traversabilityMap_, polygon);
      !polygonIterator.isPassedEnd(); ++polygonIterator) {

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

bool TraversabilityMap::updateFilter()
{
  // Reconfigure filter chain.
  filter_chain_.clear();
  if (!filter_chain_.configure("traversability_map_filters", nodeHandle_)) {
    ROS_ERROR("Could not configure the filter chain!");
    return false;
  }
  return true;
}

bool TraversabilityMap::checkForStep(const grid_map::Polygon& polygon)
{
  // Define Parameter. TODO: read this parameter;
  double windowRadius = 0.075;
  double criticalStep = 0.12;
  double gapWidth = 0.3;

  for (grid_map::PolygonIterator polygonIterator(traversabilityMap_, polygon);
      !polygonIterator.isPassedEnd(); ++polygonIterator) {
    if (traversabilityMap_.at(stepType_, *polygonIterator) == 0.0) {
      if (traversabilityMap_.isValid(*polygonIterator, "step_footprint")) {
        if (traversabilityMap_.at("step_footprint", *polygonIterator) == 0.0) return false;
        if (traversabilityMap_.at("step_footprint", *polygonIterator) == 1.0) continue;
      }
      vector<grid_map::Index> indices;
      grid_map::Position center;
      traversabilityMap_.getPosition(*polygonIterator, center);
      double height = traversabilityMap_.at("elevation", *polygonIterator);
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center, windowRadius);
          !circleIterator.isPassedEnd(); ++circleIterator) {
        if (traversabilityMap_.at("elevation", *circleIterator) > criticalStep + height && traversabilityMap_.at(stepType_, *circleIterator) == 0.0) indices.push_back(*circleIterator);
      }
      if (indices.empty()) indices.push_back(*polygonIterator);
      for (auto& index : indices) {
        grid_map::Length subMapLength_(2.5*traversabilityMap_.getResolution(), 2.5*traversabilityMap_.getResolution());
        grid_map::Position subMapPos;
        bool isSuccess;
        traversabilityMap_.getPosition(index, subMapPos);
        grid_map::Vector toCenter = center - subMapPos;
        grid_map::GridMap subMap = traversabilityMap_.getSubmap(subMapPos, subMapLength_, isSuccess);
        if (!isSuccess) {
          ROS_WARN("Traversability map: Check for step window could not retrieve submap.");
          traversabilityMap_.at("step_footprint", *polygonIterator) = 0.0;
          return false;
        }
        height = traversabilityMap_.at("elevation", index);
        for (grid_map::GridMapIterator subMapIterator(subMap); !subMapIterator.isPassedEnd(); ++subMapIterator) {
          if (subMap.at(stepType_, *subMapIterator) == 0.0 && subMap.at("elevation", *subMapIterator) < height - criticalStep) {
            grid_map::Position pos;
            subMap.getPosition(*subMapIterator, pos);
            grid_map::Vector vec = pos - subMapPos;
            if (vec.norm() < 0.025) continue;
            if (toCenter.norm() > 0.025) {
              if (toCenter.dot(vec) < 0.0) continue;
            }
            pos = subMapPos + vec;
            while ((pos - subMapPos + vec).norm() < gapWidth && traversabilityMap_.isInside(pos + vec)) pos += vec;
            grid_map::Index endIndex;
            traversabilityMap_.getIndex(pos, endIndex);
            bool gapStart = false;
            bool gapEnd = false;
            for (grid_map::LineIterator lineIterator(traversabilityMap_, index, endIndex); !lineIterator.isPassedEnd(); ++lineIterator) {
              if (traversabilityMap_.at("elevation", *lineIterator) > height + criticalStep) {
                traversabilityMap_.at("step_footprint", *polygonIterator) = 0.0;
                return false;
              }
              if (traversabilityMap_.at("elevation", *lineIterator) < height - criticalStep || !traversabilityMap_.isValid(*lineIterator, "elevation")) {
                gapStart = true;
              } else if (gapStart) {
                gapEnd = true;
                break;
              }
            }
            if (gapStart && !gapEnd) {
              traversabilityMap_.at("step_footprint", *polygonIterator) = 0.0;
              return false;
            }
          }
        }
      }
      traversabilityMap_.at("step_footprint", *polygonIterator) = 1.0;
    }
  }
  return true;
}

} /* namespace */
