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
#include <geometry_msgs/PolygonStamped.h>

// kindr
#include <kindr/Core>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

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
      zPosition_(0),
      elevationMapInitialized_(false),
      traversabilityMapInitialized_(false),
      checkForRoughness_(false),
      checkRobotInclination_(false)
{
  ROS_INFO("Traversability Map started.");

  readParameters();
  traversabilityMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("traversability_map", 1);
  footprintPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("footprint_polygon", 1, true);

  elevationMapLayers_.push_back("elevation");
  elevationMapLayers_.push_back("variance");
  // TODO: Adapt map layers to traversability filters.
  traversabilityMapLayers_.push_back(traversabilityType_);
  traversabilityMapLayers_.push_back(slopeType_);
  traversabilityMapLayers_.push_back(stepType_);
  traversabilityMapLayers_.push_back(roughnessType_);
}

TraversabilityMap::~TraversabilityMap()
{
  nodeHandle_.shutdown();
}

bool TraversabilityMap::readParameters()
{
  // Read footprint polygon.
  XmlRpc::XmlRpcValue footprint;
  if (nodeHandle_.getParam("footprint/footprint_polygon", footprint)) {
    if (footprint.size() < 3) {
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
  } else {
    ROS_WARN("Traversability Map: No footprint polygon defined.");
  }

  nodeHandle_.param("map_frame_id", mapFrameId_, string("map"));
  nodeHandle_.param("footprint/traversability_default", traversabilityDefault_, 0.5);
  nodeHandle_.param("footprint/verify_roughness_footprint", checkForRoughness_, false);
  nodeHandle_.param("footprint/check_robot_inclination", checkRobotInclination_, false);
  nodeHandle_.param("max_gap_width", maxGapWidth_, 0.3);
  nodeHandle_.param("traversability_map_filters/stepFilter/critical_value", criticalStepHeight_, 0.12);

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
  zPosition_ = msg.info.pose.position.z;
  for (auto& layer : elevationMapLayers_) {
    if (!elevationMap.exists(layer)) {
      ROS_WARN("Traversability Map: Can't set elevation map because there is no layer %s.", layer.c_str());
      return false;
    }
  }
  boost::recursive_mutex::scoped_lock scopedLockForElevationMap(elevationMapMutex_);
  elevationMap_ = elevationMap;
  elevationMapInitialized_ = true;
  return true;
}

bool TraversabilityMap::setTraversabilityMap(const grid_map_msgs::GridMap& msg)
{
  grid_map::GridMap traversabilityMap;
  grid_map::GridMapRosConverter::fromMessage(msg, traversabilityMap);
  zPosition_ = msg.info.pose.position.z;
  for (auto& layer : traversabilityMapLayers_) {
    if (!traversabilityMap.exists(layer)) {
      ROS_WARN("Traversability Map: Can't set traversability map because there exists no layer %s.", layer.c_str());
      return false;
    }
  }
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  traversabilityMap_ = traversabilityMap;
  traversabilityMapInitialized_ = true;
  return true;
}

void TraversabilityMap::publishTraversabilityMap()
{
  if (!traversabilityMapPublisher_.getNumSubscribers() < 1) {
    grid_map_msgs::GridMap mapMessage;
    boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
    grid_map::GridMapRosConverter::toMessage(traversabilityMap_, mapMessage);
    scopedLockForTraversabilityMap.unlock();
    mapMessage.info.pose.position.z = zPosition_;
    traversabilityMapPublisher_.publish(mapMessage);
  }
}

grid_map::GridMap TraversabilityMap::getTraversabilityMap()
{
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  return traversabilityMap_;
}

bool TraversabilityMap::traversabilityMapInitialized()
{
  return traversabilityMapInitialized_;
}

void TraversabilityMap::resetTraversabilityFootprintLayers()
{
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  if (traversabilityMap_.exists("step_footprint")) traversabilityMap_.clear("step_footprint");
  if (traversabilityMap_.exists("slope_footprint")) traversabilityMap_.clear("slope_footprint");
  if (traversabilityMap_.exists("traversability_footprint")) traversabilityMap_.clear("traversability_footprint");
}

bool TraversabilityMap::computeTraversability()
{
  boost::recursive_mutex::scoped_lock scopedLockForTraversabilityMap(traversabilityMapMutex_);
  grid_map::GridMap traversabilityMapCopy = traversabilityMap_;
  scopedLockForTraversabilityMap.unlock();
  boost::recursive_mutex::scoped_lock scopedLockForElevationMap(elevationMapMutex_);
  grid_map::GridMap elevationMapCopy = elevationMap_;
  scopedLockForElevationMap.unlock();

  // Initialize timer.
  ros::WallTime start = ros::WallTime::now();

  if (elevationMapInitialized_) {
    if (!filter_chain_.update(elevationMapCopy, traversabilityMapCopy)) {
      ROS_ERROR("Traversability Estimation: Could not update the filter chain! No traversability computed!");
      traversabilityMapInitialized_ = false;
      return false;
    }
  } else {
    ROS_ERROR("Traversability Estimation: Elevation map is not initialized!");
    traversabilityMapInitialized_ = false;
    return false;
  }
  traversabilityMapInitialized_ = true;
  traversabilityMapCopy.add("step_footprint");
  traversabilityMapCopy.add("slope_footprint");
  if (checkForRoughness_) traversabilityMapCopy.add("roughness_footprint");
  traversabilityMapCopy.add("traversability_footprint");

  scopedLockForTraversabilityMap.lock();
  traversabilityMap_ = traversabilityMapCopy;
  scopedLockForTraversabilityMap.unlock();
  publishTraversabilityMap();

  ROS_DEBUG("Traversability map has been updated in %f s.", (ros::WallTime::now() - start).toSec());
  return true;
}

bool TraversabilityMap::traversabilityFootprint(double footprintYaw)
{
  if (!traversabilityMapInitialized_) return false;

  // Initialize timer.
  ros::WallTime start = ros::WallTime::now();

  traversabilityMap_.add("traversability_x");
  traversabilityMap_.add("traversability_rot");

  grid_map::Position position;
  grid_map::Polygon polygonX, polygonRot;

  ROS_DEBUG_STREAM("footprint yaw: " << footprintYaw);
  // Compute Orientation
  kindr::RotationQuaternionD xquat, rquat;
  kindr::AngleAxisD rotationAxis(footprintYaw, 0.0,
                                 0.0, 1.0);
  rquat = rotationAxis * xquat;
  Eigen::Quaterniond orientationX = xquat.toImplementation();
  Eigen::Quaterniond orientationRot = rquat.toImplementation();

  for (grid_map::GridMapIterator iterator(traversabilityMap_); !iterator.isPastEnd(); ++iterator) {
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
    if (isTraversable(polygonX, traversability)) traversabilityMap_.at("traversability_x", *iterator) = traversability;
    else traversabilityMap_.at("traversability_x", *iterator) = 0.0;
    if (isTraversable(polygonRot, traversability)) traversabilityMap_.at("traversability_rot", *iterator) = traversability;
    else traversabilityMap_.at("traversability_rot", *iterator) = 0.0;
  }

  publishTraversabilityMap();

  ROS_INFO("Traversability of footprint has been computed in %f s.", (ros::WallTime::now() - start).toSec());
  return true;
}

bool TraversabilityMap::traversabilityFootprint(const double& radius, const double& offset)
{
  double traversability;
  grid_map::Position center;
  for (grid_map::GridMapIterator iterator(traversabilityMap_); !iterator.isPastEnd(); ++iterator) {
    traversabilityMap_.getPosition(*iterator, center);
    isTraversable(center, radius + offset, traversability, radius);
  }
  publishTraversabilityMap();
  return true;
}

bool TraversabilityMap::checkFootprintPath(
    const traversability_msgs::FootprintPath& path,
    traversability_msgs::TraversabilityResult& result,
    const bool publishPolygon)
{
  if (!traversabilityMapInitialized_) {
    ROS_DEBUG("Traversability Estimation: check Footprint path: Traversability map not yet initialized.");
    result.is_safe = false;
    return true;
  }

  const int arraySize = path.poses.poses.size();
  if (arraySize == 0) {
    ROS_WARN("Traversability Estimation: This path has no poses to check!");
    return false;
  }

  double radius = path.radius;
  double offset = 0.15;
  result.is_safe = false;
  result.traversability = 0.0;
  result.area = 0.0;
  double traversability = 0.0;
  double area = 0.0;
  grid_map::Position start, end;

  if (path.footprint.polygon.points.size() == 0) {
    for (int i = 0; i < arraySize; i++) {
      start = end;
      end.x() = path.poses.poses[i].position.x;
      end.y() = path.poses.poses[i].position.y;

      if (arraySize == 1) {
        if (checkRobotInclination_) {
          if (!checkInclination(end, end))
            return true;
        }
        if (!isTraversable(end, radius + offset, traversability, radius)) {
          return true;
        }
        if (publishPolygon) {
          grid_map::Polygon polygon = grid_map::Polygon::fromCircle(end, radius + offset);
          publishFootprintPolygon(polygon);
        }
        result.traversability = traversability;
      }

      if (arraySize > 1 && i > 0) {
        if (checkRobotInclination_) {
          if (!checkInclination(start, end))
            return true;
        }
        double traversabilityTemp, traversabilitySum = 0.0;
        int nLine = 0;
        grid_map::Index startIndex, endIndex;
        traversabilityMap_.getIndex(start, startIndex);
        traversabilityMap_.getIndex(end, endIndex);
        int nSkip = 3; // TODO: Remove magic number.
        for (grid_map::LineIterator lineIterator(traversabilityMap_, endIndex, startIndex); !lineIterator.isPastEnd(); ++lineIterator) {
          grid_map::Position center;
          traversabilityMap_.getPosition(*lineIterator, center);
          if (!isTraversable(center, radius + offset, traversabilityTemp, radius)) {
            return true;
          }
          if (publishPolygon) {
            grid_map::Polygon polygon = grid_map::Polygon::fromCircle(end, radius + offset);
            publishFootprintPolygon(polygon);
          }
          traversabilitySum += traversabilityTemp;
          nLine++;
          for (int j = 0; j < nSkip; j++) {
            if (!lineIterator.isPastEnd()) ++lineIterator;
          }
        }
        traversability = traversabilitySum / (double) nLine;
        double lengthSegment, lengthPreviousPath, lengthPath;
        lengthSegment = (end - start).norm();
        if (i > 1) {
          lengthPreviousPath = lengthPath;
          lengthPath += lengthSegment;
          result.traversability = (lengthSegment * traversability + lengthPreviousPath * result.traversability) / lengthPath;
        } else {
          lengthPath = lengthSegment;
          result.traversability = traversability;
        }
      }
    }
  } else {
    grid_map::Polygon polygon, polygon1, polygon2;
    polygon1.setFrameId(mapFrameId_);
    polygon1.setTimestamp(ros::Time::now().toNSec());
    polygon2 = polygon1;
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
        if (checkRobotInclination_) {
          if (!checkInclination(end, end))
            return true;
        }
        if (!isTraversable(polygon, traversability)) {
          return true;
        }
        if (publishPolygon) {
          publishFootprintPolygon(polygon);
        }
        result.traversability = traversability;
        result.area = polygon.getArea();
      }

      if (arraySize > 1 && i > 0) {
        polygon = grid_map::Polygon::convexHull(polygon1, polygon2);
        polygon.setFrameId(mapFrameId_);
        polygon.setTimestamp(ros::Time::now().toNSec());
        if (checkRobotInclination_) {
          if (!checkInclination(start, end))
            return true;
        }
        if (!isTraversable(polygon, traversability)) {
          return true;
        }
        if (publishPolygon) {
          publishFootprintPolygon(polygon);
        }
        double areaPolygon, areaPrevious;
        if (i > 1) {
          areaPrevious = result.area;
          areaPolygon = polygon.getArea() - polygon1.getArea();
          result.area += areaPolygon;
          result.traversability = (areaPolygon * traversability + areaPrevious * result.traversability) / result.area;
        } else {
          result.area = polygon.getArea();
          result.traversability = traversability;
        }
      }
    }
  }

  result.is_safe = true;
  ROS_DEBUG_STREAM("Traversability: " << result.traversability);

  return true;
}

bool TraversabilityMap::isTraversable(grid_map::Polygon& polygon, double& traversability)
{
  unsigned int nCells = 0;
  traversability = 0.0;

  // Iterate through polygon and check for traversability.
  for (grid_map::PolygonIterator polygonIterator(traversabilityMap_, polygon);
      !polygonIterator.isPastEnd(); ++polygonIterator) {

    // Check for slopes
    if (!checkForSlope(*polygonIterator)) return false;

    // Check for steps
    if (!checkForStep(*polygonIterator)) return false;

    // Check for roughness
    if (checkForRoughness_) {
      if(!checkForRoughness(*polygonIterator)) return false;
    }

    nCells++;
    if (!traversabilityMap_.isValid(*polygonIterator,
                                    traversabilityType_)) {
      traversability += traversabilityDefault_;
    } else {
      traversability += traversabilityMap_.at(traversabilityType_, *polygonIterator);
    }
  }
  // Handle cases of footprints outside of map.
  if (nCells == 0) {
    ROS_DEBUG("TraversabilityMap: isTraversable: No cells within polygon.");
    traversability = traversabilityDefault_;
    if (traversabilityDefault_ == 0.0) return false;
    return true;
  }
  traversability /= nCells;
  return true;
}

bool TraversabilityMap::isTraversable(const grid_map::Position& center, const double& radiusMax, double& traversability, const double& radiusMin)
{
  // Handle cases of footprints outside of map.
  if (!traversabilityMap_.isInside(center)) {
    traversability = traversabilityDefault_;
    if (traversabilityDefault_ == 0.0) return false;
    return true;
  }
  // Get index of center position.
  grid_map::Index indexCenter;
  traversabilityMap_.getIndex(center, indexCenter);
  if (traversabilityMap_.isValid(indexCenter, "traversability_footprint")) {
    traversability = traversabilityMap_.at("traversability_footprint", indexCenter);
    if (traversability == 0.0) return false;
    return true;
  }

  int nCells = 0;
  traversability = 0.0;

  // Iterate through polygon and check for traversability.
  for (grid_map::SpiralIterator iterator(traversabilityMap_, center, radiusMax);
      !iterator.isPastEnd(); ++iterator) {

    if (radiusMin == 0.0) {
      // Check for slopes
      if (!checkForSlope(*iterator)) {
        traversabilityMap_.at("traversability_footprint", indexCenter) = 0.0;
        return false;
      }

      // Check for steps
      if (!checkForStep(*iterator)) {
        traversabilityMap_.at("traversability_footprint", indexCenter) = 0.0;
        return false;
      }

      // Check for roughness
      if (checkForRoughness_) {
        if (!checkForRoughness(*iterator)) {
          traversabilityMap_.at("traversability_footprint", indexCenter) = 0.0;
          return false;
        }
      }
    } else {
      // Check for slopes
      if (!checkForSlope(*iterator)) {
        double radius = iterator.getCurrentRadius();
        if (radius <= radiusMin) {
          traversabilityMap_.at("traversability_footprint", indexCenter) = 0.0;
          return false;
        }
        double factor = ((radius - radiusMin) / (radiusMax - radiusMin) + 1.0) / 2.0;
        traversability *= factor / nCells;
        traversabilityMap_.at("traversability_footprint", indexCenter) = traversability;
        return true;
      }

      // Check for steps
      if (!checkForStep(*iterator)) {
        double radius = iterator.getCurrentRadius();
        if (radius <= radiusMin) {
          traversabilityMap_.at("traversability_footprint", indexCenter) = 0.0;
          return false;
        }
        double factor = ((radius - radiusMin) / (radiusMax - radiusMin) + 1.0) / 2.0;
        traversability *= factor / nCells;
        traversabilityMap_.at("traversability_footprint", indexCenter) = traversability;
        return true;
      }

      // Check for roughness
      if (checkForRoughness_) {
        if (!checkForRoughness(*iterator)) {
          double radius = iterator.getCurrentRadius();
          if (radius <= radiusMin) {
            traversabilityMap_.at("traversability_footprint", indexCenter) = 0.0;
            return false;
          }
          double factor = ((radius - radiusMin) / (radiusMax - radiusMin) + 1.0) / 2.0;
          traversability *= factor / nCells;
          traversabilityMap_.at("traversability_footprint", indexCenter) = traversability;
          return true;
        }
      }
    }

    nCells++;
    if (!traversabilityMap_.isValid(*iterator,
                                    traversabilityType_)) {
      traversability += traversabilityDefault_;
    } else {
      traversability += traversabilityMap_.at(traversabilityType_, *iterator);
    }
  }

  traversability /= nCells;
  traversabilityMap_.at("traversability_footprint", indexCenter) = traversability;
  return true;
}

bool TraversabilityMap::checkInclination(const grid_map::Position& start, const grid_map::Position& end)
{
  if (end == start) {
    if (traversabilityMap_.atPosition(robotSlopeType_, start) == 0.0) return false;
  } else {
    grid_map::Index startIndex, endIndex;
    traversabilityMap_.getIndex(start, startIndex);
    traversabilityMap_.getIndex(end, endIndex);
    for (grid_map::LineIterator lineIterator(traversabilityMap_, startIndex, endIndex); !lineIterator.isPastEnd(); ++lineIterator) {
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

bool TraversabilityMap::checkForStep(const grid_map::Index& indexStep)
{
  if (traversabilityMap_.at(stepType_, indexStep) == 0.0) {
    if (!traversabilityMap_.isValid(indexStep, "step_footprint")) {
      double windowRadiusStep = 2.5*traversabilityMap_.getResolution(); //0.075;

      vector<grid_map::Index> indices;
      grid_map::Position center;
      traversabilityMap_.getPosition(indexStep, center);
      double height = traversabilityMap_.at("elevation", indexStep);
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center, windowRadiusStep);
          !circleIterator.isPastEnd(); ++circleIterator) {
        if (traversabilityMap_.at("elevation", *circleIterator) > criticalStepHeight_ + height && traversabilityMap_.at(stepType_, *circleIterator) == 0.0) indices.push_back(*circleIterator);
      }
      if (indices.empty()) indices.push_back(indexStep);
      for (auto& index : indices) {
        grid_map::Length subMapLength(2.5*traversabilityMap_.getResolution(), 2.5*traversabilityMap_.getResolution());
        grid_map::Position subMapPos;
        bool isSuccess;
        traversabilityMap_.getPosition(index, subMapPos);
        grid_map::Vector toCenter = center - subMapPos;
        grid_map::GridMap subMap = traversabilityMap_.getSubmap(subMapPos, subMapLength, isSuccess);
        if (!isSuccess) {
          ROS_WARN("Traversability map: Check for step window could not retrieve submap.");
          traversabilityMap_.at("step_footprint", indexStep) = 0.0;
          return false;
        }
        height = traversabilityMap_.at("elevation", index);
        for (grid_map::GridMapIterator subMapIterator(subMap); !subMapIterator.isPastEnd(); ++subMapIterator) {
          if (subMap.at(stepType_, *subMapIterator) == 0.0 && subMap.at("elevation", *subMapIterator) < height - criticalStepHeight_) {
            grid_map::Position pos;
            subMap.getPosition(*subMapIterator, pos);
            grid_map::Vector vec = pos - subMapPos;
            if (vec.norm() < 0.025) continue;
            if (toCenter.norm() > 0.025) {
              if (toCenter.dot(vec) < 0.0) continue;
            }
            pos = subMapPos + vec;
            while ((pos - subMapPos + vec).norm() < maxGapWidth_ && traversabilityMap_.isInside(pos + vec)) pos += vec;
            grid_map::Index endIndex;
            traversabilityMap_.getIndex(pos, endIndex);
            bool gapStart = false;
            bool gapEnd = false;
            for (grid_map::LineIterator lineIterator(traversabilityMap_, index, endIndex); !lineIterator.isPastEnd(); ++lineIterator) {
              if (traversabilityMap_.at("elevation", *lineIterator) > height + criticalStepHeight_) {
                traversabilityMap_.at("step_footprint", indexStep) = 0.0;
                return false;
              }
              if (traversabilityMap_.at("elevation", *lineIterator) < height - criticalStepHeight_ || !traversabilityMap_.isValid(*lineIterator, "elevation")) {
                gapStart = true;
              } else if (gapStart) {
                gapEnd = true;
                break;
              }
            }
            if (gapStart && !gapEnd) {
              traversabilityMap_.at("step_footprint", indexStep) = 0.0;
              return false;
            }
          }
        }
      }
      traversabilityMap_.at("step_footprint", indexStep) = 1.0;
    } else if (traversabilityMap_.at("step_footprint", indexStep) == 0.0) {
      return false;
    }
  }
  return true;
}

bool TraversabilityMap::checkForSlope(const grid_map::Index& index)
{
  if (traversabilityMap_.at(slopeType_, index) == 0.0) {
    if (!traversabilityMap_.isValid(index, "slope_footprint")) {
      double windowRadius = 3.0*traversabilityMap_.getResolution(); // TODO: read this as a parameter?
      double criticalLength = maxGapWidth_ / 3.0;
      int nSlopesCritical = floor(2 * windowRadius * criticalLength / pow(traversabilityMap_.getResolution(), 2));

      // Requested position (center) of circle in map.
      grid_map::Position center;
      traversabilityMap_.getPosition(index, center);
      int nSlopes = 0;
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center, windowRadius);
          !circleIterator.isPastEnd(); ++circleIterator) {
        if (traversabilityMap_.at(slopeType_, *circleIterator) == 0.0)
          nSlopes++;
        if (nSlopes > nSlopesCritical) {
          traversabilityMap_.at("slope_footprint", index) = 0.0;
          return false;
        }
      }
      traversabilityMap_.at("slope_footprint", index) = 1.0;
    } else if (traversabilityMap_.at("slope_footprint", index) == 0.0) {
      return false;
    }
  }
  return true;
}

bool TraversabilityMap::checkForRoughness(const grid_map::Index& index)
{
  if (traversabilityMap_.at(roughnessType_, index) == 0.0) {
    if (!traversabilityMap_.isValid(index, "roughness_footprint")) {
      double windowRadius = 3.0*traversabilityMap_.getResolution(); // TODO: read this as a parameter?
      double criticalLength = maxGapWidth_ / 3.0;
      int nRoughnessCritical = floor(1.5 * windowRadius * criticalLength / pow(traversabilityMap_.getResolution(), 2));

      // Requested position (center) of circle in map.
      grid_map::Position center;
      traversabilityMap_.getPosition(index, center);
      int nRoughness = 0;
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center, windowRadius);
          !circleIterator.isPastEnd(); ++circleIterator) {
        if (traversabilityMap_.at(roughnessType_, *circleIterator) == 0.0)
          nRoughness++;
        if (nRoughness > nRoughnessCritical) {
          traversabilityMap_.at("roughness_footprint", index) = 0.0;
          return false;
        }
      }
      traversabilityMap_.at("roughness_footprint", index) = 1.0;
    } else if (traversabilityMap_.at("roughness_footprint", index) == 0.0) {
      return false;
    }
  }
  return true;
}

void TraversabilityMap::publishFootprintPolygon(const grid_map::Polygon& polygon)
{
  if (footprintPublisher_.getNumSubscribers() < 1) return;
  geometry_msgs::PolygonStamped polygonMsg;
  grid_map::PolygonRosConverter::toMessage(polygon, polygonMsg);
  footprintPublisher_.publish(polygonMsg);
}

} /* namespace */
