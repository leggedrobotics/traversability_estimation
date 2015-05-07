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

#include <ros/package.h>
#include "yaml-cpp/yaml.h"

// Eigen
#include <Eigen/Geometry>

using namespace std;
using namespace grid_map_msgs;

namespace traversability_estimation {

TraversabilityEstimation::TraversabilityEstimation(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      traversabilityType_("traversability"),
      slopeType_("traversability_slope"),
      stepType_("traversability_step"),
      roughnessType_("traversability_roughness"),
      filter_chain_("grid_map::GridMap"),
      getGridMap_(false)
{
  ROS_INFO("Traversability estimation node started.");

  readParameters();
  submapClient_ = nodeHandle_.serviceClient<GetGridMap>(submapServiceName_);
  footprintPolygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("footprint_polygon", 1, true);
  traversabilityMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("traversability_map", 1);

  if (!updateDuration_.isZero()) {
    updateTimer_ = nodeHandle_.createTimer(
        updateDuration_, &TraversabilityEstimation::updateTimerCallback, this);
  } else {
    ROS_WARN("Update rate is zero. No traversability map will be published.");
  }

  updateTraversabilityService_ = nodeHandle_.advertiseService("update_traversability", &TraversabilityEstimation::updateServiceCallback, this);
  footprintPathService_ = nodeHandle_.advertiseService("check_footprint_path", &TraversabilityEstimation::checkFootprintPath, this);
  updateParameters_ = nodeHandle_.advertiseService("update_parameters", &TraversabilityEstimation::updateParameter, this);

  elevationMapSub_ = nodeHandle_.subscribe(elevationMapTopic_,1,&TraversabilityEstimation::elevationMapCallback, this);

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
  nodeHandle_.param("elevation_map_topic", elevationMapTopic_,
                    string("image_to_gridmap/image_grid_map"));

  double updateRate;
  nodeHandle_.param("min_update_rate", updateRate, 1.0);
  if (updateRate != 0.0) {
    updateDuration_.fromSec(1.0 / updateRate);
  } else {
    updateDuration_.fromSec(0.0);
  }

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

  nodeHandle_.param("traversability_default", traversabilityDefault_, 0.5);

  // Configure filter chain
  if (!filter_chain_.configure("traversability_map_filters", nodeHandle_)) {
    ROS_ERROR("Could not configure the filter chain!");
  }
  return true;
}

void TraversabilityEstimation::elevationMapCallback(const grid_map_msgs::GridMap& elevationMap)
{
  grid_map::GridMapRosConverter::fromMessage(elevationMap, elevationMap_);
  elevationMap_.add("variance", 0.0);
  getGridMap_ = true;
}

void TraversabilityEstimation::updateTimerCallback(
    const ros::TimerEvent& timerEvent)
{
  computeTraversability();
}

void TraversabilityEstimation::computeTraversability()
{
  grid_map_msgs::GridMap mapMessage;
  if (!getGridMap_) {
    ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
    submapClient_.waitForExistence();
    ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
    if (getGridMap(mapMessage)) {
      grid_map::GridMap elevationMap, traversabilityMap;
      grid_map::GridMapRosConverter::fromMessage(mapMessage, elevationMap);
      updateFilters(elevationMap, traversabilityMap);
      grid_map::GridMapRosConverter::toMessage(traversabilityMap, mapMessage);
      if (!traversabilityMapPublisher_.getNumSubscribers() < 1)
        traversabilityMapPublisher_.publish(mapMessage);
    } else {
      ROS_WARN("Failed to retrieve elevation grid map.");
    }
  } else {
    grid_map::GridMap traversabilityMap;
    updateFilters(elevationMap_, traversabilityMap);
    grid_map::GridMapRosConverter::toMessage(traversabilityMap, mapMessage);
    if (!traversabilityMapPublisher_.getNumSubscribers() < 1)
      traversabilityMapPublisher_.publish(mapMessage);
  }
}

bool TraversabilityEstimation::updateServiceCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  computeTraversability();
  return true;
}

bool TraversabilityEstimation::updateParameter(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Load Yaml file.
  std::string path = ros::package::getPath("starleth_traversability_estimation");
  YAML::Node starlethParameter = YAML::LoadFile(path + "/config/starleth.yaml");
  YAML::Node filterChainNode, filterNode, filterParamsNode;

  // Get new filter parameters.
  XmlRpc::XmlRpcValue newFilterConfig;

  if (starlethParameter["traversability_map_filters"]) {
    filterChainNode = starlethParameter["traversability_map_filters"];
    newFilterConfig.setSize(filterChainNode.size());

    for (int i = 0; i < filterChainNode.size(); i++) {
      std::string filterType = filterChainNode[i]["type"].as<std::string>();
      newFilterConfig[i]["name"] = filterChainNode[i]["name"].as<std::string>();
      ROS_DEBUG("Filter type: %s", filterType.c_str());
      if (filterType == "traversabilityFilters/SurfaceNormalsFilter") {
        newFilterConfig[i]["params"]["estimation_radius"] = filterChainNode[i]["params"]["estimation_radius"].as<double>();
        newFilterConfig[i]["params"]["surface_normal_positive_axis"] = filterChainNode[i]["params"]["surface_normal_positive_axis"].as<std::string>();
      }
      if (filterType == "traversabilityFilters/SlopeFilter") {
        newFilterConfig[i]["params"]["map_type"] = filterChainNode[i]["params"]["map_type"].as<std::string>();
        newFilterConfig[i]["params"]["critical_value"] = filterChainNode[i]["params"]["critical_value"].as<double>();
      }
      if (filterType == "traversabilityFilters/StepFilter") {
        newFilterConfig[i]["params"]["map_type"] = filterChainNode[i]["params"]["map_type"].as<std::string>();
        newFilterConfig[i]["params"]["critical_value"] = filterChainNode[i]["params"]["critical_value"].as<double>();
        newFilterConfig[i]["params"]["first_window_radius"] = filterChainNode[i]["params"]["first_window_radius"].as<double>();
        newFilterConfig[i]["params"]["second_window_radius"] = filterChainNode[i]["params"]["second_window_radius"].as<double>();
        newFilterConfig[i]["params"]["critical_cell_number"] = filterChainNode[i]["params"]["critical_cell_number"].as<int>();
      }
      if (filterType == "traversabilityFilters/RoughnessFilter") {
        newFilterConfig[i]["params"]["map_type"] = filterChainNode[i]["params"]["map_type"].as<std::string>();
        newFilterConfig[i]["params"]["critical_value"] = filterChainNode[i]["params"]["critical_value"].as<double>();
        newFilterConfig[i]["params"]["estimation_radius"] = filterChainNode[i]["params"]["estimation_radius"].as<double>();
      }
      if (filterType == "gridMapFilters/WeightedSumFilter") {
        for (int j = 0; j < filterChainNode[i]["params"]["layers"].size(); j++) {
          newFilterConfig[i]["params"]["layers"][j] = filterChainNode[i]["params"]["layers"][j].as<std::string>();
          newFilterConfig[i]["params"]["weights"][j] = filterChainNode[i]["params"]["weights"][j].as<double>();
        }
        newFilterConfig[i]["params"]["layer_out"] = filterChainNode[i]["params"]["layer_out"].as<std::string>();
        newFilterConfig[i]["params"]["normalize"] = filterChainNode[i]["params"]["normalize"].as<int>();
      }
      if (filterType == "gridMapFilters/MinFilter") {
        for (int j = 0; j < filterChainNode[i]["params"]["layers"].size(); j++) {
          newFilterConfig[i]["params"]["layers"][j] = filterChainNode[i]["params"]["layers"][j].as<std::string>();
        }
        newFilterConfig[i]["params"]["layer_out"] = filterChainNode[i]["params"]["layer_out"].as<std::string>();
      }
      if (filterType == "gridMapFilters/ThresholdFilter") {
        if (filterChainNode[i]["params"]["lower_threshold"]) {
          newFilterConfig[i]["params"]["lower_threshold"] = filterChainNode[i]["params"]["lower_threshold"].as<double>();
        }
        if (filterChainNode[i]["params"]["upper_threshold"]) {
          newFilterConfig[i]["params"]["upper_threshold"] = filterChainNode[i]["params"]["upper_threshold"].as<double>();
        }
        newFilterConfig[i]["params"]["set_to"] = filterChainNode[i]["params"]["set_to"].as<double>();
        for (int j = 0; j < filterChainNode[i]["params"]["layers"].size(); j++) {
          newFilterConfig[i]["params"]["layers"][j] = filterChainNode[i]["params"]["layers"][j].as<std::string>();
        }
      }
      if (filterType == "gridMapFilters/DeletionFilter") {
        for (int j = 0; j < filterChainNode[i]["params"]["layers"].size(); j++) {
          newFilterConfig[i]["params"]["layers"][j] = filterChainNode[i]["params"]["layers"][j].as<std::string>();
        }
      }
      newFilterConfig[i]["type"] = filterChainNode[i]["type"].as<std::string>();
    }
  }

  // Set parameters.
  nodeHandle_.setParam("traversability_map_filters", newFilterConfig);

  // Reconfigure filter chain.
  filter_chain_.clear();
  if (!filter_chain_.configure("traversability_map_filters", nodeHandle_)) {
    ROS_ERROR("Could not configure the filter chain!");
  }
  return true;
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

bool TraversabilityEstimation::updateFilters(const grid_map::GridMap& elevationMap, grid_map::GridMap& traversabilityMap)
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
      polygon1.removeVertices();
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
  ROS_DEBUG_STREAM(response.traversability);
  if (isSafe) {
    ROS_DEBUG_STREAM("Safe.");
  } else {
    response.traversability = 0.0;
    ROS_DEBUG_STREAM("Not Safe.");
  }

  return true;
}

bool TraversabilityEstimation::isTraversable(const grid_map::Polygon& polygon, double& traversability)
{
  int nCells = 0, nSteps = 0;
  traversability = 0.0;
  double windowRadius = 0.1;

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
      Eigen::Vector2d center;
      traversabilityMap_.getPosition(*polygonIterator, center);

      int nSlopes = 0;
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center,
                                                   windowRadius);
          !circleIterator.isPassedEnd(); ++circleIterator) {
        if (traversabilityMap_.at(slopeType_, *circleIterator) == 0.0)
          nSlopes++;
        if (nSlopes > 20)
          return false;
      }
    }

    // Check for roughness
    if (traversabilityMap_.at(roughnessType_, *polygonIterator) == 0.0) {
      // Requested position (center) of circle in map.
      Eigen::Vector2d center;
      traversabilityMap_.getPosition(*polygonIterator, center);

      int nRoughness = 0;
      for (grid_map::CircleIterator circleIterator(traversabilityMap_, center,
                                                   windowRadius);
          !circleIterator.isPassedEnd(); ++circleIterator) {
        if (traversabilityMap_.at(roughnessType_, *circleIterator) == 0.0)
          nRoughness++;
        if (nRoughness > 15)
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
      continue;
    } else {
      traversability += traversabilityMap_.at(traversabilityType_, *polygonIterator);
    }
  }
  traversability /= nCells;
  return true;
}

} /* namespace */
