/*
 * TraversabilityEstimation.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"
#include "traversability_estimation/common.h"
#include <traversability_msgs/TraversabilityResult.h>
#include <param_io/get_param.hpp>

// ROS
#include <geometry_msgs/Pose.h>
#include <ros/package.h>

using namespace std;

namespace traversability_estimation {

TraversabilityEstimation::TraversabilityEstimation(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      acceptGridMapToInitTraversabilityMap_(false),
      traversabilityMap_(nodeHandle),
      traversabilityType_("traversability"),
      slopeType_("traversability_slope"),
      stepType_("traversability_step"),
      roughnessType_("traversability_roughness"),
      robotSlopeType_("robot_slope"),
      getImageCallback_(false),
      useRawMap_(false) {
  ROS_DEBUG("Traversability estimation node started.");
  readParameters();
  traversabilityMap_.createLayers(useRawMap_);
  submapClient_ = nodeHandle_.serviceClient<grid_map_msgs::GetGridMap>(submapServiceName_);

  if (!updateDuration_.isZero()) {
    updateTimer_ = nodeHandle_.createTimer(updateDuration_, &TraversabilityEstimation::updateTimerCallback, this);
  } else {
    ROS_WARN("Update rate is zero. No traversability map will be published.");
  }

  loadElevationMapService_ = nodeHandle_.advertiseService("load_elevation_map", &TraversabilityEstimation::loadElevationMap, this);
  updateTraversabilityService_ =
      nodeHandle_.advertiseService("update_traversability", &TraversabilityEstimation::updateServiceCallback, this);
  getTraversabilityService_ = nodeHandle_.advertiseService("get_traversability", &TraversabilityEstimation::getTraversabilityMap, this);
  footprintPathService_ = nodeHandle_.advertiseService("check_footprint_path", &TraversabilityEstimation::checkFootprintPath, this);
  updateParameters_ = nodeHandle_.advertiseService("update_parameters", &TraversabilityEstimation::updateParameter, this);
  traversabilityFootprint_ =
      nodeHandle_.advertiseService("traversability_footprint", &TraversabilityEstimation::traversabilityFootprint, this);
  saveToBagService_ = nodeHandle_.advertiseService("save_traversability_map_to_bag", &TraversabilityEstimation::saveToBag, this);
  imageSubscriber_ = nodeHandle_.subscribe(imageTopic_, 1, &TraversabilityEstimation::imageCallback, this);

  if (acceptGridMapToInitTraversabilityMap_) {
    gridMapToInitTraversabilityMapSubscriber_ = nodeHandle_.subscribe(
        gridMapToInitTraversabilityMapTopic_, 1, &TraversabilityEstimation::gridMapToInitTraversabilityMapCallback, this);
  }

  elevationMapLayers_.push_back("elevation");
  if (!useRawMap_) {
    elevationMapLayers_.push_back("upper_bound");
    elevationMapLayers_.push_back("lower_bound");
  } else {
    elevationMapLayers_.push_back("variance");
    elevationMapLayers_.push_back("horizontal_variance_x");
    elevationMapLayers_.push_back("horizontal_variance_y");
    elevationMapLayers_.push_back("horizontal_variance_xy");
    elevationMapLayers_.push_back("time");
  }
}

TraversabilityEstimation::~TraversabilityEstimation() {
  updateTimer_.stop();
  nodeHandle_.shutdown();
}

bool TraversabilityEstimation::readParameters() {
  // Read boolean to switch between raw and fused map.
  useRawMap_ = param_io::param<bool>(nodeHandle_, "use_raw_map", false);

  submapServiceName_ = param_io::param<std::string>(nodeHandle_, "submap_service", "/get_grid_map");

  double updateRate;
  updateRate = param_io::param(nodeHandle_, "min_update_rate", 1.0);
  if (updateRate != 0.0) {
    updateDuration_.fromSec(1.0 / updateRate);
  } else {
    updateDuration_.fromSec(0.0);
  }
  // Read parameters for image subscriber.
  imageTopic_ = param_io::param<std::string>(nodeHandle_, "image_topic", "/image_elevation");
  imageResolution_ = param_io::param(nodeHandle_, "resolution", 0.03);
  imageMinHeight_ = param_io::param(nodeHandle_, "min_height", 0.0);
  imageMaxHeight_ = param_io::param(nodeHandle_, "max_height", 1.0);
  imagePosition_.x() = param_io::param(nodeHandle_, "image_position_x", 0.0);
  imagePosition_.y() = param_io::param(nodeHandle_, "image_position_y", 0.0);

  robotFrameId_ = param_io::param<std::string>(nodeHandle_, "robot_frame_id", "robot");
  robot_ = param_io::param<std::string>(nodeHandle_, "robot", "robot");
  package_ = param_io::param<std::string>(nodeHandle_, "package", "traversability_estimation");

  grid_map::Position mapCenter;
  mapCenter.x() = param_io::param(nodeHandle_, "map_center_x", 0.0);
  mapCenter.y() = param_io::param(nodeHandle_, "map_center_y", 0.0);

  submapPoint_.header.frame_id = robotFrameId_;
  submapPoint_.point.x = mapCenter.x();

  submapPoint_.point.y = mapCenter.y();

  submapPoint_.point.z = 0.0;
  mapLength_.x() = param_io::param(nodeHandle_, "map_length_x", 5.0);
  mapLength_.y() = param_io::param(nodeHandle_, "map_length_y", 5.0);
  footprintYaw_ = param_io::param(nodeHandle_, "footprint_yaw", M_PI_2);

  // Grid map to initialize elevation layer
  acceptGridMapToInitTraversabilityMap_ = param_io::param<bool>(nodeHandle_, "grid_map_to_initialize_traversability_map/enable", false);
  gridMapToInitTraversabilityMapTopic_ =
      param_io::param<std::string>(nodeHandle_, "grid_map_to_initialize_traversability_map/grid_map_topic_name", "initial_elevation_map");

  return true;
}

bool TraversabilityEstimation::loadElevationMap(grid_map_msgs::ProcessFile::Request& request,
                                                grid_map_msgs::ProcessFile::Response& response) {
  ROS_INFO("TraversabilityEstimation: loadElevationMap");
  if (request.file_path.empty() || request.topic_name.empty()) {
    ROS_WARN("Fields 'file_path' and 'topic_name' in service request must be filled in.");
    response.success = static_cast<unsigned char>(false);
    return true;
  }

  grid_map::GridMap map;
  if (!grid_map::GridMapRosConverter::loadFromBag(request.file_path, request.topic_name, map)) {
    ROS_ERROR("TraversabilityEstimation: Cannot find bag '%s' or topic '%s' of the elevation map!", request.file_path.c_str(),
              request.topic_name.c_str());
    response.success = static_cast<unsigned char>(false);
  } else {
    map.setTimestamp(ros::Time::now().toNSec());
    if (!initializeTraversabilityMapFromGridMap(map)) {
      ROS_ERROR(
          "TraversabilityEstimation: loadElevationMap: it was not possible to load elevation map from bag with path '%s' and topic '%s'.",
          request.file_path.c_str(), request.topic_name.c_str());
      response.success = static_cast<unsigned char>(false);
    } else {
      response.success = static_cast<unsigned char>(true);
    }
  }

  return true;
}

void TraversabilityEstimation::imageCallback(const sensor_msgs::Image& image) {
  if (!getImageCallback_) {
    grid_map::GridMapRosConverter::initializeFromImage(image, imageResolution_, imageGridMap_, imagePosition_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", imageGridMap_.getLength().x(), imageGridMap_.getLength().y(),
             imageGridMap_.getSize()(0), imageGridMap_.getSize()(1));
    imageGridMap_.add("upper_bound", 0.0);  // TODO: Add value for layers.
    imageGridMap_.add("lower_bound", 0.0);
    imageGridMap_.add("uncertainty_range", imageGridMap_.get("upper_bound") - imageGridMap_.get("lower_bound"));
    getImageCallback_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(image, "elevation", imageGridMap_, imageMinHeight_, imageMaxHeight_);
  grid_map_msgs::GridMap elevationMap;
  grid_map::GridMapRosConverter::toMessage(imageGridMap_, elevationMap);
  traversabilityMap_.setElevationMap(elevationMap);
}

void TraversabilityEstimation::updateTimerCallback(const ros::TimerEvent& timerEvent) { updateTraversability(); }

bool TraversabilityEstimation::updateServiceCallback(grid_map_msgs::GetGridMapInfo::Request&,
                                                     grid_map_msgs::GetGridMapInfo::Response& response) {
  if (updateDuration_.isZero()) {
    if (!updateTraversability()) {
      ROS_ERROR("Traversability Estimation: Cannot update traversability!");
      return false;
    }
  }
  // Wait until traversability map is computed.
  while (!traversabilityMap_.traversabilityMapInitialized()) {
    sleep(1.0);
  }
  grid_map_msgs::GridMap msg;
  grid_map::GridMap traversabilityMap = traversabilityMap_.getTraversabilityMap();

  response.info.header.frame_id = traversabilityMap_.getMapFrameId();
  response.info.header.stamp = ros::Time::now();
  response.info.resolution = traversabilityMap.getResolution();
  response.info.length_x = traversabilityMap.getLength()[0];
  response.info.length_y = traversabilityMap.getLength()[1];
  geometry_msgs::Pose pose;
  grid_map::Position position = traversabilityMap.getPosition();
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.orientation.w = 1.0;
  response.info.pose = pose;

  return true;
}

bool TraversabilityEstimation::updateTraversability() {
  grid_map_msgs::GridMap elevationMap;
  if (!getImageCallback_) {
    ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
    if (!submapClient_.waitForExistence(ros::Duration(2.0))) {
      return false;
    }
    ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
    if (requestElevationMap(elevationMap)) {
      traversabilityMap_.setElevationMap(elevationMap);
      if (!traversabilityMap_.computeTraversability()) return false;
    } else {
      ROS_WARN_THROTTLE(periodThrottledConsoleMessages, "Failed to retrieve elevation grid map.");
      return false;
    }
  } else {
    if (!traversabilityMap_.computeTraversability()) return false;
  }

  return true;
}

bool TraversabilityEstimation::updateParameter(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
  // Load parameters file.
  string path = ros::package::getPath(package_);
  string path_filter_parameter = path + "/config/" + robot_ + "_filter_parameter.yaml";
  string path_footprint_parameter = path + "/config/" + robot_ + "_footprint_parameter.yaml";
  // Filter parameters
  string commandString = "rosparam load " + path_filter_parameter + " /traversability_estimation";
  const char* command_filter = commandString.c_str();
  if (system(command_filter) != 0) {
    ROS_ERROR("Can't update parameter.");
    return false;
  }
  // Footprint parameters.
  commandString = "rosparam load " + path_footprint_parameter + " /traversability_estimation";
  const char* command_footprint = commandString.c_str();
  if (system(command_footprint) != 0) {
    ROS_ERROR("Can't update parameter.");
    return false;
  }

  if (!traversabilityMap_.updateFilter()) return false;
  return true;
}

bool TraversabilityEstimation::requestElevationMap(grid_map_msgs::GridMap& map) {
  submapPoint_.header.stamp = ros::Time(0);
  geometry_msgs::PointStamped submapPointTransformed;

  try {
    transformListener_.transformPoint(traversabilityMap_.getMapFrameId(), submapPoint_, submapPointTransformed);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  grid_map_msgs::GetGridMap submapService;
  submapService.request.position_x = submapPointTransformed.point.x;
  submapService.request.position_y = submapPointTransformed.point.y;
  submapService.request.length_x = mapLength_.x();
  submapService.request.length_y = mapLength_.y();
  submapService.request.layers = elevationMapLayers_;

  if (!submapClient_.call(submapService)) return false;
  map = submapService.response.map;

  return true;
}

bool TraversabilityEstimation::traversabilityFootprint(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  if (!traversabilityMap_.traversabilityFootprint(footprintYaw_)) return false;

  return true;
}

bool TraversabilityEstimation::checkFootprintPath(traversability_msgs::CheckFootprintPath::Request& request,
                                                  traversability_msgs::CheckFootprintPath::Response& response) {
  const int nPaths = request.path.size();
  if (nPaths == 0) {
    ROS_WARN_THROTTLE(periodThrottledConsoleMessages, "No footprint path available to check!");
    return false;
  }

  traversability_msgs::TraversabilityResult result;
  traversability_msgs::FootprintPath path;
  for (int j = 0; j < nPaths; j++) {
    path = request.path[j];
    if (!traversabilityMap_.checkFootprintPath(path, result, true)) return false;
    response.result.push_back(result);
  }

  return true;
}

bool TraversabilityEstimation::getTraversabilityMap(grid_map_msgs::GetGridMap::Request& request,
                                                    grid_map_msgs::GetGridMap::Response& response) {
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  grid_map_msgs::GridMap msg;
  grid_map::GridMap map, subMap;
  map = traversabilityMap_.getTraversabilityMap();
  bool isSuccess;
  subMap = map.getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    vector<string> layers;
    for (const auto& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }
  return isSuccess;
}

bool TraversabilityEstimation::saveToBag(grid_map_msgs::ProcessFile::Request& request, grid_map_msgs::ProcessFile::Response& response) {
  ROS_INFO("Save to bag.");
  if (request.file_path.empty() || request.topic_name.empty()) {
    ROS_WARN("Fields 'file_path' and 'topic_name' in service request must be filled in.");
    response.success = static_cast<unsigned char>(false);
    return true;
  }

  response.success = static_cast<unsigned char>(
      grid_map::GridMapRosConverter::saveToBag(traversabilityMap_.getTraversabilityMap(), request.file_path, request.topic_name));
  return true;
}

bool TraversabilityEstimation::initializeTraversabilityMapFromGridMap(const grid_map::GridMap& gridMap) {
  if (traversabilityMap_.traversabilityMapInitialized()) {
    ROS_WARN(
        "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]: received grid map message cannot be used to initialize"
        " the traversability map, because current traversability map has been already initialized.");
    return false;
  }

  grid_map::GridMap mapWithCheckedLayers = gridMap;
  for (const auto& layer : elevationMapLayers_) {
    if (!mapWithCheckedLayers.exists(layer)) {
      mapWithCheckedLayers.add(layer, 0.0);
      ROS_INFO_STREAM("[TraversabilityEstimation::initializeTraversabilityMapFromGridMap]: Added layer '" << layer << "'.");
    }
  }
  ROS_DEBUG_STREAM("Map frame id: " << mapWithCheckedLayers.getFrameId());
  for (const auto& layer : mapWithCheckedLayers.getLayers()) {
    ROS_DEBUG_STREAM("Map layers: " << layer);
  }
  ROS_DEBUG_STREAM("Map size: " << mapWithCheckedLayers.getLength());
  ROS_DEBUG_STREAM("Map position: " << mapWithCheckedLayers.getPosition());
  ROS_DEBUG_STREAM("Map resolution: " << mapWithCheckedLayers.getResolution());

  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(mapWithCheckedLayers, message);
  traversabilityMap_.setElevationMap(message);
  if (!traversabilityMap_.computeTraversability()) {
    ROS_WARN("TraversabilityEstimation: initializeTraversabilityMapFromGridMap: cannot compute traversability.");
    return false;
  }
  return true;
}

void TraversabilityEstimation::gridMapToInitTraversabilityMapCallback(const grid_map_msgs::GridMap& message) {
  grid_map::GridMap gridMap;
  grid_map::GridMapRosConverter::fromMessage(message, gridMap);
  if (!initializeTraversabilityMapFromGridMap(gridMap)) {
    ROS_ERROR(
        "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]: "
        "It was not possible to use received grid map message to initialize traversability map.");
  } else {
    ROS_INFO(
        "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]: "
        "Traversability Map initialized using received grid map on topic '%s'.",
        gridMapToInitTraversabilityMapTopic_.c_str());
  }
}

}  // namespace traversability_estimation
