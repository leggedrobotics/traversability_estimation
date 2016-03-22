/*
 * TraversabilityEstimation.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"
#include <traversability_msgs/TraversabilityResult.h>

// ROS
#include <ros/package.h>
#include <geometry_msgs/Pose.h>

using namespace std;

namespace traversability_estimation {

TraversabilityEstimation::TraversabilityEstimation(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      traversabilityMap_(nodeHandle),
      traversabilityType_("traversability"),
      slopeType_("traversability_slope"),
      stepType_("traversability_step"),
      roughnessType_("traversability_roughness"),
      robotSlopeType_("robot_slope"),
      getImageCallback_(false)
{
  ROS_DEBUG("Traversability estimation node started.");
  readParameters();
  submapClient_ = nodeHandle_.serviceClient<grid_map_msgs::GetGridMap>(submapServiceName_);

  if (!updateDuration_.isZero()) {
    updateTimer_ = nodeHandle_.createTimer(
        updateDuration_, &TraversabilityEstimation::updateTimerCallback, this);
  } else {
    ROS_WARN("Update rate is zero. No traversability map will be published.");
  }

  loadElevationMapService_ = nodeHandle_.advertiseService("load_elevation_map", &TraversabilityEstimation::loadElevationMap, this);
  updateTraversabilityService_ = nodeHandle_.advertiseService("update_traversability", &TraversabilityEstimation::updateServiceCallback, this);
  getTraversabilityService_ = nodeHandle_.advertiseService("get_traversability", &TraversabilityEstimation::getTraversabilityMap, this);
  footprintPathService_ = nodeHandle_.advertiseService("check_footprint_path", &TraversabilityEstimation::checkFootprintPath, this);
  updateParameters_ = nodeHandle_.advertiseService("update_parameters", &TraversabilityEstimation::updateParameter, this);
  traversabilityFootprint_ = nodeHandle_.advertiseService("traversability_footprint", &TraversabilityEstimation::traversabilityFootprint, this);
  saveToBagService_ = nodeHandle_.advertiseService("save_to_bag", &TraversabilityEstimation::saveToBag, this);
  imageSubscriber_ = nodeHandle_.subscribe(imageTopic_,1,&TraversabilityEstimation::imageCallback, this);

  elevationMapLayers_.push_back("elevation");
  elevationMapLayers_.push_back("variance");
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
  nodeHandle_.param("min_update_rate", updateRate, 1.0);
  if (updateRate != 0.0) {
    updateDuration_.fromSec(1.0 / updateRate);
  } else {
    updateDuration_.fromSec(0.0);
  }
  // Read parameters for image subscriber.
  nodeHandle_.param("image_topic", imageTopic_, std::string("/image_elevation"));
  nodeHandle_.param("resolution", imageResolution_, 0.03);
  nodeHandle_.param("min_height", imageMinHeight_, 0.0);
  nodeHandle_.param("max_height", imageMaxHeight_, 1.0);
  nodeHandle_.param("image_position_x", imagePosition_.x(), 0.0);
  nodeHandle_.param("image_position_y", imagePosition_.y(), 0.0);

  nodeHandle_.param("map_frame_id", mapFrameId_, string("map"));
  nodeHandle_.param("robot_frame_id", robotFrameId_, string("robot"));
  grid_map::Position mapCenter;
  nodeHandle_.param("map_center_x", mapCenter.x(), 0.0);
  nodeHandle_.param("map_center_y", mapCenter.y(), 0.0);
  submapPoint_.header.frame_id = robotFrameId_;
  submapPoint_.point.x = mapCenter.x();
  submapPoint_.point.y = mapCenter.y();
  submapPoint_.point.z = 0.0;
  nodeHandle_.param("map_length_x", mapLength_.x(), 5.0);
  nodeHandle_.param("map_length_y", mapLength_.y(), 5.0);
  nodeHandle_.param("footprint_yaw", footprintYaw_, M_PI_2);

  nodeHandle_.param("elevation_map/topic", bagTopicName_,  std::string("grid_map"));
  nodeHandle_.param("elevation_map/path_to_bag", pathToBag_, std::string("elevation_map.bag"));

  return true;
}

bool TraversabilityEstimation::loadElevationMap(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  ROS_INFO("TraversabilityEstimation: loadElevationMap");
  grid_map::GridMap map;
  grid_map_msgs::GridMap msg;
  if (!grid_map::GridMapRosConverter::loadFromBag(pathToBag_, bagTopicName_, map)) {
    ROS_ERROR("TraversabilityEstimation: Cannot find bag or topic of the elevation map!");
    return false;
  }
  ROS_DEBUG_STREAM("Map frame id: " << map.getFrameId());
  for (auto layer : map.getLayers()) {
    ROS_DEBUG_STREAM("Map layers: " << layer);
  }
  ROS_DEBUG_STREAM("Map size: " << map.getLength());
  ROS_DEBUG_STREAM("Map position: " << map.getPosition());
  ROS_DEBUG_STREAM("Map resolution: " << map.getResolution());

  map.setTimestamp(ros::Time::now().toNSec());
  grid_map::GridMapRosConverter::toMessage(map, msg);
  traversabilityMap_.setElevationMap(msg);
  if (!traversabilityMap_.computeTraversability()) {
    ROS_WARN("TraversabilityEstimation: loadElevationMap: cannot compute traversability.");
    return false;
  }
  return true;
}

void TraversabilityEstimation::imageCallback(const sensor_msgs::Image& image)
{
  if (!getImageCallback_) {
    grid_map::GridMapRosConverter::initializeFromImage(image, imageResolution_, imageGridMap_, imagePosition_);
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", imageGridMap_.getLength().x(), imageGridMap_.getLength().y(), imageGridMap_.getSize()(0), imageGridMap_.getSize()(1));
    imageGridMap_.add("variance", 0.0); // TODO: Add value for variance.
    getImageCallback_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(image, "elevation", imageGridMap_, imageMinHeight_, imageMaxHeight_);
  grid_map_msgs::GridMap elevationMap;
  grid_map::GridMapRosConverter::toMessage(imageGridMap_, elevationMap);
  traversabilityMap_.setElevationMap(elevationMap);
}

void TraversabilityEstimation::updateTimerCallback(const ros::TimerEvent& timerEvent)
{
  updateTraversability();
}

bool TraversabilityEstimation::updateServiceCallback(grid_map_msgs::GetGridMapInfo::Request&, grid_map_msgs::GetGridMapInfo::Response& response)
{
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

  response.info.header.frame_id = mapFrameId_;
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

bool TraversabilityEstimation::updateTraversability()
{
  grid_map_msgs::GridMap elevationMap;
  if (!getImageCallback_) {
    ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
    submapClient_.waitForExistence();
    ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
    if (requestElevationMap(elevationMap)) {
      traversabilityMap_.setElevationMap(elevationMap);
      if (!traversabilityMap_.computeTraversability()) return false;
    } else {
      ROS_WARN("Failed to retrieve elevation grid map.");
      return false;
    }
  } else {
    if (!traversabilityMap_.computeTraversability()) return false;
  }

  return true;
}

bool TraversabilityEstimation::updateParameter(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  // Load parameters file.
  string path = ros::package::getPath("starleth_traversability_estimation");
  path = path + "/config/filter_parameter.yaml";
  string commandString = "rosparam load " + path + " /traversability_estimation";
  const char* command = commandString.c_str();
  if (system(command) != 0)
  {
    ROS_ERROR("Can't update parameter.");
    return false;
  }

  if (!traversabilityMap_.updateFilter()) return false;
  return true;
}

bool TraversabilityEstimation::requestElevationMap(grid_map_msgs::GridMap& map)
{
  submapPoint_.header.stamp = ros::Time(0);
  geometry_msgs::PointStamped submapPointTransformed;

  try {
    transformListener_.transformPoint(mapFrameId_, submapPoint_,
                                      submapPointTransformed);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  grid_map_msgs::GetGridMap submapService;
  submapService.request.position_x = submapPointTransformed.point.x;
  submapService.request.position_y = submapPointTransformed.point.y;
  submapService.request.length_x = mapLength_.x();
  submapService.request.length_y = mapLength_.y();
  submapService.request.layers = elevationMapLayers_;

  if (!submapClient_.call(submapService))
    return false;
  map = submapService.response.map;

  return true;
}

bool TraversabilityEstimation::traversabilityFootprint(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  if (!traversabilityMap_.traversabilityFootprint(footprintYaw_))
    return false;

  return true;
}

bool TraversabilityEstimation::checkFootprintPath(
    traversability_msgs::CheckFootprintPath::Request& request,
    traversability_msgs::CheckFootprintPath::Response& response)
{
  const int nPaths = request.path.size();
  if (nPaths == 0) {
    ROS_WARN("No footprint path available to check!");
    return false;
  }

  traversability_msgs::TraversabilityResult result;
  traversability_msgs::FootprintPath path;
  for (int j = 0; j < nPaths; j++) {
    path = request.path[j];
    if (!traversabilityMap_.checkFootprintPath(path, result, true))
      return false;
    response.result.push_back(result);
  }

  return true;
}

bool TraversabilityEstimation::getTraversabilityMap(
    grid_map_msgs::GetGridMap::Request& request,
    grid_map_msgs::GetGridMap::Response& response)
{
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

bool TraversabilityEstimation::saveToBag(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Save to bag.");
  string pathToBag = ros::package::getPath("traversability_planner");
  pathToBag += "/global_maps/traversability_map.bag";
  std::string topic = "grid_map";
  return grid_map::GridMapRosConverter::saveToBag(traversabilityMap_.getTraversabilityMap(), pathToBag, topic);
}

} /* namespace */
