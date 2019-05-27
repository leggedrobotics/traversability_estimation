/*
 * TraversabilityEstimation.hpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "traversability_estimation/TraversabilityMap.hpp"

// Grid Map
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/GetGridMapInfo.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_ros/grid_map_ros.hpp>

// Traversability estimation
#include <traversability_msgs/CheckFootprintPath.h>

// ROS
#include <filters/filter_chain.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

// STD
#include <string>
#include <vector>

namespace traversability_estimation {

/*!
 * The terrain traversability estimation main class. Coordinates the ROS
 * interfaces, the timing, and the data handling between the other classes.
 */
class TraversabilityEstimation {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  TraversabilityEstimation(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~TraversabilityEstimation();

  /*!
   * ROS service callback function to load an elevation map from a ROS bag file and to compute
   * the corresponding traversability.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
  bool loadElevationMap(grid_map_msgs::ProcessFile::Request& request, grid_map_msgs::ProcessFile::Response& response);

  /*!
   * ROS service callback function that forces an update of the traversability map,
   * given an elevation map and returns the map info of the traversability map.
   * @param request the ROS service request.
   * @param response the ROS service response containing the traversability map info.
   * @return true if successful.
   */
  bool updateServiceCallback(grid_map_msgs::GetGridMapInfo::Request& request, grid_map_msgs::GetGridMapInfo::Response& response);

  /*!
   * ROS service callback function that forces an update of the filter parameters.
   * The parameters are read from the .yaml file and put on the parameter server.
   * The filter chain is reconfigured with the new parameter.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
  bool updateParameter(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!
   * ROS service callback function to return a boolean to indicate if a path is traversable.
   * @param request the ROS service request defining footprint path.
   * @param response the ROS service response containing the traversability of the footprint path.
   * @return true if successful.
   */
  bool checkFootprintPath(traversability_msgs::CheckFootprintPath::Request& request,
                          traversability_msgs::CheckFootprintPath::Response& response);

  /*!
   * Callback function that receives an image and converts into
   * an elevation layer of a grid map.
   * @param image the received image.
   */
  void imageCallback(const sensor_msgs::Image& image);

  /*!
   * ROS service callback function that computes the traversability of a footprint
   * at each map cell position twice: first oriented in x-direction, and second
   * oriented according to the yaw angle.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
  bool traversabilityFootprint(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!
   * ROS service callback function to return the traversability map (or a submap of it).
   * @param request the ROS service request defining the location and size of the (sub-)map.
   * @param response the ROS service response containing the requested (sub-)map.
   * @return true if successful.
   */
  bool getTraversabilityMap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);

  /*!
   * Saves the traversability map with all layers to a ROS bag.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
  bool saveToBag(grid_map_msgs::ProcessFile::Request& request, grid_map_msgs::ProcessFile::Response& response);

  /*!
   * Callback to receive a grid map message that is used to initialize the traversability map, only if it is not already initialized.
   * @param message grid map message to be used to initialize the traversability map.
   */
  void gridMapToInitTraversabilityMapCallback(const grid_map_msgs::GridMap& message);

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Computes the traversability and publishes it as grid map.
   * Traversability is set between 0.0 and 1.0, where a value of 0.0 means not
   * traversable and 1.0 means fully traversable.
   * @return true if successful.
   */
  bool updateTraversability();

  /*!
   * Callback function for the update timer. Forces an update of the traversability
   * map from a new elevation map requested from the grid map service.
   * @param timerEvent the timer event.
   */
  void updateTimerCallback(const ros::TimerEvent& timerEvent);

  /*!
   * Gets the grid map for the desired submap center point.
   * @param[out] map the map that is received.
   * @return true if successful, false if ROS service call failed.
   */
  bool requestElevationMap(grid_map_msgs::GridMap& map);

  /*!
   * Initializes a new traversability map based on the given grid map. Previous traversability map is overwritten.
   * @param gridMap grid map object to be used to compute new traversability map.
   * @return true on success.
   */
  bool initializeTraversabilityMapFromGridMap(const grid_map::GridMap& gridMap);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS service server.
  ros::ServiceServer footprintPathService_;
  ros::ServiceServer updateTraversabilityService_;
  ros::ServiceServer getTraversabilityService_;
  ros::ServiceServer updateParameters_;
  ros::ServiceServer traversabilityFootprint_;
  ros::ServiceServer saveToBagService_;
  ros::ServiceServer loadElevationMapService_;

  //! Image subscriber.
  ros::Subscriber imageSubscriber_;
  std::string imageTopic_;
  grid_map::GridMap imageGridMap_;
  grid_map::Position imagePosition_;
  bool getImageCallback_;
  double imageResolution_;
  double imageMinHeight_;
  double imageMaxHeight_;

  //! Grid Map topic to initialize traversability map.
  ros::Subscriber gridMapToInitTraversabilityMapSubscriber_;
  std::string gridMapToInitTraversabilityMapTopic_;
  bool acceptGridMapToInitTraversabilityMap_;

  //! Elevation map service client.
  ros::ServiceClient submapClient_;

  //! Name of the elevation submap service.
  std::string submapServiceName_;

  //! TF listener.
  tf::TransformListener transformListener_;

  //! Center point of the requested map.
  geometry_msgs::PointStamped submapPoint_;

  //! Id of the frame of the robot.
  std::string robotFrameId_;

  //! Robot name.
  std::string robot_;

  //! Vertices of the footprint polygon in base frame.
  double footprintYaw_;

  //! Timer for the map update.
  ros::Timer updateTimer_;

  //! Duration between map updates.
  ros::Duration updateDuration_;

  //! Requested elevation map layers.
  std::vector<std::string> elevationMapLayers_;

  //! Requested map length in [m].
  grid_map::Length mapLength_;

  //! Traversability map types.
  const std::string traversabilityType_;
  const std::string slopeType_;
  const std::string stepType_;
  const std::string roughnessType_;
  const std::string robotSlopeType_;

  //! Traversability map
  TraversabilityMap traversabilityMap_;

  //! Package name where the parameters are defined.
  std::string package_;

  //! Use raw or fused map.
  bool useRawMap_;
};

}  // namespace traversability_estimation
