/*
 * TraversabilityMap.hpp
 *
 *  Created on: Jun 09, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>
#include <traversability_msgs/TraversabilityResult.h>
#include <traversability_msgs/FootprintPath.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <filters/filter_chain.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>

// Schweizer-Messer
#include <sm/timing/Timer.hpp>

// STD
#include <vector>
#include <string>

// Boost
#include <boost/thread/recursive_mutex.hpp>

namespace traversability_estimation {

/*!
 * The terrain traversability estimation core. Updates the traversbility map and
 * evaluates the traversability of single footprints on this map.
 */
class TraversabilityMap
{

 public:
  /*!
   * Constructor.
   */
  TraversabilityMap(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~TraversabilityMap();

  /*!
   * Computes the traversability based on the elevation map.
   * Traversability is set between 0.0 and 1.0, where a value of 0.0 means not
   * traversable and 1.0 means fully traversable.
   * @return true if successful.
   */
  bool computeTraversability();

  /*!
   * Checks the traversability of a footprint path and returns the traversability.
   * @param[in] path the footprint path that has to be checked.
   * @param[out] result the traversability result.
   * @return true if successful.
   */
  bool checkFootprintPath(const traversability_msgs::FootprintPath& path,
                          traversability_msgs::TraversabilityResult& result,
                          const bool publishPolygon = false);

  /*!
   * Computes the traversability of a footprint at each map cell position twice:
   * first oriented in x-direction, and second oriented according to the yaw angle.
   * @param[in] footprintYaw orientation of the footprint.
   * @return true if successful.
   */
  bool traversabilityFootprint(double footprintYaw);

  /*!
   * Computes the traversability of a circular footprint at each map cell position.
   * @param[in] radius the radius of the circular footprint.
   * @param[in] offset the offset used for radius inflation.
   * @return true if successful.
   */
  bool traversabilityFootprint(const double& radius, const double& offset);

  /*!
   * The filter chain is reconfigured with the actual parameter on the parameter server.
   * @return true if successful.
   */
  bool updateFilter();

  /*!
   * Set the traversability map from layers of a grid_map_msgs::GridMap.
   * @param[in] msg grid map with the layers of a traversability map.
   * @return true if successful.
   */
  bool setTraversabilityMap(const grid_map_msgs::GridMap& msg);

  /*!
   * Set the elevation map from layers of a grid_map_msgs::GridMap.
   * @param[in] msg grid map with a layer 'elevation'.
   * @return true if successful.
   */
  bool setElevationMap(const grid_map_msgs::GridMap& msg);

  /*!
   * Get the traversability map.
   * @return the requested traversability map.
   */
  grid_map::GridMap getTraversabilityMap();

  /*!
   * Resets the cached traversability values.
   */
  void resetTraversabilityFootprintLayers();

  /*!
   * Publishes the latest traversability map.
   */
  void publishTraversabilityMap();

  /*!
   * Checks if the traversability map is initialized.
   * @return true if the traversability map is initialized.
   */
  bool traversabilityMapInitialized();

 private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Gets the traversability value of the submap defined by the polygon. Is true if the
   * whole polygon is traversable.
   * @param[in] polygon polygon that defines submap of the traversability map.
   * @param[out] traversability traversability value of submap defined by the polygon, the traversability
   * is the mean of each cell within the polygon.
   * @return true if the whole polygon is traversable, false otherwise.
   */
  bool isTraversable(grid_map::Polygon& polygon, double& traversability);

  /*!
   * Gets the traversability value of a circular footprint.
   * @param[in] center the center position of the footprint.
   * @param[in] radiusMax the maximum radius of the footprint.
   * @param[out] traversability traversability value of the footprint.
   * @param[in] radiusMin if set (not zero), footprint inflation is applied and radiusMin is the minimum
   * valid radius of the footprint.
   * @return true if the circular footprint is traversable, false otherwise.
   */
  bool isTraversable(const grid_map::Position& center, const double& radiusMax,
                     double& traversability, const double& radiusMin = 0);

  /*!
   * Checks if the overall inclination of the robot on a line between two
   * positions is feasible.
   * @param[in] start first position of the line.
   * @param[in] end last position of the line.
   * @return true if the whole line has a feasible inclination, false otherwise.
   */
  bool checkInclination(const grid_map::Position& start,
                        const grid_map::Position& end);

  /*!
   * Checks if the map is traversable, only regarding steps, at the position defined
   * by the map index.
   * Small ditches and holes are not detected as steps.
   * @param[in] index index of the map to check.
   * @return true if no step is detected, false otherwise.
   */
  bool checkForStep(const grid_map::Index& indexStep);

  /*!
   * Checks if the map is traversable, only regarding slope, at the position defined
   * by the map index.
   * Small local slopes are not detected as slopes.
   * @param[in] index index of the map to check.
   * @return true if traversable regarding slope, false otherwise.
   */
  bool checkForSlope(const grid_map::Index& index);

  /*!
   * Checks if the map is traversable, only regarding roughness, at the position defined
   * by the map index.
   * Small local roughness is still detected as traversable terrain.
   * @param[in] index index of the map to check.
   * @return true if traversable regarding roughness, false otherwise.
   */
  bool checkForRoughness(const grid_map::Index& index);

  /*!
   * Publishes the footprint polygon.
   */
  void publishFootprintPolygon(const grid_map::Polygon& polygon);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Id of the frame of the elevation map.
  std::string mapFrameId_;

  //! Id of the frame of the robot.
  std::string robotFrameId_;

  //! Publisher of the traversability map.
  ros::Publisher traversabilityMapPublisher_;

  //! Footprint publisher.
  ros::Publisher footprintPublisher_;

  //! Vertices of the footprint polygon in base frame.
  std::vector<geometry_msgs::Point32> footprintPoints_;

  //! Robot parameter
  double maxGapWidth_;
  double circularFootprintOffset_;  // TODO: get this with FootprintPath msg.
  double criticalStepHeight_;

  //! Default value for traversability of unknown regions.
  double traversabilityDefault_;

  //! Verify footprint for roughness.
  bool checkForRoughness_;

  //! Verify overall robot inclination.
  bool checkRobotInclination_;

  //! Traversability map types.
  const std::string traversabilityType_;
  const std::string slopeType_;
  const std::string stepType_;
  const std::string roughnessType_;
  const std::string robotSlopeType_;

  //! Filter Chain
  filters::FilterChain<grid_map::GridMap> filter_chain_;

  //! Traversability map.
  grid_map::GridMap traversabilityMap_;
  std::vector<std::string> traversabilityMapLayers_;
  bool traversabilityMapInitialized_;

  //! Traversability map.
  grid_map::GridMap elevationMap_;
  std::vector<std::string> elevationMapLayers_;
  bool elevationMapInitialized_;

  //! Mutex lock for traversability map.
  boost::recursive_mutex traversabilityMapMutex_;
  boost::recursive_mutex elevationMapMutex_;

  //! Z-position of the robot pose belonging to this map.
  double zPosition_;
};

} /* namespace */
