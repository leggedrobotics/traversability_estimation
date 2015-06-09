/*
 * TraversabilityMap.hpp
 *
 *  Created on: Jun 09, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Grid Map
#include <grid_map/grid_map.hpp>
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

// Eigen
#include <Eigen/Core>

namespace traversability_estimation {

/*!
 * The terrain traversability estimation main class. Coordinates the ROS
 * interfaces, the timing, and the data handling between the other classes.
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
   * Updates the traversability map based on the current elevation map.
   */
  void updateTraversability();

  /*!
   * Checks the traversability of a footprint path and returns the traversability.
   * @param[in] path the footprint path that has to be checked.
   * @param[out] result the traversability result.
   */
  void checkFootprintPath(const traversability_msgs::FootprintPath& path, traversability_msgs::TraversabilityResult& result);

  /*!
   * Set the traversability map from the 'traversability' layer of
   * a grid_map_msgs::GridMap.
   * @param[in] msg grid map with a layer 'traversability'.
   */
  void setTraversabilityMap(const grid_map_msgs::GridMap& msg);

  /*!
   * Set the elevation map from the 'elevation' layer of
   * a grid_map_msgs::GridMap.
   * @param[in] msg grid map with a layer 'elevation'.
   */
  void setElevationMap(const grid_map_msgs::GridMap& msg);

 private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Computes the traversability based on the elevation map.
   * Traversability is set between 0.0 and 1.0, where a value of 0.0 means not
   * traversable and 1.0 means fully traversable.
   */
  void computeTraversability();

  /*!
   * Gets the traversability value of the submap defined by the polygon. Is true if the
   * whole polygon is traversable.
   * @param[in] polygon polygon that defines submap of the traversability map.
   * @param[out] traversability traversability value of submap defined by the polygon, the traversability
   * is the mean of each cell within the polygon.
   * @return true if the whole polygon is traversable, false otherwise.
   */
  bool isTraversable(const grid_map::Polygon& polygon, double& traversability);

  /*!
   * Checks if the overall inclination of the robot on a line between two
   * positions is feasible.
   * @param[in] start first position of the line.
   * @param[in] end last position of the line.
   * @return true if the whole line has a feasible inclination, false otherwise.
   */
  bool checkInclination(const grid_map::Position start,
                        const grid_map::Position end);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Id of the frame of the elevation map.
  std::string mapFrameId_;

  //! Id of the frame of the robot.
  std::string robotFrameId_;

  //! Publisher of the traversability occupancy grid.
  ros::Publisher traversabilityMapPublisher_;

  //! Publisher of the roughness filter occupancy grid.
  ros::Publisher footprintPolygonPublisher_;

  //! Vertices of the footprint polygon in base frame.
  std::vector<geometry_msgs::Point32> footprintPoints_;

  //! Default value for traversability of unknown regions.
  double traversabilityDefault_;

  //! Traversability map types.
  const std::string traversabilityType_;

  //! Filter Chain
  filters::FilterChain<grid_map::GridMap> filter_chain_;

  //! Traversability map.
  grid_map::GridMap traversabilityMap_;

  //! Traversability map.
  grid_map::GridMap elevationMap_;

  //! Timer
  std::string timerId_;
  sm::timing::Timer timer_;
};

} /* namespace */
