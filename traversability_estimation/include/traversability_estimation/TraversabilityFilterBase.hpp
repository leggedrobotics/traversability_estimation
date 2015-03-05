/*
 * TraversabilityFilterBase.hpp
 *
 *  Created on: Mar 5, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// Grid Map
#include <grid_map/GridMap.hpp>

// Eigen
#include <Eigen/Core>

// STD
#include <vector>

namespace traversability_estimation {

/*!
* Generic traversability estimation filter base class. Provides functionalities
* common to all traversability filters.
*/
class TraversabilityFilterBase
{
public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param transformListener the ROS transform listener.
   */
  TraversabilityFilterBase(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener);

	/*!
	 * Destructor.
	 */
	virtual ~TraversabilityFilterBase();



 protected:


};

} /* namespace traversability_estimation */
