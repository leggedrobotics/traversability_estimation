/*
 * SlopeFilter.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <traversability_estimation/TraversabilityFilterBase.hpp>

namespace traversabilty_estimation {

/*!
 * Slope Filter for traversability_estimation. Calculates the potential hazard value of slopes.
 */
class SlopeFilter : public TraversabilityFilterBase
{

public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param transformListener the ROS transform listener.
   */
  SlopeFilter(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener);

  /*!
   * Destructor.
   */
	virtual ~SlopeFilter();

private:
};


} /* namespace traversabilty_estimation */
