/*
 * RoughnessFilter.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <traversability_estimation/TraversabilityFilterBase.hpp>

namespace traversabilty_estimation {

/*!
 * Roughness Filter for traversability_estimation. Calculates the potential hazard value of terrain roughness.
 */
class RoughnessFilter : public TraversabilityFilterBase
{

public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param transformListener the ROS transform listener.
   */
  RoughnessFilter(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener);

  /*!
   * Destructor.
   */
	virtual ~RoughnessFilter();

private:

};


} /* namespace traversabilty_estimation */
