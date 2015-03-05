/*
 * StepFilter.hpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <traversability_estimation/TraversabilityFilterBase.hpp>

namespace traversabilty_estimation {

/*!
 * Step Filter for traversability_estimation. Calculates the potential hazard value of steps.
 */
class StepFilter : public TraversabilityFilterBase
{

public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   * @param transformListener the ROS transform listener.
   */
  StepFilter(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener);

  /*!
   * Destructor.
   */
	virtual ~StepFilter();

private:

};


} /* namespace traversabilty_estimation */
