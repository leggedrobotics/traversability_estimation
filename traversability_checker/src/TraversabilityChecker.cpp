/*
 * TraversabilityChecker.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <traversability_checker/TraversabilityChecker.hpp>

#include <traversability_msgs/CheckFootprintPath.h>

namespace traversability_checker {

TraversabilityChecker::TraversabilityChecker(const ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  readParameters();
  timer_ = nodeHandle_.createTimer(timerDuration_, &TraversabilityChecker::check, this);
  serviceClient_ = nodeHandle_.serviceClient<traversability_msgs::CheckFootprintPath>(serviceName_, true);
}

TraversabilityChecker::~TraversabilityChecker()
{
}

void TraversabilityChecker::check(const ros::TimerEvent&)
{
  ROS_DEBUG("Checking for traversability.");
}

bool TraversabilityChecker::readParameters()
{
  nodeHandle_.param("service_to_call", serviceName_,
                    std::string("/traversability_estimation/check_footprint_path"));
  double rate;
  nodeHandle_.param("rate", rate, 2.0);
  timerDuration_.fromSec(1.0 / rate);
  ROS_ASSERT(!timerDuration_.isZero());
  return true;
}


} /* namespace traversability_checker */
