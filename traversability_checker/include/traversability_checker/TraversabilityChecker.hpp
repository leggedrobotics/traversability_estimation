/*
 * TraversabilityChecker.hpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>

// STD
#include <string>

namespace traversability_checker {

class TraversabilityChecker
{
 public:
  TraversabilityChecker(const ros::NodeHandle& nodeHandle);
  virtual ~TraversabilityChecker();
  bool readParameters();
  void check(const ros::TimerEvent& timerEvent);

 private:
  ros::NodeHandle nodeHandle_;
  ros::Timer timer_;
  ros::Duration timerDuration_;
  ros::ServiceClient serviceClient_;
  std::string serviceName_;
};

} /* namespace traversability_checker */
