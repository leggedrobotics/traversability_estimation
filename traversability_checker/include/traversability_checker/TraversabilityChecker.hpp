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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>

// STD
#include <string>

namespace traversability_checker {

class TraversabilityChecker
{
 public:
  TraversabilityChecker(const ros::NodeHandle& nodeHandle);
  virtual ~TraversabilityChecker();
  void check(const ros::TimerEvent& timerEvent);
  void updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void updateRobotTwist(const geometry_msgs::TwistStamped& twist);
  void updateRobotTwistWithCovariance(const geometry_msgs::TwistWithCovarianceStamped& twist);

 private:

  bool readParameters();

  /*!
   * Publish the status of the traversability safety.
   * @param safetyStatus the status of the traversability
   * @param timeStamp the time of the status.
   */
  void publishSafetyStatus(const bool safetyStatus, const ros::Time& timeStamp);


  ros::NodeHandle nodeHandle_;
  tf::TransformListener tfListener_;
  ros::Publisher safetyPublisher_;
  ros::Timer timer_;
  ros::Duration timerDuration_;
  ros::ServiceClient serviceClient_;
  std::string serviceName_;
  ros::Subscriber robotPoseSubscriber_;
  std::string robotPoseTopic_;
  ros::Subscriber twistSubscriber_;
  std::string twistTopic_;
  bool useTwistWithCovariance_;
  double extrapolationDuration_;
  double footprintRadius_;
  geometry_msgs::PoseStamped robotPose_;
  geometry_msgs::TwistStamped twist_;

  //! Vertices of the footprint polygon in base frame.
  std::vector<geometry_msgs::Point32> footprintPoints_;

  //! Frame id in which the footprint is specified in.
  std::string footprintFrameId_;
};

} /* namespace traversability_checker */
