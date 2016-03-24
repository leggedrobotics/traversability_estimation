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
#include <any_msgs/Toggle.h>

// STD
#include <string>

// traversability msgs
#include <traversability_msgs/Overwrite.h>

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

  bool overwriteService(traversability_msgs::Overwrite::Request& request, traversability_msgs::Overwrite::Response& response);

  bool toggleTraversabilityChecking(any_msgs::Toggle::Request& request, any_msgs::Toggle::Response& response);

  /*!
   * Publish the status of the traversability safety.
   * @param safetyStatus the status of the traversability
   * @param timeStamp the time of the status.
   */
  void publishSafetyStatus(const bool safetyStatus, const ros::Time& timeStamp);

  //! ROS node handle.
  ros::NodeHandle nodeHandle_;

  //! TF listener.
  tf::TransformListener tfListener_;

  //! Publisher of the safety status.
  ros::Publisher safetyPublisher_;

  //! Check traversability safety timer.
  ros::Timer timer_;
  ros::Duration timerDuration_;

  //! Check footprint path service client.
  ros::ServiceClient checkFootprintPathServiceClient_;
  std::string checkFootprintPathServiceName_;

  //! ROS service server.
  ros::ServiceServer overwriteServiceServer_;
  std::string overwriteServiceName_;
  ros::ServiceServer toggleCheckingServer_;
  std::string toggleCheckingName_;
  bool overwrite_;
  bool isChecking_;

  //! Robot pose subscriber.
  ros::Subscriber robotPoseSubscriber_;
  std::string robotPoseTopic_;
  geometry_msgs::PoseStamped robotPose_;

  //! Robot twist subscriber.
  ros::Subscriber twistSubscriber_;
  std::string twistTopic_;
  geometry_msgs::TwistStamped twist_;
  bool useTwistWithCovariance_;

  //! Extrapolation duration.
  double extrapolationDuration_;

  //! Footprint radius.
  double footprintRadius_;

  //! Vertices of the footprint polygon in base frame.
  std::vector<geometry_msgs::Point32> footprintPoints_;

  //! Frame id in which the footprint is specified in.
  std::string footprintFrameId_;
};

} /* namespace traversability_checker */
