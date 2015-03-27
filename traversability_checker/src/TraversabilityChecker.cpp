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
//  safetyPublisher_ = nodeHandle_.advertise<???>("is_safe", 1);
  timer_ = nodeHandle_.createTimer(timerDuration_, &TraversabilityChecker::check, this);
  serviceClient_ = nodeHandle_.serviceClient<traversability_msgs::CheckFootprintPath>(serviceName_, true);
  robotPoseSubscriber_ = nodeHandle_.subscribe(robotPoseTopic_, 1, &TraversabilityChecker::updateRobotPose, this);
  robotTwistSubscriber_ = nodeHandle_.subscribe(robotTwistTopic_, 1, &TraversabilityChecker::updateRobotTwist, this);
}

TraversabilityChecker::~TraversabilityChecker()
{
}

bool TraversabilityChecker::readParameters()
{
  nodeHandle_.param("service_to_call", serviceName_,
                    std::string("/traversability_estimation/check_footprint_path"));
  nodeHandle_.param("robot_pose_topic", robotPoseTopic_, std::string("/state_estimator/pose"));
  nodeHandle_.param("robot_twist_topic", robotTwistTopic_, std::string("/state_estimator/twist"));
  nodeHandle_.param("extrapolation_duration", extrapolationDuration_, 1.0);
  double rate;
  nodeHandle_.param("rate", rate, 2.0);
  timerDuration_.fromSec(1.0 / rate);
  ROS_ASSERT(!timerDuration_.isZero());
  return true;
}

void TraversabilityChecker::check(const ros::TimerEvent&)
{
  // TODO Make sure robotPose_ and robotTwist_ were already received.
  ROS_DEBUG("Checking for traversability.");
  geometry_msgs::Pose endPose;
  // TODO Add frame checking and conversion.
  // TODO Include rotation.
  endPose.position.x = robotPose_.position.x + extrapolationDuration_ * robotTwist_.linear.x;
  endPose.position.y = robotPose_.position.y + extrapolationDuration_ * robotTwist_.linear.y;
  endPose.position.z = robotPose_.position.z + extrapolationDuration_ * robotTwist_.linear.z;
  traversability_msgs::CheckFootprintPath check;
  auto& path = check.request.path;
  path.poses.header.stamp = ros::Time::now();
  path.poses.header.frame_id = "map"; // TODO
  path.poses.poses.push_back(robotPose_); // Start pose.
  path.poses.poses.push_back(endPose);
  path.radius = 0.5; // TODO

  ROS_DEBUG("Sending request to %s.", serviceName_.c_str());
  if (!serviceClient_.call(check)) {
    ROS_ERROR("Failed to call service %s.", serviceName_.c_str());
    return;
  }

//  safetyPublisher_.publish(); // TODO.

  ROS_DEBUG("Checking for traversability.");
}

void TraversabilityChecker::updateRobotPose(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  robotPose_ = pose.pose.pose;
  ROS_DEBUG("Updated robot pose.");
}

void TraversabilityChecker::updateRobotTwist(
    const geometry_msgs::TwistWithCovarianceStamped& twist)
{
  robotTwist_ = twist.twist.twist;
  ROS_DEBUG("Updated robot twist.");
}

} /* namespace traversability_checker */
