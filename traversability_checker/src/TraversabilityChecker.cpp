/*
 * TraversabilityChecker.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <traversability_checker/TraversabilityChecker.hpp>

#include <traversability_msgs/CheckFootprintPath.h>
#include <any_msgs/SafetyCheck.h>

namespace traversability_checker {

TraversabilityChecker::TraversabilityChecker(const ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  readParameters();
  safetyPublisher_ = nodeHandle_.advertise<any_msgs::SafetyCheck>("safety_status", 1);
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
  nodeHandle_.param("footprint_radius", footprintRadius_, 0.25);
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
  const geometry_msgs::Pose& startPose = robotPose_.pose;
  geometry_msgs::Pose endPose;

  geometry_msgs::Vector3Stamped linearVelocityInTwistFrame, linearVelocityInBaseFrame;
  linearVelocityInTwistFrame.header = robotTwist_.header;
  linearVelocityInTwistFrame.vector = robotTwist_.twist.linear;
  try {
    tfListener_.transformVector(robotPose_.header.frame_id, linearVelocityInTwistFrame, linearVelocityInBaseFrame);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Add footprint // TODO: Move polygon points to yaml file
  geometry_msgs::Point32 fl, fr, bl, br;
  fl.x = 0.355;
  fl.y = 0.32;
  fl.z = 0.0;
  fr.x = 0.355;
  fr.y = -0.32;
  fr.z = 0.0;
  bl.x = -0.355;
  bl.y = 0.32;
  bl.z = 0.0;
  br.x = -0.355;
  br.y = -0.32;
  br.z = 0.0;


  // TODO Include rotation.
  endPose.position.x = startPose.position.x + extrapolationDuration_ * linearVelocityInBaseFrame.vector.x;
  endPose.position.y = startPose.position.y + extrapolationDuration_ * linearVelocityInBaseFrame.vector.y;
  endPose.position.z = startPose.position.z + extrapolationDuration_ * linearVelocityInBaseFrame.vector.z;
  endPose.orientation = startPose.orientation;
  traversability_msgs::CheckFootprintPath check;
  auto& path = check.request.path;
  path.poses.header = robotPose_.header;
  path.poses.poses.push_back(startPose);
  path.poses.poses.push_back(endPose);
  path.radius = footprintRadius_;
  path.footprint.polygon.points.push_back(fl);
  path.footprint.polygon.points.push_back(fr);
  path.footprint.polygon.points.push_back(br);
  path.footprint.polygon.points.push_back(bl);
  path.footprint.header = robotPose_.header;
  path.footprint.header.frame_id = "base";
  ROS_INFO("robot pose frame id = %s", path.poses.header.frame_id.c_str());

  ROS_DEBUG("Sending request to %s.", serviceName_.c_str());
  serviceClient_.waitForExistence();
  ROS_DEBUG("Sending request to %s.", serviceName_.c_str());
  if (!serviceClient_.call(check)) {
    ROS_ERROR("Failed to call service %s.", serviceName_.c_str());
    return;
  }

  any_msgs::SafetyCheck safetyStatusMessage;
  safetyStatusMessage.stamp = robotPose_.header.stamp;
  safetyStatusMessage.is_safe = check.response.is_safe;
  safetyPublisher_.publish(safetyStatusMessage);
  if (safetyStatusMessage.is_safe) {
    ROS_DEBUG("Safe.");
  } else {
    ROS_DEBUG("Not safe.");
  }
}

void TraversabilityChecker::updateRobotPose(
    const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  robotPose_.header = pose.header;
  robotPose_.pose = pose.pose.pose;
  ROS_DEBUG("Updated robot pose.");
}

void TraversabilityChecker::updateRobotTwist(
    const geometry_msgs::TwistWithCovarianceStamped& twist)
{
  robotTwist_.header = twist.header;
  robotTwist_.twist = twist.twist.twist;
  ROS_DEBUG("Updated robot twist.");
}

} /* namespace traversability_checker */
