/*
 * TraversabilityChecker.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Péter Fankhauser, Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <traversability_checker/TraversabilityChecker.hpp>

#include <traversability_msgs/CheckFootprintPath.h>
#include <any_msgs/SafetyCheck.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace traversability_checker {

TraversabilityChecker::TraversabilityChecker(const ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      useTwistWithCovariance_(false)
{
  readParameters();
  safetyPublisher_ = nodeHandle_.advertise<any_msgs::SafetyCheck>("safety_status", 1);
  timer_ = nodeHandle_.createTimer(timerDuration_, &TraversabilityChecker::check, this);
  serviceClient_ = nodeHandle_.serviceClient<traversability_msgs::CheckFootprintPath>(serviceName_);
  robotPoseSubscriber_ = nodeHandle_.subscribe(robotPoseTopic_, 1, &TraversabilityChecker::updateRobotPose, this);
  if (useTwistWithCovariance_) twistSubscriber_ = nodeHandle_.subscribe(twistTopic_, 1, &TraversabilityChecker::updateRobotTwistWithCovariance, this);
  else twistSubscriber_ = nodeHandle_.subscribe(twistTopic_, 1, &TraversabilityChecker::updateRobotTwist, this);
}

TraversabilityChecker::~TraversabilityChecker()
{
}

bool TraversabilityChecker::readParameters()
{
  nodeHandle_.param("service_to_call", serviceName_,
                    std::string("/traversability_estimation/check_footprint_path"));
  nodeHandle_.param("robot_pose_topic", robotPoseTopic_, std::string("pose"));
  nodeHandle_.param("twist_topic", twistTopic_, std::string("twist"));
  nodeHandle_.param("use_twist_with_covariance", useTwistWithCovariance_, false);
  nodeHandle_.param("extrapolation_duration", extrapolationDuration_, 1.0);
  double rate;
  nodeHandle_.param("rate", rate, 1.0);
  timerDuration_.fromSec(1.0 / rate);
  ROS_ASSERT(!timerDuration_.isZero());

  // Read footprint polygon.
  XmlRpc::XmlRpcValue footprint;
  nodeHandle_.getParam("footprint_polygon", footprint);
  if (footprint.size() < 2) {
    ROS_WARN(
        "Footprint polygon must consist of at least 3 points. Only %i points found.",
        footprint.size());
    footprintPoints_.clear();
  } else {
    geometry_msgs::Point32 pt;
    pt.z = 0.0;
    for (int i = 0; i < footprint.size(); i++) {
      pt.x = (double) footprint[i][0];
      pt.y = (double) footprint[i][1];
      footprintPoints_.push_back(pt);
    }
  }

  nodeHandle_.param("footprint_frame_id", footprintFrameId_, std::string("base"));

  return true;
}

void TraversabilityChecker::check(const ros::TimerEvent&)
{
  ROS_DEBUG("Checking for traversability.");
  if (robotPose_.header.stamp.isZero()) {
    ROS_WARN_STREAM(nodeHandle_.getNamespace() << ": No robot pose received yet.");
    return;
  }

  if (ros::Time::now() - twist_.header.stamp >= ros::Duration(10.0)) {
    ROS_INFO_STREAM(nodeHandle_.getNamespace() << ": Twist too old, checking traversability with zero velocity.");
    twist_.header = robotPose_.header;
    twist_.twist.linear.x = 0.0;
    twist_.twist.linear.y = 0.0;
    twist_.twist.linear.z = 0.0;
    twist_.twist.angular.x = 0.0;
    twist_.twist.angular.y = 0.0;
    twist_.twist.angular.z = 0.0;
    return;
  }

  const geometry_msgs::Pose& startPose = robotPose_.pose;
  geometry_msgs::Pose endPose;

  geometry_msgs::Vector3Stamped linearVelocityInTwistFrame, linearVelocityInBaseFrame, angularVelocityInTwistFrame, angularVelocityInBaseFrame;
  linearVelocityInTwistFrame.header = twist_.header;
  linearVelocityInTwistFrame.vector = twist_.twist.linear;
  angularVelocityInTwistFrame.header = twist_.header;
  angularVelocityInTwistFrame.vector = twist_.twist.angular;
  try {
    tfListener_.waitForTransform(twist_.header.frame_id, robotPose_.header.frame_id, robotPose_.header.stamp, timerDuration_);
    tfListener_.transformVector(robotPose_.header.frame_id, linearVelocityInTwistFrame, linearVelocityInBaseFrame);
    tfListener_.transformVector(robotPose_.header.frame_id, angularVelocityInTwistFrame, angularVelocityInBaseFrame);
  } catch (tf::TransformException ex) {
    ROS_ERROR_STREAM(nodeHandle_.getNamespace() << ": " << ex.what());
    return;
  }

  // Extrapolation of linear velocity.
  endPose.position.x = startPose.position.x + extrapolationDuration_ * linearVelocityInBaseFrame.vector.x;
  endPose.position.y = startPose.position.y + extrapolationDuration_ * linearVelocityInBaseFrame.vector.y;
  endPose.position.z = startPose.position.z + extrapolationDuration_ * linearVelocityInBaseFrame.vector.z;

  // Extrapolation of angular velocity.
  Eigen::Vector3f rotationAxis;
  Eigen::Quaternionf startPoseOrientation, endPoseOrientation, startToEnd;

  if (angularVelocityInBaseFrame.vector.x != 0 && angularVelocityInBaseFrame.vector.y != 0
      && angularVelocityInBaseFrame.vector.z != 0) {
    rotationAxis.x() = angularVelocityInBaseFrame.vector.x;
    rotationAxis.y() = angularVelocityInBaseFrame.vector.y;
    rotationAxis.z() = angularVelocityInBaseFrame.vector.z;
    double angle = rotationAxis.norm() * extrapolationDuration_;
    rotationAxis.normalize();
    startToEnd = Eigen::AngleAxis<float>(angle, rotationAxis);
    startPoseOrientation.x() = startPose.orientation.x;
    startPoseOrientation.y() = startPose.orientation.y;
    startPoseOrientation.z() = startPose.orientation.z;
    startPoseOrientation.w() = startPose.orientation.w;
    endPoseOrientation = startToEnd * startPoseOrientation;
    endPose.orientation.x = endPoseOrientation.x();
    endPose.orientation.y = endPoseOrientation.y();
    endPose.orientation.z = endPoseOrientation.z();
    endPose.orientation.w = endPoseOrientation.w();
  } else {
    endPose.orientation = startPose.orientation;
  }

  // Create service request.
  traversability_msgs::CheckFootprintPath check;
  auto& path = check.request.path;
  path.poses.header = robotPose_.header;
  path.poses.poses.push_back(startPose);
  path.poses.poses.push_back(endPose);
  path.radius = footprintRadius_;
  path.footprint.polygon.points = footprintPoints_;
  path.footprint.header.stamp = robotPose_.header.stamp;
  path.footprint.header.frame_id = footprintFrameId_;

  // Sending service request.
  ROS_DEBUG("Sending request to %s.", serviceName_.c_str());
  serviceClient_.waitForExistence();
  ROS_DEBUG("Sending request to %s.", serviceName_.c_str());
  if (!serviceClient_.call(check)) {
    ROS_ERROR("Failed to call service %s.", serviceName_.c_str());
    return;
  }

  publishSafetyStatus(check.response.is_safe, robotPose_.header.stamp);
}

void TraversabilityChecker::publishSafetyStatus(const bool safetyStatus, const ros::Time& timeStamp)
{
  any_msgs::SafetyCheck safetyStatusMessage;
  safetyStatusMessage.stamp = timeStamp;
  safetyStatusMessage.is_safe = safetyStatus;
  safetyPublisher_.publish(safetyStatusMessage);
  if (safetyStatus) {
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
    const geometry_msgs::TwistStamped& twist)
{
  twist_ = twist;
  ROS_DEBUG("Updated robot twist with covariance.");
}

void TraversabilityChecker::updateRobotTwistWithCovariance(
    const geometry_msgs::TwistWithCovarianceStamped& twist)
{
  twist_.header = twist.header;
  twist_.twist = twist.twist.twist;
  ROS_DEBUG("Updated robot twist.");
}

} /* namespace traversability_checker */
