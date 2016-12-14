/*
 * TraversabilityChecker.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: Péter Fankhauser, Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <traversability_checker/TraversabilityChecker.hpp>

#include <traversability_msgs/CheckFootprintPath.h>
#include <traversability_msgs/FootprintPath.h>
#include <navigation_msgs/DetectObstacle.h>
#include <any_msgs/State.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace traversability_checker {

TraversabilityChecker::TraversabilityChecker(const ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      useTwistWithCovariance_(false),
      overwrite_(false),
      isChecking_(false),
      isCheckingForObstacle_(false)
{
  readParameters();
  safetyPublisher_ = nodeHandle_.advertise<any_msgs::State>("safety_status", 1);
  timer_ = nodeHandle_.createTimer(timerDuration_, &TraversabilityChecker::check, this, false, false);
  checkFootprintPathServiceClient_ = nodeHandle_.serviceClient<traversability_msgs::CheckFootprintPath>(checkFootprintPathServiceName_);
  checkObstaclesServiceClient_ = nodeHandle_.serviceClient<navigation_msgs::DetectObstacle>(checkObstacleServiceName_);
  toggleCheckingServer_ = nodeHandle_.advertiseService(toggleCheckingName_, &TraversabilityChecker::toggleTraversabilityChecking, this);
  toggleDetectObstacleServer_ = nodeHandle_.advertiseService(toggleDetectObstacleName_, &TraversabilityChecker::toggleObstacleDetection, this);
  overwriteServiceServer_ = nodeHandle_.advertiseService(overwriteServiceName_, &TraversabilityChecker::overwriteService, this);
  robotPoseSubscriber_ = nodeHandle_.subscribe(robotPoseTopic_, 1, &TraversabilityChecker::updateRobotPose, this);
  if (useTwistWithCovariance_) twistSubscriber_ = nodeHandle_.subscribe(twistTopic_, 1, &TraversabilityChecker::updateRobotTwistWithCovariance, this);
  else twistSubscriber_ = nodeHandle_.subscribe(twistTopic_, 1, &TraversabilityChecker::updateRobotTwist, this);
}

TraversabilityChecker::~TraversabilityChecker()
{
}

bool TraversabilityChecker::readParameters()
{
  nodeHandle_.param("check_traversability_service_name", checkFootprintPathServiceName_, std::string("/traversability_estimation/check_footprint_path"));
  nodeHandle_.param("check_obstacle_service_name", checkObstacleServiceName_, std::string("/elevation_obstacle_detection/detect_obstacle"));
  nodeHandle_.param("is_checking_for_obstacle", isCheckingForObstacle_, false);
  nodeHandle_.param("overwrite_service_name", overwriteServiceName_, std::string("overwrite"));
  nodeHandle_.param("toggle_checking_service_name", toggleCheckingName_, std::string("toggle"));
  nodeHandle_.param("toggle_detect_obstacle_service_name", toggleDetectObstacleName_, std::string("detect_obstacle"));
  nodeHandle_.param("robot_pose_topic", robotPoseTopic_, std::string("pose"));
  nodeHandle_.param("twist_topic", twistTopic_, std::string("twist"));
  nodeHandle_.param("use_twist_with_covariance", useTwistWithCovariance_, false);
  nodeHandle_.param("extrapolation_duration", extrapolationDuration_, 1.0);
  nodeHandle_.param("footprint/footprint_frame_id", footprintFrameId_, std::string("base"));
  double rate;
  nodeHandle_.param("rate", rate, 1.0);
  timerDuration_.fromSec(1.0 / rate);
  ROS_ASSERT(!timerDuration_.isZero());

  // Read footprint polygon.
  XmlRpc::XmlRpcValue footprint;
  nodeHandle_.getParam("footprint/footprint_polygon", footprint);
  if (footprint.size() < 3) {
    ROS_WARN("Footprint polygon must consist of at least 3 points. Only %i points found.", footprint.size());
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
  return true;
}

bool TraversabilityChecker::toggleTraversabilityChecking(any_msgs::Toggle::Request& request, any_msgs::Toggle::Response& response)
{
  if (request.enable) {
    if (isChecking_) {
      ROS_INFO_STREAM("TraversabilityChecker: Traversability Checking already active.");
    } else {
      ROS_INFO_STREAM("TraversabilityChecker: Start Traversability Checking.");
      timer_.start();
      isChecking_ = true;
    }
  } else {
    if (isChecking_) {
      ROS_INFO_STREAM("TraversabilityChecker: Stop Traversability Checking.");
      timer_.stop();
      isChecking_ = false;
    } else {
      ROS_INFO_STREAM("TraversabilityChecker: Traversability Checking already inactive.");
    }
  }
  response.success = true;
  return true;
}

bool TraversabilityChecker::toggleObstacleDetection(any_msgs::Toggle::Request& request, any_msgs::Toggle::Response& response)
{
  if (request.enable == isCheckingForObstacle_)
  {
    ROS_INFO_STREAM("TraversabilityChecker: toggleObstacleDetection: Checking is already " << (isCheckingForObstacle_ ? "enabled." : "disabled."));
  }
  isCheckingForObstacle_ = request.enable;
  response.success = true;
  return true;
}

bool TraversabilityChecker::overwriteService(traversability_msgs::Overwrite::Request& request, traversability_msgs::Overwrite::Response& response)
{
  overwrite_ = request.enable;
  return true;
}

void TraversabilityChecker::check(const ros::TimerEvent&)
{
  if (overwrite_) {
    ROS_DEBUG("Traversability checking is overwritten, publish is safe.");
    publishSafetyStatus(true, ros::Time::now());
    return;
  }

  ROS_DEBUG("Checking for traversability.");
  if (robotPose_.header.stamp.isZero()) {
    ROS_WARN_STREAM_THROTTLE(5, nodeHandle_.getNamespace() << ": No robot pose received yet.");
    return;
  }

  if (twist_.header.stamp.isZero() || ros::Time::now() - twist_.header.stamp >= ros::Duration(10.0)) {
    ROS_INFO_STREAM(nodeHandle_.getNamespace() << ": Twist not received yet or too old, checking traversability with zero velocity.");
    twist_.header = robotPose_.header;
    twist_.twist.linear.x = 0.0;
    twist_.twist.linear.y = 0.0;
    twist_.twist.linear.z = 0.0;
    twist_.twist.angular.x = 0.0;
    twist_.twist.angular.y = 0.0;
    twist_.twist.angular.z = 0.0;
  }

  const geometry_msgs::Pose& startPose = robotPose_.pose;
  geometry_msgs::Pose endPose;

  geometry_msgs::Vector3Stamped linearVelocityInTwistFrame, linearVelocityInBaseFrame, angularVelocityInTwistFrame, angularVelocityInBaseFrame;
  linearVelocityInTwistFrame.header = twist_.header;
  linearVelocityInTwistFrame.vector = twist_.twist.linear;
  angularVelocityInTwistFrame.header = twist_.header;
  angularVelocityInTwistFrame.vector = twist_.twist.angular;
  try {
    if (tfListener_.waitForTransform(twist_.header.frame_id, robotPose_.header.frame_id, twist_.header.stamp, ros::Duration(1.0))) {
      tfListener_.transformVector(robotPose_.header.frame_id, linearVelocityInTwistFrame, linearVelocityInBaseFrame);
      tfListener_.transformVector(robotPose_.header.frame_id, angularVelocityInTwistFrame, angularVelocityInBaseFrame);
    } else {
      ROS_ERROR_STREAM(nodeHandle_.getNamespace() << ": " << "Waiting for transform has timed out. Footprint is not checked for validity.");
      return;
    }
  } catch (tf::TransformException& ex) {
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

  if (angularVelocityInBaseFrame.vector.x != 0 &&
      angularVelocityInBaseFrame.vector.y != 0 &&
      angularVelocityInBaseFrame.vector.z != 0) {
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
  traversability_msgs::FootprintPath footprintPath;
  footprintPath.poses.header = robotPose_.header;
  footprintPath.poses.poses.push_back(startPose);
  footprintPath.poses.poses.push_back(endPose);
  footprintPath.radius = footprintRadius_;
  footprintPath.footprint.polygon.points = footprintPoints_;
  footprintPath.footprint.header.stamp = robotPose_.header.stamp;
  footprintPath.footprint.header.frame_id = footprintFrameId_;
  check.request.path.push_back(footprintPath);
  // Sending service request to check for traversability.
  bool isTraversable = false;
  ROS_DEBUG("Sending request to %s.", checkFootprintPathServiceName_.c_str());
  checkFootprintPathServiceClient_.waitForExistence();
  ROS_DEBUG("Sending request to %s.", checkFootprintPathServiceName_.c_str());
  if (!checkFootprintPathServiceClient_.call(check)) {
    ROS_ERROR("Failed to call service %s.", checkFootprintPathServiceName_.c_str());
  } else {
    isTraversable = check.response.result[0].is_safe;
  }
  // Sending service request to check for obstacle.
  bool hasObstacle = false;
  if (isCheckingForObstacle_) {
    navigation_msgs::DetectObstacle checkObstacle;
    checkObstacle.request.path.push_back(footprintPath);
    ROS_DEBUG("Sending request to %s.", checkObstacleServiceName_.c_str());
    checkObstaclesServiceClient_.waitForExistence();
    ROS_DEBUG("Sending request to %s.", checkObstacleServiceName_.c_str());
    if (!checkObstaclesServiceClient_.call(checkObstacle)) {
      ROS_ERROR("Failed to call service %s.", checkObstacleServiceName_.c_str());
    } else {
      hasObstacle = !checkObstacle.response.obstacles[0].obstacles.empty();
    }
  }

  publishSafetyStatus(isTraversable && !hasObstacle, robotPose_.header.stamp);
}

void TraversabilityChecker::publishSafetyStatus(const bool safetyStatus, const ros::Time& timeStamp)
{
  any_msgs::State safetyStatusMessage;
  safetyStatusMessage.stamp = timeStamp;
  safetyStatusMessage.is_ok = safetyStatus;
  safetyPublisher_.publish(safetyStatusMessage);
  if (safetyStatus) {
    ROS_DEBUG("Safe.");
  } else {
    ROS_DEBUG("Not safe.");
  }
}

void TraversabilityChecker::updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  robotPose_.header = pose.header;
  robotPose_.pose = pose.pose.pose;
  ROS_DEBUG("Updated robot pose.");
}

void TraversabilityChecker::updateRobotTwist(const geometry_msgs::TwistStamped& twist)
{
  twist_ = twist;
  ROS_DEBUG("Updated robot twist.");
}

void TraversabilityChecker::updateRobotTwistWithCovariance(const geometry_msgs::TwistWithCovarianceStamped& twist)
{
  twist_.header = twist.header;
  twist_.twist = twist.twist.twist;
  ROS_DEBUG("Updated robot twist with covariance.");
}

} /* namespace traversability_checker */
