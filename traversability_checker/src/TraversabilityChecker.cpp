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

  // Read footprint
  std::string fullParamName;
  std::string fullRadiusParamName;

  if (nodeHandle_.searchParam("footprint_polygon", fullParamName)) {
    XmlRpc::XmlRpcValue footprintXmlrpc;
    nodeHandle_.getParam(fullParamName, footprintXmlrpc);
    if (footprintXmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString) {
      readFootprintFromString(std::string(footprintXmlrpc));
    } else if (footprintXmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      readFootprintFromXMLRPC(footprintXmlrpc, fullParamName);
    }
  } else if (nodeHandle_.searchParam("footprint_radius", fullRadiusParamName)) {
    nodeHandle_.param(fullRadiusParamName, footprintRadius_, 0.25);
  }
  return true;
}

void TraversabilityChecker::readFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprintXmlrpc, const std::string& fullParamName)
{
  // Make sure we have an array of at least 3 elements.
  if (footprintXmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray
      || footprintXmlrpc.size() < 3) {
    ROS_ERROR(
        "The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
        fullParamName.c_str(), std::string(footprintXmlrpc).c_str());
    throw std::runtime_error(
        "The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  geometry_msgs::Point32 pt;

  for (int i = 0; i < footprintXmlrpc.size(); ++i) {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = footprintXmlrpc[i];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray
        || point.size() != 2) {
      ROS_ERROR(
          "The footprint (parameter %s) must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
          fullParamName.c_str());
      throw std::runtime_error(
          "The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x = getNumberFromXMLRPC(point[0], fullParamName);
    pt.y = getNumberFromXMLRPC(point[1], fullParamName);
    pt.z = 0.0;

    footprintPoints_.push_back(pt);
  }
}

bool TraversabilityChecker::readFootprintFromString(const std::string& footprintString)
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(footprintString, error);
  if (error != "") {
    ROS_ERROR("Error parsing footprint parameter: '%s'", error.c_str());
    ROS_ERROR("  Footprint string was '%s'.", footprintString.c_str());
    return false;
  }

  // convert vvf into points.
  if (vvf.size() < 3) {
    ROS_ERROR(
        "You must specify at least three points for the robot footprint.");
    return false;
  }

  footprintPoints_.reserve(vvf.size());
  for (unsigned int i = 0; i < vvf.size(); i++) {
    if (vvf[i].size() == 2) {
      geometry_msgs::Point32 point;
      point.x = vvf[i][0];
      point.y = vvf[i][1];
      point.z = 0;
      footprintPoints_.push_back(point);
    } else {
      ROS_ERROR(
          "Points in the footprint specification must be pairs of numbers. Found a point with %d numbers.",
          int(vvf[i].size()));
      return false;
    }
  }
  return true;
}

std::vector<std::vector<float> > TraversabilityChecker::parseVVF(const std::string& input, std::string& errorReturn)
{
  std::vector<std::vector<float> > result;

  std::stringstream inputStringStream(input);
  int depth = 0;
  std::vector<float> currentVector;
  while (!!inputStringStream && !inputStringStream.eof()) {
    switch (inputStringStream.peek()) {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth > 2) {
          errorReturn = "Array depth greater than 2";
          return result;
        }
        inputStringStream.get();
        currentVector.clear();
        break;
      case ']':
        depth--;
        if (depth < 0) {
          errorReturn = "More close ] than open [";
          return result;
        }
        inputStringStream.get();
        if (depth == 1) {
          result.push_back(currentVector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        inputStringStream.get();
        break;
      default:  // All other characters should be part of the numbers.
        if (depth != 2) {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '"
                 << char(inputStringStream.peek()) << "'.";
          errorReturn = err_ss.str();
          return result;
        }
        float value;
        inputStringStream >> value;
        if (!!inputStringStream) {
          currentVector.push_back(value);
        }
        break;
    }
  }

  if (depth != 0) {
    errorReturn = "Unterminated vector string.";
  } else {
    errorReturn = "";
  }

  return result;
}

double TraversabilityChecker::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& fullParamName)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt
      && value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
    std::string& valueString = value;
    ROS_ERROR(
        "Values in the footprint specification (param %s) must be numbers. Found value %s.",
        fullParamName.c_str(), valueString.c_str());
    throw std::runtime_error(
        "Values in the footprint specification must be numbers");
  }
  return
      value.getType() == XmlRpc::XmlRpcValue::TypeInt ?
          (int) (value) : (double) (value);
}

void TraversabilityChecker::check(const ros::TimerEvent&)
{
  // TODO Make sure robotPose_ and robotTwist_ were already received.
  ROS_DEBUG("Checking for traversability.");
  const geometry_msgs::Pose& startPose = robotPose_.pose;
  geometry_msgs::Pose endPose;

  geometry_msgs::Vector3Stamped linearVelocityInTwistFrame, linearVelocityInBaseFrame, angularVelocityInTwistFrame, angularVelocityInBaseFrame;
  linearVelocityInTwistFrame.header = twist_.header;
  linearVelocityInTwistFrame.vector = twist_.twist.linear;
  angularVelocityInTwistFrame.header = twist_.header;
  angularVelocityInTwistFrame.vector = twist_.twist.angular;
  try {
    ros::Time now = ros::Time::now();
    tfListener_.waitForTransform(twist_.header.frame_id, robotPose_.header.frame_id, now, timerDuration_);
    tfListener_.transformVector(robotPose_.header.frame_id, linearVelocityInTwistFrame, linearVelocityInBaseFrame);
    tfListener_.transformVector(robotPose_.header.frame_id, angularVelocityInTwistFrame, angularVelocityInBaseFrame);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Extrapolation of linear velocity
  endPose.position.x = startPose.position.x + extrapolationDuration_ * linearVelocityInBaseFrame.vector.x;
  endPose.position.y = startPose.position.y + extrapolationDuration_ * linearVelocityInBaseFrame.vector.y;
  endPose.position.z = startPose.position.z + extrapolationDuration_ * linearVelocityInBaseFrame.vector.z;
  // Extrapolation of angular velocity
  Eigen::Vector3f rotationAxis;
  Eigen::Quaternionf startPoseOrientation, endPoseOrientation, startToEnd;
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
  endPoseOrientation = startToEnd*startPoseOrientation;
  endPose.orientation.x = endPoseOrientation.x();
  endPose.orientation.y = endPoseOrientation.y();
  endPose.orientation.z = endPoseOrientation.z();
  endPose.orientation.w = endPoseOrientation.w();

  traversability_msgs::CheckFootprintPath check;
  auto& path = check.request.path;
  path.poses.header = robotPose_.header;
  path.poses.poses.push_back(startPose);
  path.poses.poses.push_back(endPose);
  path.radius = footprintRadius_;
  path.footprint.polygon.points = footprintPoints_;
  path.footprint.header = robotPose_.header;
  path.footprint.header.frame_id = "base";

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
