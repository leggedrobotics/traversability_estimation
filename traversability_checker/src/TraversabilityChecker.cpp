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

  // Read footprint
  std::string full_param_name;
  std::string full_radius_param_name;

  if (nodeHandle_.searchParam("footprint_polygon", full_param_name)) {
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    nodeHandle_.getParam(full_param_name, footprint_xmlrpc);
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeString) {
      readFootprintFromString(std::string(footprint_xmlrpc));
    } else if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      readFootprintFromXMLRPC(footprint_xmlrpc, full_param_name);
    }
  } else if (nodeHandle_.searchParam("footprint_radius", full_radius_param_name)) {
//    nodeHandle_.param("footprint_radius", footprintRadius_, 0.25);
  }
  return true;
}

void TraversabilityChecker::readFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name)
{
  // Make sure we have an array of at least 3 elements.
  if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray
      || footprint_xmlrpc.size() < 3) {
    ROS_ERROR(
        "The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
        full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
    throw std::runtime_error(
        "The footprint must be specified as list of lists on the parameter server with at least 3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
  }

  geometry_msgs::Point32 pt;

  for (int i = 0; i < footprint_xmlrpc.size(); ++i) {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue point = footprint_xmlrpc[i];
    if (point.getType() != XmlRpc::XmlRpcValue::TypeArray
        || point.size() != 2) {
      ROS_ERROR(
          "The footprint (parameter %s) must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
          full_param_name.c_str());
      throw std::runtime_error(
          "The footprint must be specified as list of lists on the parameter server eg: [[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x = getNumberFromXMLRPC(point[0], full_param_name);
    pt.y = getNumberFromXMLRPC(point[1], full_param_name);
    pt.z = 0.0;

    footprintPoints_.push_back(pt);
  }
}

bool TraversabilityChecker::readFootprintFromString(const std::string& footprint_string)
{
  std::string error;
  std::vector<std::vector<float> > vvf = parseVVF(footprint_string, error);
  if (error != "") {
    ROS_ERROR("Error parsing footprint parameter: '%s'", error.c_str());
    ROS_ERROR("  Footprint string was '%s'.", footprint_string.c_str());
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
          "Points in the footprint specification must be pairs of numbers.  Found a point with %d numbers.",
          int(vvf[i].size()));
      return false;
    }
  }
  return true;
}

std::vector<std::vector<float> > TraversabilityChecker::parseVVF( const std::string& input, std::string& error_return )
{
  std::vector<std::vector<float> > result;

  std::stringstream input_ss(input);
  int depth = 0;
  std::vector<float> current_vector;
  while (!!input_ss && !input_ss.eof()) {
    switch (input_ss.peek()) {
      case EOF:
        break;
      case '[':
        depth++;
        if (depth > 2) {
          error_return = "Array depth greater than 2";
          return result;
        }
        input_ss.get();
        current_vector.clear();
        break;
      case ']':
        depth--;
        if (depth < 0) {
          error_return = "More close ] than open [";
          return result;
        }
        input_ss.get();
        if (depth == 1) {
          result.push_back(current_vector);
        }
        break;
      case ',':
      case ' ':
      case '\t':
        input_ss.get();
        break;
      default:  // All other characters should be part of the numbers.
        if (depth != 2) {
          std::stringstream err_ss;
          err_ss << "Numbers at depth other than 2. Char was '"
                 << char(input_ss.peek()) << "'.";
          error_return = err_ss.str();
          return result;
        }
        float value;
        input_ss >> value;
        if (!!input_ss) {
          current_vector.push_back(value);
        }
        break;
    }
  }

  if (depth != 0) {
    error_return = "Unterminated vector string.";
  } else {
    error_return = "";
  }

  return result;
}

double TraversabilityChecker::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt
      && value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
    std::string& value_string = value;
    ROS_FATAL(
        "Values in the footprint specification (param %s) must be numbers. Found value %s.",
        full_param_name.c_str(), value_string.c_str());
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

  geometry_msgs::Vector3Stamped linearVelocityInTwistFrame, linearVelocityInBaseFrame;
  linearVelocityInTwistFrame.header = robotTwist_.header;
  linearVelocityInTwistFrame.vector = robotTwist_.twist.linear;
  try {
    ros::Time now = ros::Time::now();
    tfListener_.waitForTransform(robotTwist_.header.frame_id, robotPose_.header.frame_id, now, ros::Duration(0.05));
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
