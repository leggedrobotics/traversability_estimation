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
#include <tf/transform_listener.h>

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
  void updateRobotPose(const geometry_msgs::PoseWithCovarianceStamped& pose);
  void updateRobotTwist(const geometry_msgs::TwistStamped& twist);
  void updateRobotTwistWithCovariance(const geometry_msgs::TwistWithCovarianceStamped& twist);

  /*!
   * Set the footprint from the given string. (If footprint is defined as string).
   * @param[in] footprintString the string which contains the footprint.
   * @return true if successful.
   */
  bool readFootprintFromString(const std::string& footprintString);

  /*!
   * Set the footprint from the given XmlRpcValue. (If footprint is defined as XmlRpcValue).
   * @param[in] footprintXmlrpc XML/RPC value which contains the fooprint.
   * @param[in] fullParamName Parameter name of the footprint.
   */
  void readFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprintXmlrpc, const std::string& fullParamName);

  /*!
   * Converts the string containing the footprint into a vector of floats.
   * @param[in] input string of the footprint.
   * @param[in] errorReturn string with error message. Empty if no error occurred.
   * @return float vector containing the footprint points.
   */
  std::vector<std::vector<float>> parseVVF(const std::string& input, std::string& errorReturn);

  /*!
   * Get the number from a XML/RPC value.
   * @param[in] value XML/RPC value that should be converted to a double.
   * @param[in] fullParamName Parameter name.
   * @return XML/RPC value converted to a double.
   */
  double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& fullParamName);

 private:
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
};

} /* namespace traversability_checker */
