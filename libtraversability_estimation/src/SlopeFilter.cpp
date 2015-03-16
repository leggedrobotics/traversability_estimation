/*
 * SlopeFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/SlopeFilter.h"
#include <pluginlib/class_list_macros.h>

namespace filters {

template<typename T>
SlopeFilter<T>::SlopeFilter()
    : weight_(0.0),
      slopeCritical_(M_PI_4),
      traversabilityType_("traversability")
{

}

template<typename T>
SlopeFilter<T>::~SlopeFilter()
{

}

template<typename T>
bool SlopeFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("weight"), weight_)) {
    ROS_ERROR("SlopeFilter did not find param weight");
    return false;
  }

  if (weight_ > 1.0 || weight_ < 0.0) {
    ROS_ERROR("SlopeFilter weight must be in the interval [0, 1]");
    return false;
  }

  ROS_INFO("SlopeFilter weight = %f", weight_);

  if (!FilterBase<T>::getParam(std::string("slopeCritical"), slopeCritical_)) {
    ROS_ERROR("SlopeFilter did not find param slopeCritical");
    return false;
  }

  if (slopeCritical_ > M_PI_2 || slopeCritical_ < 0.0) {
    ROS_ERROR("Critical slope must be in the interval [0, PI/2]");
    return false;
  }

  ROS_INFO("critical Slope = %f", slopeCritical_);
  return true;
}

template<typename T>
bool SlopeFilter<T>::update(const T& elevation_map, T& slope_map)
{
  slope_map = elevation_map;
  slope_map.add("slope_danger_value", elevation_map.get("surface_normal_z"));

  std::vector<std::string> clearTypes, validTypes;
  clearTypes.push_back("slope_danger_value");
  validTypes.push_back("surface_normal_z");
  slope_map.setClearTypes(clearTypes);
  slope_map.clear();

  double slope, slopeMax = 0.0;

  for (grid_map_lib::GridMapIterator iterator(slope_map);
      !iterator.isPassedEnd(); ++iterator) {

    // Check if there is a surface normal (empty cell).
    if (!slope_map.isValid(*iterator, validTypes)) continue;

    // Compute slope from surface normal z
    slope = acos(slope_map.at("surface_normal_z", *iterator));

    if (slope < slopeCritical_) {
      slope_map.at("slope_danger_value", *iterator) = weight_ * slope / slopeCritical_;
    }

    if (slope > slopeMax) slopeMax = slope;
  }

  ROS_INFO("slope max = %f", slopeMax);

  if (!slope_map.exists(traversabilityType_)) {
    slope_map.add(traversabilityType_, slope_map.get("slope_danger_value"));
  }
  else{
    Eigen::MatrixXf traversabliltyMap_ = slope_map.get(traversabilityType_);
    Eigen::MatrixXf slopeMap_ = slope_map.get("slope_danger_value");
    traversabliltyMap_ += slopeMap_;
    slope_map.add(traversabilityType_, slope_map.get(traversabilityType_));
  }
  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(SlopeFilter, filters::SlopeFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
