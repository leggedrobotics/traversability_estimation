/*
 * RoughnessFilter.cpp
 *
 *  Created on: Mar 13, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/RoughnessFilter.h"
#include <pluginlib/class_list_macros.h>

namespace filters {

template<typename T>
RoughnessFilter<T>::RoughnessFilter()
    : weight_(0.0),
      roughnessCritical_(0.3),
      traversabilityType_("traversability")
{

}

template<typename T>
RoughnessFilter<T>::~RoughnessFilter()
{

}

template<typename T>
bool RoughnessFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("weight"), weight_)) {
    ROS_ERROR("RoughnessFilter did not find param weight");
    return false;
  }

  if (weight_ > 1.0 || weight_ < 0.0) {
    ROS_ERROR("RoughnessFilter weight must be in the interval [0, 1]");
    return false;
  }

  ROS_INFO("RoughnessFilter weight = %f", weight_);

  if (!FilterBase<T>::getParam(std::string("roughnessCritical"), roughnessCritical_)) {
    ROS_ERROR("RoughnessFilter did not find param roughnessCritical");
    return false;
  }

  if (roughnessCritical_ < 0.0) {
    ROS_ERROR("Critical roughness must be greater than zero");
    return false;
  }

  ROS_INFO("Critical roughness = %f", roughnessCritical_);
  return true;
}

template<typename T>
bool RoughnessFilter<T>::update(const T& elevation_map, T& roughness_map)
{
  roughness_map = elevation_map;
  ROS_INFO("Roughness Filter");

//  if (!step_map.exists("traversability")) {
//    step_map.add("traversability", step_map.get("step_danger_value"));
//  }
//  else{
//    Eigen::MatrixXf traversabliltyMap_ = step_map.get("traversability");
//    Eigen::MatrixXf stepMap_ = step_map.get("step_danger_value");
////    std::cout << "traversability map =\n" << traversabliltyMap_ << std::endl;
//    traversabliltyMap_ += stepMap_;
////    std::cout << "traversability map =\n" << traversabliltyMap_ << std::endl;
//    step_map.add("traversability", step_map.get("traversability"));
//  }

  return true;
}
;

} /* namespace */

PLUGINLIB_REGISTER_CLASS(RoughnessFilter, filters::RoughnessFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
