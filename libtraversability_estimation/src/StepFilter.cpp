/*
 * StepFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/StepFilter.h"
#include <pluginlib/class_list_macros.h>

namespace filters {

template<typename T>
StepFilter<T>::StepFilter()
    : weight_(0.0),
      stepCritical_(0.3),
      traversabilityType_("traversability")
{

}

template<typename T>
StepFilter<T>::~StepFilter()
{

}

template<typename T>
bool StepFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("weight"), weight_)) {
    ROS_ERROR("StepFilter did not find param weight");
    return false;
  }

  if (weight_ > 1.0 || weight_ < 0.0) {
    ROS_ERROR("StepFilter weight must be in the interval [0, 1]");
    return false;
  }

  ROS_INFO("StepFilter weight = %f", weight_);

  if (!FilterBase<T>::getParam(std::string("stepCritical"), stepCritical_)) {
    ROS_ERROR("StepFilter did not find param stepCritical");
    return false;
  }

  if (stepCritical_ < 0.0) {
    ROS_ERROR("Critical step height must be greater than zero");
    return false;
  }

  ROS_INFO("critical step height = %f", stepCritical_);
  return true;
}

template<typename T>
bool StepFilter<T>::update(const T& elevation_map, T& step_map)
{
  step_map = elevation_map;
  step_map.add("step_height", elevation_map.get("elevation"));
  step_map.add("step_danger_value", elevation_map.get("elevation"));
  std::vector<std::string> clearTypes_, validTypes_;
  clearTypes_.push_back("step_height");
  validTypes_.push_back("elevation");
  step_map.setClearTypes(clearTypes_);
  step_map.clear();

  Eigen::Array2i submapBufferSize(3, 3);
  double height, stepHeight, stepHeightMax, subMapHeight;

  for (grid_map_lib::GridMapIterator mapIterator(step_map);
      !mapIterator.isPassedEnd(); ++mapIterator) {
    if (step_map.isValid(*mapIterator, validTypes_)) {
      height = step_map.at("elevation", *mapIterator);
      stepHeightMax = 0.0;

      for (grid_map_lib::SubmapIterator subMapIterator(step_map, *mapIterator,
                                                       submapBufferSize);
          !subMapIterator.isPassedEnd(); ++subMapIterator) {
        if (step_map.isValid(*subMapIterator, validTypes_)) {
          stepHeight = abs(height - step_map.at("elevation", *subMapIterator));
          if (stepHeight > stepHeightMax) stepHeightMax = stepHeight;
        }
        subMapHeight = step_map.at("elevation", *subMapIterator);
      }

      if (!stepHeightMax == 0.0) {
        step_map.at("step_height", *mapIterator) = stepHeightMax;
      }
    }
  }
  return true;
}
;

} /* namespace */

PLUGINLIB_REGISTER_CLASS(StepFilter, filters::StepFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
