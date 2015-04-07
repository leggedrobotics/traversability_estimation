/*
 * StepFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/StepFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map
#include <grid_map/grid_map.hpp>

using namespace grid_map;

namespace filters {

template<typename T>
StepFilter<T>::StepFilter()
    : criticalValue_(0.3),
      windowRadius_(0.3),
      type_("traversability_step")
{

}

template<typename T>
StepFilter<T>::~StepFilter()
{

}

template<typename T>
bool StepFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("critical_value"), criticalValue_)) {
    ROS_ERROR("StepFilter did not find param critical_value");
    return false;
  }

  if (criticalValue_ < 0.0) {
    ROS_ERROR("Critical step height must be greater than zero");
    return false;
  }

  ROS_INFO("Critical step height = %f", criticalValue_);

  if (!FilterBase<T>::getParam(std::string("window_radius"), windowRadius_)) {
    ROS_ERROR("StepFilter did not find param window_radius");
    return false;
  }

  if (windowRadius_ < 0.0) {
    ROS_ERROR("windowRadius_ must be greater than zero");
    return false;
  }

  ROS_INFO("Window radius of step filter = %f", windowRadius_);

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    ROS_ERROR("StepFilter did not find param map_type");
    return false;
  }

  ROS_INFO("Step map type = %s", type_.c_str());

  return true;
}

template<typename T>
bool StepFilter<T>::update(const T& mapIn, T& mapOut)
{
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);

  double height, stepHeight, stepHeightMax = 0.0;
  bool stepHeightExists, heightExists;

  for (GridMapIterator iterator(mapOut);
      !iterator.isPassedEnd(); ++iterator) {
    double heightMin, heightMax;
    stepHeightExists = false;
    heightExists = false;

    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Get the highest step in the circular window.
    for (CircleIterator submapIterator(mapOut, center, windowRadius_);
        !submapIterator.isPassedEnd(); ++submapIterator) {
      if (mapOut.isValid(*submapIterator, "elevation")) {
        height = mapOut.at("elevation", *submapIterator);
        if (!heightExists) {
          heightMin = height;
          heightMax = height;
          heightExists = true;
          continue;
        }
        if (height > heightMax) heightMax = height;
        if (height < heightMin) heightMin = height;
        stepHeightExists = true;
      }
    }

    if (stepHeightExists) {
      stepHeight = heightMax - heightMin;
      if (stepHeight < criticalValue_) {
        mapOut.at(type_, *iterator) = 1.0 - stepHeight / criticalValue_;
      }
      else {
        mapOut.at(type_, *iterator) = 0.0;
      }

      if (stepHeight > stepHeightMax) stepHeightMax = stepHeight;

    }
  }

  ROS_DEBUG("step height max = %f", stepHeightMax);

  return true;
}
;

} /* namespace */

PLUGINLIB_REGISTER_CLASS(StepFilter, filters::StepFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
