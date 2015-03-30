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
#include <grid_map/GridMap.hpp>

// Grid Map lib
#include <grid_map_lib/GridMap.hpp>
#include <grid_map_lib/iterators/GridMapIterator.hpp>
#include <grid_map_lib/iterators/CircleIterator.hpp>
#include <grid_map_lib/iterators/SubmapIterator.hpp>

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
  mapOut.add(type_, mapIn.get("elevation"));

  // Set clear and valid types.
  std::vector<std::string> clearTypes, validTypes;
  clearTypes.push_back(type_);
  validTypes.push_back("elevation");
  mapOut.setClearTypes(clearTypes);
  mapOut.clear();

  double height, stepHeight, stepHeightMax = 0.0, windowStepHeightMax;
  bool stepHeightExists;

  for (grid_map_lib::GridMapIterator iterator(mapOut);
      !iterator.isPassedEnd(); ++iterator) {
    // Check if this is an empty cell (hole in the map).
    if (!mapOut.isValid(*iterator, validTypes)) continue;

    height = mapOut.at("elevation", *iterator);
    windowStepHeightMax = 0.0;
    stepHeightExists = false;

    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Get the highest step in the circular window.
    for (grid_map_lib::CircleIterator submapIterator(mapOut, center, windowRadius_);
        !submapIterator.isPassedEnd(); ++submapIterator) {
      if (mapOut.isValid(*submapIterator, validTypes)) {
        stepHeight = std::abs(height - mapOut.at("elevation", *submapIterator));
        if (stepHeight > windowStepHeightMax) {
          windowStepHeightMax = stepHeight;
          stepHeightExists = true;
        }
      }
    }

    if (stepHeightExists) {
      if (windowStepHeightMax < criticalValue_) {
        mapOut.at(type_, *iterator) = 1.0 - windowStepHeightMax / criticalValue_;
      }
      else {
        mapOut.at(type_, *iterator) = 0.0;
      }

      if (windowStepHeightMax > stepHeightMax) stepHeightMax = windowStepHeightMax;

    }
  }

  ROS_DEBUG("step height max = %f", stepHeightMax);

  return true;
}
;

} /* namespace */

PLUGINLIB_REGISTER_CLASS(StepFilter, filters::StepFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
