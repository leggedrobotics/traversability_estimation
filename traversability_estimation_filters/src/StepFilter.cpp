/*
 * StepFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/StepFilter.hpp"
#include <pluginlib/class_list_macros.h>
#include <algorithm>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;

namespace filters {

template<typename T>
StepFilter<T>::StepFilter()
    : criticalValue_(0.3),
      firstWindowRadius_(0.08),
      secondWindowRadius_(0.08),
      nCellCritical_(5),
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
    ROS_ERROR("Step filter did not find param critical_value.");
    return false;
  }

  if (criticalValue_ < 0.0) {
    ROS_ERROR("Critical step height must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Critical step height = %f.", criticalValue_);

  if (!FilterBase<T>::getParam(std::string("first_window_radius"),
                               firstWindowRadius_)) {
    ROS_ERROR("Step filter did not find param 'first_window_radius'.");
    return false;
  }

  if (firstWindowRadius_ < 0.0) {
    ROS_ERROR("'first_window_radius' must be greater than zero.");
    return false;
  }

  ROS_DEBUG("First window radius of step filter = %f.", firstWindowRadius_);

  if (!FilterBase<T>::getParam(std::string("second_window_radius"),
                               secondWindowRadius_)) {
    ROS_ERROR("Step filter did not find param 'second_window_radius'.");
    return false;
  }

  if (secondWindowRadius_ < 0.0) {
    ROS_ERROR("'second_window_radius' must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Second window radius of step filter = %f.", secondWindowRadius_);

  if (!FilterBase<T>::getParam(std::string("critical_cell_number"),
                               nCellCritical_)) {
    ROS_ERROR("Step filter did not find param 'critical_cell_number'.");
    return false;
  }

  if (nCellCritical_ <= 0) {
    ROS_ERROR("'critical_cell_number' must be greater than zero.");
    return false;
  }

  ROS_DEBUG("Number of critical cells of step filter = %d.", nCellCritical_);

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    ROS_ERROR("Step filter did not find param map_type.");
    return false;
  }

  ROS_DEBUG("Step map type = %s.", type_.c_str());

  return true;
}

template<typename T>
bool StepFilter<T>::update(const T& mapIn, T& mapOut)
{
  // Add new layers to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);
  mapOut.add("step_height");

  double height, step;

  // First iteration through the elevation map.
  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    if (!mapOut.isValid(*iterator, "elevation"))
      continue;
    height = mapOut.at("elevation", *iterator);
    double heightMax, heightMin;

    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Get the highest step in the circular window.
    bool init = false;
    for (CircleIterator submapIterator(mapOut, center, firstWindowRadius_);
        !submapIterator.isPastEnd(); ++submapIterator) {
      if (!mapOut.isValid(*submapIterator, "elevation"))
        continue;
      height = mapOut.at("elevation", *submapIterator);
      // Init heightMax and heightMin
      if (!init) {
        heightMax = height;
        heightMin = height;
        init = true;
        continue;
      }
      if (height > heightMax)
        heightMax = height;
      if (height < heightMin)
        heightMin = height;
    }

    if (init)
      mapOut.at("step_height", *iterator) = heightMax - heightMin;
  }

  // Second iteration through the elevation map.
  for (GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
    int nCells = 0;
    double stepMax = 0.0;
    bool isValid = false;

    // Requested position (center) of circle in map.
    Eigen::Vector2d center;
    mapOut.getPosition(*iterator, center);

    // Compute the step height.
    for (CircleIterator submapIterator(mapOut, center, secondWindowRadius_);
        !submapIterator.isPastEnd(); ++submapIterator) {
      if (!mapOut.isValid(*submapIterator, "step_height"))
        continue;
      isValid = true;
      if (mapOut.at("step_height", *submapIterator) > stepMax) {
        stepMax = mapOut.at("step_height", *submapIterator);
      }
      if (mapOut.at("step_height", *submapIterator) > criticalValue_)
        nCells++;
    }

    if (isValid) {
      step = std::min(stepMax,
                      (double) nCells / (double) nCellCritical_ * stepMax);
      if (step < criticalValue_) {
        mapOut.at(type_, *iterator) = 1.0 - step / criticalValue_;
      } else {
        mapOut.at(type_, *iterator) = 0.0;
      }
    }
  }
  // Remove unnecessary layer.
  mapOut.erase("step_height");
  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(filters::StepFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
