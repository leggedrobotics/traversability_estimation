/*
 * SlopeFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/SlopeFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map
#include <grid_map/GridMap.hpp>

// Grid Map lib
#include <grid_map_lib/iterators/GridMapIterator.hpp>

namespace filters {

template<typename T>
SlopeFilter<T>::SlopeFilter()
    : criticalValue_(M_PI_4),
      type_("traversability_slope")
{

}

template<typename T>
SlopeFilter<T>::~SlopeFilter()
{

}

template<typename T>
bool SlopeFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("criticalValue"), criticalValue_)) {
    ROS_ERROR("SlopeFilter did not find param criticalValue");
    return false;
  }

  if (criticalValue_ > M_PI_2 || criticalValue_ < 0.0) {
    ROS_ERROR("Critical slope must be in the interval [0, PI/2]");
    return false;
  }

  ROS_INFO("critical Slope = %f", criticalValue_);
  return true;
}

template<typename T>
bool SlopeFilter<T>::update(const T& mapIn, T& mapOut)
{
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_, mapIn.get("elevation"));

  // Set clear and valid types.
  std::vector<std::string> clearTypes, validTypes;
  clearTypes.push_back(type_);
  validTypes.push_back("surface_normal_z");
  mapOut.setClearTypes(clearTypes);
  mapOut.clear();

  double slope, slopeMax = 0.0;

  for (grid_map_lib::GridMapIterator iterator(mapOut);
      !iterator.isPassedEnd(); ++iterator) {

    // Check if there is a surface normal (empty cell).
    if (!mapOut.isValid(*iterator, validTypes)) continue;

    // Compute slope from surface normal z
    slope = acos(mapOut.at("surface_normal_z", *iterator));

    if (slope < criticalValue_) {
      mapOut.at(type_, *iterator) = 1.0 - slope / criticalValue_;
    }
    else {
      mapOut.at(type_, *iterator) = 0.0;
    }

    if (slope > slopeMax) slopeMax = slope;
  }

  ROS_INFO("slope max = %f", slopeMax);

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(SlopeFilter, filters::SlopeFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
