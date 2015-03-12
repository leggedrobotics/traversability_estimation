#include "filters/SlopeFilter.h"
#include <pluginlib/class_list_macros.h>
#include <grid_map_lib/iterators/GridMapIterator.hpp>

namespace filters {

template <typename T>
SlopeFilter<T>::SlopeFilter()
      : weight_(0.0),
        slopeCritical_(M_PI_4),
        traversabilityType_("traversability")

{

}

template <typename T>
SlopeFilter<T>::~SlopeFilter()
{

}

template <typename T>
bool SlopeFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("weight"), weight_))
  {
    ROS_ERROR("SlopeFilter did not find param weight");
    return false;
  }

  if (weight_ > 1.0 || weight_ < 0.0) {
    ROS_ERROR("SlopeFilter weight must be in the interval [0, 1]");
    return false;
  }

  ROS_INFO("SlopeFilter weight = %f",weight_);

  if (!FilterBase<T>::getParam(std::string("slopeCritical"), slopeCritical_))
  {
    ROS_ERROR("SlopeFilter did not find param slopeCritical");
    return false;
  }

  if (slopeCritical_ > M_PI_2 || slopeCritical_ < 0.0) {
    ROS_ERROR("SlopeFilter weight must be in the interval [0, PI/2]");
    return false;
  }

  ROS_INFO("critical Slope = %f",slopeCritical_);
  return true;
}

template <typename T>
bool SlopeFilter<T>::update(const T& elevation_map, T& slope_map)
{
  slope_map = elevation_map;
  slope_map.add("slope_danger_value", elevation_map.get("surface_normal_z"));

  double slope_;
  for (grid_map_lib::GridMapIterator iterator(slope_map); !iterator.isPassedEnd(); ++iterator) {
//    ROS_INFO("elevation = %f",slope_map.at("elevation", *iterator));
//    ROS_INFO("surface normal z = %f",slope_map.at("surface_normal_z", *iterator));
    slope_ = acos(slope_map.at("surface_normal_z", *iterator));
    if (slope_ < slopeCritical_) {
      slope_map.at("slope_danger_value", *iterator) = slope_ / slopeCritical_ * weight_;
//      ROS_INFO("slope danger value = %f",slope_ / slopeCritical_ * weight_);
    }
    else {
      slope_map.at("slope_danger_value", *iterator) = NAN;
    }
  }
  return true;
};

} /* namespace */

PLUGINLIB_REGISTER_CLASS(SlopeFilter, filters::SlopeFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
