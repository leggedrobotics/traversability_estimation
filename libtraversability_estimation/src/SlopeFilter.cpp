#include "filters/SlopeFilter.h"
#include <pluginlib/class_list_macros.h>

using namespace filters;

template <typename T>
SlopeFilter<T>::SlopeFilter()
{

}

template <typename T>
SlopeFilter<T>::~SlopeFilter()
{

}

template <typename T>
bool SlopeFilter<T>::configure()
{
  return true;
}

template <typename T>
bool SlopeFilter<T>::update(const T & elevation_map, T& slope_map)
{
  ROS_INFO("Slope Filter update.");
  slope_map = elevation_map;
  return true;
};

PLUGINLIB_REGISTER_CLASS(SlopeFilter, filters::SlopeFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
