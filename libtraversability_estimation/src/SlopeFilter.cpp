#include "filters/SlopeFilter.h"
#include <pluginlib/class_list_macros.h>

using namespace filters;

template <typename T>
SlopeFilter<T>::SlopeFilter()
      : weight_(0.0)
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

  if (weight_ > 1.0 || weight_ < 0.0)
  {
    ROS_ERROR("SlopeFilter weight must be in the interval [0, 1]");
    return false;
  }

  ROS_INFO("SlopeFilter weight = %f",weight_);
  return true;
}

template <typename T>
bool SlopeFilter<T>::update(const T& elevation_map, T& slope_map)
{
  ROS_INFO("Slope Filter update.");
  slope_map = elevation_map;
  return true;
};

PLUGINLIB_REGISTER_CLASS(SlopeFilter, filters::SlopeFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
