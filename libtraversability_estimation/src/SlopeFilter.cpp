#include "filters/SlopeFilter.h"
#include <pluginlib/class_list_macros.h>

using namespace filters;

SlopeFilter::SlopeFilter()
{

}

SlopeFilter::~SlopeFilter()
{

}

bool SlopeFilter::configure()
{
  return true;
}

bool SlopeFilter::update(grid_map::GridMap & elevation_map, grid_map::GridMap& slope_map)
{
  slope_map = elevation_map;
  return true;
};

PLUGINLIB_REGISTER_CLASS(SlopeFilter, traversability_estimation::SlopeFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
