#ifndef SLOPEFILTER_H
#define SLOPEFILTER_H
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <boost/scoped_ptr.hpp>
#include <filters/filter_base.h>

namespace filters {

class SlopeFilter: public FilterBase<grid_map::GridMap>
{
public:
  SlopeFilter();
  virtual ~SlopeFilter();
  virtual bool configure();
  virtual bool update(const grid_map::GridMap & elevation_map, grid_map::GridMap& slope_map);
};

}

#endif
