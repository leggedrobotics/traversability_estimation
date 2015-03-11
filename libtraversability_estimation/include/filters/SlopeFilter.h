#ifndef SLOPEFILTER_H
#define SLOPEFILTER_H
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <boost/scoped_ptr.hpp>
#include <filters/filter_base.h>

// Grid Map
#include <grid_map/GridMap.hpp>

namespace filters {

template <typename T>
class SlopeFilter: public FilterBase<T>
{
public:

  SlopeFilter();
  virtual ~SlopeFilter();
  virtual bool configure();
  virtual bool update(const T & elevation_map, T& slope_map);

private:

  double weight_;
};

} /* namespace */

#endif
