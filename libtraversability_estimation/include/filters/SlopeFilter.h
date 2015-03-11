#ifndef SLOPEFILTER_H
#define SLOPEFILTER_H
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <boost/scoped_ptr.hpp>
#include <filters/filter_base.h>

// Grid Map
#include <grid_map/GridMap.hpp>

// Grid Map lib
#include <grid_map_lib/iterators/GridMapIterator.hpp>

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

  //! Weight parameter of the slope filter.
  double weight_;

  //! Maximum allowed slope.
  double slopeCritical_;

  //! Traversability map type.
  const std::string traversabilityType_;
};

} /* namespace */

#endif
