/*
 * SlopeFilter.h
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

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

/*!
 * Slope Filter class to compute the slope danger value of an elevation map.
 */
template<typename T>
class SlopeFilter : public FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  SlopeFilter();

  /*!
   * Destructor.
   */
  virtual ~SlopeFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Computes the slope danger value based on an elevation map
   * The slope danger value is set between 0.0 and 1.0, where a value of 0.0 means fully
   * traversable and 1.0 means barely traversable. NAN indicates that the terrain
   * is not traversable.
   * @param elevationMap the map for which the slope danger value is computed.
   * @param slope_map gridMap with slope danger value.
   */
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
