/*
 * StepFilter.h
 *
 *  Created on: Mar 12, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef STEPFILTER_H
#define STEPFILTER_H
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
 * Step Filter class to compute the step danger value of an elevation map.
 */
template <typename T>
class StepFilter: public FilterBase<T>
{

public:
  /*!
   * Constructor
   */
  StepFilter();

  /*!
  * Destructor.
  */
  virtual ~StepFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Computes the step danger value based on an elevation map
   * The step danger value is set between 0.0 and 1.0, where a value of 0.0 means fully
   * traversable and 1.0 means barely traversable. NAN indicates that the terrain
   * is not traversable.
   * @param elevationMap the map for which the step danger value is computed.
   * @param step_map gridMap with step danger value.
   */
  virtual bool update(const T & elevation_map, T& step_map);

private:

  //! Weight parameter of the step filter.
  double weight_;

  //! Maximum allowed step.
  double stepCritical_;

  //! Traversability map type.
  const std::string traversabilityType_;
};

} /* namespace */

#endif
