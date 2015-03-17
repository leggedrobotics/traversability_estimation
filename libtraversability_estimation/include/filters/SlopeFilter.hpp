/*
 * SlopeFilter.hpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef SLOPEFILTER_HPP
#define SLOPEFILTER_HPP

#include <filters/filter_base.h>

namespace filters {

/*!
 * Slope Filter class to compute the slope traversability value of an elevation map.
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
   * Computes the slope traversability value based on an elevation map
   * The slope traversability value is set between 0.0 and 1.0, where a value of 1.0 means fully
   * traversable and 0.0 means barely traversable. NAN indicates that the terrain
   * is not traversable.
   * @param elevationMap the map for which the slope traversability value is computed.
   * @param slope_map gridMap with slope traversability value.
   */
  virtual bool update(const T& elevation_map, T& slope_map);

 private:

  //! Weight parameter of the slope filter.
  double weight_;

  //! Maximum allowed slope.
  double criticalValue_;

  //! Traversability map type.
  const std::string traversabilityType_;
};

} /* namespace */

#endif
