/*
 * RoughnessFilter.hpp
 *
 *  Created on: Mar 13, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef ROUGHNESSFILTER_HPP
#define ROUGHNESSFILTER_HPP

#include <filters/filter_base.h>

namespace filters {

/*!
 * Roughness Filter class to compute the roughness traversability value of an elevation map.
 */
template<typename T>
class RoughnessFilter : public FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  RoughnessFilter();

  /*!
   * Destructor.
   */
  virtual ~RoughnessFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * Computes the roughness traversability value based on an elevation map
   * The roughness traversability value is set between 0.0 and 1.0, where a value of 1.0 means fully
   * traversable and 0.0 means barely traversable. NAN indicates that the terrain
   * is not traversable.
   * @param elevationMap the map for which the roughness traversability value is computed.
   * @param roughness_map gridMap with roughness traversability value.
   */
  virtual bool update(const T& elevation_map, T& roughness_map);

 private:

  //! Weight parameter of the roughness filter.
  double weight_;

  //! Maximum allowed roughness.
  double criticalValue_;

  //! Radius of submap for roughness estimation.
  double estimationRadius_;

  //! Traversability map type.
  const std::string traversabilityType_;
};

} /* namespace */

#endif
