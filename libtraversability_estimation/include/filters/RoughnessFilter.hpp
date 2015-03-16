/*
 * RoughnessFilter.hpp
 *
 *  Created on: Mar 13, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef ROUGHNESSFILTER_HPP
#define ROUGHNESSFILTER_HPP
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <boost/scoped_ptr.hpp>
#include <filters/filter_base.h>

// Grid Map
#include <grid_map/GridMap.hpp>

// Grid Map lib
#include <grid_map_lib/GridMap.hpp>
#include <grid_map_lib/GridMapMath.hpp>
#include <grid_map_lib/iterators/GridMapIterator.hpp>
#include <grid_map_lib/iterators/SubmapIterator.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace filters {

/*!
 * Roughness Filter class to compute the roughness danger value of an elevation map.
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
   * Computes the roughness danger value based on an elevation map
   * The roughness danger value is set between 0.0 and 1.0, where a value of 0.0 means fully
   * traversable and 1.0 means barely traversable. NAN indicates that the terrain
   * is not traversable.
   * @param elevationMap the map for which the roughness danger value is computed.
   * @param roughness_map gridMap with roughness danger value.
   */
  virtual bool update(const T & elevation_map, T& roughness_map);

 private:

  //! Weight parameter of the roughness filter.
  double weight_;

  //! Maximum allowed roughness.
  double roughnessCritical_;

  //! Radius of submap for roughness estimation.
  double roughnessEstimationRadius_;

  //! Traversability map type.
  const std::string traversabilityType_;
};

} /* namespace */

#endif
