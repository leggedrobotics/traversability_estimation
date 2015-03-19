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
#include <string>

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
   * Computes the roughness traversability value based on an elevation map and
   * saves it as additional grid map layer.
   * The roughness traversability is set between 0.0 and 1.0, where a value of 1.0 means fully
   * traversable and 0.0 means not traversable. NAN indicates unknown values (terrain).
   * @param mapIn grid map containing elevation map and surface normals.
   * @param mapOut grid map containing mapIn and roughness traversability values.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! Maximum allowed roughness.
  double criticalValue_;

  //! Radius of submap for roughness estimation.
  double estimationRadius_;

  //! Roughness map type.
  std::string type_;
};

} /* namespace */

#endif
