/*
 * StepFilter.hpp
 *
 *  Created on: Mar 12, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef STEPFILTER_HPP
#define STEPFILTER_HPP

#include <filters/filter_base.h>

namespace filters {

/*!
 * Step Filter class to compute the step traversability value of an elevation map.
 */
template<typename T>
class StepFilter : public FilterBase<T>
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
   * Computes the step traversability value based on an elevation map
   * The step traversability value is set between 0.0 and 1.0, where a value of 1.0 means fully
   * traversable and 0.0 means barely traversable. NAN indicates that the terrain
   * is not traversable.
   * @param elevationMap the map for which the step traversability value is computed.
   * @param step_map gridMap with step traversability value.
   */
  virtual bool update(const T& elevation_map, T& step_map);

 private:

  //! Weight parameter of the step filter.
  double weight_;

  //! Maximum allowed step.
  double criticalValue_;

  //! Window size for step filter
  int windowSize_;

  //! Traversability map type.
  const std::string traversabilityType_;
};

} /* namespace */

#endif
