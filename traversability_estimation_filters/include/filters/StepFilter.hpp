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

#include <string>

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
   * Computes the step traversability value based on an elevation map and
   * saves it as additional grid map layer.
   * The step traversability is set between 0.0 and 1.0, where a value of 1.0 means fully
   * traversable and 0.0 means not traversable. NAN indicates unknown values (terrain).
   * @param mapIn grid map containing elevation map and surface normals.
   * @param mapOut grid map containing mapIn and step traversability values.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! Maximum allowed step.
  double criticalValue_;

  //! Window sizes for step filter
  double firstWindowRadius_, secondWindowRadius_;

  //! Critical number of cells greater than maximums allowed step.
  int nCellCritical_;

  //! Step map type.
  std::string type_;
};

} /* namespace */

#endif
