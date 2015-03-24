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
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>

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
   * Computes the slope traversability value based on an elevation map and
   * saves it as additional grid map layer.
   * The slope traversability is set between 0.0 and 1.0, where a value of 1.0 means fully
   * traversable and 0.0 means not traversable. NAN indicates unknown values (terrain).
   * @param mapIn grid map containing elevation map and surface normals.
   * @param mapOut grid map containing mapIn and slope traversability values.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! Maximum allowed slope.
  double criticalValue_;

  //! slope map type.
  std::string type_;
};

} /* namespace */

#endif
