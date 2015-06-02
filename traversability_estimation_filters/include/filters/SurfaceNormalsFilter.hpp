/*
 * SurfaceNormalsFilter.hpp
 *
 *  Created on: May 05, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef SURFACENORMALSFILTER_HPP
#define SURFACENORMALSFILTER_HPP

#include <filters/filter_base.h>
#include <ros/ros.h>

#include <string>

#include <Eigen/Core>

namespace filters {

/*!
 * Surface Normals Filter class to compute the surface normals of an elevation map.
 */
template<typename T>
class SurfaceNormalsFilter : public FilterBase<T>
{

 public:
  /*!
   * Constructor
   */
  SurfaceNormalsFilter();

  /*!
   * Destructor.
   */
  virtual ~SurfaceNormalsFilter();

  /*!
   * Configures the filter from parameters on the Parameter Server
   */
  virtual bool configure();

  /*!
   * TODO: Update description.
   * Computes the surface normals based on an elevation map and
   * saves it as additional grid map layer.
   * The slope traversability is set between 0.0 and 1.0, where a value of 1.0 means fully
   * traversable and 0.0 means not traversable. NAN indicates unknown values (terrain).
   * @param mapIn grid map containing elevation map and surface normals.
   * @param mapOut grid map containing mapIn and slope traversability values.
   */
  virtual bool update(const T& mapIn, T& mapOut);

 private:

  //! Radius of submap for surface normal estimation.
  double estimationRadius_;

  //! Surface normal positive axis.
  Eigen::Vector3d surfaceNormalPositiveAxis_;
};

} /* namespace */

#endif
