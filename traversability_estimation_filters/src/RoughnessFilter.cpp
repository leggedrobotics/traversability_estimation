/*
 * RoughnessFilter.cpp
 *
 *  Created on: Mar 13, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/RoughnessFilter.hpp"
#include <pluginlib/class_list_macros.h>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

using namespace Eigen;
using namespace grid_map;

namespace filters {

template<typename T>
RoughnessFilter<T>::RoughnessFilter()
    : criticalValue_(0.3),
      estimationRadius_(0.3),
      type_("traversability_roughness")
{

}

template<typename T>
RoughnessFilter<T>::~RoughnessFilter()
{

}

template<typename T>
bool RoughnessFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("critical_value"), criticalValue_)) {
    ROS_ERROR("RoughnessFilter did not find param critical_value");
    return false;
  }

  if (criticalValue_ < 0.0) {
    ROS_ERROR("Critical roughness must be greater than zero");
    return false;
  }

  ROS_DEBUG("Critical roughness = %f", criticalValue_);

  if (!FilterBase<T>::getParam(std::string("estimation_radius"), estimationRadius_)) {
    ROS_ERROR("RoughnessFilter did not find param estimation_radius");
    return false;
  }

  if (estimationRadius_ < 0.0) {
    ROS_ERROR("Roughness estimation radius must be greater than zero");
    return false;
  }

  ROS_DEBUG("Roughness estimation radius = %f", estimationRadius_);

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    ROS_ERROR("RoughnessFilter did not find param map_type");
    return false;
  }

  ROS_DEBUG("Roughness map type = %s", type_.c_str());

  return true;
}

template<typename T>
bool RoughnessFilter<T>::update(const T& mapIn, T& mapOut)
{
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);
  double roughnessMax = 0.0;

  for (GridMapIterator iterator(mapOut);
      !iterator.isPastEnd(); ++iterator) {

    // Check if this is an empty cell (hole in the map).
    if (!mapOut.isValid(*iterator, "surface_normal_x")) continue;

    // Prepare data computation.
    const int maxNumberOfCells = ceil(pow(2*estimationRadius_/mapOut.getResolution(),2));
    MatrixXd points(3, maxNumberOfCells);

    // Requested position (center) of circle in map.
    Position center;
    mapOut.getPosition(*iterator, center);

    // Gather surrounding data.
    size_t nPoints = 0;
    for (CircleIterator submapIterator(mapOut, center, estimationRadius_);
        !submapIterator.isPastEnd(); ++submapIterator) {
      if (!mapOut.isValid(*submapIterator, "elevation")) continue;
      Vector3d point;
      mapOut.getPosition3("elevation", *submapIterator, point);
      points.col(nPoints) = point;
      nPoints++;
    }

    const Vector3d mean = points.leftCols(nPoints).rowwise().sum() / nPoints;

    // Compute standard deviation of submap.
    double normalX = mapOut.at("surface_normal_x", *iterator);
    double normalY = mapOut.at("surface_normal_y", *iterator);
    double normalZ = mapOut.at("surface_normal_z", *iterator);
    double planeParameter = mean.x()*normalX + mean.y()*normalY + mean.z()*normalZ;
    double sum = 0.0;
    for (int i = 0; i < nPoints; i++) {
      double dist = normalX*points(0,i) + normalY*points(1,i) + normalZ*points(2,i) - planeParameter;
      sum += pow(dist,2);
    }
    double roughness = sqrt(sum / (nPoints -1));

    if (roughness < criticalValue_) {
      mapOut.at(type_, *iterator) = 1.0 - roughness / criticalValue_;
    }
    else {
      mapOut.at(type_, *iterator) = 0.0;
    }

    if (roughness > roughnessMax) roughnessMax = roughness;
  }

  ROS_DEBUG("roughness max = %f", roughnessMax);

  return true;
}

} /* namespace */

PLUGINLIB_EXPORT_CLASS(filters::RoughnessFilter<grid_map::GridMap>, filters::FilterBase<grid_map::GridMap>)
