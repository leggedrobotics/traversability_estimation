/*
 * RoughnessFilter.cpp
 *
 *  Created on: Mar 13, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/RoughnessFilter.h"
#include <pluginlib/class_list_macros.h>

using namespace Eigen;

namespace filters {

template<typename T>
RoughnessFilter<T>::RoughnessFilter()
    : weight_(0.0),
      roughnessCritical_(0.3),
      traversabilityType_("traversability")
{

}

template<typename T>
RoughnessFilter<T>::~RoughnessFilter()
{

}

template<typename T>
bool RoughnessFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("weight"), weight_)) {
    ROS_ERROR("RoughnessFilter did not find param weight");
    return false;
  }

  if (weight_ > 1.0 || weight_ < 0.0) {
    ROS_ERROR("RoughnessFilter weight must be in the interval [0, 1]");
    return false;
  }

  ROS_INFO("RoughnessFilter weight = %f", weight_);

  if (!FilterBase<T>::getParam(std::string("roughnessCritical"), roughnessCritical_)) {
    ROS_ERROR("RoughnessFilter did not find param roughnessCritical");
    return false;
  }

  if (roughnessCritical_ < 0.0) {
    ROS_ERROR("Critical roughness must be greater than zero");
    return false;
  }

  ROS_INFO("Critical roughness = %f", roughnessCritical_);
  return true;
}

template<typename T>
bool RoughnessFilter<T>::update(const T& elevation_map, T& roughness_map)
{
  roughness_map = elevation_map;
  roughness_map.add("roughness_danger_value", elevation_map.get("elevation"));

  ROS_INFO("Roughness Filter");
  // Hack! has to be replaced by yaml-file
  double surfaceNormalEstimationRadius_ = 0.3;
  double roughnessMax = 0.0;

  std::vector<std::string> clearTypes_;
  clearTypes_.push_back("surface_normal_x");
  clearTypes_.push_back("elevation");
  roughness_map.setClearTypes(clearTypes_);

  for (grid_map_lib::GridMapIterator iterator(roughness_map);
      !iterator.isPassedEnd(); ++iterator) {

    // Clear roughness danger value
    roughness_map.at("roughness_danger_value", *iterator) = NAN;

    // Check if this is an empty cell (hole in the map).
    if (!roughness_map.isValid(*iterator)) continue;

    // Size of submap area for surface normal estimation.
    Array2d submapLength = Array2d::Ones() * (2.0 * surfaceNormalEstimationRadius_);

    // Requested position (center) of submap in map.
    Vector2d submapPosition;
    roughness_map.getPosition(*iterator, submapPosition);
    Array2i submapTopLeftIndex, submapBufferSize, requestedIndexInSubmap;
    grid_map_lib::getSubmapInformation(submapTopLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap, submapPosition, submapLength,
                                       roughness_map.getLength(), roughness_map.getPosition(), roughness_map.getResolution(), roughness_map.getBufferSize(), roughness_map.getBufferStartIndex());

    // Prepare data computation.
    const int maxNumberOfCells = submapBufferSize.prod();
    MatrixXd points(3, maxNumberOfCells);

    // Gather surrounding data.
    size_t nPoints = 0;
    for (grid_map_lib::SubmapIterator submapIterator(roughness_map, submapTopLeftIndex, submapBufferSize); !submapIterator.isPassedEnd(); ++submapIterator) {
      if (!roughness_map.isValid(*submapIterator)) continue;
      Vector3d point;
      roughness_map.getPosition3d("elevation", *submapIterator, point);
      points.col(nPoints) = point;
      nPoints++;
    }

    const Vector3d mean = points.leftCols(nPoints).rowwise().sum() / nPoints;
//    ROS_INFO("mean z = %f",mean.z());

    double normalX_ = roughness_map.at("surface_normal_x", *iterator);
    double normalY_ = roughness_map.at("surface_normal_y", *iterator);
    double normalZ_ = roughness_map.at("surface_normal_z", *iterator);
    double planeParameter = mean.x()*normalX_ + mean.y()*normalY_ + mean.z()*normalZ_;
    double sum = 0.0;
    for (int i = 0; i < nPoints; i++) {
      double dist = normalX_*points(0,i) + normalY_*points(1,i) + normalZ_*points(2,i) - planeParameter;
      sum += pow(dist,2);
    }
    double roughness = sqrt(sum / (nPoints -1));
    if (roughness > roughnessMax) roughnessMax = roughness;
    roughness_map.at("roughness_danger_value", *iterator) = weight_ * roughness / roughnessCritical_;
  }

//  if (!step_map.exists("traversability")) {
//    step_map.add("traversability", step_map.get("step_danger_value"));
//  }
//  else{
//    Eigen::MatrixXf traversabliltyMap_ = step_map.get("traversability");
//    Eigen::MatrixXf stepMap_ = step_map.get("step_danger_value");
////    std::cout << "traversability map =\n" << traversabliltyMap_ << std::endl;
//    traversabliltyMap_ += stepMap_;
////    std::cout << "traversability map =\n" << traversabliltyMap_ << std::endl;
//    step_map.add("traversability", step_map.get("traversability"));
//  }

  ROS_INFO("roughness max = %f",roughnessMax);
  return true;
}
;

} /* namespace */

PLUGINLIB_REGISTER_CLASS(RoughnessFilter, filters::RoughnessFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
