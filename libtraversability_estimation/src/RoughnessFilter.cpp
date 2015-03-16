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

  if (!FilterBase<T>::getParam(std::string("roughnessEstimationRadius"), roughnessEstimationRadius_)) {
    ROS_ERROR("RoughnessFilter did not find param roughnessEstimationRadius");
    return false;
  }

  if (roughnessEstimationRadius_ < 0.0) {
    ROS_ERROR("Roughness estimation radius must be greater than zero");
    return false;
  }

  ROS_INFO("Roughness estimation = %f", roughnessEstimationRadius_);

  return true;
}

template<typename T>
bool RoughnessFilter<T>::update(const T& elevation_map, T& roughness_map)
{
  roughness_map = elevation_map;
  roughness_map.add("roughness_danger_value", elevation_map.get("elevation"));

  double roughnessMax = 0.0;

  std::vector<std::string> clearTypes, validTypes;
  validTypes.push_back("surface_normal_x");
  validTypes.push_back("elevation");
  clearTypes.push_back("roughness_danger_value");
  roughness_map.setClearTypes(clearTypes);
  roughness_map.clear();

  for (grid_map_lib::GridMapIterator iterator(roughness_map);
      !iterator.isPassedEnd(); ++iterator) {

    // Check if this is an empty cell (hole in the map).
    if (!roughness_map.isValid(*iterator, validTypes)) continue;

    // Size of submap area for surface normal estimation.
    Array2d submapLength = Array2d::Ones() * (2.0 * roughnessEstimationRadius_);

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
      if (!roughness_map.isValid(*submapIterator, validTypes)) continue;
      Vector3d point;
      roughness_map.getPosition3d("elevation", *submapIterator, point);
      points.col(nPoints) = point;
      nPoints++;
    }

    const Vector3d mean = points.leftCols(nPoints).rowwise().sum() / nPoints;
//    ROS_INFO("mean z = %f",mean.z());

    double normalX = roughness_map.at("surface_normal_x", *iterator);
    double normalY = roughness_map.at("surface_normal_y", *iterator);
    double normalZ = roughness_map.at("surface_normal_z", *iterator);
    double planeParameter = mean.x()*normalX + mean.y()*normalY + mean.z()*normalZ;
    double sum = 0.0;
    for (int i = 0; i < nPoints; i++) {
      double dist = normalX*points(0,i) + normalY*points(1,i) + normalZ*points(2,i) - planeParameter;
      sum += pow(dist,2);
    }
    double roughness = sqrt(sum / (nPoints -1));

    if (roughness < roughnessCritical_) {
      roughness_map.at("roughness_danger_value", *iterator) = weight_ * roughness / roughnessCritical_;
    }

    if (roughness > roughnessMax) roughnessMax = roughness;
  }

  ROS_INFO("roughness max = %f",roughnessMax);

  // Add danger value to traversability map
  if (!roughness_map.exists(traversabilityType_)) {
    roughness_map.add(traversabilityType_, roughness_map.get("roughness_danger_value"));
  }
  else{
    Eigen::MatrixXf traversabliltyMap = roughness_map.get(traversabilityType_);
    Eigen::MatrixXf roughnessMap = roughness_map.get("roughness_danger_value");
    traversabliltyMap += roughnessMap;
    roughness_map.add(traversabilityType_, traversabliltyMap);
  }

  return true;
}
;

} /* namespace */

PLUGINLIB_REGISTER_CLASS(RoughnessFilter, filters::RoughnessFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
