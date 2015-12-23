/*
 * SurfaceNormalsFilter.cpp
 *
 *  Created on: May 05, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/SurfaceNormalsFilter.hpp"
#include <pluginlib/class_list_macros.h>

#include <vector>

// Eigenvalues
#include <Eigen/Core>
#include <Eigen/Dense>

// Grid Map
#include <grid_map/grid_map.hpp>

using namespace grid_map;

namespace filters {

template<typename T>
SurfaceNormalsFilter<T>::SurfaceNormalsFilter()
    : estimationRadius_(0.05)
{

}

template<typename T>
SurfaceNormalsFilter<T>::~SurfaceNormalsFilter()
{

}

template<typename T>
bool SurfaceNormalsFilter<T>::configure()
{
  if (!FilterBase<T>::getParam(std::string("estimation_radius"), estimationRadius_)) {
    ROS_ERROR("Surface normals filter did not find param estimation_radius");
    return false;
  }

  if (estimationRadius_ < 0.0) {
    ROS_ERROR("Surface normals filter estimation radius must be greater than zero");
    return false;
  }

  ROS_DEBUG("Surface normals estimation radius = %f", estimationRadius_);

  std::string surfaceNormalPositiveAxis;
  if (!FilterBase<T>::getParam(std::string("surface_normal_positive_axis"), surfaceNormalPositiveAxis)) {
    ROS_ERROR("Surface normals filter did not find param surface_normal_positive_axis");
    return false;
  }
  if (surfaceNormalPositiveAxis == "z") {
    surfaceNormalPositiveAxis_ = Vector3::UnitZ();
  } else if (surfaceNormalPositiveAxis == "y") {
    surfaceNormalPositiveAxis_ = Vector3::UnitY();
  } else if (surfaceNormalPositiveAxis == "x") {
    surfaceNormalPositiveAxis_ = Vector3::UnitX();
  } else {
    ROS_ERROR("The surface normal positive axis '%s' is not valid.", surfaceNormalPositiveAxis.c_str());
  }

  return true;
}

template<typename T>
bool SurfaceNormalsFilter<T>::update(const T& mapIn, T& mapOut)
{
  mapOut = mapIn;
  mapOut.add("surface_normal_x");
  mapOut.add("surface_normal_y");
  mapOut.add("surface_normal_z");

  std::vector<std::string> surfaceNormalTypes;
  surfaceNormalTypes.push_back("surface_normal_x");
  surfaceNormalTypes.push_back("surface_normal_y");
  surfaceNormalTypes.push_back("surface_normal_z");

  // For each cell in requested area.
  for (GridMapIterator iterator(mapOut);
      !iterator.isPastEnd(); ++iterator) {
    // Check if this is an empty cell (hole in the map).
    if (!mapOut.isValid(*iterator, "elevation")) continue;
    // Check if surface normal for this cell has already been computed earlier.
    if (mapOut.isValid(*iterator, surfaceNormalTypes)) continue;

    // Requested position (center) of circle in map.
    Position center;
    mapOut.getPosition(*iterator, center);

    // Prepare data computation.
    const int maxNumberOfCells = ceil(pow(2*estimationRadius_/mapOut.getResolution(),2));
    Eigen::MatrixXd points(3, maxNumberOfCells);

    // Gather surrounding data.
    size_t nPoints = 0;
    for (CircleIterator submapIterator(mapOut, center, estimationRadius_);
        !submapIterator.isPastEnd(); ++submapIterator) {
      if (!mapOut.isValid(*submapIterator, "elevation")) continue;
      Position3 point;
      mapOut.getPosition3("elevation", *submapIterator, point);
      points.col(nPoints) = point;
      nPoints++;
    }
    points.conservativeResize(3, nPoints); // TODO Eigen version?

    // Compute eigenvectors.
    const Position3 mean = points.leftCols(nPoints).rowwise().sum() / nPoints;
    const Eigen::MatrixXd NN = points.leftCols(nPoints).colwise() - mean;

    const Eigen::Matrix3d covarianceMatrix(NN * NN.transpose());
    Vector3 eigenvalues = Vector3::Ones();
    Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
    // Ensure that the matrix is suited for eigenvalues calculation
    if (covarianceMatrix.fullPivHouseholderQr().rank() >= 3) {
      const Eigen::EigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
      eigenvalues = solver.eigenvalues().real();
      eigenvectors = solver.eigenvectors().real();
    } else {
      ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated. Expected cause: no noise in data (nPoints = %i)", (int) nPoints);
      // Use z-axis as default surface normal.
      eigenvalues.z() = 0.0;
    }
    // Keep the smallest eigenvector as surface normal
    int smallestId(0);
    double smallestValue(std::numeric_limits<double>::max());
    for (int j = 0; j < eigenvectors.cols(); j++) {
      if (eigenvalues(j) < smallestValue) {
        smallestId = j;
        smallestValue = eigenvalues(j);
      }
    }
    Vector3 eigenvector = eigenvectors.col(smallestId);
    if (eigenvector.dot(surfaceNormalPositiveAxis_) < 0.0) eigenvector = -eigenvector;
    mapOut.at("surface_normal_x", *iterator) = eigenvector.x();
    mapOut.at("surface_normal_y", *iterator) = eigenvector.y();
    mapOut.at("surface_normal_z", *iterator) = eigenvector.z();
  }

  return true;
}

} /* namespace */

PLUGINLIB_REGISTER_CLASS(SurfaceNormalsFilter, filters::SurfaceNormalsFilter<grid_map::GridMap>,
                         filters::FilterBase<grid_map::GridMap>)
