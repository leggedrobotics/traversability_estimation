# Traversability Estimation

## Overview

Traversability mapping for mobile rough terrain navigation.

The Traversability Estimation package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Authors: Martin Wermelinger, Peter Fankhauser, Ralf Kaestner  
Contact: Martin Wermelinger, martiwer@mavt.ethz.ch  
Affiliation: Autonomous Systems Lab, ETH Zurich**

[![Build Status](https://ci.leggedrobotics.com/buildStatus/icon?job=github_ethz-asl/traversability_estimation/master)](https://ci.leggedrobotics.com/job/github_ethz-asl/job/traversability_estimation/job/master/)

## Installation

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionaly, the Traversability Estimation depends on following software:

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library),
- [kindr](http://github.com/ethz-asl/kindr) (kinematics and dynamics library for robotics),
- [Grid Map](https://github.com/ethz-asl/grid_map) (grid map library for mobile robots),
- [Elevation Map](https://github.com/ethz-asl/elevation_mapping) (elevation mapping with a mobile robot),

### Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	git clone https://github.com/ethz-asl/traversability_estimation.git
	cd ../
	catkin build traversability_estimation


### Unit Tests

No unit tests so far.


## Basic Usage

In order to get the Traversability estimation to run with your robot, you will need to adapt a few parameters in the config-files. It is the easiest if you duplicate the file `robot.yaml`, `robot_footprint.yaml`, and `filter_parameter.yaml` in [`traversability_estimation/config`](https://github.com/ethz-asl/traversability_estimation/tree/master/traversability_estimation/config) and adapt all the parameters you need to change. Then, duplicate the launch-file `traversability_estimation/launch/traversability_estimation.launch` and change the entries to point at your files. You can then launch the traversability map node with

	roslaunch traversability_estimation traversability_estimation.launch

Proceed in the same way for the traversability map visualization by adapting the launch-file `traversability_estimation/launch/visualization.launch`. You can then launch the traversability map visualization node with

     	roslaunch traversability_estimation visualization.launch

Use [rviz](http://wiki.ros.org/rviz) to visualize the traversability map.


## Nodes

### traversability_estimation

This is the main Traversability Estimation node. It uses the elevation map and the traversability estimation filters to generate a traversability map.


#### Subscribed Topics

* **`/image_elevation`** ([sensor_msgs/Image])

    	It is possible to subscribe to an image. The image is converted into a grayscale image and the values are mapped into an elevation map.


#### Published Topics

* **`traversability_map`** ([grid_map_msgs/GridMap])

	The current traversability map. The traversability map can be configured with the traversability filters.


#### Services

* **`load_elevation_map`** ([std_srvs/Empty])

    Trigger the loading of an elevation map from a [rosbag] file. Trigger the loading of the map with

        rosservice call /traversability_estimation/load_elevation_map

* **`update_traversability`** ([grid_map_msgs/GetGridMapInfo])

    Forces an update of the traversability map, using the current elevation map. The update can be triggered with

        rosservice call /traversability_estimation/update_traversability

* **`get_traversability`** ([grid_map_msgs/GetGridMap])

    Request the current traversability map or a submap of it. For example, you can get the traversability submap at position (-1.0, 0.0) and size (2.5, 2.0) and safe it to a text file form the console with

        rosservice call -- /traversability_estimation/get_traversability -1.0 0.0 2.5 2.0 []

* **`check_footprint_path`** ([traversability_msgs/CheckFootprintPath])

    This service is used to check the traversability of a single footprint or a path of several footprints. The current traversability map is used to evaluate the footprints.

* **`update_parameters`** ([std_srvs/Empty])

    Use this service to update the parameters of the traversability estimation filters. It reloads the parameter file and sets the new parameters. Trigger the parameter update with

        rosservice call /traversability_estimation/update_parameters

* **`traversability_footprint`** ([std_srvs/Empty])

    Computes the traversability of a circular footprint at each map position and stores the values in an additional map layer. This service is intended for visualizing the traversability and debugging. Trigger the traversability computation with

        rosservice call /traversability_estimation/traversability_footprint

* **`save_to_bag`** ([std_srvs/Empty])

    Save all layers of the current traversability map to a [rosbag] file. Save the traversability map with

        rosservice call /traversability_estimation/save_to_bag

#### Parameters

* **`submap_service`** (string, default: "/elevation_mapping/get_grid_map")

	The name of the service to get the elevation submap.
	
* **`robot_frame_id`** (string, default: "base")
	
	The id of the robot tf frame.

* **`map_frame_id`** (string, default: "map")
	
	The id of the tf frame of the traversability map.

* **`update_rate`** (double, default: 4.0)
	
	The update rate (in \[Hz\]) at which the traversability map is updated.

* **`map_center_x`, `map_center_y`** (double, default: 1.5, 0.0)
	
	The position of the traversability map (center) in the traversability map frame.

* **`map_length_x`, `map_length_y`** (double, default: 5.0)
	
	The size (in \[m\]) of the traversability map.

* **`traversability_map_filters:`** (filter_chain)
	
	Defines the different filters that are used to generate the traversability map.

### Traversability Estimation Filters

The traversability estimation filters can be applied to an elevation map. Each filter adds an additional layer to the elevation map and computes a value for every cell of the map.

* *[Surface Normals Filter:](traversability_estimation_filters/src/SurfaceNormalsFilter.cpp)* Computes the surface normal of each cell of an elevation map. Each component of the surface normal is saved separatly.

* *[Slope Filter:](traversability_estimation_filters/src/SlopeFilter.cpp)* Computes the slope traversability value based on an elevation map.

* *[Roughness Filter:](traversability_estimation_filters/src/RoughnessFilter.cpp)* Computes the step traversability value based on an elevation map.

* *[Step Filter:](traversability_estimation_filters/src/StepFilter.cpp)* Compute the roughness traversability value based on an elevation map.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).
