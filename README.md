# Traversability Estimation

## Overview

Traversability mapping for mobile rough terrain navigation.

The Traversability Estimation package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE).

**Authors: Peter Fankhauser, Martin Wermelinger, Ralf Kaestner  
Contact: Peter Fankhauser, pfankhauser@ethz.ch  
Affiliation: Autonomous Systems Lab, ETH Zurich**


## Installation

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library),
- [kindr](http://github.com/ethz-asl/kindr) (kinematics and dynamics library for robotics),
- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing),
- [Grid Map](https://github.com/ethz-asl/grid_map) (grid map library for mobile robots),
- [Elevation Map](https://github.com/ethz-asl/elevation_mapping) (elevation mapping with a mobile robot),
- [starleth_ros_common](http://bitbucket.org/ethz-asl-lr/c_starleth_ros_common) (common [ROS] packages for StarlETH robot).
- [Gazebo](http://gazebosim.org/) (robot simulator)

		sudo apt-get install ros-indigo-gazebo-ros-pkgs

	More information on the installation of [Gazebo] for [ROS] is given [here](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros),
- [Schweizer-Messer](http://github.com/ethz-asl/Schweizer-Messer) (programming tools for robotics),
	
		sudo add-apt-repository ppa:ethz-asl/common
		sudo apt-get update
		sudo apt-get install schweizer-messer-common-dev schweizer-messer-timing-dev


### Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_ws/src
	git clone https://github.com/ethz-asl/traversability_estimation.git
	cd ../
	catkin_make


### Unit Tests

No unit tests so far.


## Usage

In order to get the Traversability estimation to run with your robot, you will need to adapt a few parameters in the config-file. It is the easiest if you duplicate the config-file `starleth.yaml` in `traversability_estimation/config` and adapt all the parameters you need to change. Then, duplicate the launch-file `traversability_estimation/launch/starleth.launch` and change the entries to point at your config-file. You can then launch the traversability map node with

	roslaunch traversability_estimation your_launch_file.launch

Use rviz to visualize the traversability map.


## Nodes

### traversability_estimation

This is the main Traversability Estimation node. It uses the elevation map and the surface normals to generate a traversability map.


#### Subscribed Topics

* 


#### Published Topics

* **`slope_map`** ([nav_msgs/OccupancyGrid])

	The traversability map only considering the slope. This topic is only published if the slope filter is activated.

* **`step_map`** ([nav_msgs/OccupancyGrid])

	The traversability map only considering the step heights. This topic is only published if the step filter is activated.

* **`roughness_map`** ([nav_msgs/OccupancyGrid])

	The traversability map only considering the roughness of the surface. This topic is only published if the roughness filter is activated.

* **`traversability_map`** ([nav_msgs/OccupancyGrid])

	The resulting traversability map. The traversability map can be defined with a filter chain and should consist at least of one of the above mentioned topics.


#### Services

* 


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


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[Gazebo]: http://gazebosim.org/
[rviz]: http://wiki.ros.org/rviz
[starleth_msgs/SeActuatorCommands]: https://bitbucket.org/ethz-asl-lr/c_starleth_ros_common/raw/master/starleth_msgs/msg/SeActuatorCommands.msg
[grid_map_msg/GridMap]: https://github.com/ethz-asl/grid_map/blob/master/grid_map_msg/msg/GridMap.msg
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[visualization_msgs/Marker]: http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html
[nav_msgs/OccupancyGrid]: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
