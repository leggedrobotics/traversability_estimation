/*
 * TraversabilityEstimation.hpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Grid Map
#include <grid_map/GridMap.hpp>

// ROS
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

// STD
#include <vector>

// Eigen
#include <Eigen/Core>

namespace traversability_estimation {

  /*!
  * The terrain traversability estimation main class. Coordinates the ROS
  * interfaces, the timing, and the data handling between the other classes.
  */
  class TraversabilityEstimation {

  public:
    /*!
    * Constructor.
    * @param nodeHandle the ROS node handle.
    */
    TraversabilityEstimation(ros::NodeHandle& nodeHandle);

    /*!
    * Destructor.
    */
    virtual ~TraversabilityEstimation();

    /*!
     * Computes the traversability and add it as type to the elevation map.
     * Traversability is set between 0.0 and 1.0, where a value of 0.0 means not
     * traversable and 1.0 means fully traversable.
     * @param[in/out] elevationMap the map for which the traversability is computed.
     */
    void computeTraversability(grid_map::GridMap& elevationMap);

    /*!
     * Publishes the traversability grid map as occupancy grid.
     * @param map the traversability map to publish.
     */
    void publishAsOccupancyGrid(const grid_map::GridMap& map) const;

  private:

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! Elevation map service client.
    ros::ServiceClient submapClient_;

    //! Name of the elevation submap service.
    std::string submapServiceName_;
  
    //! Publisher of the occupancy grid.
    ros::Publisher occupancyGridPublisher_;
  
    //! TF listener.
    tf::TransformListener transformListener_;

    //! Center point of the requested map.
    geometry_msgs::PointStamped submapPoint_;

    //! Id of the frame of the elevation map.
    std::string mapFrameId_;
  
    //! Timer for the map update.
    ros::Timer updateTimer_;
  
    //! Duration between map updates.
    ros::Duration updateDuration_;
    
    //! Requested map cell types.
    std::vector<std::string> requestedMapTypes_;

    //! Traversability map type.
    const std::string traversabilityType_;

    //! Requested map length in [m].
    Eigen::Array2d mapLength_;
  
    /*!
    * Reads and verifies the ROS parameters.
    * @return true if successful.
    */
    bool readParameters();  
    
    /*!
    * Callback function for the update timer. Forces an update of the traversability
    * map from a new elevation map requested from the grid map service.
    * @param timerEvent the timer event.
    */
    void updateTimerCallback(const ros::TimerEvent& timerEvent);

    /*!
     * Gets the grid map for the desired submap center point.
     * @param[out] map the map that is received.
     * @return true if successful, false if ROS service call failed.
     */
    bool getGridMap(grid_map_msg::GridMap& map);
  };

} /* namespace */
