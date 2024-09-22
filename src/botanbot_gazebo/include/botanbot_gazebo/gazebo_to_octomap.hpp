/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright 2021 Fetullah Atas, Norwegian University of Life Sciences

 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PLUGIN_BUILD_OCTOMAP_HH
#define PLUGIN_BUILD_OCTOMAP_HH

#include <iostream>
#include <math.h>

#include <gazebo_ros/node.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <octomap/octomap.h>
#include <sdf/sdf.hh>
#include <std_srvs/srv/empty.hpp>
#include <rclcpp/rclcpp.hpp>
#include <botanbot_gazebo/gazebo_to_octomap_common.hpp>
#include <vox_nav_msgs/srv/get_octomap.hpp>

namespace gazebo
{

/// \brief    Octomap plugin for Gazebo.
/// \details  This plugin is dependent on ROS, and is not built if NO_ROS=TRUE is provided to
///           CMakeLists.txt. The PX4/Firmware build does not build this file.
  class OctomapFromGazeboWorld : public WorldPlugin
  {
  public:
    OctomapFromGazeboWorld()
    : WorldPlugin(), octomap_(NULL) {}
    virtual ~OctomapFromGazeboWorld();

  protected:
    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the world that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    bool CheckIfInterest(
      const ignition::math::Vector3d & central_point,
      gazebo::physics::RayShapePtr ray,
      const double leaf_size);

    void FloodFill(
      const ignition::math::Vector3d & seed_point,
      const ignition::math::Vector3d & bounding_box_origin,
      const ignition::math::Vector3d & bounding_box_lengths,
      const double leaf_size);

    /*! \brief Creates octomap by floodfilling freespace.
    *
    * Creates an octomap of the environment in 3 steps:
    *   -# Casts rays along the central X,Y and Z axis of each cell. Marks any
    *     cell where a ray intersects a mesh as occupied
    *   -# Floodfills the area from the top and bottom marking all connected
    *     space that has not been set to occupied as free.
    *   -# Labels all remaining unknown space as occupied.
    *
    * Can give incorrect results in the following situations:
    *   -# The top central cell or bottom central cell are either occupied or
    *     completely enclosed by occupied cells.
    *   -# A completely enclosed hollow space will be marked as occupied.
    *   -# Cells containing a mesh that does not intersect its central axes will
    *     be marked as unoccupied
    */
    void CreateOctomap(const vox_nav_msgs::srv::GetOctomap::Request & msg);

  private:
    physics::WorldPtr world_;
    gazebo_ros::Node::SharedPtr node_;
    rclcpp::Service<vox_nav_msgs::srv::GetOctomap>::SharedPtr srv_;
    octomap::OcTree * octomap_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;

  };

} // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_GAZEBO_OCTOMAP_PLUGIN_H
