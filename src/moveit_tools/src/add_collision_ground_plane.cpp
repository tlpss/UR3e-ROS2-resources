/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Ghent University.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*     Created   : 06/10/2021
 *      Author    : Thomas Lips
 * Executable to add ground plane collision object to Moveit Planning Scene
 */

#include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

rclcpp::Node::SharedPtr node_;

/* CONFIGURATION */
std::string BASE_LINK = "base_link";                   // UR3e base link
std::string PLANNING_SCENE_TOPIC = "/planning_scene";  // MOVEIT SERVO configuration

// set up the node, planning_scene_monitor, and collision object
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.use_intra_process_comms(true);
  node_ = std::make_shared<rclcpp::Node>("temp_ground_plane_collision_node", node_options);

  auto collision_pub_ = node_->create_publisher<moveit_msgs::msg::PlanningScene>(PLANNING_SCENE_TOPIC, 10);

  // Create collision object, in the way of servoing
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = BASE_LINK;
  collision_object.id = "ground_plane";

  shape_msgs::msg::SolidPrimitive ground_plane;
  ground_plane.type = ground_plane.CYLINDER;
  // set dimensions (height, radius) of the cylinder
  // 0.6 is approx workspace size of UR3e
  ground_plane.dimensions = { 0.02, 0.6 };

  // attach to robot base
  geometry_msgs::msg::Pose ground_plane_pose;
  ground_plane_pose.position.x = 0.0;
  ground_plane_pose.position.y = 0.0;
  ground_plane_pose.position.z = 0.0;

  collision_object.primitives.push_back(ground_plane);
  collision_object.primitive_poses.push_back(ground_plane_pose);

  collision_object.operation = collision_object.ADD;

  moveit_msgs::msg::PlanningSceneWorld psw;
  psw.collision_objects.push_back(collision_object);

  auto ps = std::make_unique<moveit_msgs::msg::PlanningScene>();
  ps->world = psw;
  ps->is_diff = true;
  collision_pub_->publish(std::move(ps));

  rclcpp::shutdown();
  return 0;
}
