#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "cartesian_waypoint_execution/json.hpp"
#include "cartesian_waypoint_execution/trajectory_processing.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

using namespace moveit::core;
using namespace moveit::planning_interface;
// using namespace moveit_msgs::msg;
using namespace geometry_msgs::msg;
using Eigen::Isometry3d;
using Eigen::Vector3d;

using nlohmann::json;

static Point vec_to_point(Vector3d v)
{
  // Converts an Eigen::Vector3d to a geometry_msgs::msg::Point
  Point point;
  point.x = v.x();
  point.y = v.y();
  point.z = v.z();
  return point;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("waypoints", node_options);

  std::string path_to_waypoints = "/home/idlab185/waypoints2.json";
  json j;
  std::ifstream i(path_to_waypoints);
  i >> j;
  json locations = j["waypoints"]["locations"];

  std::vector<Vector3d> target_positions;
  std::vector<double> times;

  for (auto& l : locations)
  {
    double x = l["x"];
    double y = l["y"];
    double z = l["z"];

    target_positions.push_back(Vector3d(x, y, z));
    times.push_back(0.041666);
  }

  // for (auto &t : j["waypoints"]["times"]) {
  //   times.push_back();
  // }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "ur_manipulator";
  MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  PlanningSceneInterface planning_scene_interface;

  RobotStatePtr current_state = move_group.getCurrentState(10);

  const JointModelGroup* joint_model_group = current_state->getJointModelGroup(PLANNING_GROUP);

  MoveGroupInterface::Plan plan;

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // joint_group_positions[0] = -1.0; // radians
  // joint_group_positions[2] = 1.0;  // radians

  move_group.setJointValueTarget(joint_group_positions);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Vector3d text_position(0.0, 0.0, 0.8);
  Isometry3d text_pose = Isometry3d::Identity();
  text_pose.translation() = text_position;
  visual_tools.publishText(text_pose, "Waypoints", rvt::WHITE, rvt::XLARGE);

  // Adding the collision object
  // Now, let's define a collision object ROS message for the robot to avoid.
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 1.5;
  primitive.dimensions[primitive.BOX_Y] = 1.5;
  primitive.dimensions[primitive.BOX_Z] = 0.1;

  // Define a pose for the box (specified relative to frame_id).
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.05;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  RCLCPP_INFO(LOGGER, "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Creating and executing the cartesian path.
  std::vector<Pose> target_poses;

  Pose current_pose = move_group.getCurrentPose().pose;
  visual_tools.publishAxis(current_pose);

  const double pi = 3.14159;
  tf2::Quaternion q;
  q.setRPY(-pi, 0.0, 0.0);

  for (Vector3d target_position : target_positions)
  {
    visual_tools.publishSphere(target_position, rvt::GREEN, rvt::MEDIUM);

    Pose target_pose;
    tf2::convert(q, target_pose.orientation);
    target_pose.position = vec_to_point(target_position);

    target_poses.push_back(target_pose);
  }

  visual_tools.publishAxis(target_poses[0]);

  // target_poses

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  move_group.computeCartesianPath({ target_poses[0] }, eef_step, jump_threshold, trajectory);

  visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
  visual_tools.prompt("NEXT");

  move_group.execute(trajectory);
  visual_tools.prompt("NEXT2");

  double fraction = move_group.computeCartesianPath(target_poses, eef_step, jump_threshold, trajectory);

  // move_group.
  RCLCPP_INFO(LOGGER, "FRACTION");
  RCLCPP_INFO(LOGGER, std::to_string(fraction));

  visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
  visual_tools.publishPath(target_poses, rvt::LIME_GREEN, rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to visualize the retimed trjaectory.");

  robot_trajectory::RobotTrajectory rt(move_group.getRobotModel(), joint_model_group);

  rt.setRobotTrajectoryMsg(*current_state, trajectory);
  trajectory_processing::updateTrajectory(rt, times);
  rt.getRobotTrajectoryMsg(trajectory);
  visual_tools.publishTrajectoryLine(trajectory, joint_model_group);

  move_group.execute(trajectory);

  RCLCPP_INFO(LOGGER, "WAYPOINTS");
  RCLCPP_INFO(LOGGER, std::to_string(rt.getWayPointCount()));

  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}
