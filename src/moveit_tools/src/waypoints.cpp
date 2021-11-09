#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

using namespace moveit::core;
using namespace moveit::planning_interface;
using namespace moveit_msgs::msg;
using namespace geometry_msgs::msg;
using Eigen::Isometry3d;
using Eigen::Vector3d;

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
  auto move_group_node = rclcpp::Node::make_shared("waypoint", node_options);

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

  // Now, let's modify one of the joints, plan to the new joint space goal and
  // visualize the plan.
  joint_group_positions[0] = -1.0;  // radians
  joint_group_positions[2] = 1.0;   // radians

  move_group.setJointValueTarget(joint_group_positions);

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "base_link", "move_group_tutorial",
                                                      move_group.getRobotModel());

  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();

  Vector3d text_position(0.0, 0.0, 0.8);
  Isometry3d text_pose = Isometry3d::Identity();
  text_pose.translation() = text_position;
  visual_tools.publishText(text_pose, "Waypoint", rvt::WHITE, rvt::XLARGE);

  //   std::vector<Vector3d> target_positions{Vector3d(-0.3, 0.3, 0.1),
  //                                          Vector3d(0.0, 0.3, 0.3),
  //                                          Vector3d(0.3, 0.3, 0.1)};

  std::vector<Vector3d> target_positions;

  double radius = 0.3;
  double pi = 3.14159265358979323846;
  int n_steps = 100;
  for (double theta = 0.0; theta <= pi; theta += pi / n_steps)
  {
    double x = std::cos(theta) * radius;
    double y = 0.3;
    double z = std::sin(theta) * radius;
    target_positions.push_back(Vector3d(x, y, z));
  }

  std::vector<Pose> target_poses;

  Pose current_pose = move_group.getCurrentPose().pose;

  for (Vector3d target_position : target_positions)
  {
    visual_tools.publishSphere(target_position, rvt::GREEN, rvt::MEDIUM);
    Pose target_pose = current_pose;
    target_pose.position = vec_to_point(target_position);
    target_poses.push_back(target_pose);
  }

  RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group.computeCartesianPath(target_poses, eef_step, jump_threshold, trajectory);

  visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
  visual_tools.publishPath(target_poses, rvt::LIME_GREEN, rvt::SMALL);
  //   move_group.execute(trajectory);

  //   move_group.setPoseTarget(target_pose0);
  //   bool success = (move_group.plan(plan) == MoveItErrorCode::SUCCESS);
  //   visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  rclcpp::shutdown();
  return 0;
}
