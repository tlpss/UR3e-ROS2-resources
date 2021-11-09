#include <moveit/robot_trajectory/robot_trajectory.h>

namespace trajectory_processing
{
void updateTrajectory(robot_trajectory::RobotTrajectory& rob_trajectory, const std::vector<double>& time_diff);
}  // namespace trajectory_processing
