# ur3e_tutorials
A collection of moveit related tools and examples for the UR3e.

## move_group_interface_tutorial
Initial port of the [move_group_interface tutorial](http://moveit2_tutorials.picknik.ai/doc/move_group_interface/move_group_interface_tutorial.html) from the panda robot to the UR3e.
The basic functionalitly works, e.g. moving to a waypoints of to a joint space goal, however the move advanced features like obstacle avoidance are still quite buggy.

Prerequisites:
* `moveit_visual_tools` and `rviz_visual_tools` have to be cloned
* `warehouse_ros_mongo` has to be installed, this can be done by rerunning `rosdep`

To start the tutorial run:

```c
ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3e robot_ip:=10.42.0.162 use_fake_hardware:=true launch_rviz:=false

ros2 launch moveit_tools move_group.launch.py

ros2 launch ur3e_tutorials move_group_interface_tutorial.launch.py
```
