# UR3e-ROS2-resources
Repository containing code for controlling the UR3e @ AIRO Ghent University using ROS2 and Moveit2


## Installation of the repository and its dependencies

All steps assume ubuntu 20.04 as OS, since this is the default for ROS2-foxy.

- install [ROS2 foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- install ROS2's package mananger colcon: `sudo apt install python3-colcon-common-extensions'
- install Moveit2 for ROS2 foxy using `sudo apt install ros-foxy-moveit`
- install vcs-tools using `sudo apt-get install python3-vcstool`
- make sure that you are in the root folder of this repository
- pull the official UR3e driver repositories and its dependencies using `vcs import src < ur3e_ROS2_drivers.repos`
- fetch all ROS dependencies using `rosdep install --from-paths src --ignore-src -r -y`
- build the workspace using `colcon build --symlink-install` and make sure that there are no build errors


## connect to UR3e 
** always make sure that you have the big red button within reach. **
- turn on the UR controller (green button on the switch)
- connect to the controller using the ethernet cable
- create a network profile (name e.g. UR3e) and select under IPv4: "shared with other computers". 
- activate the network profile using `nmcli connection up <profile_name>`. 
- now turn the URCap into manual mode (third icon in the top right corner) and fill in the password. 
- now check on URCap in settings>system>network if the UR3 is connected. Its IP address should be 10.42.0.162 and your host IP should be 10.42.0.1
- load the "ROS-external-control-program" on the UR3e and start up the robot (button in the left lower corner)

## spin up ROS controllers and joint publishers for the UR3e
This spins up nodes for receiving joint information and controlling the joint actuators and connects the UR3e to the ROS nodes on your pc. 
It relies on the UR3e driver package from Universal Robotics, which can be found [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/foxy)

- move to the root folder of the worskpace
- build the workspace using `colcon build`
- source the packages in your terminal using `source install/setup.bash`
- bringup the robot using `ros2 launch ur_bringup ur_control.launch.py ur_type:=ur3e robot_ip:=10.42.0.162 use_fake_hardware:=false launch_rviz:=true`
- verify if the joint states are correctly published by comparing the pose in RVIZ with the pose of the real robot
- to allow for control by the ROS controllers, start the remote control program on URCap by pressing the play icon
- you should now see this message in your terminal: `Robot connected to reverse interface. Ready to receive control commands.`


## Motion Planning with Moveit2
This starts up the Moveit planner for trajectory planning and connects it with the joint information publishers and controllers of the UR3e. More information on Moveit: [website]() and [official tutorials](http://moveit2_tutorials.picknik.ai/index.html)
### Launch the Moveit planner using RVIZ
- bringup the robot using the steps above (optionally set `rviz:=false`)
- spin the moveit planner nodes using `ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur3e robot_ip:=10.42.0.162  use_fake_hardware:=false launch_rviz:=true`
- you should now see the planner for moveit and can start planning trajectories in RVIZ for the UR3e. 
- To configure the planner beyond the default settings, see the Moveit tutorials.

### Connect to trajectory API through python
- Moveit2 python movegroup interface not yet ported, TODO -> search for minimal effort to send trajectory goals from python.

## Teleop Servoing
This starts up nodes to teleoperate the UR3e with a game controller in realtime using moveit servo with custom configuration for the UR3e.

### Spinning up the nodes
- bringup the robot using the steps above (optionally set `rviz:=false`)
- set the speed at 20% in URCap (to avoid jerky motion)
- connect a Logitech F310 (or similar) controller to the laptop
- launch the Joy and Servo nodes using `ros2 launch teleop servo_teleop.launch.py`

### attaching collision objects to the scene
- to attach a groundplane to the UR3e, run `ros2 run moveit_tools add_collision_ground_plane`

