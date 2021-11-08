ARG ROS_DISTRO=foxy
FROM moveit/moveit2:${ROS_DISTRO}-release

# set default shell to bash
SHELL ["/bin/bash", "-c"] 


# install CI for Moveit
RUN apt-get update && apt-get install -y clang-format-10 && \ 
apt-get install -y python3-pip && \ 
rm -rf /var/lib/apt/lists/* && \
pip install pre-commit

 WORKDIR /home/colcon_ws
COPY src src/
COPY ur3e_ros2_drivers.repos .

# install dependencies and build all local packages
RUN source /opt/ros/${ROS_DISTRO}/setup.bash 
RUN vcs import src < ur3e_ros2_drivers.repos
RUN apt-get update && \
#apt-get install ros-foxy-graph-msgs && \ 
rosdep install --from-paths src --ignore-src -r -y && \
rm -rf /var/lib/apt/lists/*

RUN colcon build --symlink-install