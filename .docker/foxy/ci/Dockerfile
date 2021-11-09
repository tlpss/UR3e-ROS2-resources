ARG ROS_DISTRO=foxy
FROM moveit/moveit2:${ROS_DISTRO}-release

# copy source files
WORKDIR /home/colcon_ws
COPY src src/
COPY ur3e_ros2_drivers.repos .

# install dependencies and build all local packages
# all commands for building / installing are combined in a single command
# cf https://github.com/ros-planning/moveit2/blob/main/.docker/source/Dockerfile
# and bc each RUN command has its own context and hence needs to resource ROS
# apt cache is cleand to reduce image size
RUN vcs import src < ur3e_ros2_drivers.repos && \
apt-get update && \
. /opt/ros/${ROS_DISTRO}/setup.sh && \
# ros2 pkg list && \ # used to check if everything is sourced as expected
rosdep install --from-paths src --ignore-src -r -y && \
rm -rf /var/lib/apt/lists/* && \
colcon build --symlink-install
