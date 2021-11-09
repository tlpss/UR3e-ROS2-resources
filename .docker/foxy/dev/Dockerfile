# Dockerfile for use with Vscode remote containers

ARG ROS_DISTRO=foxy
FROM moveit/moveit2:${ROS_DISTRO}-release


RUN \
    # Update apt package list as previous containers clear the cache
    apt-get -q update && \
    apt-get -q -y dist-upgrade && \
    #
    # Install some base dependencies
    apt-get -q install --no-install-recommends -y \
        # Some basic requirements
        sudo curl \
        # Preferred build tools
        clang clang-format-10 clang-tidy clang-tools && \
    rm -rf /var/lib/apt/lists/*


# install CI for ROS
RUN  apt-get update && \
apt-get install -y python3-pip && \
rm -rf /var/lib/apt/lists/* && \
pip install pre-commit
