FROM ros:humble

# build-time arguments
ARG ROS_WS=/ros2_ws

# install dependencies and clear APT's package index cache
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    libpcl-dev
RUN rm -rf /var/lib/apt/lists/*

# create workspace
RUN mkdir -p "${ROS_WS}/src/"
COPY src/radix_ros2_pkg  "${ROS_WS}/src/radix_ros2_pkg"
COPY src/radix_msgs_ros2_pkg "${ROS_WS}/src/radix_msgs_ros2_pkg"

# build project
WORKDIR ${ROS_WS}
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select radix_ros radix_msgs --symlink-install

# entrypoint to source ROS and workspace then run command
ENTRYPOINT ["/bin/bash", "-c", "\
  source /opt/ros/humble/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash && \
  exec \"$0\" \"$@\" \
"]
# default: run semantic behavior
CMD ["ros2", "launch", "radix_ros", "radix_semantic.launch.py"]
