ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash \
    ca-certificates \
    curl \
    nodejs \
    npm \
    python3-colcon-common-extensions \
    python3-flask \
    python3-serial \
    python3-yaml \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-simple-commander \
    ros-${ROS_DISTRO}-rosapi \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-web-video-server \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY . /workspace

RUN bash scripts/build_frontend.sh
RUN bash scripts/sync_frontend_to_ros.sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && bash scripts/build_ros.sh

ENTRYPOINT ["/workspace/scripts/container_entrypoint.sh"]
