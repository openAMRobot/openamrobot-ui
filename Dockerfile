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
    git \
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
    ros-${ROS_DISTRO}-xacro \
  && rm -rf /var/lib/apt/lists/*

# Robot Description page: openamrobot_description from openamr-platform-sw,
# pinned to the validated commit recorded in docs/installation.md. Only that
# package is fetched and built, into its own workspace, which the entrypoint
# sources before the UI workspace.
ARG OPENAMR_PLATFORM_SW_REPO=https://github.com/openAMRobot/openamr-platform-sw.git
ARG OPENAMR_PLATFORM_SW_COMMIT=174731c6a593870276fa47d8f92db2ef79eda79a
RUN git init -q /tmp/openamr-platform-sw \
  && cd /tmp/openamr-platform-sw \
  && git fetch -q --depth 1 "${OPENAMR_PLATFORM_SW_REPO}" "${OPENAMR_PLATFORM_SW_COMMIT}" \
  && git checkout -q FETCH_HEAD \
  && test "$(git rev-parse HEAD)" = "${OPENAMR_PLATFORM_SW_COMMIT}" \
  && mkdir -p /opt/openamr_platform_sw/src \
  && cp -r ros2/src/openamrobot_description /opt/openamr_platform_sw/src/ \
  && echo "${OPENAMR_PLATFORM_SW_COMMIT}" > /opt/openamr_platform_sw/PLATFORM_SW_COMMIT \
  && rm -rf /tmp/openamr-platform-sw \
  && cd /opt/openamr_platform_sw \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --packages-select openamrobot_description \
  && rm -rf build log

WORKDIR /workspace
COPY . /workspace

RUN bash scripts/build_frontend.sh
RUN bash scripts/sync_frontend_to_ros.sh
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && bash scripts/build_ros.sh

ENTRYPOINT ["/workspace/scripts/container_entrypoint.sh"]
