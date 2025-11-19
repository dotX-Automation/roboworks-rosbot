ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
  build-essential \
  git \
  iproute2 \
  iputils-ping \
  less \
  net-tools \
  python3-colcon-common-extensions \
  python3-pip \
  vim \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
  joystick \
  libgl1-mesa-glx \
  libglu1-mesa \
  ros-${ROS_DISTRO}-joy \
  ros-${ROS_DISTRO}-rqt \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  ros-${ROS_DISTRO}-teleop-twist-joy \
  ros-${ROS_DISTRO}-rmw-zenoh-cpp \
  && rm -rf /var/lib/apt/lists/*

ENV LIBGL_ALWAYS_SOFTWARE=1

ENV ROS_DISTRO=${ROS_DISTRO}
ENV ROS_WS=/roboworks-rosbot
WORKDIR ${ROS_WS}

COPY docker/aliases.sh /etc/profile.d/aliases.sh
RUN chmod +x /etc/profile.d/aliases.sh

COPY docker/prompt.sh /etc/profile.d/prompt.sh
RUN chmod +x /etc/profile.d/prompt.sh

ENV QT_X11_NO_MITSHM=1

RUN echo 'if [ -f /etc/profile.d/aliases.sh ]; then . /etc/profile.d/aliases.sh; fi' >> /root/.bashrc \
  && echo 'if [ -f /etc/profile.d/prompt.sh ]; then . /etc/profile.d/prompt.sh; fi' >> /root/.bashrc \
  && echo 'if [ -f /opt/ros/'${ROS_DISTRO}'/setup.bash ]; then . /opt/ros/'${ROS_DISTRO}'/setup.bash; fi' >> /root/.bashrc \
  && echo 'if [ -f '${ROS_WS}'/install/setup.bash ]; then . '${ROS_WS}'/install/setup.bash; fi' >> /root/.bashrc

CMD ["bash"]
