FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ARG ROS_DISTRO=jazzy
ARG TURTLEBOT3_BRANCH=jazzy

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
ENV NAV_WS=/home/${USERNAME}/nav_ws

SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    curl \
    dirmngr \
    git \
    gnupg \
    iproute2 \
    iputils-ping \
    less \
    locales \
    lsb-release \
    nano \
    net-tools \
    openssh-client \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    software-properties-common \
    sudo \
    tzdata \
    wget \
    xauth && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
      gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME}") main" \
      > /etc/apt/sources.list.d/ros2.list

RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    fastdds-tools \
    gz-harmonic \
    ros-dev-tools \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-minimal-tb3-sim \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp && \
    rm -rf /var/lib/apt/lists/*

RUN groupadd --gid "${USER_GID}" "${USERNAME}" && \
    useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" && \
    usermod -aG sudo "${USERNAME}" && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" && \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"

RUN rosdep init

COPY docker/entrypoint.sh /usr/local/bin/navws-entrypoint
COPY docker/portable_bashrc.sh /usr/local/share/navws/portable_bashrc.sh
COPY docker/ros_network.env.example /usr/local/share/navws/ros_network.env.example

RUN chmod +x /usr/local/bin/navws-entrypoint

COPY src /tmp/nav_ws/src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths /tmp/nav_ws/src --ignore-src --rosdistro ${ROS_DISTRO} -r -y && \
    rm -rf /var/lib/apt/lists/*

USER ${USERNAME}
WORKDIR /home/${USERNAME}

RUN mkdir -p /home/${USERNAME}/turtlebot3_ws/src && \
    git clone -b ${TURTLEBOT3_BRANCH} https://github.com/ROBOTIS-GIT/DynamixelSDK.git /home/${USERNAME}/turtlebot3_ws/src/DynamixelSDK && \
    git clone -b ${TURTLEBOT3_BRANCH} https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git /home/${USERNAME}/turtlebot3_ws/src/turtlebot3_msgs && \
    git clone -b ${TURTLEBOT3_BRANCH} https://github.com/ROBOTIS-GIT/turtlebot3.git /home/${USERNAME}/turtlebot3_ws/src/turtlebot3 && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths /home/${USERNAME}/turtlebot3_ws/src --ignore-src --rosdistro ${ROS_DISTRO} -r -y && \
    cd /home/${USERNAME}/turtlebot3_ws && \
    colcon build --symlink-install

RUN printf '\nexport NAV_WS=${NAV_WS:-$HOME/nav_ws}\n[ -f /usr/local/share/navws/portable_bashrc.sh ] && source /usr/local/share/navws/portable_bashrc.sh\n' >> /home/${USERNAME}/.bashrc

WORKDIR ${NAV_WS}
ENTRYPOINT ["/usr/local/bin/navws-entrypoint"]
CMD ["sleep", "infinity"]
