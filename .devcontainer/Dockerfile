# Faucon Development image
FROM osrf/ros:jazzy-desktop-full

 
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Simulation / visualization
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-twist-mux \
    # Navigation
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    # Localisation
    ros-jazzy-robot-localization \
    # Perception
    ros-jazzy-pcl-ros \
    ros-jazzy-cv-bridge \
    ros-jazzy-rtabmap-ros \
    # Web interface
    ros-jazzy-rosbridge-suite \
    ros-jazzy-web-video-server \
    # Build tools
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    # Python runtime deps
    python3-pyproj \
    python3-numpy \
    python3-opencv \
    python3-utm \
    # Dev utilities
    git \
    curl \
    wget \
    vim \
    htop \
    && rm -rf /var/lib/apt/lists/*

#  Node.js 20 (IHM React/Vite dev server) 
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

#  rosdep / colcon bootstrap 
RUN rosdep update --rosdistro jazzy

RUN colcon mixin update default 2>/dev/null || true

#  Non-root dev user (matches VSCode devcontainer convention) 
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN \
    # GID may already exist in the base image (Ubuntu 24.04 ships GID 1000)
    EXISTING_GRP=$(getent group ${USER_GID} | cut -d: -f1 2>/dev/null) \
    && if [ -z "$EXISTING_GRP" ]; then \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    elif [ "$EXISTING_GRP" != "${USERNAME}" ]; then \
        groupmod --new-name ${USERNAME} "$EXISTING_GRP"; \
    fi \
    # UID may already exist in the base image
    && EXISTING_USR=$(getent passwd ${USER_UID} | cut -d: -f1 2>/dev/null) \
    && if [ -z "$EXISTING_USR" ]; then \
        useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME}; \
    elif [ "$EXISTING_USR" != "${USERNAME}" ]; then \
        usermod -l ${USERNAME} -s /bin/bash -g ${USER_GID} "$EXISTING_USR" \
        && usermod -d /home/${USERNAME} -m ${USERNAME} 2>/dev/null || true; \
    fi \
    && apt-get update \
    && apt-get install -y --no-install-recommends sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

#  Shell environment 
USER ${USERNAME}

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ~/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

WORKDIR /workspace
