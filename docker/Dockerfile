# Faucon Autonomy Stack — Production Image
# Multi-stage build:
#   1. ihm-builder  — compiles the React/Vite web interface
#   2. ros-builder  — builds the full ROS2 workspace with colcon
#   3. runtime      — minimal image with only runtime dependencies

#  Stage 1: IHM (React/Vite) 
FROM node:20-slim AS ihm-builder

WORKDIR /ihm-build

# Install deps before copying source to leverage Docker layer cache
COPY faucon_ihm/ihm/package*.json ./
RUN npm ci --prefer-offline

# Build the React app
COPY faucon_ihm/ihm/ ./
RUN npm run build


#  Stage 2: ROS2 workspace build 
FROM osrf/ros:jazzy-ros-base AS ros-builder

# Install all packages needed at build time (build deps + runtime deps in one
# layer — saves image layers in the builder; runtime stage installs separately)
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    # Simulation / control
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher \
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
    # Python runtime
    python3-pyproj \
    python3-numpy \
    python3-opencv \
    python3-utm \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update --rosdistro jazzy

# Copy the full workspace source
COPY . /workspace/
WORKDIR /workspace

# Install any remaining rosdep-declared deps not covered by the apt list above
RUN . /opt/ros/jazzy/setup.sh \
    && rosdep install --from-paths . --ignore-src -y --rosdistro jazzy 2>/dev/null || true

# Colcon build — Release mode, merged install tree, no symlinks (production safe)
RUN . /opt/ros/jazzy/setup.sh \
    && colcon build \
      --merge-install \
      --install-base /opt/faucon \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
      --parallel-workers "$(nproc)" \
      --event-handlers console_direct+


#  Stage 3: Runtime image 
FROM osrf/ros:jazzy-ros-base AS runtime

LABEL org.opencontainers.image.title="Faucon Autonomy Stack" \
      org.opencontainers.image.vendor="UMOJA Robotics" \
      org.opencontainers.image.description="Hardware-agnostic autonomous robotics stack — ROS2 Jazzy" \
      org.opencontainers.image.source="https://github.com/umoja-robotics/faucon"

# Runtime dependencies only — no build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Simulation / control
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-joint-state-publisher \
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
    # Python runtime
    python3-pyproj \
    python3-numpy \
    python3-opencv \
    python3-utm \
    && rm -rf /var/lib/apt/lists/*

# Copy the built ROS2 workspace
COPY --from=ros-builder /opt/faucon /opt/faucon

# Copy the built IHM static files into the package share directory
# (mirrors what CMakeLists.txt would install from ihm/dist/ → share/faucon_ihm/ihm/)
COPY --from=ihm-builder /ihm-build/dist /opt/faucon/share/faucon_ihm/ihm

# Runtime entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Ports:
#   9090 — rosbridge WebSocket
#   8080 — web_video_server (camera streams)
#   3000 — IHM HTTP server
EXPOSE 9090 8080 3000

ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/entrypoint.sh"]

# Default: IHM only (headless, for real hardware deployment)
# Override with: docker run ... ros2 launch faucon_base_desc view.launch.py use_sim_time:=true
CMD ["ros2", "launch", "faucon_ihm", "ihm_launch.py"]
