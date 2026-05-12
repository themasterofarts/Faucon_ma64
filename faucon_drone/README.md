# faucon_drone

Drone integration for the **Faucon Autonomy Stack** (ROS 2 Jazzy / Gazebo Harmonic).

Provides a PX4-powered X500 quadrotor that shares the same Gazebo world as the UGV,
with a clean hardware-agnostic interface that hides all PX4 internals from the rest
of the stack.

---

## Architecture

```
Gazebo (virtual_maize_field)
    x500_mono_cam model  ←→  GZ transport (IMU, GPS, Magnetometer, Baro, Camera)
          ↕
    PX4 SITL  (standalone mode — connects to running Gazebo)
          ↕  XRCE-DDS / UDP 8888
    MicroXRCEAgent
          ↕  ROS 2 DDS  (/fmu/*)
    PX4X500Adapter
          ↕
    /faucon/drone/*  (Faucon stable interface)
```

**Key design choice**: PX4 runs in `PX4_GZ_STANDALONE=1` mode — it connects to the
already-running UGV Gazebo instance instead of launching its own. Both agents share
a single Gazebo world.

---

## Faucon Interface

All topics are under `/faucon/drone/`. Nothing above this node sees PX4-specific
message types, frame conventions (NED), or internal topics.

### Published

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/faucon/drone/odom` | `nav_msgs/Odometry` | 50 Hz | Position + velocity, ENU frame |
| `/faucon/drone/gps` | `sensor_msgs/NavSatFix` | ~5 Hz | WGS-84 GPS fix |
| `/faucon/drone/imu` | `sensor_msgs/Imu` | ~250 Hz | Attitude + rates, ENU body frame |
| `/faucon/drone/state` | `std_msgs/String` | ~5 Hz | `DISARMED` / `ARMED` / `OFFBOARD` / … |
| `/faucon/drone/camera/image` | `sensor_msgs/Image` | 10-30 Hz | RGB camera, model-dependent orientation |
| `/faucon/drone/trajectory/status` | `std_msgs/String` | 20 Hz | Current trajectory phase: `WAITING` / `PREARM` / `TAKEOFF` / `WP_N/TOTAL` / `HOVER` |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/faucon/drone/arm` | `std_msgs/Bool` | `true` = arm + offboard mode, `false` = disarm |
| `/faucon/drone/cmd_vel` | `geometry_msgs/Twist` | Body-ENU velocity (forward/left/up + yaw rate) |

### Frame conventions

| Field | Convention |
|-------|-----------|
| Position / velocity | Local ENU (East-North-Up) |
| Body velocity (`cmd_vel`) | `linear.x` = forward, `linear.y` = left, `linear.z` = up |
| Yaw rate | CCW positive (standard ROS) |

The PX4X500Adapter performs NED ↔ ENU conversion internally.

---

## Prerequisites

### 1. PX4-Autopilot (built inside this workspace)

```
~/umoja_project/robotics/Faucon/PX4-Autopilot/build/px4_sitl_default/
```

`px4_dir` defaults to `~/umoja_project/robotics/Faucon/PX4-Autopilot`.
The launch files reject PX4 paths outside the Faucon workspace, so the drone
package does not depend on files under `~/Faucon/drone_space` or any other
external checkout.

### 2. MicroXRCEAgent

```bash
sudo apt install micro-xrce-dds-agent
# or: pip install micro-xrce-dds-agent
```

### 3. px4_msgs in the Faucon workspace

`px4_msgs` is included directly in this workspace.

### 4. Build

```bash
cd ~/umoja_project/robotics/Faucon
colcon build --symlink-install
source install/setup.bash
```

---

## Quick start

### Full simulation (UGV + drone)

```bash
ros2 launch faucon_drone drone_sim.launch.py
```

Sequence:
- `t=0s`  — Gazebo starts with `virtual_maize_field`, UGV spawns
- `t=15s` — UGV ros2_control stack fully active
- `t=20s` — X500 spawned into Gazebo, MicroXRCEAgent starts
- `t=26s` — PX4 SITL starts in standalone mode
- `t=32s` — PX4X500Adapter + camera bridge start
- `t≈67s` — auto takeoff arms the drone, climbs to 3 m, then holds hover

Optional arguments:

```bash
ros2 launch faucon_drone drone_sim.launch.py \
  px4_dir:=~/umoja_project/robotics/Faucon/PX4-Autopilot \
  drone_spawn_x:=5.5 \        # outside the maize rows by default
  drone_spawn_y:=0.0 \
  drone_spawn_z:=1.5 \
  auto_takeoff:=true \        # arm + takeoff + hover (disabled by auto_trajectory)
  takeoff_altitude:=3.0 \
  auto_trajectory:=false \    # set true to run drone_trajectory instead
  trajectory_waypoints:="" \  # comma-separated "x0,y0,z0,x1,y1,z1,…" (world-ENU)
  use_rviz:=true
```

### Drone only (into running Gazebo)

```bash
ros2 launch faucon_drone spawn_drone_px4.launch.py
```

---

## Piloting

### Automatic takeoff and hover

By default (`auto_takeoff:=true`, `auto_trajectory:=false`), the `drone_takeoff_hover`
node handles:

1. wait for odometry and PX4/EKF startup (`start_delay`),
2. pre-stream zero offboard setpoints (`prearm_setpoint_time`),
3. arm + switch to offboard mode,
4. climb to `takeoff_altitude`,
5. hold the initial XY position with a P controller.

Disable for manual piloting:

```bash
ros2 launch faucon_drone drone_sim.launch.py auto_takeoff:=false
```

### Autonomous waypoint trajectory

`drone_trajectory` replaces `drone_takeoff_hover` when `auto_trajectory:=true`.
It runs the same arm + takeoff sequence, then visits each waypoint in order.

```bash
# Take off to 3 m then follow a square (world-ENU, metres)
ros2 launch faucon_drone drone_sim.launch.py \
  auto_trajectory:=true \
  trajectory_waypoints:="7.0,2.0,3.0,9.0,2.0,3.0,9.0,-2.0,3.0,7.0,-2.0,3.0"
```

Monitor progress:
```bash
ros2 topic echo /faucon/drone/trajectory/status
# WAITING → PREARM → TAKEOFF → WP_1/4 → WP_2/4 → … → HOVER
```

Fine-tuning parameters (via `--ros-args` or YAML param file):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `waypoints` | `[]` | Flat list `[x0,y0,z0, …]` (alternative to `trajectory_waypoints`) |
| `takeoff_altitude` | `3.0` m | Altitude before first waypoint |
| `waypoint_tolerance` | `0.5` m | 3-D arrival radius per waypoint |
| `cruise_speed` | `1.2` m/s | Max horizontal speed per leg |
| `return_home` | `false` | Append home XY as final waypoint |
| `start_delay` | `20.0` s | Delay before arming (let PX4/EKF settle) |

### Manual piloting

#### Step 1 — Wait for EKF2

After launch, wait ~40 seconds then verify:

```bash
ros2 topic echo /faucon/drone/state --once
# → data: DISARMED
```

Also check EKF2 yaw alignment:

```bash
ros2 topic echo /fmu/out/estimator_status_flags --once | grep cs_yaw_align
# → cs_yaw_align: true
```

#### Step 2 — Arm and switch to offboard

```bash
ros2 topic pub --once /faucon/drone/arm std_msgs/msg/Bool "data: true"
```

This sends arm + offboard mode to PX4. The adapter publishes `OffboardControlMode`
at 20 Hz as keepalive — PX4 exits offboard if this stops for more than 500 ms.

#### Step 3 — Takeoff

```bash
ros2 topic pub -r 20 /faucon/drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 1.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### Step 4 — Hover / navigate

```bash
# Move forward at 1 m/s
ros2 topic pub -r 20 /faucon/drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate left
ros2 topic pub -r 20 /faucon/drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

#### Step 5 — Land

```bash
# Descend slowly
ros2 topic pub -r 20 /faucon/drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: -0.3}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### Disarm

```bash
ros2 topic pub --once /faucon/drone/arm std_msgs/msg/Bool "data: false"
```

---

## Monitoring

```bash
# Drone state
ros2 topic echo /faucon/drone/state

# Altitude
ros2 topic echo /faucon/drone/odom --field pose.pose.position.z

# GPS fix
ros2 topic echo /faucon/drone/gps

# Camera
ros2 run rqt_image_view rqt_image_view /faucon/drone/camera/image

# EKF2 health
ros2 topic echo /fmu/out/estimator_status_flags --once

# PX4 arming failures
ros2 topic echo /fmu/out/failsafe_flags --once | grep "true"
```

---

## Package structure

```
faucon_drone/
├── adapters/
│   └── px4_x500_adapter         # PX4 ↔ Faucon interface node (executable)
├── config/
│   ├── drone_bridge.yaml        # ros_gz_bridge config (sim fallback)
│   └── px4_params/
│       └── 4010_gz_x500_mono_cam.post   # PX4 airframe overrides (reference copy)
├── description/
│   ├── drone.urdf.xacro         # Sim-fallback URDF drone
│   └── …
├── launch/
│   ├── drone_sim.launch.py      # Full UGV + drone simulation
│   ├── spawn_drone_px4.launch.py # PX4 drone spawner
│   └── spawn_drone.launch.py    # Sim-fallback drone spawner
├── scripts/
│   ├── drone_takeoff_hover.py        # PX4 auto arm/takeoff/hover helper
│   ├── drone_trajectory.py           # Waypoint trajectory node (arm+takeoff+waypoints)
│   └── drone_velocity_controller.py  # Velocity controller (sim fallback)
├── CMakeLists.txt
└── package.xml
```

---

## PX4 co-simulation notes

### World requirements

`virtual_maize_field` must have these Gazebo system plugins loaded
(already added to `faucon_base_desc/worlds/virtual_maize_field/generated.world`):

```xml
<plugin filename="gz-sim-magnetometer-system"
        name="gz::sim::systems::Magnetometer"/>
<plugin filename="gz-sim-air-pressure-system"
        name="gz::sim::systems::AirPressure"/>
```

Without them, the x500_mono_cam magnetometer and barometer sensors never publish
GZ topics. PX4 cannot read them → EKF2 cannot align yaw → arming denied.

### PX4 airframe overrides

`config/px4_params/4010_gz_x500_mono_cam.post` is kept as a versioned reference
inside this package. `spawn_drone_px4.launch.py` copies both the airframe and
`.post` file into the local PX4 build before SITL starts. It sets:

```sh
param set NAV_DLL_ACT 0         # no GCS/datalink failsafe in simulation
param set COM_DLL_EXCEPT 4
param set COM_ARM_WO_GPS 2
param set EKF2_EV_CTRL 15       # fuse adapter visual odometry
```

### Resetting PX4 parameters

If PX4 misbehaves (wrong saved params from a previous session):

```bash
rm -f ~/umoja_project/robotics/Faucon/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson
rm -f ~/umoja_project/robotics/Faucon/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters_backup.bson
```

Then restart the simulation.

### PX4 console access

When launched via `ros2 launch`, PX4's NSH shell is not directly interactive.
To access it, use `screen`:

```bash
screen /tmp/px4-sock-0
# Quit: Ctrl+A then D
```

---

## Known limitations

- EKF2 convergence takes ~15-20 seconds after PX4 start. The default
  `auto_takeoff_delay` is intentionally conservative; if arming is denied,
  increase it or verify `cs_yaw_align: true`.
- The x500_mono_cam model has a forward camera only. No downward camera for
  precise landing.
