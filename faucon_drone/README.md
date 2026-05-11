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
- `t=0s` — Gazebo starts with `virtual_maize_field`, UGV spawns
- `t=12s` — X500 spawned into Gazebo, MicroXRCEAgent starts
- `t=16s` — PX4 SITL starts in standalone mode
- `t=22s` — PX4X500Adapter + camera bridge start
- `t=35-45s` — EKF2 converges (mag + GPS), drone ready to arm

Optional arguments:

```bash
ros2 launch faucon_drone drone_sim.launch.py \
  controller:=px4 \        # px4 (default) or sim (lightweight fallback)
  px4_dir:=~/umoja_project/robotics/Faucon/PX4-Autopilot \
  drone_spawn_x:=2.0 \     # spawn offset from UGV (m)
  drone_spawn_y:=0.0 \
  drone_spawn_z:=0.3 \
  use_rviz:=true
```

### Drone only (into running Gazebo)

```bash
ros2 launch faucon_drone spawn_drone_px4.launch.py
```

### Lightweight fallback (no PX4)

```bash
ros2 launch faucon_drone drone_sim.launch.py controller:=sim
```

Uses a simple velocity controller with a custom URDF drone. No PX4, no XRCE-DDS.
Control interface is the same (`/faucon/drone/cmd_vel`, `/faucon/drone/arm`).

---

## Piloting

### Step 1 — Wait for EKF2

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

### Step 2 — Arm and switch to offboard

```bash
ros2 topic pub --once /faucon/drone/arm std_msgs/msg/Bool "data: true"
```

This sends arm + offboard mode to PX4. The adapter publishes `OffboardControlMode`
at 20 Hz as keepalive — PX4 exits offboard if this stops for more than 500 ms.

### Step 3 — Takeoff

```bash
ros2 topic pub -r 20 /faucon/drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 1.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Step 4 — Hover / navigate

```bash
# Move forward at 1 m/s
ros2 topic pub -r 20 /faucon/drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate left
ros2 topic pub -r 20 /faucon/drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

### Step 5 — Land

```bash
# Descend slowly
ros2 topic pub -r 20 /faucon/drone/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: -0.3}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Disarm

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
inside this package. It is not copied into PX4 during `colcon build`; apply it
explicitly to the PX4 checkout inside this workspace when needed. It sets:

```sh
param set GCS_CONN_LOST_ACT 0   # no GCS failsafe in simulation
param set COM_ARM_MAG_ANG 360   # permissive during EKF2 initial alignment
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

- No position hold controller — drone drifts when `cmd_vel` is zero. A position
  controller node using `/faucon/drone/odom` feedback should be added in the
  orchestration layer.
- EKF2 convergence takes ~15-20 seconds after PX4 start. Do not attempt to arm
  before `cs_yaw_align: true`.
- The x500_mono_cam model has a forward camera only. No downward camera for
  precise landing.
