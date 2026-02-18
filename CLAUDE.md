# AVROS - Autonomous Vehicle ROS2 Platform

## Project Overview

Migration of the AV2.1-API autonomous vehicle codebase to ROS2 (Humble). Five custom packages + upstream drivers (Velodyne, RealSense, Xsens) + Nav2 + robot_localization.

**Source reference:** `~/AV2.1-API`
**Build:** `cd ~/AVROS && colcon build --symlink-install`
**Source overlay:** `source install/setup.bash`
**Target hardware:** NVIDIA Jetson Orin (Ubuntu, `ssh jetson` = 100.93.121.3 via Tailscale, user `dinosaur`)

---

## Build & Test

```bash
# Build all packages (avros_msgs must build first for message generation)
cd ~/AVROS
colcon build --symlink-install --packages-select avros_msgs
colcon build --symlink-install

# Test static TF
ros2 launch avros_bringup sensors.launch.py
ros2 run tf2_tools view_frames

# Test actuator (bench)
ros2 launch avros_bringup actuator.launch.py
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}}'

# Test localization
ros2 launch avros_bringup localization.launch.py

# Keyboard teleop (bench test)
ros2 launch avros_bringup teleop.launch.py
# Keys: i=forward, ,=backward, j=left, l=right, k=stop, q/z=speed up/down

# Phone web UI (bench test — proportional joystick + mode buttons)
ros2 launch avros_bringup webui.launch.py
# Open https://<jetson-ip>:8000 on phone (accept self-signed cert)

# Full autonomous stack
ros2 launch avros_bringup navigation.launch.py

# Plan a route
ros2 service call /avros/plan_route avros_msgs/srv/PlanRoute \
  '{destination_lat: 34.058, destination_lon: -117.82}'

# E-stop
ros2 topic pub --once /avros/actuator_command avros_msgs/msg/ActuatorCommand \
  '{estop: true}'
```

---

## Workspace Structure

```
AVROS/
├── CLAUDE.md
├── requirements.txt              # pip deps: osmnx, fastapi, uvicorn, websockets
├── src/
│   ├── avros_msgs/               # ament_cmake — ActuatorCommand, ActuatorState, PlanRoute
│   ├── avros_bringup/            # ament_python — launch, config, URDF, RViz
│   │   ├── launch/               # sensors, localization, actuator, navigation, teleop, webui
│   │   ├── config/               # ekf, navsat, nav2, actuator, velodyne, realsense, xsens, webui, cyclonedds
│   │   ├── urdf/                 # avros.urdf.xacro
│   │   └── rviz/                 # avros.rviz
│   ├── avros_control/            # ament_python — actuator_node (cmd_vel → Teensy UDP)
│   ├── avros_webui/              # ament_python — webui_node (phone joystick → ActuatorCommand)
│   │   └── static/               # index.html, app.js (nipplejs joystick UI)
│   └── avros_navigation/         # ament_python — route_planner_node (OSMnx → Nav2)
├── firmware/                     # Teensy CAN code (unchanged from AV2.1-API)
└── docs/
```

---

## Packages

| Package | Build Type | Purpose |
|---------|-----------|---------|
| `avros_msgs` | ament_cmake | ActuatorCommand.msg, ActuatorState.msg, PlanRoute.srv |
| `avros_bringup` | ament_python | Launch files, URDF, all YAML configs, RViz config |
| `avros_control` | ament_python | `actuator_node`: cmd_vel → PID → Ackermann inverse → Teensy UDP |
| `avros_webui` | ament_python | `webui_node`: phone joystick WebSocket → ActuatorCommand (direct control) |
| `avros_navigation` | ament_python | `route_planner_node`: GPS → OSMnx route → Nav2 waypoints |

No `avros_sensors` — upstream drivers (velodyne, realsense2_camera, xsens_mti_driver) used directly via official packages.

---

## Network Inventory (192.168.13.0/24)

| IP | Device | MAC | Notes |
|----|--------|-----|-------|
| 192.168.13.10 | Jetson Orin | — | Compute platform, runs all ROS2 nodes |
| 192.168.13.11 | Velodyne VLP-16 | 60:76:88:38:0F:20 | Reconfigured from factory 192.168.1.201 |
| 192.168.13.31 | Gateway/router | — | Network gateway |
| 192.168.13.177 | Teensy (PJRC) | 04:E9:E5:1C:70:4A | Actuator MCU, UDP port 5005 |

---

## Sensors

### Velodyne VLP-16

- **Package:** `ros-humble-velodyne` (apt, official)
- **Config:** `avros_bringup/config/velodyne.yaml`
- **IP:** 192.168.13.11, port 2368
- **Topics:** `/velodyne_packets` (~32 Hz), `/velodyne_points` (~70 Hz)
- **Nodes:** `velodyne_driver_node` (raw UDP → packets), `velodyne_convert_node` (packets → PointCloud2)
- **Range filter:** min 1.0m (car body), max 50.0m
- **Verified working** — data confirmed via tcpdump and topic echo

### Intel RealSense D435i

- **Package:** `ros-humble-realsense2-camera` (apt, or build from source on Jetson with CUDA)
- **Config:** `avros_bringup/config/realsense.yaml`
- **Resolution:** 1280x720 @ 30fps (color + depth)
- **Features:** depth align enabled, pointcloud disabled (Nav2 uses VoxelLayer instead)
- **Jetson notes:** May need librealsense2 built from source with `-DBUILD_WITH_CUDA=true` and kernel module patching (jetsonhacks)

### Xsens MTi-680G (IMU + GNSS)

- **Package:** `xsens_ros_mti_driver` (build from source, ros2 branch)
- **Config:** `avros_bringup/config/xsens.yaml`
- **Port:** `/dev/ttyUSB0` at 115200 baud
- **Output rate:** 100 Hz
- **Topics:** `/imu/data`, `/gnss` (NavSatFix), plus quaternion, euler, acceleration, mag, twist, NMEA
- **GNSS lever arm:** `[0.0, 0.0, 0.0]` — TODO: measure antenna offset on vehicle
- **u-Blox platform:** Automotive (type 4)

---

## TF Tree

```
map                                    ← navsat_transform_node
 └── odom                              ← robot_localization EKF
      └── base_link                    ← robot_state_publisher (URDF)
           ├── imu_link                ← static (URDF) — TODO: measure mount position
           ├── velodyne                ← static (URDF) — TODO: measure mount position
           ├── camera_link             ← static (URDF) — TODO: measure mount position
           │    ├── camera_color_optical_frame  ← realsense driver
           │    └── camera_depth_optical_frame  ← realsense driver
           └── base_footprint          ← static (URDF)
```

Sensor mount positions in URDF (`avros.urdf.xacro`) are approximate — measure on real vehicle.

---

## Key Topics

| Topic | Type | Source |
|-------|------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller / teleop_twist_keyboard |
| `/avros/actuator_state` | `avros_msgs/ActuatorState` | actuator_node @ 20 Hz |
| `/avros/actuator_command` | `avros_msgs/ActuatorCommand` | webui_node (direct control) / e-stop |
| `/avros/route_waypoints` | `nav_msgs/Path` | route_planner_node |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | velodyne_convert_node (~70 Hz) |
| `/velodyne_packets` | `velodyne_msgs/VelodyneScan` | velodyne_driver_node (~32 Hz) |
| `/imu/data` | `sensor_msgs/Imu` | xsens_mti_node (100 Hz) |
| `/gnss` | `sensor_msgs/NavSatFix` | xsens_mti_node |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (robot_localization) |
| `/camera/color/image_raw` | `sensor_msgs/Image` | realsense2_camera_node |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | realsense2_camera_node |

---

## Messages

### ActuatorCommand.msg
```
std_msgs/Header header
bool estop
float32 throttle      # 0.0-1.0
string mode           # N, D, S, R
float32 brake         # 0.0-1.0
float32 steer         # -1.0-1.0 (normalized)
```

### ActuatorState.msg
```
std_msgs/Header header
bool estop
float32 throttle
string mode
float32 brake
float32 steer
bool watchdog_active
```

### PlanRoute.srv
```
float64 destination_lat
float64 destination_lon
---
bool success
string message
float64 distance_meters
uint32 num_waypoints
```

---

## Actuator Control

### UDP Protocol

Teensy at `192.168.13.177:5005`. Watchdog: 500 ms. Keepalive at 200 ms.

```
A E=0 T=0.500 M=D B=0.000 S=0.100    # all-in-one command
→ {"e":0,"t":0.500,"m":"D","b":0.000,"s":0.100,"w":1}
```

### Control Priority (actuator_node)

The actuator node has a 3-way priority system:
1. **Fresh ActuatorCommand** (< timeout) → direct control, skip PID (used by webui)
2. **Fresh cmd_vel** (< timeout) → PID speed control (used by Nav2/teleop)
3. **Neither** → brake to stop

This enables seamless handoff: when webui stops publishing, timeout expires and Nav2's cmd_vel takes over automatically.

---

## Vehicle Parameters

- Wheelbase: 1.23 m
- Track width: 0.9 m
- Max steering: 28 deg (0.489 rad)
- Min turning radius: 2.31 m
- Steering sign: -1 (hardware convention)
- PID: Kp=0.55, Ki=0.055, Kd=0.08
- Max throttle: 0.6 (actuator_node), 0.55 (webui safety limit)
- Robot radius: 0.8 m, inflation: 0.7 m

---

## Nav2 Config

- **Planner:** SmacPlannerHybrid (DUBIN, min radius 2.31 m)
- **Controller:** Regulated Pure Pursuit (lookahead 3-20 m)
- **Local costmap:** VoxelLayer (LiDAR) + InflationLayer, 10x10 m
- **Global costmap:** ObstacleLayer + InflationLayer, 100x100 m rolling
- **Goal tolerance:** 2.0 m xy, 0.5 rad yaw

---

## Web UI (avros_webui)

Phone-based joystick controller for bench testing. FastAPI + WebSocket + nipplejs.

- **Launch:** `ros2 launch avros_bringup webui.launch.py`
- **URL:** `https://<jetson-ip>:8000` (self-signed cert required for phone WebSocket)
- **Control path:** phone joystick → WebSocket → webui_node → `/avros/actuator_command` → actuator_node → Teensy UDP
- **Priority:** ActuatorCommand (direct) takes precedence over cmd_vel (PID). When webui stops publishing, timeout expires and Nav2's cmd_vel takes over.
- **Safety:** WebSocket disconnect → e-stop published automatically
- **Features:** proportional joystick (throttle/brake/steer), E-STOP button, drive mode buttons (N/D/S/R), live telemetry from ActuatorState
- **Config:** `avros_bringup/config/webui_params.yaml` — port, SSL paths, max_throttle
- **SSL setup:** `mkdir -p ~/avros_certs && openssl req -x509 -newkey rsa:2048 -keyout ~/avros_certs/key.pem -out ~/avros_certs/cert.pem -days 365 -nodes -subj '/CN=AVROS'`
- **Pip deps:** `pip install fastapi uvicorn[standard] websockets`
- **On Jetson:** SSL cert paths set in webui_params.yaml to `/home/dinosaur/avros_certs/{cert,key}.pem`

---

## Launch Files

| Launch File | What it starts |
|-------------|---------------|
| `sensors.launch.py` | robot_state_publisher + velodyne driver/convert + realsense + xsens |
| `actuator.launch.py` | actuator_node only |
| `teleop.launch.py` | actuator_node + teleop_twist_keyboard |
| `webui.launch.py` | actuator_node + webui_node |
| `localization.launch.py` | EKF + navsat_transform |
| `navigation.launch.py` | Full stack: sensors + localization + Nav2 + route_planner |

---

## Config Files

| Config | Used By |
|--------|---------|
| `actuator_params.yaml` | actuator_node — Teensy IP/port, PID gains, vehicle geometry |
| `velodyne.yaml` | velodyne_driver_node + velodyne_convert_node |
| `realsense.yaml` | realsense2_camera_node |
| `xsens.yaml` | xsens_mti_node — IMU/GNSS, lever arm, output rate |
| `webui_params.yaml` | webui_node — port, SSL, max throttle |
| `ekf.yaml` | robot_localization EKF |
| `navsat.yaml` | navsat_transform_node |
| `nav2_params.yaml` | Nav2 (planner, controller, costmaps, BT) |
| `cyclonedds.xml` | CycloneDDS shared memory + socket buffer config |

---

## DDS Config

CycloneDDS with shared memory enabled (`cyclonedds.xml`):
- Socket receive buffer: 10 MB minimum
- Shared memory: enabled
- Network: auto-detect interface
- Set via: `CYCLONEDDS_URI=file://<path>` in sensors.launch.py

---

## Ported Code

| AV2.1-API Source | AVROS Destination |
|------------------|-------------------|
| `actuators/udp.py` | `avros_control/actuator_node.py` (UDP protocol) |
| `control/pid.py` | `avros_control/actuator_node.py` (PID class) |
| `control/ackermann_vehicle.py` | `avros_control/actuator_node.py` (Ackermann inverse) |
| `planning/navigator.py` | `avros_navigation/route_planner_node.py` (OSMnx routing) |
| `config/default.yaml` | Split into per-component YAML in `avros_bringup/config/` |
| `webui/server_standalone.py` | `avros_webui/webui_node.py` (ROS2 ActuatorCommand instead of raw UDP) |
| `webui/static/` | `avros_webui/static/` (voice features removed) |

---

## Replaced by Upstream

| AV2.1-API | Upstream Package |
|-----------|-----------------|
| `sensors/xsens_receiver.py` | `xsens_mti_driver` |
| `sensors/lidar_interface.py` | `velodyne` |
| `sensors/camera_interface.py` | `realsense2_camera` |
| `perception/occupancy_grid.py` | `nav2_costmap_2d` VoxelLayer |
| `perception/costmap.py` | `nav2_costmap_2d` InflationLayer |
| `control/pure_pursuit.py` | Nav2 Regulated Pure Pursuit |
| `planning/dwa.py` | Nav2 SmacPlannerHybrid |
| `runner_*.py` | Nav2 Behavior Trees + launch files |

---

## Known Issues & Fixes

| Issue | Fix |
|-------|-----|
| `rclpy.time.Time()` clock type mismatch | Use `rclpy.time.Time(clock_type=self.get_clock().clock_type)` |
| Starlette StaticFiles 404 with `--symlink-install` | Add `follow_symlink=True` to `StaticFiles()` |
| sensors.launch.py xacro YAML parse error (Humble) | Wrap in `ParameterValue(Command([...]), value_type=str)` |
| Port 8000 held after webui crash/disconnect | `fuser -k 8000/tcp` before relaunch |

---

## TODOs

- [ ] Measure physical sensor mount positions on vehicle (URDF imu_link, velodyne, camera_link)
- [ ] Calibrate GNSS lever arm in xsens.yaml (antenna offset from IMU)
- [ ] Test full sensors.launch.py (all sensors together)
- [ ] Verify RealSense D435i camera connected and working
- [ ] Verify Xsens MTi-680G on /dev/ttyUSB0
- [ ] Test localization stack (EKF + navsat)
- [ ] Test full Nav2 navigation stack
- [ ] Commit SSL cert paths for Jetson (currently only set locally)
