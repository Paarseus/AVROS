# AVROS - Autonomous Vehicle ROS2 Platform

## Project Overview

Migration of the AV2.1-API autonomous vehicle codebase to ROS2 (Humble/Jazzy). Five custom packages + upstream drivers (Velodyne, RealSense, Xsens) + Nav2 + robot_localization.

**Source:** `~/AV2.1-API`
**Build:** `cd ~/AVROS && colcon build --symlink-install`
**Source overlay:** `source install/setup.bash`

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
├── src/
│   ├── avros_msgs/          # ament_cmake — ActuatorCommand, ActuatorState, PlanRoute
│   ├── avros_bringup/       # ament_python — launch, config, URDF, RViz
│   │   ├── launch/          # sensors, localization, actuator, navigation, teleop
│   │   ├── config/          # ekf, navsat, nav2, actuator, velodyne, realsense, xsens, cyclonedds
│   │   ├── urdf/            # avros.urdf.xacro
│   │   └── rviz/            # avros.rviz
│   ├── avros_control/       # ament_python — actuator_node (cmd_vel → Teensy UDP)
│   ├── avros_webui/         # ament_python — webui_node (phone joystick → ActuatorCommand)
│   │   └── static/          # index.html, app.js (nipplejs joystick UI)
│   └── avros_navigation/    # ament_python — route_planner_node (OSMnx → Nav2)
├── firmware/                # Teensy CAN code (unchanged from AV2.1-API)
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

No `avros_sensors` — upstream drivers (velodyne, realsense2_camera, xsens_mti_driver) used directly.

---

## TF Tree

```
map                                    ← navsat_transform_node
 └── odom                              ← robot_localization EKF
      └── base_link                    ← robot_state_publisher (URDF)
           ├── imu_link                ← static (URDF)
           ├── velodyne                ← static (URDF)
           ├── camera_link             ← static (URDF)
           │    ├── camera_color_optical_frame  ← realsense driver
           │    └── camera_depth_optical_frame  ← realsense driver
           └── base_footprint          ← static (URDF)
```

Sensor mount positions in URDF are approximate — measure on real vehicle.

---

## Key Topics

| Topic | Type | Source |
|-------|------|--------|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller |
| `/avros/actuator_state` | `avros_msgs/ActuatorState` | actuator_node @ 20Hz |
| `/avros/actuator_command` | `avros_msgs/ActuatorCommand` | webui_node (direct control) / e-stop |
| `/avros/route_waypoints` | `nav_msgs/Path` | route_planner_node |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | velodyne driver |
| `/imu/data` | `sensor_msgs/Imu` | xsens driver |
| `/gnss` | `sensor_msgs/NavSatFix` | xsens driver |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF |

---

## Actuator UDP Protocol

Teensy at `192.168.13.177:5005`. Watchdog: 500ms. Keepalive at 200ms.

```
A E=0 T=0.500 M=D B=0.000 S=0.100    # all-in-one command
→ {"e":0,"t":0.500,"m":"D","b":0.000,"s":0.100,"w":1}
```

---

## Vehicle Parameters

- Wheelbase: 1.23m
- Max steering: 28° (0.489 rad)
- Min turning radius: 2.31m
- Steering sign: -1 (hardware convention)
- PID: Kp=0.55, Ki=0.055, Kd=0.08
- Max throttle: 0.6
- Robot radius: 0.8m, inflation: 0.7m

---

## Nav2 Config

- **Planner:** SmacPlannerHybrid (DUBIN, min radius 2.31m)
- **Controller:** Regulated Pure Pursuit (lookahead 3-20m)
- **Local costmap:** VoxelLayer (LiDAR) + InflationLayer, 10x10m
- **Global costmap:** ObstacleLayer + InflationLayer, 100x100m rolling
- **Goal tolerance:** 2.0m xy, 0.5 rad yaw

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

## Web UI (avros_webui)

Phone-based joystick controller for bench testing. FastAPI + WebSocket + nipplejs.

- **Launch:** `ros2 launch avros_bringup webui.launch.py`
- **URL:** `https://<jetson-ip>:8000` (self-signed cert)
- **Control path:** phone joystick → WebSocket → webui_node → `/avros/actuator_command` → actuator_node → Teensy UDP
- **Priority:** ActuatorCommand (direct) takes precedence over cmd_vel (PID). When webui stops publishing, timeout expires and Nav2's cmd_vel takes over.
- **Safety:** WebSocket disconnect → e-stop published automatically
- **Features:** proportional joystick (throttle/brake/steer), E-STOP button, drive mode buttons (N/D/S/R), live telemetry from ActuatorState
- **SSL setup:** `mkdir -p ~/avros_certs && openssl req -x509 -newkey rsa:2048 -keyout ~/avros_certs/key.pem -out ~/avros_certs/cert.pem -days 365 -nodes -subj '/CN=AVROS'`
- **Pip deps:** `pip install fastapi uvicorn[standard] websockets`

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
