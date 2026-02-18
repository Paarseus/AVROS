# AVROS - Autonomous Vehicle ROS2 Platform

## Project Overview

Migration of the AV2.1-API autonomous vehicle codebase to ROS2 using official packages. The original codebase is a pure Python (~23K lines) + C++ Teensy firmware (~1K lines) autonomous vehicle platform with zero ROS dependencies, implementing a complete sense-plan-control-act pipeline for GPS-guided autonomous navigation with obstacle avoidance.

**Source repo:** `Paarseus/AV2.1-API`
**Target ROS2 distro:** Humble (LTS) or Jazzy
**Build system:** colcon + ament_python / ament_cmake

---

## Hardware Platform

| Component | Hardware | Interface |
|-----------|----------|-----------|
| **GNSS/INS** | Xsens MTi-680G | Serial (xsensdeviceapi SDK), 100Hz, RTK support |
| **LiDAR** | Velodyne VLP-16 | UDP port 2368, 360° FOV, 16 rings, 100m range |
| **Camera (RGB-D)** | Intel RealSense (D400 series) | USB3, BGR8 + depth uint16 (mm) |
| **Camera (RGB)** | USB/CSI camera | OpenCV, 640x480 default |
| **Actuators** | Teensy 4.1 CAN master | UDP 192.168.13.177:5005, ASCII protocol |
| **CAN slaves** | Teensy nodes (steer/throttle/brake) | CAN bus 250kbps |

### Vehicle Parameters
- Wheelbase: 1.23m
- Max steering: 28° (0.489 rad)
- Collision radius: 0.5m
- Safety margin: 0.2m

---

## ROS2 Official Packages to Use

| Component | Package | Install |
|-----------|---------|---------|
| **LiDAR** | `ros-${ROS_DISTRO}-velodyne` | `sudo apt install ros-${ROS_DISTRO}-velodyne` |
| **RealSense** | `ros-${ROS_DISTRO}-realsense2-camera` | `sudo apt install ros-${ROS_DISTRO}-realsense2-camera` |
| **Xsens IMU** | `ros-${ROS_DISTRO}-xsens-mti-driver` | `sudo apt install ros-${ROS_DISTRO}-xsens-mti-driver` |
| **Xsens RTK/Ntrip** | `xsenssupport/Xsens_MTi_ROS_Driver_and_Ntrip_Client` (ros2 branch) | Build from source |
| **Navigation** | `ros-${ROS_DISTRO}-navigation2` | `sudo apt install ros-${ROS_DISTRO}-navigation2` |
| **Localization** | `ros-${ROS_DISTRO}-robot-localization` | `sudo apt install ros-${ROS_DISTRO}-robot-localization` |
| **IPM** | Custom node (wrap existing IPMProcessor) | In-workspace |
| **Transforms** | `ros-${ROS_DISTRO}-robot-state-publisher` | `sudo apt install ros-${ROS_DISTRO}-robot-state-publisher` |

---

## Workspace Structure

```
AVROS/
├── CLAUDE.md
├── src/
│   ├── avros_bringup/          # Launch files, top-level configs
│   │   ├── launch/
│   │   ├── config/
│   │   └── urdf/
│   ├── avros_description/      # URDF/Xacro vehicle model
│   ├── avros_sensors/          # Custom sensor nodes (if needed)
│   ├── avros_perception/       # IPM, occupancy grid, costmap wrappers
│   │   ├── avros_perception/
│   │   └── launch/
│   ├── avros_navigation/       # OSMnx route planner, waypoint publisher
│   │   ├── avros_navigation/
│   │   └── launch/
│   ├── avros_control/          # Actuator interface (UDP to Teensy)
│   │   ├── avros_control/
│   │   └── launch/
│   ├── avros_msgs/             # Custom message/service definitions
│   └── avros_webui/            # Web-based remote control (FastAPI)
├── firmware/                   # Teensy CAN node code (unchanged from AV2.1-API)
│   ├── master.cpp
│   ├── steer.cpp
│   ├── throttle.cpp
│   └── brake.cpp
└── docs/
```

---

## TF Tree (REP-105)

```
map
 └── odom                          (published by robot_localization EKF)
      └── base_link                (vehicle center, ground plane)
           ├── velodyne_link       (LiDAR mount position)
           ├── camera_link         (RealSense mount position)
           ├── camera_color_optical_frame
           ├── camera_depth_optical_frame
           └── imu_link            (Xsens MTi-680G position)
```

- `map -> odom`: Published by `robot_localization` navsat_transform_node (GPS-based) or AMCL
- `odom -> base_link`: Published by `robot_localization` EKF (fusing Xsens IMU + GPS)
- `base_link -> sensor_frames`: Published by `robot_state_publisher` from URDF

---

## Coordinate Conventions

| Frame | Origin | X | Y | Z | Usage |
|-------|--------|---|---|---|-------|
| **map** | UTM origin | East | North | Up | Global planning |
| **odom** | Start position | Forward | Left | Up | Odometry (REP-103 ENU) |
| **base_link** | Vehicle center | Forward | Left | Up | Vehicle body frame |
| **camera_optical** | Camera center | Right | Down | Forward | OpenCV convention |

The original AV2.1-API uses ENU for heading and WGS84 for GPS. ROS2 follows REP-103 (ENU) and REP-105 (map/odom/base_link), so conventions are compatible.

---

## Nav2 Configuration Strategy

### Planner
- **SmacPlannerHybrid** (Hybrid-A*) for Ackermann-feasible global paths
- Min turning radius derived from: `wheelbase / tan(max_steering_angle)` = `1.23 / tan(28°)` = ~2.31m

### Controller
- **Regulated Pure Pursuit** for path following (mirrors existing Pure Pursuit implementation)
- Alternative: **MPPI Controller** for obstacle-aware predictive control (replaces Ackermann DWA)
- `enable_stamped_cmd_vel: true` (Jazzy+)

### Costmap
- **Global costmap:** StaticLayer (if using pre-built map) + VoxelLayer (LiDAR) + InflationLayer
- **Local costmap:** VoxelLayer (LiDAR + RealSense depth) + InflationLayer
- `robot_radius: 0.5`
- `inflation_radius: 0.7` (robot_radius + safety_margin)

### Localization
- `robot_localization` EKF fusing:
  - Xsens MTi-680G IMU (`sensor_msgs/Imu`)
  - Xsens MTi-680G GPS (`sensor_msgs/NavSatFix`)
- `navsat_transform_node` for GPS -> map frame conversion

### Route Planning
- Keep OSMnx-based `navigator.py` as a standalone ROS2 node
- Publishes waypoints to Nav2 `NavigateThroughPoses` action or `WaypointFollower`

---

## Migration Mapping (AV2.1-API -> AVROS)

| AV2.1-API File | AVROS Replacement | Notes |
|----------------|-------------------|-------|
| `sensors/xsens_receiver.py` | `xsens_mti_driver` package | Official ROS2 package, configure via YAML |
| `sensors/lidar_interface.py` | `velodyne` package | Official ROS2 package |
| `sensors/camera_interface.py` | `realsense2_camera` package | Official ROS2 package |
| `perception/occupancy_grid.py` | `nav2_costmap_2d` VoxelLayer | Built into Nav2 |
| `perception/costmap.py` | `nav2_costmap_2d` InflationLayer | Built into Nav2 |
| `perception/preprocessing/ipm_processor.py` | `avros_perception/ipm_node.py` | Wrap existing code as ROS2 node |
| `planning/navigator.py` | `avros_navigation/route_planner_node.py` | Wrap as ROS2 node publishing waypoints |
| `planning/ackermann_dwa_costmap.py` | Nav2 MPPI Controller | Configure for Ackermann kinematics |
| `control/pure_pursuit.py` | Nav2 Regulated Pure Pursuit | Built-in Nav2 controller plugin |
| `control/pid.py` | Nav2 Velocity Smoother | Or custom node if needed |
| `control/ackermann_vehicle.py` | URDF + Nav2 kinematics config | Vehicle model in URDF |
| `actuators/udp.py` | `avros_control/actuator_node.py` | Custom ROS2 node subscribing to cmd_vel |
| `utils/config.py` + `config/default.yaml` | ROS2 parameter YAML files | Per-node parameter files |
| `runner*.py` | Nav2 Behavior Trees + launch files | BT XML replaces imperative Python runners |
| matplotlib/Open3D viewers | RViz2 | Standard ROS2 visualization |
| `webui/server.py` | `avros_webui/` package | Keep FastAPI, subscribe to ROS2 topics |

---

## Actuator UDP Protocol (Unchanged)

The Teensy CAN gateway protocol stays the same. The `avros_control/actuator_node.py` translates between ROS2 and UDP:

```
Commands (ASCII over UDP to 192.168.13.177:5005):
  E 1|0          -> E-stop on/off
  T 0..1         -> Throttle (normalized)
  M N|D|S|R      -> Mode (Neutral/Drive/Sport/Reverse)
  B 0..1         -> Brake (normalized)
  S -1..1        -> Steering (-1=left, +1=right)
  C              -> Center steering
  A E=0 T=0.5... -> All-in-one command
  P              -> Poll state

Response (JSON):
  {"e":0, "t":0.500, "m":"D", "b":0.000, "s":0.100, "w":1}
```

Watchdog timeout: 500ms. Keepalive must be sent at < 500ms intervals.

---

## Migration Phases

### Phase 1: Sensor Drivers (1-2 weeks)
- [ ] Set up colcon workspace
- [ ] Create URDF with sensor frames
- [ ] Configure and test velodyne driver
- [ ] Configure and test realsense2_camera
- [ ] Configure and test xsens_mti_driver
- [ ] Verify all TF static transforms in RViz2

### Phase 2: TF + Localization (2-3 weeks)
- [ ] Configure robot_localization EKF (IMU + GPS fusion)
- [ ] Configure navsat_transform_node (GPS -> map frame)
- [ ] Verify map -> odom -> base_link TF chain
- [ ] Test with recorded rosbag data

### Phase 3: Actuator Node (1 week)
- [ ] Create avros_control package
- [ ] Implement actuator_node.py (cmd_vel -> UDP)
- [ ] Implement keepalive and watchdog handling
- [ ] Publish vehicle state as nav_msgs/Odometry
- [ ] Add lifecycle node support

### Phase 4: Nav2 Integration (3-5 weeks)
- [ ] Configure Nav2 costmaps (VoxelLayer for LiDAR)
- [ ] Configure SmacPlannerHybrid for Ackermann planning
- [ ] Configure Regulated Pure Pursuit or MPPI controller
- [ ] Port OSMnx route planner as waypoint publisher node
- [ ] Create Behavior Tree XML for autonomous navigation
- [ ] Configure lifecycle manager
- [ ] Tune controller parameters on real vehicle

### Phase 5: Perception Nodes (1-2 weeks)
- [ ] Wrap IPMProcessor as ROS2 composable node
- [ ] Feed IPM output into Nav2 costmap (optional custom layer)
- [ ] Integrate YOLOPv2 perception (if needed)

### Phase 6: Integration Testing
- [ ] End-to-end autonomous navigation test
- [ ] Obstacle avoidance validation
- [ ] RTK GPS waypoint following
- [ ] Emergency stop behavior
- [ ] Web UI integration

---

## Coding Conventions

- **Python:** Follow ROS2 Python node conventions (rclpy)
- **Naming:** snake_case for packages, nodes, topics, services, parameters
- **Node naming:** `<package>_<function>_node` (e.g., `avros_actuator_node`)
- **Topic naming:** Use standard names where possible (`/cmd_vel`, `/odom`, `/scan`, `/imu/data`)
- **Parameters:** Declare all parameters in node constructor, load from YAML
- **Launch files:** Python launch files (`.launch.py`)
- **QoS:** Match sensor drivers (typically `best_effort` for sensors, `reliable` for control)
- **Lifecycle:** All custom nodes should implement lifecycle node pattern
- **Error handling:** Use ROS2 logging (`self.get_logger().error/warn/info`)

---

## Key Dependencies

```xml
<!-- package.xml common dependencies -->
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>

<!-- Navigation -->
<depend>nav2_msgs</depend>
<depend>robot_localization</depend>

<!-- Perception -->
<depend>cv_bridge</depend>
<depend>image_transport</depend>

<!-- Custom -->
<depend>numpy</depend>
<depend>scipy</depend>
<depend>opencv-python</depend>
<depend>osmnx</depend>
<depend>pyproj</depend>
</content>
