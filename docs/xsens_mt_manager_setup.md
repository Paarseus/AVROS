# Xsens MTi-680G — MT Manager Output Configuration

## Why This Is Needed

The MTi-680G's stored output configuration is missing **CalibratedAcceleration**.
Without it, `/imu/data` publishes `linear_acceleration = (0, 0, 0)` with `covariance[0] = -1`
(not available). The EKF needs raw acceleration (with gravity) to do dead reckoning
between GPS updates.

### Current device output (verified live 2026-02-25):
| Data | Status | ROS Topic |
|------|--------|-----------|
| Orientation (quaternion) | OK | `/imu/data` orientation |
| Calibrated Gyroscope | OK | `/imu/data` angular_velocity |
| Free Acceleration | OK | `/filter/free_acceleration` (gravity removed) |
| **Calibrated Acceleration** | **MISSING** | `/imu/data` linear_acceleration = zeros |
| GNSS Position | OK | `/gnss` |
| UTC Time | OK | `/imu/utctime` |

### After fix:
- `/imu/data` linear_acceleration will show real values (~0, 0, 9.81 at rest)
- EKF uses `imu0_remove_gravitational_acceleration: true` to subtract gravity

---

## Requirements

- Windows or Linux PC with [MT Manager](https://www.movella.com/support/software-documentation) installed
- USB cable to connect MTi-680G
- Device ID: **0080005BF5** (verify with `rs-enumerate-devices` or MT Manager)

---

## Steps

### 1. Connect and open device

1. Connect the MTi-680G to your PC via USB
2. Open MT Manager
3. Click **Scan** — the device should appear as `MTi-680G-8A1G6 (0080005BF5)`
4. Click **Connect**

### 2. Open Output Configuration

1. Go to **Device Settings** (gear icon or menu)
2. Select the **Output Configuration** tab
3. You'll see the current output list

### 3. Add CalibratedAcceleration

1. Click **Add** (or the + button)
2. From the data type list, select **Acceleration** → **CalibratedAcceleration**
3. Set the output rate to **100 Hz** (matches the other outputs)
4. Set coordinate system to **ENU** (East-North-Up) if prompted

### 4. Verify the full output list

Ensure all of these are present at 100 Hz:

| Output | Rate | Notes |
|--------|------|-------|
| Orientation (Quaternion) | 100 Hz | For EKF orientation |
| CalibratedGyroscopeData | 100 Hz | For EKF angular velocity |
| **CalibratedAcceleration** | **100 Hz** | **Add this — for EKF linear acceleration** |
| FreeAcceleration | 100 Hz | Gravity-free (used by filter, not EKF) |
| UTC Time | - | Timestamps |
| StatusWord | - | Device status |
| PositionLLA | 100 Hz | GNSS lat/lon/alt |
| VelocityXYZ | 100 Hz | GNSS velocity |
| MagneticField | 100 Hz | Magnetometer |

Keep everything that's already there. Only add CalibratedAcceleration.

### 5. Write to device

1. Click **Apply** or **Write to Device**
2. The configuration is stored in flash — persists across power cycles
3. No need to redo this unless you factory reset

### 6. Verify

1. Disconnect from MT Manager
2. Reconnect the MTi-680G to the Jetson
3. Run:
```bash
source /opt/ros/humble/setup.bash
source ~/AVROS/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py &
sleep 8
ros2 topic echo /imu/data --field linear_acceleration --once
```
4. Expected output (device stationary, upright):
```
x: ~0.0
y: ~0.0
z: ~9.81
```
If z is near 9.81, CalibratedAcceleration is working (gravity included).

---

## Alternative: ROS Driver Auto-Config (No MT Manager)

If you don't have access to MT Manager, the ROS driver can configure the device:

1. Edit the Xsens params file on the Jetson:
```bash
nano ~/AVROS/install/xsens_mti_ros2_driver/share/xsens_mti_ros2_driver/param/xsens_mti_node.yaml
```

2. Change `enable_deviceConfig: false` to `enable_deviceConfig: true`

3. Launch once:
```bash
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py
```

4. The driver will write the output config (including `pub_acceleration: true`) to the device

5. **Set `enable_deviceConfig` back to `false`** — the config is now stored on the device

6. Verify with the same echo command above

---

## After Configuring: EKF Changes

Once CalibratedAcceleration is confirmed working, the EKF config (`ekf.yaml`) needs:

```yaml
imu0_config: [false, false, false,    # position
              true,  true,  true,     # orientation
              false, false, false,    # velocity
              true,  true,  true,     # angular velocity
              true,  true,  true]     # acceleration (NOW AVAILABLE)
imu0_remove_gravitational_acceleration: true   # subtract 9.81 from z
```

This is already the current config — just add `imu0_remove_gravitational_acceleration: true`
once the device is outputting real acceleration data.
