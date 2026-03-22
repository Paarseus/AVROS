#!/bin/bash
# Deploy and relaunch nav2 on Jetson
# Usage: ./scripts/deploy.sh [nav|webui|sensors]
#   nav (default): full navigation stack
#   webui: webui + actuator
#   sensors: sensors only

set -e

JETSON="jetson"
AVROS_DIR="~/AVROS"
MODE="${1:-nav}"

echo "=== Killing all ROS processes on Jetson ==="
ssh $JETSON "ps aux | grep -E 'ros|ekf|nav|xsens|ntrip|actuator|foxglove|webui|bag' | grep -v grep | awk '{print \$2}' | xargs kill -9 2>/dev/null || true"
sleep 2
ssh $JETSON "fuser -k /dev/ttyUSB0 2>/dev/null || true"
sleep 1

# Verify clean
REMAINING=$(ssh $JETSON "ps aux | grep -E 'ros|ekf|nav|xsens|ntrip|actuator|foxglove|webui|bag' | grep -v grep" 2>/dev/null || true)
if [ -n "$REMAINING" ]; then
    echo "Stragglers found, force killing..."
    ssh $JETSON "ps aux | grep -E 'ros|ekf|nav|xsens|ntrip|actuator|foxglove|webui|bag' | grep -v grep | awk '{print \$2}' | xargs kill -9 2>/dev/null || true"
    sleep 1
fi
echo "All clean."

echo "=== Syncing files to Jetson ==="
rsync -avz --delete \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    src/avros_bringup/ $JETSON:$AVROS_DIR/src/avros_bringup/
rsync -avz --delete \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    src/avros_control/ $JETSON:$AVROS_DIR/src/avros_control/
rsync -avz --delete \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    src/avros_webui/ $JETSON:$AVROS_DIR/src/avros_webui/

echo "=== Building on Jetson ==="
ssh $JETSON "cd $AVROS_DIR && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select avros_bringup avros_control avros_webui 2>&1 | tail -5"

echo "=== Launching ($MODE) ==="
case $MODE in
    nav)
        ssh $JETSON "source /opt/ros/humble/setup.bash && source $AVROS_DIR/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://\$(ros2 pkg prefix avros_bringup)/share/avros_bringup/config/cyclonedds.xml && ros2 launch avros_bringup navigation.launch.py enable_velodyne:=false enable_realsense:=false"
        ;;
    webui)
        ssh $JETSON "source /opt/ros/humble/setup.bash && source $AVROS_DIR/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://\$(ros2 pkg prefix avros_bringup)/share/avros_bringup/config/cyclonedds.xml && ros2 launch avros_bringup webui.launch.py"
        ;;
    sensors)
        ssh $JETSON "source /opt/ros/humble/setup.bash && source $AVROS_DIR/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://\$(ros2 pkg prefix avros_bringup)/share/avros_bringup/config/cyclonedds.xml && ros2 launch avros_bringup sensors.launch.py"
        ;;
    *)
        echo "Unknown mode: $MODE (use nav, webui, or sensors)"
        exit 1
        ;;
esac
