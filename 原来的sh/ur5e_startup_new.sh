#!/bin/bash

# UR5e ROS2 Startup Script
# 2026.1.26

USER_HOME="/home/$(whoami)"
UR_WS="${UR_WS:-$USER_HOME/ur_sim_ws}"
MODEL_WS="${MODEL_WS:-$USER_HOME/ur_sim_ws}"
MOVEIT_WS="${MOVEIT_WS:-$USER_HOME/ws_moveit2}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"
DESCRIPTION_PACKAGE="${DESCRIPTION_PACKAGE:-mobile_ur_description}"
DESCRIPTION_FILE="${DESCRIPTION_FILE:-ur5e_on_new_base_with_gripper.urdf.xacro}"

check_status() {
    if [ $? -eq 0 ]; then
        echo "✓ $1 succeeded"
    else
        echo "✗ $1 failed"
        exit 1
    fi
}

# Wait for ROS2 daemon to start (may be needed on system boot)
sleep 1

# 1. Load ROS2 environment
echo "Step 1: Loading ROS2 environment..."
source /opt/ros/humble/setup.bash
source "$MOVEIT_WS/install/setup.bash"
source "$UR_WS/install/setup.bash"
if [ "$MODEL_WS" != "$UR_WS" ] && [ -f "$MODEL_WS/install/setup.bash" ]; then
    source "$MODEL_WS/install/setup.bash"
fi
check_status "Loading ROS2 environment"

# 2. Start UR5e robot driver
echo "Step 2: Starting UR5e robot driver..."
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:="$ROBOT_IP" \
    description_package:="$DESCRIPTION_PACKAGE" \
    description_file:="$DESCRIPTION_FILE" \
    launch_rviz:=false &
DRIVER_PID=$!
sleep 5  # Wait for driver to fully start
check_status "Starting UR5e robot driver"

# 3. Start MoveIt2 configuration
echo "Step 3: Starting MoveIt2 configuration..."
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    use_sim_time:=false \
    launch_rviz:=true \
    launch_servo:=true \
    description_package:="$DESCRIPTION_PACKAGE" \
    description_file:="$DESCRIPTION_FILE" &
MOVEIT_PID=$!
sleep 5  # Wait for MoveIt2 to fully start
check_status "Starting MoveIt2 configuration"

# 4. Controller state switching
echo "Step 4: Configuring controller states..."
# Deactivate trajectory controller
ros2 control set_controller_state scaled_joint_trajectory_controller inactive
sleep 3
check_status "Deactivating trajectory controller"

# Activate position controller
ros2 control set_controller_state forward_position_controller active
sleep 3
check_status "Activating position controller"

# 5. Start servo service
echo "Step 5: Starting servo service..."
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
sleep 5
check_status "Starting servo service"

echo "========================================"
echo "UR5e ROS2 startup completed!"
echo "All services are started and running"
echo "Driver process PID: $DRIVER_PID"
echo "MoveIt2 process PID: $MOVEIT_PID"
echo "========================================"

echo "Monitoring... Press Ctrl+C to exit"
wait
