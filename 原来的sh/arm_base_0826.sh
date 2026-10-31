#!/usr/bin/env bash

# UR5e + movable SZAR base startup
#
# Start the base first with /home/rob/init.sh.  That script starts
# szar_base_node (which publishes odom -> base_footprint) and the keyboard
# publisher (/cmd_vel).  This script starts only the arm side and the two TF
# bridges needed by the RViz model.

# Do not use `set -u` here: ROS setup.bash files intentionally reference
# optional, initially-unset environment variables on Humble.

USER_HOME="${HOME:-/home/rob}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"

# The mobile model deliberately has no world -> mobile_base_link fixed joint.
DESCRIPTION_PACKAGE="mobile_ur_description"
DESCRIPTION_FILE="ur5e_on_new_base_with_gripper_mobile.urdf.xacro"
MOVEIT_CONFIG_PACKAGE="ur_moveit_config"
MOVEIT_CONFIG_FILE="mobile_ur.srdf.xacro"

# Fixed installation transform.  Change these values only if the odometry
# base frame and the model's mobile_base_link origin are not coincident.
BASE_OFFSET_X="${BASE_OFFSET_X:-0}"
BASE_OFFSET_Y="${BASE_OFFSET_Y:-0}"
BASE_OFFSET_Z="${BASE_OFFSET_Z:-0}"
BASE_OFFSET_YAW="${BASE_OFFSET_YAW:-0}"

DRIVER_PID=""
MOVEIT_PID=""
WORLD_ODOM_PID=""
BASE_BRIDGE_PID=""

fail() {
    echo "ERROR: $*" >&2
    exit 1
}

stop_children() {
    trap - INT TERM EXIT
    for pid in "$MOVEIT_PID" "$DRIVER_PID" "$BASE_BRIDGE_PID" "$WORLD_ODOM_PID"; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
}
trap stop_children INT TERM EXIT

wait_for_service() {
    local service="$1"
    local timeout_s="${2:-30}"
    local elapsed=0
    while [ "$elapsed" -lt "$timeout_s" ]; do
        if ros2 service list 2>/dev/null | grep -Fxq "$service"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

controller_state_is() {
    local controller="$1"
    local state="$2"
    ros2 control list_controllers 2>/dev/null \
        | sed -r 's/\x1B\[[0-9;]*[A-Za-z]//g' \
        | grep -E "^${controller}[[:space:]]" \
        | grep -qw "$state"
}

set_controller_state() {
    local controller="$1"
    local state="$2"
    controller_state_is "$controller" "$state" && return 0
    ros2 control set_controller_state "$controller" "$state"
}

echo "Step 1: Loading ROS2 environments..."
[ -f /opt/ros/humble/setup.bash ] || fail "missing /opt/ros/humble/setup.bash"
source /opt/ros/humble/setup.bash || fail "cannot source ROS 2"

for setup_file in \
    "$USER_HOME/ws_moveit2/install/setup.bash" \
    "$USER_HOME/ur_ros2_ws/install/setup.bash" \
    "$USER_HOME/ur_sim_ws/install/setup.bash"; do
    [ -f "$setup_file" ] || fail "missing $setup_file"
    source "$setup_file" || fail "cannot source $setup_file"
done

for package in ur_robot_driver "$MOVEIT_CONFIG_PACKAGE" "$DESCRIPTION_PACKAGE"; do
    ros2 pkg prefix "$package" >/dev/null 2>&1 || fail "ROS package not found: $package"
done
echo "  description: $DESCRIPTION_PACKAGE/$DESCRIPTION_FILE"
echo "  MoveIt SRDF: $MOVEIT_CONFIG_PACKAGE/$MOVEIT_CONFIG_FILE"

echo "Starting TF bridge: world -> odom"
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id world --child-frame-id odom \
    --ros-args -r __node:=world_to_odom_tf &
WORLD_ODOM_PID=$!

echo "Starting TF bridge: base_footprint -> mobile_base_link"
ros2 run tf2_ros static_transform_publisher \
    --x "$BASE_OFFSET_X" --y "$BASE_OFFSET_Y" --z "$BASE_OFFSET_Z" \
    --roll 0 --pitch 0 --yaw "$BASE_OFFSET_YAW" \
    --frame-id base_footprint --child-frame-id mobile_base_link \
    --ros-args -r __node:=base_footprint_to_mobile_base_tf &
BASE_BRIDGE_PID=$!

echo "Step 2: Starting UR5e driver..."
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:="$ROBOT_IP" \
    description_package:="$DESCRIPTION_PACKAGE" \
    description_file:="$DESCRIPTION_FILE" \
    launch_rviz:=false &
DRIVER_PID=$!

if ! wait_for_service "/controller_manager/list_controllers" 30; then
    fail "UR driver did not provide controller_manager/list_controllers"
fi
echo "  UR driver is ready (robot_ip=$ROBOT_IP)"

echo "Step 3: Starting MoveIt2, RViz and Servo..."
ros2 launch "$MOVEIT_CONFIG_PACKAGE" ur_moveit.launch.py \
    ur_type:=ur5e \
    use_sim_time:=false \
    launch_rviz:=true \
    launch_servo:=true \
    description_package:="$DESCRIPTION_PACKAGE" \
    description_file:="$DESCRIPTION_FILE" \
    moveit_config_package:="$MOVEIT_CONFIG_PACKAGE" \
    moveit_config_file:="$MOVEIT_CONFIG_FILE" &
MOVEIT_PID=$!

if ! wait_for_service "/servo_node/start_servo" 30; then
    fail "MoveIt Servo service was not created"
fi
echo "  MoveIt/RViz/Servo is ready"

echo "Step 4: Configuring controllers..."
set_controller_state scaled_joint_trajectory_controller inactive \
    || fail "could not deactivate scaled_joint_trajectory_controller"
set_controller_state forward_position_controller active \
    || fail "could not activate forward_position_controller"
echo "  scaled_joint_trajectory_controller: inactive"
echo "  forward_position_controller: active"

echo "Step 5: Starting Servo..."
SERVO_RESULT="$(ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}' 2>&1)" \
    || fail "failed to call /servo_node/start_servo"
echo "$SERVO_RESULT"
echo "$SERVO_RESULT" | grep -q "success=True" \
    || echo "WARNING: Servo service did not report success (it may already be running)."

echo "========================================"
echo "UR5e + movable base startup completed"
echo "Real base TF source: szar_base_node (odom -> base_footprint)"
echo "Do not start base_cmd_vel_driver.py; it would publish a duplicate TF."
echo "Press Ctrl+C to stop the arm-side processes."
echo "========================================"

wait
