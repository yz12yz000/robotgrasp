#!/usr/bin/env bash

# UR5e + movable SZAR base + Logitech F710 + gripper startup
#
# Start /home/rob/init.sh first.  It owns the real base driver and publishes
# odom -> base_footprint from /cmd_vel.  This script starts the arm side,
# the TF bridges, the gripper serial node, joystick Servo node, Mid-360 and
# Orbbec Gemini2L.  Each optional peripheral is launched in its own terminal.

# Do not use `set -u`: ROS Humble setup files reference optional unset vars.

USER_HOME="${HOME:-/home/rob}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"

DESCRIPTION_PACKAGE="mobile_ur_description"
DESCRIPTION_FILE="ur5e_on_new_base_with_gripper_mobile.urdf.xacro"
MOVEIT_CONFIG_PACKAGE="ur_moveit_config"
MOVEIT_CONFIG_FILE="mobile_ur.srdf.xacro"

# Gripper and joystick defaults from ROS2+UR_v6.docx.
GRIPPER_WS="${GRIPPER_WS:-$USER_HOME/handControl2_ws}"
GRIPPER_PORT="${GRIPPER_PORT:-/dev/ttyUSB1}"
GRIPPER_BAUDRATE="${GRIPPER_BAUDRATE:-115200}"
JOYSTICK_WS="${JOYSTICK_WS:-$USER_HOME/joystickTele_ws}"
# Empty means auto-detect the Logitech F710 below.  A caller can still set
# JOYSTICK_DEVICE explicitly when more than one gamepad is connected.
JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-}"
START_GRIPPER="${START_GRIPPER:-true}"
START_JOYSTICK="${START_JOYSTICK:-true}"

# Sensor workspaces and startup switches.  The spelling `orbbect_ws` is the
# existing workspace name on this machine and is kept intentionally.
LIVOX_WS="${LIVOX_WS:-$USER_HOME/ws_livox}"
ORBBEC_WS="${ORBBEC_WS:-$USER_HOME/orbbect_ws}"
START_LIVOX="${START_LIVOX:-true}"
START_ORBBEC="${START_ORBBEC:-true}"

# Fixed installation transform.  Adjust only if the physical odometry frame
# and the model's mobile_base_link origin are offset.
BASE_OFFSET_X="${BASE_OFFSET_X:-0}"
BASE_OFFSET_Y="${BASE_OFFSET_Y:-0}"
BASE_OFFSET_Z="${BASE_OFFSET_Z:-0}"
BASE_OFFSET_YAW="${BASE_OFFSET_YAW:-0}"

DRIVER_PID=""
MOVEIT_PID=""
WORLD_ODOM_PID=""
BASE_BRIDGE_PID=""
GRIPPER_PID=""
JOYSTICK_PID=""
GRIPPER_TERMINAL_PID=""
JOYSTICK_TERMINAL_PID=""
LIVOX_TERMINAL_PID=""
ORBBEC_TERMINAL_PID=""
LIVOX_PROCESS_PID=""
ORBBEC_PROCESS_PID=""

fail() {
    echo "ERROR: $*" >&2
    exit 1
}

normalize_bool() {
    case "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')" in
        true|1|yes|on) printf 'true' ;;
        false|0|no|off) printf 'false' ;;
        *) return 1 ;;
    esac
}

stop_pid() {
    local pid="$1"
    [ -n "$pid" ] || return 0
    if kill -0 "$pid" 2>/dev/null; then
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
}

stop_tree() {
    local root="$1"
    local signal="$2"
    local child
    [ -n "$root" ] || return 0
    # Stop descendants first so ros2/launch wrappers cannot leave their
    # component processes behind when the parent receives Ctrl+C.
    for child in $(pgrep -P "$root" 2>/dev/null || true); do
        stop_tree "$child" "$signal"
    done
    kill -"$signal" "$root" 2>/dev/null || true
}

cleanup() {
    local rc=$?
    trap - EXIT INT TERM
    # Send SIGINT to complete process trees.  This avoids orphaned ROS nodes
    # when a launch wrapper has already detached one of its children.
    stop_tree "$JOYSTICK_PID" INT
    stop_tree "$GRIPPER_PID" INT
    stop_tree "$JOYSTICK_TERMINAL_PID" INT
    stop_tree "$GRIPPER_TERMINAL_PID" INT
    stop_tree "$LIVOX_PROCESS_PID" INT
    stop_tree "$ORBBEC_PROCESS_PID" INT
    stop_tree "$LIVOX_TERMINAL_PID" INT
    stop_tree "$ORBBEC_TERMINAL_PID" INT
    stop_tree "$MOVEIT_PID" INT
    stop_tree "$DRIVER_PID" INT
    stop_tree "$BASE_BRIDGE_PID" INT
    stop_tree "$WORLD_ODOM_PID" INT
    sleep 1
    stop_tree "$JOYSTICK_PID" TERM
    stop_tree "$GRIPPER_PID" TERM
    stop_tree "$JOYSTICK_TERMINAL_PID" TERM
    stop_tree "$GRIPPER_TERMINAL_PID" TERM
    stop_tree "$LIVOX_PROCESS_PID" TERM
    stop_tree "$ORBBEC_PROCESS_PID" TERM
    stop_tree "$LIVOX_TERMINAL_PID" TERM
    stop_tree "$ORBBEC_TERMINAL_PID" TERM
    stop_tree "$MOVEIT_PID" TERM
    stop_tree "$DRIVER_PID" TERM
    stop_tree "$BASE_BRIDGE_PID" TERM
    stop_tree "$WORLD_ODOM_PID" TERM
    exit "$rc"
}
trap cleanup EXIT INT TERM

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

wait_for_node() {
    local node="$1"
    local timeout_s="${2:-20}"
    local elapsed=0
    while [ "$elapsed" -lt "$timeout_s" ]; do
        if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

wait_for_topic() {
    local topic="$1"
    local timeout_s="${2:-20}"
    local elapsed=0
    while [ "$elapsed" -lt "$timeout_s" ]; do
        if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

wait_for_any_node() {
    local timeout_s="${1:-20}"
    shift
    local elapsed=0 node candidate
    while [ "$elapsed" -lt "$timeout_s" ]; do
        node="$(ros2 node list 2>/dev/null || true)"
        for candidate in "$@"; do
            if printf '%s\n' "$node" | grep -Fxq "$candidate"; then
                return 0
            fi
        done
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
        | awk -v c="$controller" -v s="$state" '$1 == c && $3 == s { found=1 } END { exit(found ? 0 : 1) }'
}

controller_state() {
    local controller="$1"
    ros2 control list_controllers 2>/dev/null \
        | sed -r 's/\x1B\[[0-9;]*[A-Za-z]//g' \
        | awk -v c="$controller" '$1 == c { print $3; exit }'
}

wait_for_controller() {
    local controller="$1"
    local timeout_s="${2:-30}"
    local elapsed=0
    while [ "$elapsed" -lt "$timeout_s" ]; do
        [ -n "$(controller_state "$controller")" ] && return 0
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

ensure_velocity_controller() {
    local scaled_state forward_state
    wait_for_controller scaled_joint_trajectory_controller 30 \
        || fail "scaled_joint_trajectory_controller was not spawned"
    wait_for_controller forward_position_controller 30 \
        || fail "forward_position_controller was not spawned"

    scaled_state="$(controller_state scaled_joint_trajectory_controller)"
    forward_state="$(controller_state forward_position_controller)"
    if [ "$scaled_state" = "active" ] || [ "$forward_state" != "active" ]; then
        local switch_args=()
        [ "$scaled_state" = "active" ] && switch_args+=(--deactivate scaled_joint_trajectory_controller)
        [ "$forward_state" != "active" ] && switch_args+=(--activate forward_position_controller)
        ros2 control switch_controllers "${switch_args[@]}" \
            || fail "could not switch controllers to Servo velocity mode"
    fi

    controller_state_is scaled_joint_trajectory_controller inactive \
        || fail "scaled_joint_trajectory_controller is not inactive"
    controller_state_is forward_position_controller active \
        || fail "forward_position_controller is not active"
}

load_workspace() {
    local workspace="$1"
    local package="$2"
    [ -f "$workspace/install/setup.bash" ] \
        || fail "missing workspace setup: $workspace/install/setup.bash"
    source "$workspace/install/setup.bash" \
        || fail "cannot source workspace: $workspace/install/setup.bash"
    ros2 pkg prefix "$package" >/dev/null 2>&1 \
        || fail "ROS package not found after sourcing $workspace: $package"
}

check_for_existing_arm() {
    local existing
    existing="$(pgrep -af 'ros2 launch ur_robot_driver ur_control.launch.py|ur_robot_driver/ur_ros2_control_node' || true)"
    if [ -n "$existing" ]; then
        echo "ERROR: an existing UR driver is already running:" >&2
        echo "$existing" >&2
        echo "Stop the old UR/MoveIt startup first, then run this script once." >&2
        return 1
    fi
    if ros2 node list 2>/dev/null | grep -Fxq "/controller_manager"; then
        echo "ERROR: /controller_manager already exists; refusing to create a duplicate arm driver." >&2
        echo "Stop the previous UR startup first, then run this script once." >&2
        return 1
    fi
}

check_for_existing_sensors() {
    local existing
    if [ "$START_LIVOX" = "true" ]; then
        existing="$(pgrep -af 'ros2 launch livox_ros_driver2 rviz_MID360_launch.py|livox_ros_driver2_node' || true)"
        if [ -n "$existing" ] || ros2 node list 2>/dev/null | grep -Fxq "/livox_lidar_publisher"; then
            echo "ERROR: a Mid-360 driver is already running; refusing a duplicate." >&2
            [ -n "$existing" ] && echo "$existing" >&2
            return 1
        fi
    fi
    if [ "$START_ORBBEC" = "true" ]; then
        existing="$(pgrep -af 'gemini2L.launch.py|component_container.*camera_container' || true)"
        if [ -n "$existing" ] || ros2 node list 2>/dev/null | grep -Fxq "/camera/camera"; then
            echo "ERROR: an Orbbec Gemini2L driver is already running; refusing a duplicate." >&2
            [ -n "$existing" ] && echo "$existing" >&2
            return 1
        fi
    fi
}

detect_joystick_device() {
    [ -n "$JOYSTICK_DEVICE" ] && return 0
    if command -v udevadm >/dev/null 2>&1; then
        local device properties
        for device in /dev/input/js*; do
            [ -e "$device" ] || continue
            properties="$(udevadm info --query=property --name="$device" 2>/dev/null || true)"
            if printf '%s\n' "$properties" | grep -Fq 'ID_VENDOR=Logitech' \
                && printf '%s\n' "$properties" | grep -Fq 'ID_INPUT_JOYSTICK=1'; then
                JOYSTICK_DEVICE="$device"
                return 0
            fi
        done
    fi
    # Fallback for systems without udevadm; this is the usual first joystick.
    [ -e /dev/input/js0 ] && JOYSTICK_DEVICE=/dev/input/js0
    [ -n "$JOYSTICK_DEVICE" ]
}

START_GRIPPER="$(normalize_bool "$START_GRIPPER")" \
    || fail "START_GRIPPER must be true or false"
START_JOYSTICK="$(normalize_bool "$START_JOYSTICK")" \
    || fail "START_JOYSTICK must be true or false"
START_LIVOX="$(normalize_bool "$START_LIVOX")" \
    || fail "START_LIVOX must be true or false"
START_ORBBEC="$(normalize_bool "$START_ORBBEC")" \
    || fail "START_ORBBEC must be true or false"

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

if [ "$START_LIVOX" = "true" ]; then
    [ -f "$LIVOX_WS/install/setup.bash" ] \
        || fail "missing Livox workspace setup: $LIVOX_WS/install/setup.bash"
    [ -e /usr/local/lib/liblivox_lidar_sdk_shared.so ] \
        || fail "missing Livox SDK: /usr/local/lib/liblivox_lidar_sdk_shared.so"
    [ -f "$LIVOX_WS/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json" ] \
        || fail "missing Mid-360 configuration in $LIVOX_WS"
    bash -c "source /opt/ros/humble/setup.bash && source '$LIVOX_WS/install/setup.bash' && ros2 pkg prefix livox_ros_driver2 >/dev/null" \
        || fail "ROS package livox_ros_driver2 is unavailable in $LIVOX_WS"
fi
if [ "$START_ORBBEC" = "true" ]; then
    [ -f "$ORBBEC_WS/install/setup.bash" ] \
        || fail "missing Orbbec workspace setup: $ORBBEC_WS/install/setup.bash"
    bash -c "source /opt/ros/humble/setup.bash && source '$ORBBEC_WS/install/setup.bash' && ros2 pkg prefix orbbec_camera >/dev/null" \
        || fail "ROS package orbbec_camera is unavailable in $ORBBEC_WS"
fi
check_for_existing_sensors || exit 1
check_for_existing_arm || exit 1
if [ "$START_JOYSTICK" = "true" ]; then
    detect_joystick_device \
        || fail "no joystick device found (expected Logitech F710 under /dev/input/js*)"
fi
echo "  description: $DESCRIPTION_PACKAGE/$DESCRIPTION_FILE"
echo "  MoveIt SRDF: $MOVEIT_CONFIG_PACKAGE/$MOVEIT_CONFIG_FILE"
echo "  Mid-360: $START_LIVOX (workspace $LIVOX_WS)"
echo "  Orbbec Gemini2L: $START_ORBBEC (workspace $ORBBEC_WS)"

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
wait_for_service "/controller_manager/list_controllers" 30 \
    || fail "UR driver did not provide controller_manager/list_controllers"
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
wait_for_service "/servo_node/start_servo" 30 \
    || fail "MoveIt Servo service was not created"
echo "  MoveIt/RViz/Servo is ready"

echo "Step 4: Configuring controllers..."
ensure_velocity_controller
echo "  scaled_joint_trajectory_controller: inactive"
echo "  forward_position_controller: active"

echo "Step 5: Starting Servo..."
SERVO_RESULT="$(ros2 service call /servo_node/start_servo std_srvs/srv/Trigger '{}' 2>&1)" \
    || fail "failed to call /servo_node/start_servo"
echo "$SERVO_RESULT"
echo "$SERVO_RESULT" | grep -q "success=True" \
    || echo "WARNING: Servo did not report success (it may already be running)."

# The joystick package invokes `source handControl2_ws/install/setup.bash`
# when Y toggles the gripper, so run it from HOME regardless of the caller's
# current directory (for example, when launched from the Desktop).
cd "$USER_HOME" || fail "cannot change directory to $USER_HOME"

echo "Step 6: Starting gripper control in a separate terminal..."
if [ "$START_GRIPPER" = "true" ]; then
    [ -e "$GRIPPER_PORT" ] || fail "gripper serial port not found: $GRIPPER_PORT"
    command -v gnome-terminal >/dev/null 2>&1 \
        || fail "gnome-terminal is required for the claw console"
    # --wait keeps this terminal process alive for the lifetime of the claw
    # command, so cleanup can close exactly the terminal started here.
    gnome-terminal --disable-factory --wait --title="claw" -- bash -lc \
        "source /opt/ros/humble/setup.bash && source '$GRIPPER_WS/install/setup.bash' && exec ros2 launch hand_control hand_control.launch.py port_name:='$GRIPPER_PORT' baudrate:='$GRIPPER_BAUDRATE'" &
    GRIPPER_TERMINAL_PID=$!
    GRIPPER_PID="$GRIPPER_TERMINAL_PID"
    wait_for_node "/hand_control_node" 15 \
        || fail "gripper node did not start"
    echo "  gripper is ready on $GRIPPER_PORT"
else
    echo "  gripper disabled (START_GRIPPER=$START_GRIPPER)"
fi

echo "Step 7: Starting Logitech F710 joystick control in a separate terminal..."
if [ "$START_JOYSTICK" = "true" ]; then
    [ -e "$JOYSTICK_DEVICE" ] || fail "joystick device not found: $JOYSTICK_DEVICE"
    command -v gnome-terminal >/dev/null 2>&1 \
        || fail "gnome-terminal is required for the joy console"
    gnome-terminal --disable-factory --wait --title="joy" -- bash -lc \
        "cd '$USER_HOME' && source /opt/ros/humble/setup.bash && source '$JOYSTICK_WS/install/setup.bash' && exec ros2 run joy_servo_control logitech_f710_servo_twist --ros-args -p device:='$JOYSTICK_DEVICE'" &
    JOYSTICK_TERMINAL_PID=$!
    JOYSTICK_PID="$JOYSTICK_TERMINAL_PID"
    wait_for_node "/logitech_f710_servo_twist" 15 \
        || fail "joystick node did not start"
    echo "  joystick is ready on $JOYSTICK_DEVICE"
else
    echo "  joystick disabled (START_JOYSTICK=$START_JOYSTICK)"
fi

echo "Step 8: Starting Mid-360 in a separate terminal..."
if [ "$START_LIVOX" = "true" ]; then
    command -v gnome-terminal >/dev/null 2>&1 \
        || fail "gnome-terminal is required for the Mid-360 console"
    gnome-terminal --disable-factory --wait --title="Mid-360" -- bash -lc \
        "source /opt/ros/humble/setup.bash && source '$LIVOX_WS/install/setup.bash' && export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH:-} && exec ros2 launch livox_ros_driver2 rviz_MID360_launch.py" &
    LIVOX_TERMINAL_PID=$!
    wait_for_node "/livox_lidar_publisher" 20 \
        || fail "Mid-360 node /livox_lidar_publisher did not start"
    LIVOX_PROCESS_PID="$(pgrep -n -f 'ros2 launch livox_ros_driver2 rviz_MID360_launch.py' || true)"
    echo "  Mid-360 driver is ready (config from $LIVOX_WS)"
else
    echo "  Mid-360 disabled (START_LIVOX=$START_LIVOX)"
fi

echo "Step 9: Starting Orbbec Gemini2L in a separate terminal..."
if [ "$START_ORBBEC" = "true" ]; then
    command -v gnome-terminal >/dev/null 2>&1 \
        || fail "gnome-terminal is required for the Orbbec RGBD Camera console"
    # Gemini2L publishes coloured, depth-to-colour-aligned PointCloud2 on
    # /camera/depth_registered/points.  HW is the device-supported alignment
    # mode for this model and keeps RGB values spatially registered to depth.
    gnome-terminal --disable-factory --wait --title="Orbbec RGBD Camera" -- bash -lc \
        "source /opt/ros/humble/setup.bash && source '$ORBBEC_WS/install/setup.bash' && exec ros2 launch orbbec_camera gemini2L.launch.py depth_format:=Y14 enable_point_cloud:=true enable_colored_point_cloud:=true depth_registration:=true ordered_pc:=true align_mode:=HW" &
    ORBBEC_TERMINAL_PID=$!
    # Humble runs Gemini2L as a composable node.  Depending on the Orbbec
    # driver version either the component or its container is listed first.
    wait_for_any_node 20 "/camera/camera" "/camera/camera_container" "/camera_container" \
        || fail "Orbbec Gemini2L node/container did not start"
    wait_for_topic "/camera/depth_registered/points" 20 \
        || fail "Orbbec RGB point-cloud topic did not start"
    ORBBEC_PROCESS_PID="$(pgrep -n -f 'ros2 launch orbbec_camera gemini2L.launch.py' || true)"
    echo "  Orbbec Gemini2L is ready (RGB point cloud: /camera/depth_registered/points)"
else
    echo "  Orbbec RGBD Camera disabled (START_ORBBEC=$START_ORBBEC)"
fi

echo "========================================"
echo "UR5e + movable base + joystick + gripper + Mid-360 + Orbbec startup completed"
echo "Real base TF source: szar_base_node (odom -> base_footprint)"
echo "Joystick: F710 axes -> /servo_node/delta_twist_cmds"
echo "Joystick Y button: gripper Open / Close"
echo "Mid-360 console: Mid-360"
echo "Orbbec console: Orbbec RGBD Camera"
echo "Orbbec RGB point cloud: /camera/depth_registered/points (RViz: Color Transformer = RGB8)"
echo "Do not start base_cmd_vel_driver.py; it would publish duplicate base TF."
echo "Press Ctrl+C to stop all arm-side, joystick, gripper, lidar and camera processes."
echo "========================================"

wait
