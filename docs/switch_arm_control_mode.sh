#!/usr/bin/env bash

# Switch a running UR5e between joystick/MoveIt Servo control and the FZI
# Cartesian controller used by grasp_executor. This script never starts the
# grasp pipeline and never publishes a robot motion command.

set -euo pipefail

USER_HOME="${HOME:-/home/rob}"
MODE="${1:-status}"
CONTROLLER_MANAGER="${CONTROLLER_MANAGER:-/controller_manager}"
JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-}"
JOYSTICK_LOG="${JOYSTICK_LOG:-/tmp/robot_grasp_joystick.log}"
ROS_CLI_TIMEOUT="${ROS_CLI_TIMEOUT:-5}"

fail() {
    echo "ERROR: $*" >&2
    exit 1
}

source_if_present() {
    local setup_file="$1"
    if [ -f "$setup_file" ]; then
        # ROS setup files can reference optional unset variables.
        set +u
        source "$setup_file"
        set -u
    fi
}

source_if_present /opt/ros/humble/setup.bash
source_if_present "$USER_HOME/ws_moveit2/install/setup.bash"
source_if_present "$USER_HOME/ur_ros2_ws/install/setup.bash"
source_if_present "$USER_HOME/ur_sim_ws/install/setup.bash"
source_if_present "$USER_HOME/joystickTele_ws/install/setup.bash"

command -v ros2 >/dev/null 2>&1 || fail "ros2 is unavailable"

controller_state() {
    local controller="$1"
    timeout "$ROS_CLI_TIMEOUT" ros2 control list_controllers -c "$CONTROLLER_MANAGER" 2>/dev/null \
        | sed -r 's/\x1B\[[0-9;]*[A-Za-z]//g' \
        | awk -v c="$controller" '$1 == c { print $3; exit }'
}

wait_for_controller_state() {
    local controller="$1" expected="$2" elapsed=0
    while [ "$elapsed" -lt 10 ]; do
        if [ "$(controller_state "$controller")" = "$expected" ]; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

node_exists() {
    timeout "$ROS_CLI_TIMEOUT" ros2 node list 2>/dev/null | grep -Fxq "$1"
}

wait_for_node_state() {
    local node="$1" expected="$2" elapsed=0
    while [ "$elapsed" -lt 10 ]; do
        if node_exists "$node"; then
            [ "$expected" = present ] && return 0
        else
            [ "$expected" = absent ] && return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

call_trigger() {
    local service="$1" output
    output="$(ros2 service call "$service" std_srvs/srv/Trigger '{}' 2>&1)" || {
        echo "$output" >&2
        return 1
    }
    echo "$output"
    printf '%s\n' "$output" | grep -Eq 'success[=:][[:space:]]*[Tt]rue'
}

stop_joystick() {
    local pid found=false
    while read -r pid; do
        [ -n "$pid" ] || continue
        found=true
        echo "Stopping joystick process PID $pid"
        kill -INT "$pid" 2>/dev/null || true
    done < <(pgrep -f '/joy_servo_control/logitech_f710_servo_twist([[:space:]]|$)' || true)

    if [ "$found" = true ]; then
        wait_for_node_state /logitech_f710_servo_twist absent \
            || fail "joystick node did not stop; Cartesian mode was not enabled"
    elif node_exists /logitech_f710_servo_twist; then
        fail "joystick node exists but its process could not be identified; stop it manually"
    fi
}

detect_joystick() {
    [ -n "$JOYSTICK_DEVICE" ] && return 0
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
    return 1
}

start_joystick() {
    if node_exists /logitech_f710_servo_twist; then
        echo "Joystick node is already running"
        return 0
    fi
    detect_joystick || fail "Logitech joystick was not found under /dev/input/js*"
    ros2 pkg executables joy_servo_control 2>/dev/null \
        | grep -Eq '^joy_servo_control[[:space:]]+logitech_f710_servo_twist$' \
        || fail "joy_servo_control/logitech_f710_servo_twist is unavailable"
    echo "Starting joystick on $JOYSTICK_DEVICE (log: $JOYSTICK_LOG)"
    nohup ros2 run joy_servo_control logitech_f710_servo_twist \
        --ros-args -p device:="$JOYSTICK_DEVICE" >"$JOYSTICK_LOG" 2>&1 &
    wait_for_node_state /logitech_f710_servo_twist present \
        || fail "joystick node did not start; inspect $JOYSTICK_LOG"
}

show_status() {
    echo "forward_position_controller: ${1:-$(controller_state forward_position_controller || true)}"
    echo "cartesian_motion_controller: ${2:-$(controller_state cartesian_motion_controller || true)}"
    if node_exists /logitech_f710_servo_twist; then
        echo "joystick: running"
    else
        echo "joystick: stopped"
    fi
}

require_base_services() {
    timeout "$ROS_CLI_TIMEOUT" ros2 service list 2>/dev/null | grep -Fxq "$CONTROLLER_MANAGER/list_controllers" \
        || fail "$CONTROLLER_MANAGER/list_controllers is unavailable"
    timeout "$ROS_CLI_TIMEOUT" ros2 service list 2>/dev/null | grep -Fxq /servo_node/stop_servo \
        || fail "/servo_node/stop_servo is unavailable"
    timeout "$ROS_CLI_TIMEOUT" ros2 service list 2>/dev/null | grep -Fxq /servo_node/start_servo \
        || fail "/servo_node/start_servo is unavailable"
}

switch_to_cartesian() {
    require_base_services
    [ "$(controller_state forward_position_controller)" = active ] \
        || fail "forward_position_controller is not active; refusing an unexpected starting state"

    stop_joystick
    echo "Stopping MoveIt Servo"
    call_trigger /servo_node/stop_servo \
        || fail "MoveIt Servo did not confirm that it stopped"

    local cartesian_state
    cartesian_state="$(controller_state cartesian_motion_controller || true)"
    if [ -z "$cartesian_state" ]; then
        echo "Loading cartesian_motion_controller as inactive"
        ros2 control load_controller cartesian_motion_controller \
            --set-state inactive -c "$CONTROLLER_MANAGER" \
            || fail "could not load cartesian_motion_controller"
    elif [ "$cartesian_state" = unconfigured ]; then
        ros2 control set_controller_state cartesian_motion_controller inactive \
            -c "$CONTROLLER_MANAGER" \
            || fail "could not configure cartesian_motion_controller"
    elif [ "$cartesian_state" != inactive ]; then
        fail "unexpected cartesian_motion_controller state: $cartesian_state"
    fi

    echo "Switching forward_position_controller -> cartesian_motion_controller"
    ros2 control switch_controllers \
        --deactivate forward_position_controller \
        --activate cartesian_motion_controller \
        --strict \
        -c "$CONTROLLER_MANAGER" \
        || fail "controller switch was rejected"

    wait_for_controller_state forward_position_controller inactive \
        || fail "forward_position_controller did not become inactive"
    wait_for_controller_state cartesian_motion_controller active \
        || fail "cartesian_motion_controller did not become active"
    [ "$(ros2 param get /cartesian_motion_controller robot_base_link 2>/dev/null)" = "String value is: base_link" ] \
        || fail "cartesian controller robot_base_link is not base_link"
    ros2 param get /cartesian_motion_controller end_effector_link
    ros2 topic info /cartesian_motion_controller/target_frame -v \
        | grep -q 'Subscription count: 1' \
        || fail "cartesian target_frame does not have exactly one controller subscription"

    show_status inactive active
    echo "Cartesian mode is ready. The grasp pipeline was NOT started."
}

switch_to_servo() {
    require_base_services
    if node_exists /grasp_executor; then
        fail "stop /grasp_executor before returning to Servo mode"
    fi
    [ "$(controller_state cartesian_motion_controller)" = active ] \
        || fail "cartesian_motion_controller is not active; refusing an unexpected starting state"

    echo "Switching cartesian_motion_controller -> forward_position_controller"
    ros2 control switch_controllers \
        --deactivate cartesian_motion_controller \
        --activate forward_position_controller \
        --strict --switch-timeout 10 \
        -c "$CONTROLLER_MANAGER" \
        || fail "controller switch was rejected"
    wait_for_controller_state cartesian_motion_controller inactive \
        || fail "cartesian_motion_controller did not become inactive"
    wait_for_controller_state forward_position_controller active \
        || fail "forward_position_controller did not become active"

    echo "Starting MoveIt Servo"
    call_trigger /servo_node/start_servo \
        || fail "MoveIt Servo did not confirm that it started"
    start_joystick
    show_status active inactive
    echo "Servo/joystick mode is ready."
}

case "$MODE" in
    cartesian) switch_to_cartesian ;;
    servo) switch_to_servo ;;
    status) show_status ;;
    *) fail "usage: $0 {cartesian|servo|status}" ;;
esac
