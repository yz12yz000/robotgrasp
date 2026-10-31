#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_robot_prepare.sh [--check]
启动 UR5e、TF、MoveIt/RViz、夹爪、Gemini2L 和 Mid-360；不启动 Servo/手柄。
机械臂运动控制器保持 inactive，由另外两个模式脚本选择。
--check 只检查环境、设备占用和重复节点，不启动硬件。
可配置：ROBOT_IP、GRIPPER_PORT、GRIPPER_BAUDRATE、START_GRIPPER、
START_ORBBEC、START_LIVOX、LAUNCH_RVIZ、BASE_OFFSET_X/Y/Z/YAW。
移动底盘驱动仍由外部提供 odom -> base_footprint；本脚本不启动人工遥控。
Ctrl+C 停止本脚本启动的硬件进程。详细说明：docs/0921/startup_scripts.md
EOF
    exit 0
fi
[[ $# -eq 0 || ( $# -eq 1 && "$1" = --check ) ]] || { echo '用法：./run_robot_prepare.sh [--check|--help]' >&2; exit 2; }
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
START_GRIPPER="$(bool_value "${START_GRIPPER:-true}")"
START_ORBBEC="$(bool_value "${START_ORBBEC:-true}")"
START_LIVOX="$(bool_value "${START_LIVOX:-true}")"
LAUNCH_RVIZ="$(bool_value "${LAUNCH_RVIZ:-true}")"
GRIPPER_WS="${GRIPPER_WS:-$ROBOT_USER_HOME/handControl2_ws}"
# Verified gripper adapter; use its stable identity instead of USB enumeration order.
GRIPPER_PORT="${GRIPPER_PORT:-/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0}"
ORBBEC_WS="${ORBBEC_WS:-$ROBOT_USER_HOME/orbbect_ws}"
LIVOX_WS="${LIVOX_WS:-$ROBOT_USER_HOME/ws_livox}"

exec 8>"$ROBOT_RUNTIME_DIR/prepare.lock"
flock -n 8 || fail '预备脚本已经运行，不会重复启动硬件。'
lock_mode
for package in ur_robot_driver ur_moveit_config mobile_ur_description; do require_package "$package"; done
absent=(controller_manager move_group robot_state_publisher servo_node logitech_f710_servo_twist grasp_executor world_to_odom_tf base_footprint_to_mobile_base_tf)
if [[ "$START_GRIPPER" = true ]]; then
    [[ -c "$GRIPPER_PORT" && -r "$GRIPPER_PORT" && -w "$GRIPPER_PORT" ]] || fail "夹爪串口不存在或不可读写：$GRIPPER_PORT"
    command -v fuser >/dev/null || fail '缺少 fuser（psmisc），无法检查夹爪串口占用'
    if fuser "$GRIPPER_PORT" >/dev/null 2>&1; then
        fail "串口 $GRIPPER_PORT 已被占用。底盘与夹爪不能共用串口，请设置实际 GRIPPER_PORT。"
    fi
    (source_setup "$GRIPPER_WS/install/setup.bash"; require_package hand_control)
    absent+=(hand_control_node)
fi
if [[ "$START_ORBBEC" = true ]]; then
    (source_setup "$ORBBEC_WS/install/setup.bash"; require_package orbbec_camera)
    absent+=(camera camera_container)
fi
if [[ "$START_LIVOX" = true ]]; then
    (source_setup "$LIVOX_WS/install/setup.bash"; require_package livox_ros_driver2)
    [[ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]] || fail '缺少 Livox SDK 动态库'
    LIVOX_SHARE="$(source_setup "$LIVOX_WS/install/setup.bash"; ros2 pkg prefix --share livox_ros_driver2)"
    [[ -f "$LIVOX_SHARE/config/MID360_config.json" ]] || fail '缺少 MID360_config.json'
    absent+=(livox_lidar_publisher)
fi
probe absent "${absent[@]}"
if [[ "${1:-}" = --check ]]; then echo '预备环境检查通过，未启动硬件。'; exit 0; fi
install_cleanup

start_owned world_tf ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id world --child-frame-id odom --ros-args -r __node:=world_to_odom_tf
start_owned base_tf ros2 run tf2_ros static_transform_publisher \
    --x "${BASE_OFFSET_X:-0}" --y "${BASE_OFFSET_Y:-0}" --z "${BASE_OFFSET_Z:-0}" \
    --roll 0 --pitch 0 --yaw "${BASE_OFFSET_YAW:-0}" \
    --frame-id base_footprint --child-frame-id mobile_base_link --ros-args -r __node:=base_footprint_to_mobile_base_tf
start_owned ur_driver ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e robot_ip:="$ROBOT_IP" \
    description_package:=mobile_ur_description description_file:=ur5e_on_new_base_with_gripper_mobile.urdf.xacro \
    launch_rviz:=false initial_joint_controller:=scaled_joint_trajectory_controller activate_joint_controller:=false
probe driver-ready
start_owned moveit ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e use_sim_time:=false launch_rviz:="$LAUNCH_RVIZ" launch_servo:=false \
    description_package:=mobile_ur_description description_file:=ur5e_on_new_base_with_gripper_mobile.urdf.xacro \
    moveit_config_package:=ur_moveit_config moveit_config_file:=mobile_ur.srdf.xacro
if [[ "$START_GRIPPER" = true ]]; then
    start_owned gripper bash -c 'set -eo pipefail; source "$1"; exec ros2 launch hand_control hand_control.launch.py port_name:="$2" baudrate:="$3"' \
        robot-gripper "$GRIPPER_WS/install/setup.bash" "$GRIPPER_PORT" "${GRIPPER_BAUDRATE:-115200}"
fi
if [[ "$START_ORBBEC" = true ]]; then
    start_owned camera bash -c 'set -eo pipefail; source "$1"; exec ros2 launch "$2"' \
        robot-camera "$ORBBEC_WS/install/setup.bash" "$ROBOT_WORKSPACE/tools/grasp_camera.launch.py"
fi
if [[ "$START_LIVOX" = true ]]; then
    # Same driver parameters as rviz_MID360_launch.py; use the MoveIt RViz
    # already started above instead of forcing a second GUI when RViz is off.
    start_owned lidar bash -c 'set -eo pipefail; source "$1"; export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"; exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args -r __node:=livox_lidar_publisher -p xfer_format:=0 -p multi_topic:=0 -p data_src:=0 -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame -p lvx_file_path:=/home/livox/livox_test.lvx -p cmdline_input_bd_code:=livox0000000001 -p user_config_path:="$2"' \
        robot-lidar "$LIVOX_WS/install/setup.bash" "$LIVOX_SHARE/config/MID360_config.json"
fi
probe planning-ready
[[ "$START_GRIPPER" = false ]] || probe wait-gripper
[[ "$START_ORBBEC" = false ]] || probe wait-camera
[[ "$START_LIVOX" = false ]] || probe wait-lidar
probe driver-ready
flock -u 9
exec 9>&-
echo '预备完成：无手柄/Servo，运动控制器 inactive。保持此终端运行。'
echo '另开终端选择 ./run_grasp_position_control.sh 或 ./run_joystick_control.sh。'
echo '底盘 odom -> base_footprint 由现有底盘驱动提供；请勿重复发布该 TF。'
supervise
