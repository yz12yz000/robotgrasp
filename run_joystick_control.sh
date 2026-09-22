#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_joystick_control.sh [--check]
在已有 UR/MoveIt 上启动独立 Servo 和 F710，启用 forward_position_controller。
--check 只检查，不启动节点、不切换控制器。
可配置 JOYSTICK_DEVICE=/dev/input/jsN、JOYSTICK_WS。
脚本运行期间独占模式锁；Ctrl+C 停止本次 Servo/手柄并停用运动控制器。
没有手柄、存在抓取节点/未完成运动任务、已有 Servo/手柄时拒绝启动。
EOF
    exit 0
fi
[[ $# -eq 0 || ( $# -eq 1 && "$1" = --check ) ]] || { echo '用法：./run_joystick_control.sh [--check|--help]' >&2; exit 2; }
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
lock_mode
source_setup "${JOYSTICK_WS:-$ROBOT_USER_HOME/joystickTele_ws}/install/setup.bash"
require_package joy_servo_control
require_package moveit_servo
JOYSTICK_DEVICE="${JOYSTICK_DEVICE:-}"
if [[ -z "$JOYSTICK_DEVICE" ]]; then
    candidates=()
    for device in /dev/input/js*; do
        [[ -e "$device" ]] || continue
        properties="$(udevadm info --query=property --name="$device" 2>/dev/null || true)"
        if [[ "$properties" = *ID_VENDOR=Logitech* && "$properties" = *ID_INPUT_JOYSTICK=1* ]]; then
            candidates+=("$device")
        fi
    done
    [[ ${#candidates[@]} -eq 1 ]] || fail '无法唯一识别 Logitech 手柄，请设置 JOYSTICK_DEVICE=/dev/input/jsN'
    JOYSTICK_DEVICE="${candidates[0]}"
fi
[[ -c "$JOYSTICK_DEVICE" && -r "$JOYSTICK_DEVICE" ]] || fail "手柄设备不存在或不可读：$JOYSTICK_DEVICE"
probe check-joystick
if [[ "${1:-}" = --check ]]; then echo '手柄模式检查通过，未启动或切换。'; exit 0; fi

SERVO_PARAMS="$(mktemp "$ROBOT_RUNTIME_DIR/servo-XXXXXX.yaml")"
JOYSTICK_SWITCH_MARKER="$SERVO_PARAMS.switched"
cleanup_joystick_mode() {
    local rc=0
    if [[ -f "$JOYSTICK_SWITCH_MARKER" ]]; then
        # The legacy F710 X button can temporarily select trajectory mode.
        # All owned processes have stopped before releasing these interfaces.
        probe stop-joystick || rc=1
    fi
    rm -f -- "$SERVO_PARAMS" "$JOYSTICK_SWITCH_MARKER"
    return "$rc"
}
ROBOT_CLEANUP_HOOK=cleanup_joystick_mode
install_cleanup
probe servo-config "$SERVO_PARAMS"
# The helper records ownership immediately before the switch request. A failed
# preflight must never make cleanup deactivate another task's controller.
probe joystick "$JOYSTICK_SWITCH_MARKER"
start_owned servo ros2 run moveit_servo servo_node_main --ros-args -r __node:=servo_node --params-file "$SERVO_PARAMS"
probe start-servo
# Existing F710 gripper commands use a HOME-relative handControl2_ws path.
cd "$ROBOT_USER_HOME"
start_owned joystick ros2 run joy_servo_control logitech_f710_servo_twist --ros-args -p device:="$JOYSTICK_DEVICE"
probe joystick-ready
echo '手柄模式已启动。保持此终端运行；退出后才能运行位置控制脚本。'
supervise
