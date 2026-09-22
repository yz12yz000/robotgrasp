#!/usr/bin/env bash
# Shared by the three root-level startup scripts. No robot commands on source.
set -euo pipefail

ROBOT_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_USER_HOME="${HOME:-/home/rob}"
ROBOT_PYTHON=/usr/bin/python3
CHECK_TIMEOUT="${CHECK_TIMEOUT:-30}"
ROBOT_IP="${ROBOT_IP:-192.168.10.102}"
ROBOT_CHILDREN=()
ROBOT_LOG_DIR=""
ROBOT_CLEANUP_HOOK=""

fail() { echo "错误：$*" >&2; exit 1; }

source_setup() {
    [[ -f "$1" ]] || fail "环境文件不存在：$1"
    set +u
    source "$1"
    set -u
}

load_robot_environment() {
    source_setup /opt/ros/humble/setup.bash
    source_setup "${MOVEIT_WS:-$ROBOT_USER_HOME/ws_moveit2}/install/setup.bash"
    source_setup "${UR_DRIVER_WS:-$ROBOT_USER_HOME/ur_ros2_ws}/install/setup.bash"
    source_setup "${UR_MODEL_WS:-$ROBOT_USER_HOME/ur_sim_ws}/install/setup.bash"
    [[ "$CHECK_TIMEOUT" =~ ^[1-9][0-9]*$ ]] || fail 'CHECK_TIMEOUT 必须是正整数秒数'
    [[ "${ROS_DOMAIN_ID:-0}" =~ ^[0-9]+$ ]] || fail 'ROS_DOMAIN_ID 必须是非负整数'
    command -v flock >/dev/null || fail '缺少 flock'
    command -v setsid >/dev/null || fail '缺少 setsid'
    # Same user/domain shares ownership even when invoked from another cwd.
    ROBOT_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/robot-startup-$UID}/robot-grasp-${ROS_DOMAIN_ID:-0}"
    mkdir -p "$ROBOT_RUNTIME_DIR"
    chmod 700 "$ROBOT_RUNTIME_DIR"
}

probe() {
    "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/robot_control.py" --timeout "$CHECK_TIMEOUT" "$@"
}

lock_mode() {
    exec 9>"$ROBOT_RUNTIME_DIR/arm-mode.lock"
    flock -n 9 || fail '另一个模式脚本正在运行。手柄模式需先在其终端 Ctrl+C 退出，再启用位置控制。'
}

bool_value() {
    case "${1,,}" in
        true|1|yes|on) echo true ;;
        false|0|no|off) echo false ;;
        *) fail "无效布尔值：$1" ;;
    esac
}

require_package() { ros2 pkg prefix "$1" >/dev/null || fail "缺少 ROS 包：$1"; }

start_owned() {
    local name="$1"
    shift
    if [[ -z "$ROBOT_LOG_DIR" ]]; then
        mkdir -p "$ROBOT_RUNTIME_DIR/logs"
        ROBOT_LOG_DIR="$(mktemp -d "$ROBOT_RUNTIME_DIR/logs/$(date +%Y%m%d-%H%M%S)-XXXXXX")"
        echo "本次日志：$ROBOT_LOG_DIR"
    fi
    # Non-interactive bash has job control disabled. setsid gives this process
    # and all ROS launch descendants a private group. Never inherit lock FDs.
    setsid "$@" >"$ROBOT_LOG_DIR/$name.log" 2>&1 8>&- 9>&- &
    ROBOT_CHILDREN+=("$!")
    echo "已启动 $name，PID=$!，日志：$ROBOT_LOG_DIR/$name.log"
}

stop_children() {
    local pid sig
    for sig in INT TERM KILL; do
        for pid in "${ROBOT_CHILDREN[@]}"; do
            kill -"$sig" -- "-$pid" 2>/dev/null || true
        done
        [[ "$sig" = KILL ]] || sleep 1
    done
    for pid in "${ROBOT_CHILDREN[@]}"; do wait "$pid" 2>/dev/null || true; done
    ROBOT_CHILDREN=()
}

cleanup_owned() {
    local rc=$?
    trap - EXIT INT TERM
    stop_children
    if [[ -n "$ROBOT_CLEANUP_HOOK" ]]; then
        "$ROBOT_CLEANUP_HOOK" || rc=1
    fi
    [[ -z "$ROBOT_LOG_DIR" ]] || echo "日志保留在：$ROBOT_LOG_DIR"
    exit "$rc"
}

install_cleanup() {
    trap cleanup_owned EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
}

supervise() {
    # Any required launch exiting means the session is no longer ready.
    local rc=0
    wait -n "${ROBOT_CHILDREN[@]}" || rc=$?
    echo "错误：一个受管进程退出（返回码 $rc），正在清理本次启动的进程。查看 $ROBOT_LOG_DIR" >&2
    return 1
}
