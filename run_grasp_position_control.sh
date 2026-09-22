#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_grasp_position_control.sh [--check]
检查 UR/External Control、实时关节状态、TF、MoveIt 和轨迹 Action；
停止闲置 Servo，严格切换到 scaled_joint_trajectory_controller。
--check 只检查，不停止 Servo、不切换控制器、不发送目标。
默认执行成功后退出，位置控制器保持 active。可以重复执行。
手柄脚本运行、抓取节点存在或有未完成运动 Action 时拒绝切换。
此脚本只准备控制接口。预览：./run_grasp_executor.sh；执行：加 --execute。
EOF
    exit 0
fi
[[ $# -eq 0 || ( $# -eq 1 && "$1" = --check ) ]] || { echo '用法：./run_grasp_position_control.sh [--check|--help]' >&2; exit 2; }
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
lock_mode
if [[ "${1:-}" = --check ]]; then
    probe check-position
else
    probe position
    echo '位置控制已就绪：scaled_joint_trajectory_controller active；未发送运动目标。'
    echo '本次未发送目标。执行抓取：./run_grasp_executor.sh --execute；默认只预览。'
fi
