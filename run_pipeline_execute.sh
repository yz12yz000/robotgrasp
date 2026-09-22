#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_pipeline_execute.sh [--vision-config 配置.json] [--timeout 600] [--output-dir 日志目录]
真实抓取：自动开启位置控制 → 抓取等待 → 定位等待 → YOLO → 规划通过后自动抓取。
前提：run_robot_prepare.sh 已就绪；示教器已运行 External Control。
默认使用当前桌面区域 table_roi.example.json；相机视角变化后需修改区域。
会移动机械臂并开合夹爪，不会再次询问确认。不能与预览或其他项目节点同时运行。
Ctrl+C 请求取消本次任务。结束后保留硬件与位置控制器，结果写入 debug_output/live_execution。
EOF
    exit 0
fi
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
probe absent grasp_executor rim_locator yolo_vision
probe wait-camera
probe planning-ready
exec "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/run_live_grasp.py" "$@"
