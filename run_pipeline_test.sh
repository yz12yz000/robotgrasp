#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_pipeline_test.sh [--vision-config /绝对路径/视觉配置.json] [--timeout 240]
硬件准备完成后，一次启动“抓取预览 → 定位 → YOLO”，打印各模块结果并保存日志。
只做规划预览，不切换控制器、不移动机械臂、不驱动夹爪。
测试完成后关闭本次三个节点，保留硬件。已存在项目节点时拒绝重复启动。
桌面示例：--vision-config src/yolo_vision/config/table_roi.example.json
该示例仅适用于当前相机视角；更换相机位置后需重新设置区域。
EOF
    exit 0
fi
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
probe absent grasp_executor rim_locator yolo_vision
probe wait-camera
probe planning-ready
exec "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/test_live_pipeline.py" "$@"
