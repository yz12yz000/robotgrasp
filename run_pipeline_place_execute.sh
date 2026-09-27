#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_pipeline_place_execute.sh [--vision-config 配置.json] [--timeout 900]
每轮只抓放最近一只碗，完整退回后重新观测和规划；0 个结束且不启动抓取。
不再选择单／多物体模式；默认全视野，显式配置 ROI 时只处理该区域内的碗。
默认继承 place_config.json，--vision-config 只覆盖显式字段，不改写源配置。
可用 --max-reobservations 2 --max-observation-rounds 30 --max-completed-objects 20。
仅运动前候选耗尽/几何不足可重新观察，任务总 --timeout 不重置；预抓高度为 10 cm。
有有效定位目标后才启动位置控制器并真实移动机械臂、开合夹爪。
执行前确认固定放置点已通过 plan-only，且放置区域不在取物 ROI 内，避免重复抓回。
EOF
    exit 0
fi
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
# Do not start a controller until observation has produced graspable targets.
exec "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/test_live_place_pipeline.py" --execute "$@"
