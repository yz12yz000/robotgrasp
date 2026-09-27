#!/usr/bin/env bash
set -euo pipefail
if [[ "${1:-}" = --help ]]; then
    cat <<'EOF'
用法：./run_pipeline_place_test.sh [--vision-config 配置.json] [--timeout 360]
预览一份新观测：0 个结束，N 个从近到远逐个规划；同碗候选按原顺序回退。
不再选择单／多物体模式；默认全视野，显式配置 ROI 时只处理该区域内的碗。
默认继承 place_config.json，--vision-config 只覆盖显式字段，不改写源配置。
可用 --max-reobservations 2 --max-observation-rounds 30 --max-completed-objects 20。
仅运动前候选耗尽/几何不足可重新观察，任务总 --timeout 不重置；预抓高度为 10 cm。
日志按 round_0001 等轮次保存配置快照、分组候选、失败原因和实际采集时间戳。
只做 plan-only，不切换控制器、不移动机械臂、不驱动夹爪；不会覆盖旧 run_pipeline_test.sh。
前提：run_robot_prepare.sh 已完成，相机、TF、MoveIt 和工作空间已就绪。
EOF
    exit 0
fi
source "$(dirname "${BASH_SOURCE[0]}")/tools/robot_startup_common.sh"
load_robot_environment
source_setup "$ROBOT_WORKSPACE/install/setup.bash"
# Vision validates one actual RGB/depth observation using the effective config.
# A camera timeout is distinct from a successful observation with zero targets.
exec "$ROBOT_PYTHON" -B "$ROBOT_WORKSPACE/tools/test_live_place_pipeline.py" "$@"
