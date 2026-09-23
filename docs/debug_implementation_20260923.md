# IMPLEMENT_debug.md 实施与验证记录

已按实施方案完成工作区核对、补齐代码和文档，并运行可执行的离线测试。未启动相机、UR 硬件驱动或真实执行入口。

开始时工作区已有大量未提交修改，四项主要修复及多数回归用例已存在。本轮沿用这些实现，没有重置或覆盖为 Git 旧版本。
初始差异清单保存在 [status.txt](../debug_output/implement_debug_20260923/status.txt)，初始已跟踪文件差异保存在
[tracked.diff](../debug_output/implement_debug_20260923/tracked.diff)。下表区分已有实现与本轮补齐内容，避免将此前修改计为本轮新增。

## 已核对并通过回归的主要实现

| 文件 | 已完成行为 |
| --- | --- |
| `src/yolo_vision/yolo_vision/config.py` | 先合并显式 JSON 字段再实例化配置；严格校验；独立 load_config 语义保持不变 |
| `src/yolo_vision/yolo_vision/main.py` | 单／多实例首次成功前按固定总期限重试漏检和深度不足；数据错误终止；超时结果不发布 |
| `tools/test_live_place_pipeline.py` | 互斥单／多模式；三份配置快照；最终模式与来源记录；下游首次输入等待至少 timeout+35 秒 |
| `src/rim_locator/rim_locator/config.py`、`rim_detection.py` | 每实例 26 邻域体素连通过滤，按原点数量取优势组件；锚点、中心、PCA 共用同一过滤结果 |
| `src/rim_locator/config/config.json`、`place_config.json` | 显式启用 8 mm 体素、60% 主组件比例 |
| `src/grasp_executor/grasp_executor/grasp.py` | 批次基座坐标偏移恰好一次；补偿后排序和预览；原始观测认领摘要保持兼容 |
| `src/yolo_vision/test/test_config_modes.py`、`test_retry.py` | 配置优先级、非法输入、单／多模式重试及总期限回归 |
| `src/rim_locator/test/test_outlier_filter.py`、`test_multi_pose.py` | 高位杂点、倾斜碗沿、壁面法向、组件歧义、实例隔离和大点云回归 |
| `src/grasp_executor/test/test_place.py`、`tools/test_place_runner.py` | 单／双目标六段流程、偏移、越界、认领兼容、相对路径和配置快照回归 |

## 本轮实际补充修改文件

| 文件 | 修改 |
| --- | --- |
| `tools/test_live_place_pipeline.py` | 快照写入前检查与输入配置路径冲突，包括符号链接；拒绝覆盖输入文件；更新运行器说明 |
| `tools/test_place_runner.py` | 两项输入配置保护回归，确认报错发生在 ROS、子进程和控制器启动之前 |
| `src/yolo_vision/test/test_retry.py` | 四项输出处理期间超时回归，以及第二个 mask 格式错误时禁止发布部分批次的回归 |
| `tools/test_moveit_grasp.py` | 双目标模拟预览和执行使用 `(0.001, 0, 0.002)` m 偏移；验证偏移仅一次、姿态和固定放置不变、预览与执行一致；结果文件记录偏移 |
| `run_pipeline_place_test.sh`、`run_pipeline_place_execute.sh` | 帮助文本说明单／多模式、选择规则、配置继承和快照 |
| `New_run/README.md` | 单／多碗完整流程命令、配置优先级、相对路径、重试期限、输出目录和旧入口区别 |
| `docs/0921/startup_scripts.md` | 推荐统一 place 入口；说明等待预算、实例过滤和调试点云含义 |
| `docs/hardware_adapter.md` | PoseArray 原始锚点、基座偏移、补偿后排序和原始观测认领约定 |
| `docs/debug_implementation_20260923.md` | 本交付记录 |

place 正式配置中的抓取偏移保持为零。实际工作区的旧三段 `src/grasp_executor/config/config.json`
在本轮开始前已有 `grasp_offset={x:0,y:0,z:0.02}`，与方案中“正式配置均为零”的描述不同；
按保留用户已有改动的要求，该值原样保留，未新增经验偏移。
没有改变冻结批次、夹持／释放反馈、场景清理、固定放置、工具姿态与进退方向、标定或运动超时。

## 实际测试结果

1. 完整离线入口：`PYTHONNOUSERSITE=1 ./run_pipeline_place_offline_test.sh`，退出码 0。
   `yolo_vision`、`rim_locator`、`grasp_executor` 三包构建成功。此时单元回归为 191 passed、7 skipped。
   隔离 MoveIt 使用随机空闲 domain 219、localhost 和模拟控制接口，11 项检查全部通过：包含旧三段、六段、
   双目标连续规划、双目标 12 段模拟执行、非零偏移、工具转换、不可达拒绝及碰撞拒绝。
   详见 [离线日志](../debug_output/implement_debug_20260923/offline.log) 和
   [MoveIt 结果](../debug_output/implement_debug_20260923/moveit/result.json)。
2. 最后补充的 7 项回归加入后，重新运行完整单元测试：**198 passed、7 skipped**，4.24 秒，退出码 0。
   [最终单元日志](../debug_output/implement_debug_20260923/unit_final.log) 列出每项跳过原因。
3. 默认跳过的 7 项是 `ROBOT_STARTUP_ROS_TEST` 未开启时的隔离 ROS 编排测试。
   已另行开启该变量运行 `tools/test_robot_control.py`：**18 tests，全部 OK，0 skipped**，25.942 秒；
   其中包含上述 7 项，其余是该文件自带的纯逻辑和进程管理测试。不是在真实控制器上执行。
   详见 [ROS 编排日志](../debug_output/implement_debug_20260923/startup_integration.log)。
4. 48 个相关 Python 文件使用 Python `compile(..., 'exec')` 完成语法编译检查；两份根目录 place 脚本 `bash -n`、
   两份 `--help`、`git diff --check` 均通过。
5. 几何回归中 9,600 个碗壁点 + 80 个高位杂点通过位置误差 ≤2 mm、法向夹角 ≤2° 断言。
   大点云用例 307,200 点、624 个占用体素，本机本轮过滤耗时 0.437 秒；该数值不是通用实时性能保证。
6. 历史 `debug_output/live/20260918_151517/base_cloud.npy` 有 4,106 点；启用／关闭过滤均保留全部点，
   返回抓取点和法向一致。[对比结果](../debug_output/implement_debug_20260923/recorded_clouds.json) 无人工真值，不作为实机成功率证据。

最终单元命令：

```bash
source /opt/ros/humble/setup.bash
export PYTHONNOUSERSITE=1
export PYTHONPATH="$PWD/src/grasp_executor:$PWD/src/rim_locator:$PWD/src/yolo_vision:${PYTHONPATH:-}"
unset ROBOT_STARTUP_ROS_TEST
/usr/bin/python3 -B -m pytest -q -rs \
  src/grasp_executor/test src/rim_locator/test src/yolo_vision/test \
  tools/test_place_runner.py tools/test_pipeline_runner.py \
  tools/test_robot_control.py tools/test_joint_stationarity.py
```

隔离 ROS 编排命令（先加载现有 ROS/MoveIt/UR 模型工作区）：

```bash
PYTHONNOUSERSITE=1 ROBOT_STARTUP_ROS_TEST=1 /usr/bin/python3 -B tools/test_robot_control.py
```

## 尚未解决或保留的限制

- 本次范围内没有失败的测试或阻塞问题。未做真实相机漏检重试、真实机械臂运动、夹持或释放成功率验收。
- 8 mm、60% 是初始参数；与碗连通的污染仍可能保留，稀疏或碎裂深度可能被拒绝，需现场调参。
- 同步模型推理不能被 timer 强制抢占；仅在回调检查点拒绝超时结果。
- 按方案保留冻结批次，不逐碗重新检查时效或重检测；夹爪仍无夹持／释放传感反馈；没有恢复桌面或携带物碰撞模型。
- 单独启动节点时需自行协调输入等待时间；place 运行器的等待预算调整只存在于本次快照。

所有测试均在假接口或隔离模拟环境完成，模拟通过不等同于实机抓取成功。
