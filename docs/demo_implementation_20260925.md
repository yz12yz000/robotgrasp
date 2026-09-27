# IMPLEMENT_demo 实施与离线验证记录

日期：2026-09-25。实施依据：[IMPLEMENT_demo.md](../IMPLEMENT_demo.md)。本次未连接机器人或相机；仅修改代码、构建、离线回放与隔离 ROS 测试。没有运行真实硬件准备、真实抓放入口或真实夹爪命令。

## 已实现行为

1. **同碗多候选贯通。** 定位器沿用原局部壁面搜索，默认最多 8 个候选、空间去重间距 4 mm。候选 0 保留旧算法的首选语义。使用版本化 JSON 分组，保留任务/观测/物体/候选编号、原采集时间、坐标系、姿态和几何诊断；执行器只共享纯 Python 协议校验器。
2. **保持原选择顺序。** 碗仍按原首候选加已有抓取偏移后的基座 XY 距离处理；候选按原几何搜索次序尝试；不增加综合评分、不更换 IK 分支排序。旧 PoseArray 仍是一碗一个首选姿态，旧 point/PoseArray 抓取入口继续支持。
3. **运动前换候选。** 首候选的抓点/预抓位置越界、无 IK、明确路径不可行等仅拒绝该候选。必须整条六段路径通过原有轨迹校验后才认领物体并发送动作。失败候选不留下可执行计划；迟到规划线程无法提交已作废计划。
4. **固定 10 cm 预抓。** 两份执行器 JSON 及 Python 默认值由 0.15 m 改为 0.10 m；抬升复用预抓位。固定放置参考姿态、20 cm 放置下降和 1 s 等待保持不变。
5. **抓放完成后重新观察。** 执行入口每轮只转发最近一只碗；全部抓取、释放、退回命令成功后，结束本轮节点，再采集新 RGB/深度、生成新候选、从当前实测关节重新规划。上轮剩余目标、候选和虚拟预览起点不跨轮复用。
6. **有限恢复。** 仅在尚未发送动作时允许候选回退；候选耗尽/明确几何不足允许重新观察。默认最多额外 2 次无进展观察，整个任务最多 30 轮、20 次完整抓放，执行总时间默认 900 s。成功一次可清零无进展计数，总轮数、动作次数和任务截止时间不重置。
7. **隔离每轮输入。** 每任务 UUID、每轮 namespace、私有候选转发和轨迹预览话题共同隔离旧 retained 消息。检查任务身份、原采集时间、坐标系及物体集合；每轮输入必须晚于该轮采集门槛。RGB/深度旧帧直接跳过，不通过改写时间戳使其变新。
8. **执行中失败停止。** 运动/夹爪失败、取消、超时、控制器/TF/通信或未知错误均保留停止及错误上报，不自动释放、换候选或重抓。无法确认本轮进程组清理完成也不进入下一轮。

预览入口对一份有效快照规划全部有效物体，保持虚拟终态衔接；没有真实取走物体时不无限重复预览。纯几何失败/候选耗尽仍受有限重新观察规则约束。

## 结果与日志语义

每轮保存有效配置快照、`candidates.json`、实际转发的 `selected_candidates.json`、失败候选记录、节点日志与轮次结果；总目录保存任务配置、汇总结果及停止原因。尚未到达的阶段不产生对应文件；全部几何拒绝时仍保存空 groups 加 rejected 的候选消息。已有输出目录不可覆盖。

- `completed_count` 只统计成功完成六段抓放及夹爪命令的次数，预览始终为 0。
- `grasp_verified=false`、`placement_verified=false`：没有物体反馈，命令完成不等于实物抓取/释放已验证。
- `object_id` 只在本轮有效。不使用 YOLO 编号作为跨帧身份、已完成黑名单或重试计数器。
- 因本期不做跨帧实体关联，历史失败或被拒绝的目标不能被“后来另一次成功”自动销账。最后观察为空时仍可得到 `partial / remaining_unknown`；恢复成功和实际完成次数保留，但不推算初始物体已全部处理。
- 零目标只来自有效观测；没有相机帧、配对失败和模型故障均不能冒充空场景。

## 修改文件

以下为本次代码/测试/使用说明清单。已有 `IMPLEMENT_demo.md` 和先前分析文档保留，没有改写其内容。

### 定位与分组协议

- `src/rim_locator/rim_locator/candidate_protocol.py`（新增）：严格版本化分组 JSON 校验/编解码。
- `src/rim_locator/rim_locator/rim_detection.py`：有界多候选生成与原首选兼容。
- `src/rim_locator/rim_locator/main.py`：分组发布、质量诊断、全部拒绝输出；保留旧消息。
- `src/rim_locator/rim_locator/config.py`。
- `src/rim_locator/config/config.json`、`src/rim_locator/config/place_config.json`。
- `src/rim_locator/launch/rim_locator.launch.py`。
- 新增测试 `src/rim_locator/test/test_candidate_protocol.py`、`test_grasp_candidates.py`；更新 `test_multi_pose.py`、`test_outlier_filter.py`、`test_supported_contacts.py`。

### 执行器与规划

- `src/grasp_executor/grasp_executor/candidates.py`（新增）：同物体候选回退、完整规划、单物体执行、预览与全局截止时间。
- `src/grasp_executor/grasp_executor/planning_errors.py`（新增）：可恢复候选拒绝类型。
- `src/grasp_executor/grasp_executor/grasp.py`：按观测和物体认领，candidate_id 不改变防重键。
- `src/grasp_executor/grasp_executor/main.py`：分组订阅、身份/时间门槛、结果与取消处理。
- `src/grasp_executor/grasp_executor/moveit_ros.py`：候选失败分类、缓存失效、迟到提交防护、私有预览话题。
- `src/grasp_executor/grasp_executor/interfaces.py`、`config.py`。
- `src/grasp_executor/config/config.json`、`src/grasp_executor/config/place_config.json`。
- `src/grasp_executor/launch/grasp_executor.launch.py`、`pipeline.launch.py`；`src/grasp_executor/package.xml`。
- 新增测试 `src/grasp_executor/test/test_candidate_execution.py`、`test_candidate_planning.py`；更新 `test_grasp.py`、`test_place.py`。

### 视觉与多轮任务

- `src/yolo_vision/yolo_vision/main.py`：任务上下文与原采集时间门槛，不更改分割模型和深度还原算法。
- `src/yolo_vision/launch/yolo_vision.launch.py`。
- 新增 `src/yolo_vision/test/test_observation_context.py`；更新 `test_observation_pipeline.py`、`test_retry.py`。
- `tools/test_live_place_pipeline.py`：按轮编排、单物体转发、每轮重新观察、配置快照、日志、任务互斥和所属进程组清理。
- `tools/place_rounds.py`（新增）：独立于 ROS 的预算、恢复白名单、对象选择与结果策略。
- `tools/test_place_runner.py`；`tools/test_place_rounds.py`（新增）。

### 测试与入口文档

- `tools/test_moveit_grasp.py`：扩展真实 MoveIt 规划与模拟控制器候选/新观测测试。
- `tools/test_observation_transport.py`（新增）：实际定位节点和 retained 分组消息传输。
- `tools/test_place_launch_contract.py`（新增）：实际 launch 参数、namespace、输入类型和私有预览话题。
- `tools/isolated_ros.py`（新增）、`tools/test_robot_control.py`：隔离 ROS 测试进程互斥、空域发现与诊断。
- `run_pipeline_place_offline_test.sh`、`run_pipeline_place_test.sh`、`run_pipeline_place_execute.sh`。
- `New_run/README.md`、`docs/README.md`、本记录。

## 实际验证结果

所有下列检查已运行通过。原始证据归档在 [debug_output/demo_implementation_20260925](../debug_output/demo_implementation_20260925/)；结构化汇总见 [verification_summary.json](../debug_output/demo_implementation_20260925/verification_summary.json)。该目录属于本地调试产物，受工作区现有忽略规则管理。

| 检查 | 结果 | 主要证据 |
|---|---:|---|
| colcon 构建 | 3 个包成功 | [build.log](../debug_output/demo_implementation_20260925/build.log) |
| 单元/纯逻辑/假接口回归 | 498 passed | [unit_tests.log](../debug_output/demo_implementation_20260925/unit_tests.log) |
| 隔离 ROS 启动检查 | 7 passed | [startup_ros_tests.log](../debug_output/demo_implementation_20260925/startup_ros_tests.log) |
| 实际定位节点/消息传输 | 14 项通过 | [transport/result.json](../debug_output/demo_implementation_20260925/transport/result.json) |
| 实际 launch 契约 | 30 项通过 | [launch_contract/result.json](../debug_output/demo_implementation_20260925/launch_contract/result.json) |
| 真实 MoveIt＋模拟控制器 | 15 项通过 | [moveit/result.json](../debug_output/demo_implementation_20260925/moveit/result.json) |
| 历史点云回放 | 26 项通过 | [candidate_replay_audit.json](../debug_output/demo_implementation_20260925/candidate_replay_audit.json) |
| 静态检查 | 76 个 Python、7 个 JSON、23 个 Shell；diff 检查通过 | [static_checks.json](../debug_output/demo_implementation_20260925/static_checks.json) |

单元阶段显示的 7 个 skipped 是可选 `FakeRosTests`，随后已在隔离 ROS 阶段全部启用并通过，没有将其当作未执行后直接略过。启动单独运行时的 14 个 deselected 是前一单元阶段已经通过的其他检查。

关键验证覆盖：

- 0/1/2/3 个目标、根目录与 `New_run` 调用、预览/执行两种入口、每轮单物体转发、抓放后新位置和重新分配 ID、旧消息/旧回调隔离、总预算、零恢复预算、失败阶段分类及停止。
- 协议非法字段/编号/NaN/重复键/坐标系/时间戳拒绝；多个候选不会扩展旧 PoseArray 的物体数量；retained 消息允许晚订阅。
- 候选 A 越界或不可行、B 成功只认领一次；六段/夹爪各阶段失败不自动换候选；过期输入、控制器异常、迟到规划返回、失效缓存和取消均保持拒绝。
- **真实 MoveIt 拒绝与回退**：A 无 IK、B 完整规划成功；执行模式下两候选全部无 IK，动作/夹爪命令/认领记录均为零；成功候选只执行六段模拟轨迹；下一新观测重复本轮 ID 但目标改变，从前轮退回后的实测模拟关节重新规划。原三物体 PoseArray 兼容预览、18 段模拟执行、碰撞拒绝与 FK/TF 检查继续通过。
- **历史点云**：172955（6819 点）默认得到 8 候选，扩展上限得到 25 个；134903 的物体 0（6934 点）得到 8 个，物体 1（3028 点）仍因壁面证据不足拒绝。与旧提交 `95da7301` 的算法独立回放，候选 0 的位置、姿态、诊断逐值一致，原搜索次序和失败诊断保留。回放不等于这些候选均可达。

调试期间发现并修复了预览话题被 launch 默认值覆盖、只清理启动进程而漏掉后代、首候选工作区越界未回退、失败历史被无关联成功清除等问题。另一次并行 ROS 夹具运行出现反馈/服务干扰；保留初次失败日志，给所有隔离测试加文件互斥后串行运行，启动、消息、launch 和 MoveIt 均通过，没有为使测试通过而降低业务轨迹/关节稳定性门槛。

## 可重复运行

```bash
cd /home/rob/robot_grasp_ws
./run_pipeline_place_offline_test.sh
```

脚本构建三个包，执行纯逻辑/假接口回归，然后顺序运行隔离 ROS 启动、消息传输、launch 契约和真实 MoveIt 集成。测试使用系统 Python 并设置 `PYTHONNOUSERSITE=1`，避免用户目录的 NumPy 与系统 ROS 二进制依赖不匹配。隔离测试使用同用户文件锁避免并行发现竞争，仍检查所选 DDS 域为空，并仅使用 localhost。

MoveIt 集成运行真实 OMPL、Cartesian、碰撞/轨迹检查、FK/TF 和 ROS Action 路径；动作服务器、关节反馈、夹爪结果由测试模拟。分割模型没有在这些合成测试中实际推理。

## 尚待实机验证及本期边界

1. **相机与机器人未连接。** 实际 RGB/深度时钟、标定、每次退回后的成像条件、UR 控制反馈、实物路径、夹爪接触和成功率需连接设备后验证；本次结果不证明接近 100% 实物抓取。
2. **取物区域与放置区域需现场分开。** 默认仍为全视野检测；必须确认固定放置点不在取物区域，必要时填写真实取物 ROI，否则下一轮可能识别刚放下的碗。代码记录 `placement_region_separation_verified=false`，本次没有编造 ROI。
3. **局部几何缺失仍可能无候选。** 新功能增加尝试机会，不会凭空补出不存在的壁面证据；有限重新观察仍失败即结束。按约定未增加全局碗沿模型或多帧融合。
4. **没有夹持/释放反馈、跨帧跟踪和放置区容量感知。** 固定放置位姿及场景模型保持原设计；需要现场确认连续放置空间。历史失败的完成判定会保守地保留不确定性。
5. **可达性受机械臂与完整路径约束。** 同碗所有候选都不可达时仍会失败；未增加综合排序、变高度、全流程 IK 回溯或执行后自动恢复。

连接设备后应先运行 plan-only 检查真实目标和固定放置路径，再按单碗、多碗顺序验收：同碗 A 候选不可行换 B、完成退回后重新采集、旁碗位置变化后采用新坐标、候选耗尽有限重观测、执行中失败立即停止。逐项检查真实夹爪接触与物体是否移动，不能只看命令成功计数。
