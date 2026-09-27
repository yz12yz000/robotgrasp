# 抓取工作区说明

当前主流程由三个 ROS 2 Humble Python 包组成，使用 MoveIt 规划。完整抓放入口支持按碗分组的多个候选，每次抓放一只后重新观察剩余目标。

| 模块 | 当前职责 |
|---|---|
| `yolo_vision` | 同步 RGB、对齐深度与 CameraInfo，一次分割取物区内的碗；保留原始采集时间和观测内物体编号 |
| `rim_locator` | 按原局部壁面算法生成同碗多候选；保留几何诊断、坐标系和时间戳 |
| `grasp_executor` | 按候选顺序预检完整六段路径，运动前不可行则换候选；执行一只或预览整份快照 |
| `tools/test_live_place_pipeline.py` | 每轮独立话题与节点，执行后重新观察，管理总时间、轮数及有限恢复 |

物体仍按到基座的 XY 距离从近到远处理，候选与 IK 分支排序保持原设计。预抓/抬升高度为 10 cm；固定放置位姿、放置下降 20 cm 保持不变。

## 使用入口

完整现场顺序见 [New_run 使用说明](../New_run/README.md)。以下命令在工作区根目录运行；`New_run/` 有同名转发入口。

```bash
# 当前无设备时可运行，构建与隔离离线测试
./run_pipeline_place_offline_test.sh

# 以下等机器人与相机接好、准备环境就绪后再运行
./run_pipeline_place_test.sh       # 仅规划一份新观测，不发送动作
./run_pipeline_place_execute.sh    # 实际抓放，每次一只，随后重新观察
```

实际执行要求取物 ROI 与放置区域分开；默认配置仍是全视野，不能据此证明固定放置点在取物区外。现场需要确认 ROI、固定放置点、接收区容量和完整路径。

## 接口与日志

`std_msgs/String` 中的版本化 JSON 是新增分组接口，默认独立调试话题为 `/rim_locator/grasp_candidates`。校验器位于 `src/rim_locator/rim_locator/candidate_protocol.py`，执行器声明对 `rim_locator` 的运行依赖，只共享此纯 Python 协议。

每个物体有候选列表，带 `task_id / observation_id / source_stamp_ns / object_id / candidate_id / geometry`。`object_id` 不作为跨帧身份。旧 `PoseArray` 仍然一只碗对应一个姿态；旧 point/PoseArray 模式和三段抓取入口保留。

完整抓放运行器使用每轮独立 namespace、保留消息和私有预览话题，保存 `task_config.json`、每轮配置/候选/诊断、汇总 `result.json`。使用位置及字段解释见 [New_run 日志说明](../New_run/README.md)。

动作命令成功不等于传感器确认抓到或放下，结果明确保留 `grasp_verified=false` 和 `placement_verified=false`。历史失败后的空观测、未解决拒绝与预算耗尽会如实报告，不能当作所有碗均已确认抓完。

## 本次变更与历史资料

实施依据是 [IMPLEMENT_demo.md](../IMPLEMENT_demo.md)，完成记录与测试结果见 [demo_implementation_20260925.md](demo_implementation_20260925.md)。

连接设备后的执行结果、现场 ROI、运行器监督修复和仍未覆盖的情况，见 [2026-09-27 实机验证记录](live_demo_verification_20260927.md)。

本次保留现有机器人 URDF 碰撞和轨迹检查，不增加桌面模型、全局碗沿建模、夹持反馈或执行中恢复。历史硬件文档和早期验证记录保留在本目录；旧文档可能描述单点或旧控制器路径，当前入口和实际代码以上述说明为准。
