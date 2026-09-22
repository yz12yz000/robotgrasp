# 三个模块现场测试结果（2026-09-21）

最终结果：在当前相机视角、桌面区域配置下，**真实相机识别 → 定位 → 真实机器人状态下的 MoveIt 三段规划预览全部通过**。
没有发送机械臂轨迹或夹爪动作。真机抓取执行未在本次测试中验证。

## 每个模块的结果

| 模块 | 最终现场结果 | 证据 |
|---|---|---|
| yolo_vision | success；bowl 置信度 0.6594，提取 4306 个点 | 使用可选桌面 ROI 配置，原图坐标掩码 |
| rim_locator | success；760 个碗沿候选点、76 个局部点 | base_link 下目标约 (0.59064, 0.14699, -0.02551) m |
| grasp_executor | planned；全部三段规划完成 | 接近 17.94 s、下降 29.68 s、抬升 29.87 s 的规划时长 |

[最终现场结果 JSON](../debug_output/validation_20260921/live_roi/result.json)
及[现场测试控制台](../debug_output/validation_20260921/live_roi_console.log)。
这是一次成功的现场预览，不是对所有物体位置、光照或真实跟踪精度的保证。

## 发现并处理的问题

1. 原相机 global 时间域曾冻结、跳到未来，RGB/点云时间差约 8.6 秒；YOLO 无法配对。
   本项目相机入口改为 SDK 系统采集时间戳，不在视觉节点重新盖时间戳。
2. 原相机同时发布 30 FPS 红外和彩色点云等当前流程不用的数据。
   改为 15 FPS RGB/深度，关闭红外和额外彩色点云；保留深度注册、有序 XYZ，使用 SENSOR_DATA QoS。
   最后检查 RGB/点云最小时间差约 2.38 ms，并且时间戳持续更新。
3. 原准备检查只确认收到了数据，可能将冻结数据误认为就绪。
   现在要求两组新鲜、时间戳更新、1280×800、坐标系相同且配对差不超过 50 ms 的数据。
4. 同一现场照片全图缩放到 512 后漏检绿色碗；仅截取桌面后相同模型检测到碗（诊断裁剪约 0.63）。
   新增可选 `inference_roi`，掩码还原到原始图像坐标后再取 XYZ；默认全图行为保持不变。
5. 第一轮曾识别出工作空间外目标，抓取正确拒绝；后续桌面 ROI 测试得到范围内目标并完成规划。
   没有扩大工作空间，也没有跳过不可达/碰撞检查。
6. 测试中曾与用户重新启动相机发生重叠，出现重复组件占用。已清理重复组件并恢复。
   最终仅有一个 `/camera/camera` 组件，UR 运动控制器仍为 inactive。

最初全图现场测试未通过；不是一直都成功。完整记录保留在
`debug_output/validation_20260921/live_01`、`live_02` 和最终 `live_roi` 中。

## 简化后的使用方式

硬件准备已经运行、没有其他 YOLO/定位/抓取节点时，在工作区执行：

```bash
./run_pipeline_test.sh --vision-config src/yolo_vision/config/table_roi.example.json
```

此入口检查相机、MoveIt，按顺序启动三个模块，保存结果，结束后只退出本次项目节点。
不切换运动控制器、不驱动夹爪、不具备真实执行模式。
若需要全图诊断，去掉 `--vision-config ...` 即可。

区域 `[320,350,740,550]` 是这次相机视角的示例。相机或桌面位置改变后需要重新确认，
不能把裁剪区域当作桌面标定或碰撞模型。
完整启动说明在 [0921/startup_scripts.md](0921/startup_scripts.md)。

## 修改文件

新增：

- [run_pipeline_test.sh](../run_pipeline_test.sh)：一条命令运行三模块预览测试。
- [tools/test_live_pipeline.py](../tools/test_live_pipeline.py)：启动、状态收集、结果保存及所属进程清理。
- [tools/grasp_camera.launch.py](../tools/grasp_camera.launch.py)：本项目相机配置。
- [table_roi.example.json](../src/yolo_vision/config/table_roi.example.json)：当前桌面区域示例。
- [test_roi.py](../src/yolo_vision/test/test_roi.py)：区域边界、原图掩码还原、默认全图兼容测试。
- 本报告。

修改：

- [run_robot_prepare.sh](../run_robot_prepare.sh)：采用本项目相机入口。
- [robot_control.py](../tools/robot_control.py)、[test_robot_control.py](../tools/test_robot_control.py)：新鲜成对相机检查及回归测试。
- [config.py](../src/yolo_vision/yolo_vision/config.py)、[segmentation.py](../src/yolo_vision/yolo_vision/segmentation.py)：可选 ROI 和掩码还原。
- [setup.py](../src/yolo_vision/setup.py)、[package.xml](../src/yolo_vision/package.xml)：声明 pytest，确保 colcon 能发现测试。
- [启动说明](0921/startup_scripts.md)、[run.md](../run.md)、[后端说明链接](hardware_adapter.md)。

本次未修改定位算法，也未修改抓取运动后端或放宽其保护条件。

## 回归验证

- 三个包 colcon build 成功。
- colcon test：视觉新增 8 项、抓取 38 项，全部通过。
- 启动工具 18 项测试（含隔离 ROS 模拟接口），全部通过。
- 真实 MoveIt/项目模型隔离检查 5 项通过，执行端为模拟控制器。
- 历史点云定位复算得到 (0.64488536, 0.25507101, -0.01934073)，与记录结果一致。
- Bash 语法、Python 编译检查通过。
- 实际单实例相机检查通过，最终现场三模块预览通过。

日志在 [continued](../debug_output/validation_20260921/continued) 和
[live_roi](../debug_output/validation_20260921/live_roi) 中。

## 尚未完成的真机验证

桌面没有碰撞模型；相机外参、TCP、抓取姿态仍需要现场核实。
夹爪尚无抓取成功反馈；本次没有验证真实下抓、夹紧、抬升效果。
不要把 `planned` 当作真实抓取成功。控制器目前 inactive 是本次预览测试的正常状态。
