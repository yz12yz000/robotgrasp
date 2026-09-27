# New_run 启动顺序

这些脚本是工作区根目录脚本的统一入口，代码仍然只维护在上一级目录。
先进入本目录：

```bash
cd /home/rob/robot_grasp_ws/New_run
```

## 1. 每次开机先启动准备环境

在第一个终端运行并保持窗口不退出：

```bash
./run_robot_prepare.sh
```

它启动 UR 驱动、TF、MoveIt/RViz、夹爪、相机和雷达；不启动手柄、Servo，也不激活机械臂运动控制器。

当前版本不加载桌面碰撞模型或额外夹爪保护盒，也不读取旧的 `table_model.json`。
准备脚本会自动清除 MoveIt 场景中残留的 `grasp_table` 和 `grasp_gripper_envelope`；
抓取后端在规划前也执行同一检查。RViz 中这两个额外碰撞对象随场景更新消失。
机器人 URDF 自带的机械臂/夹爪外观与碰撞几何、自碰撞检查继续保留。
不再运行 `run_table_model.sh`，该入口已移除。

## 2. 观察视野并预览整批抓放

第二个终端运行：

```bash
./run_pipeline_place_test.sh
```

默认使用整个相机视野，检测当前视野中的所有碗，不再选择单物体／多物体模式。
检测到 0 个就正常结束，不启动抓取执行器；1 个就规划 1 个；2、3 或更多个就按抓取点到基座的 XY 距离从近到远，逐个规划六段抓取放置。
预览只检查一次有效观测中的目标，不因为没有真实取走物体而重复观察。每只碗携带多个候选，按原顺序选择第一组能通过完整六段规划的候选；仍保留按基座 XY 距离从近到远的物体顺序，不引入综合评分。默认目标类别仍为 `bowl`。

只显示规划，不移动机械臂、不控制夹爪。结果会打印检测数量、有效定位数量；深度无效或定位拒绝会单独记录，不能把部分完成当作全部完成。
相机没有图像／深度数据或时间戳无法配对时是数据错误，不等于检测到 0 个。

如明确只处理某个区域，可另外指定 ROI 配置：

```bash
./run_pipeline_place_test.sh --vision-config ../src/yolo_vision/config/table_roi.example.json
```

此时只处理 ROI 内目标。配置按字段默认值 → place_config.json → 自定义 JSON 显式字段合并，不写回源文件。
相对配置路径以调用目录为基准；模型路径仍以视觉包 share 为基准。
`--single` 和 `--multi` 已移除。旧 JSON 中布尔型 `multi_instance` 仅兼容读取并忽略，不再影响目标数量。

旧 `run_pipeline_test.sh`、`run_pipeline_execute.sh` 只保留三段抓取用途，不包含放置；完整流程统一使用 place 入口。

## 3. 依次抓取、放置和释放

现场确认规划后运行：

```bash
./run_pipeline_place_execute.sh
```

先观察、定位，再启动执行器。有有效目标才调用位置控制脚本；0 个目标不切换控制器、不发送机械臂或夹爪命令。
每轮只抓取当前最近的一只碗：接近 → 开爪 → 下降 → 闭爪 → 抬升 → 固定放置点 → 下降 → 等待 1 秒 → 开爪 → 退回。完整退回、结束本轮节点后，重新采集 RGB/深度、定位剩余碗并规划下一轮；不会继续使用上轮未处理目标的旧坐标或旧 YOLO 编号。

预抓位置固定高出抓点 **0.10 m**，抓取抬升复用此位置。放置下降仍为 **0.20 m**，固定放置位姿不变；接收区域须能容纳连续放下的物体。

在机械臂和夹爪均未收到本轮命令时，某候选规划不可行会自动换同一只碗的下一个候选。候选全部不可行或几何点不足，默认最多额外重新观察 2 次；成功完成一次抓放后，无进展次数归零。运动开始后的失败、控制器/TF/通信故障、取消和超时直接停止，不自动释放或重抓。

```bash
./run_pipeline_place_execute.sh --max-reobservations 2 --max-observation-rounds 30 --max-completed-objects 20 --timeout 900
```

`--timeout` 是整个任务的总时间，重观测不重新计时。总轮数包含最后一次确认没有目标的观察；达到任一上限均如实结束并记录原因。

**实机使用条件：取物区域与放置区必须分开。** 默认仍是全视野检测；若固定放置点在视野内，需先配置实际取物 ROI，确保放下的碗不会再次进入取物检测。示例 ROI 不是现场标定结果，本次未自动填入虚构区域。

YOLO 编号仅在本轮有效；本期没有跨帧物体跟踪。历史失败/拒绝后即使后续抓放成功，也不能证明成功的是之前那一只，因此日志保留未解决状态；最后观察为空可报告 `partial / remaining_unknown`。这不阻止允许的重观测和后续动作，但不会把不确定目标算成已确认处理。

## 4. 手柄模式（与抓取模式互斥）

不需要手柄时不要运行。需要手柄时，在准备脚本保持运行的前提下另开终端：

```bash
./run_joystick_control.sh
```

退出手柄终端后，才能再次运行位置控制或抓取脚本。

## 5. 固定放置准备位姿

放置参考位姿保存在：

```text
../src/grasp_executor/config/place_config.json
```

其中 `place_reference_*` 保存用户指定的固定 TCP 参考，执行器通过静态 TF 转成 `base_link/grasp_center`，并与保存的 `place_ready_*` 交叉核对。程序启动时不会读取当前机械臂位置来覆盖它。

真实执行结果保存在：

```text
debug_output/place_execution/
```

规划预览结果保存在：

```text
debug_output/place_validation/
```

默认输出路径相对当前终端目录，也可通过 `--output-dir` 指定。每次任务使用新目录，不覆盖既有结果。结构如下：

```text
<任务目录>/
  task_config.json             # 任务配置及总预算
  result.json                  # 汇总结果、每轮结果、停止原因
  round_0001/
    effective_vision_config.json
    effective_rim_config.json
    effective_grasp_config.json
    candidates.json           # 分组候选；定位全部拒绝时也保存
    selected_candidates.json  # 执行仅转发一只碗；预览转发本轮全部
    candidate_attempts.json   # 失败候选、阶段及原因
    result.json
    yolo_vision.log / rim_locator.log / grasp_executor.log
  round_0002/ ...
```

未达到相应阶段的文件不会生成。开启调试时，每轮还保存检测叠加图和基座系点云。
任务与每轮均有独立编号、namespace、输入和预览话题；只接受晚于本轮门槛的原始采集时间戳。`object_id` 是观测内编号，`candidate_id` 是同碗内候选编号。

`completed_count` 只统计完成全部抓放命令的次数；预览始终为 0。`grasp_verified`、`placement_verified` 始终为 false，因为没有物体夹持/释放反馈。`no_targets` 仅表示有效观测未检测到目标；历史失败后为空、部分拒绝、预算耗尽等不会宣称全部物体已处理。

视觉在 `input_timeout` 内等待一组尺寸、坐标系、采集时间和配对误差符合配置的 RGB／对齐深度图及 CameraInfo，完成一次全部目标检测。
0 个目标正常结束；未收到合格相机数据、模型故障、检测到目标但全部缺少有效深度会报告失败。
深度图、彩色图先读取序列化消息头，选中同步帧后才完整解码，避免不断解码未使用的大帧。
放置入口不再提前运行固定参数的“两组相机帧”检查，而由视觉节点按最终配置验证实际使用的这一组数据。
独立 `wait-camera` 探针仍用于硬件准备，它检查的是连续传感帧而不是物体数量，并输出两路收到的帧数、帧大小、坐标系和时间差。

放置运行器仅在本次快照中将 rim/grasp 首次输入等待延长至至少 `--timeout + 35` 秒，运行器仍有总体截止时间。
执行失败后停止批次，不自动释放、不重试、不处理后续目标。夹持／释放仍无物体传感反馈，流程成功不等于已确认真实抓住物体。

place 默认使用对齐深度图 `/camera/depth/image_raw` 和 `/camera/depth/camera_info` 还原 XYZ，保留采集时间和原像素位置。
16UC1 深度由毫米转换为米；坐标系、尺寸、内参与时间不匹配则拒绝。
旧 PointCloud2 输入仍可通过 `depth_image_topic: null` 配置使用，不涉及目标数量模式。

## 6. 不连接机器人和相机的回归测试

```bash
./run_pipeline_place_offline_test.sh
```

此入口构建三个包、运行单元测试，然后在独立空闲 DDS 域和 localhost 上运行消息传输、launch 参数、模拟控制器和真实 MoveIt 规划测试。它不会启动相机、UR 驱动或真实夹爪命令。隔离测试的轨迹执行和关节反馈是合成的，不能作为实机抓取成功率验收。

实现说明、文件清单和本次结果见 [实施验证记录](../docs/demo_implementation_20260925.md)。

## 7. 2026-09-27 实机验证配置

当前相机和工作台布置的取物 ROI 已保存，可按本目录的相对路径使用：

```bash
./run_pipeline_place_test.sh --vision-config ../src/yolo_vision/config/live_pick_roi_20260927.json
./run_pipeline_place_execute.sh --vision-config ../src/yolo_vision/config/live_pick_roi_20260927.json
```

该区域排除本次固定放置区；相机位置改变后需重新核对。运行器正常收到 SIGINT、SIGTERM、SIGHUP 会清理本次节点；执行器也会通过 PID/启动时钟检测运行器异常退出，并触发现有停止流程。旧独立执行器入口不传监督参数时保持兼容。详情见 [实机验证与修复记录](../docs/live_demo_verification_20260927.md)。
