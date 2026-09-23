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
一次有效观测确定本批目标，处理过程中不重新识别。默认目标类别仍为 `bowl`，不能保证识别任意类别物体。

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
每个目标完整完成接近 → 开爪 → 下降 → 闭爪 → 抬升 → 固定放置点 → 下降 → 等待 1 秒 → 开爪 → 退回，然后处理下一目标。
放置位置和高度仍固定；接收区域须能容纳连续放下的物体。

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

默认输出路径相对当前终端目录，也可通过 `--output-dir` 指定。每次保存三份
`effective_vision_config.json`、`effective_rim_config.json`、`effective_grasp_config.json`；
`result.json` 记录 `detected_count`、`target_count`、`completed_count`、拒绝原因和配置快照；`mode` 表示预览或执行，`outcome=no_targets` 表示本次观察为 0 个。

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
