# 项目启动说明

本文档保留视觉和碗沿定位的使用方法。完整硬件准备、MoveIt 抓取预览和执行流程见 [docs/0921/startup_scripts.md](docs/0921/startup_scripts.md)。

## 1. 打开终端并准备 ROS 环境

在每个新终端中执行：

```bash
cd /home/rob/robot_grasp_ws
source /opt/ros/humble/setup.bash
source /home/rob/robot_grasp_ws/install/setup.bash
```

如果源码有更新，先重新编译：

```bash
cd /home/rob/robot_grasp_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
```

## 2. 启动相机

打开终端 A，启动 Orbbec Gemini2L。相机必须发布 RGB、深度和注册点云：

```bash
source /opt/ros/humble/setup.bash
source /home/rob/orbbect_ws/install/setup.bash  # 如本机相机工作区路径不同，请改成实际路径

ros2 launch /home/rob/robot_grasp_ws/tools/grasp_camera.launch.py
```

保持该终端运行。可用下面命令确认相机已经发布数据：

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/points
```

## 3. 启动 yolo_vision

打开终端 B，执行：

```bash
cd /home/rob/robot_grasp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch yolo_vision yolo_vision.launch.py
```

该节点读取：

- `/camera/color/image_raw`
- `/camera/depth/points`

并发布目标点云、mask 和 overlay。默认使用包内模型
`src/yolo_vision/yolo26n-seg_openvino_model/`，目标类别为 `bowl`。

## 4. 启动 rim_locator

打开终端 C，执行：

```bash
cd /home/rob/robot_grasp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch rim_locator rim_locator.launch.py
```

它订阅 `/yolo_vision/object_cloud`，通过 TF 转换到 `base_link`，然后发布：

```text
/rim_locator/rim_point
/rim_locator/base_cloud
/rim_locator/rim_candidates
/rim_locator/local_points
```

确认两个节点运行：

```bash
ros2 node list | grep -E 'yolo|rim'
ros2 topic echo /rim_locator/rim_point --qos-durability transient_local
```

## 5. 本次已验证的结果

一次成功运行的结果保存在：

```text
/home/rob/robot_grasp_ws/debug_output/live/20260918_151517
```

其中包括 `rgb.png`、`mask.png`、`overlay.png`、`object_cloud.npy`、
`base_cloud.npy`、`rim_candidates.npy`、`local_points.npy` 和 `result.json`。

本次识别到 `bowl`，置信度约为 `0.276965`；定位结果坐标系为 `base_link`，
目标点约为 `[0.644885, 0.255071, -0.019341]`。

## 6. 一键启动感知和定位（不启动抓取）

相机已经启动后，也可以只用一个 launch 启动感知和定位：

```bash
cd /home/rob/robot_grasp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch grasp_executor pipeline.launch.py enable_grasp:=false
```

`enable_grasp:=false` 很重要，它不会启动 `grasp_executor`，因此不会向机械臂发送动作。

## 7. 机械臂抓取

见 [新的启动顺序](docs/0921/startup_scripts.md#5-抓取预览和执行顺序)。
`./run_grasp_executor.sh` 默认只规划，`./run_grasp_executor.sh --execute` 才执行。
`pipeline.launch.py` 默认不启用抓取；即使 `enable_grasp:=true`，默认也是 `plan_only:=true`。
推荐分终端启动“硬件和控制器 → 抓取/定位等待 → YOLO”，便于保证是本次的新目标。

## 8. 停止

在各个 ROS 终端按 `Ctrl+C`，建议顺序为：先停止项目节点，再停止相机。确认没有机械臂动作后再关闭机器人驱动。

## 9. 常见检查

```bash
ros2 topic list | grep -E 'camera|yolo_vision|rim_locator'
ros2 topic echo /yolo_vision/status --qos-durability transient_local
ros2 topic echo /rim_locator/status --qos-durability transient_local
```

如果视觉节点提示没有输入，先检查相机是否发布 `/camera/color/image_raw` 和 `/camera/depth/points`，以及相机点云是否开启注册对齐；如果定位提示 TF 错误，检查 `camera_color_optical_frame` 到 `base_link` 的 TF 是否存在。
