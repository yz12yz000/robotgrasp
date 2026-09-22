# 硬件准备、手柄和抓取位置控制

抓取已改用 **MoveIt OMPL + Cartesian 路径 + scaled_joint_trajectory_controller**，无需 Pilz。
`run_grasp_position_control.sh` 负责就绪检查和控制器切换；`run_grasp_executor.sh` 负责规划和抓取。
默认只预览，明确加 `--execute` 才会执行。视觉模块新增可选桌面区域识别；定位算法保持原样。
完整后端说明、参数和测试见 [hardware_adapter.md](../hardware_adapter.md)；
本次修改文件清单和实测结果见 [moveit_grasp_test_report.md](moveit_grasp_test_report.md)。

## 1. 硬件准备

关闭原 `arm_base_joy_claw_lidar_camcolor0917.sh` 启动的程序，避免重复节点。
在工作区运行：

```bash
cd /home/rob/robot_grasp_ws
./run_robot_prepare.sh
```

启动 UR5e、原有两条静态 TF、MoveIt/RViz、夹爪、Gemini2L、Mid-360。
沿用 `mobile_ur_description` 模型、`mobile_ur.srdf.xacro` 和现有 OMPL 配置。
不启动 Servo、F710，也不检查手柄设备。
UR 驱动使用 `activate_joint_controller:=false`，运动控制器加载为 inactive；
关节状态、IO 等 broadcaster 正常启用。

底盘驱动仍需单独提供 `odom -> base_footprint`，脚本只提供
`world -> odom` 和 `base_footprint -> mobile_base_link`。
本脚本不会运行 `init.sh` 中的键盘控制，也不重复启动底盘驱动。
默认夹爪使用实测有回复的 FTDI 设备：
`/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_00000000-if00-port0`（本次对应 `/dev/ttyUSB0`，夹爪 ID 3）。
不再默认连接 `/dev/ttyUSB1`。串口不存在或已占用时脚本会拒绝启动；
更换 USB 转接器后，用 `GRIPPER_PORT` 指定实际设备。修改只在下次启动预备脚本时生效。

环境变量示例（将路径替换为实际设备）：

```bash
GRIPPER_PORT=/dev/serial/by-id/实际夹爪设备 ./run_robot_prepare.sh
# 如此次不需要雷达或 RViz：
START_LIVOX=false LAUNCH_RVIZ=false ./run_robot_prepare.sh
```

可选开关 `START_GRIPPER`、`START_ORBBEC`、`START_LIVOX`、`LAUNCH_RVIZ` 默认 true。
`ROBOT_IP` 默认 `192.168.10.102`；`GRIPPER_BAUDRATE` 默认 `115200`。
`BASE_OFFSET_X/Y/Z/YAW` 默认 0，与原脚本一致。
相机使用本项目 `tools/grasp_camera.launch.py`：1280×800、15 FPS、系统采集时间戳、
SENSOR_DATA QoS；关闭未使用的红外和彩色点云，保留对齐后的有序深度点云。
启动检查要求至少两组新鲜、时间戳更新且相差不超过 50 ms 的 RGB/点云。
RViz 如需订阅图像/点云，Reliability 选择 Best Effort。
雷达复用原 MID360 JSON 和 PointCloud2 配置，并等待 `/livox/lidar` 数据，
使用上面的 MoveIt/RViz 窗口，不再强制打开第二个雷达 RViz。

终端持续运行；各组件日志路径会打印在终端，默认位于用户运行目录的
`robot-grasp-<ROS_DOMAIN_ID>/logs/` 下。无图形终端也可以启动（关闭 RViz）。
任一受管启动进程退出会结束准备会话并清理本次进程。

## 2. 选择抓取位置控制

在示教器准备好机器人并启动 External Control 后，另开终端：

```bash
cd /home/rob/robot_grasp_ws
./run_grasp_position_control.sh
```

该脚本检查：

- 控制器已加载且类型正确；MoveIt 模型、规划服务和执行 Action 可用。
- External Control 正在运行，机器人为 RUNNING，安全状态为 NORMAL 或 REDUCED。
- 六个关节的位置/速度反馈完整、时间戳新鲜，机械臂持续静止。
- `base_link -> tool0` 和 `base_link -> grasp_center` TF 有效且新鲜。
- 不存在抓取/手柄节点，没有 accepted、executing 或 canceling 的 Action 任务。

检查后停止已存在的闲置 Servo，严格停用已知冲突运动控制器，激活
`scaled_joint_trajectory_controller` 并核对结果。未知控制器占用机械臂关节时拒绝切换。
不会自动启动 External Control，不会发送轨迹或抓取目标，不会自动启动抓取节点。
成功后脚本退出，控制器保持 active；再次执行只检查现有状态，不重复激活。

## 3. 选择手柄控制

```bash
cd /home/rob/robot_grasp_ws
./run_joystick_control.sh
# 多个设备或无法自动识别时，指定实际手柄：
JOYSTICK_DEVICE=/dev/input/js1 ./run_joystick_control.sh
```

自动检测唯一的 Logitech 手柄，复用运行中的 MoveIt 的 URDF/SRDF 和规划参数，
单独启动 `servo_node_main`，不会再启动 MoveIt/RViz/UR 驱动。
切换到 `forward_position_controller`，等待 Servo 启动服务和状态码 0 后启动 F710。
保留现有 F710 节点的按钮功能，包括 Y 控制夹爪、X 调用原有回零脚本；
回零脚本仍须由现有手柄工作区提供。

该终端持续持有模式锁，位置控制脚本此时会拒绝切换。
Ctrl+C 停止本次启动的手柄和 Servo，并停用手柄使用的 forward/轨迹控制器，
再释放锁。随后可执行位置控制脚本。
从位置模式进入手柄模式前，先退出抓取节点及其他运动任务；即使抓取节点
已经报告完成，只要该节点仍存在，脚本也会要求先关闭它。

## 4. 检查、退出和环境

三个脚本均支持 `--help` 和 `--check`。
`--check` 不启动硬件节点、不调用 Servo 启停、不切换控制器；
两个模式脚本的检查需要现有硬件/MoveIt 正常运行。

建议先退出手柄或抓取任务，最后在准备终端 Ctrl+C。
清理只针对脚本创建的进程组，日志保留；不会用全局 `pkill` 清理其他程序。
准备脚本本身有独立进程锁，手柄和位置切换共用模式锁；这些锁不能约束用户
在外部直接发布命令或调用控制器服务。

默认环境加载顺序：`/opt/ros/humble` → `~/ws_moveit2` → `~/ur_ros2_ws` → `~/ur_sim_ws`。
可分别用 `MOVEIT_WS`、`UR_DRIVER_WS`、`UR_MODEL_WS` 指定路径；外设工作区用
`GRIPPER_WS`、`ORBBEC_WS`、`LIVOX_WS`、`JOYSTICK_WS` 指定。
保持所有终端的 ROS_DOMAIN_ID 和 DDS 设置一致。当前支持无关节前缀的单 UR5e，
控制器管理器为 `/controller_manager`。`CHECK_TIMEOUT` 默认每项等待 30 秒。
三个准备/模式脚本直接运行。抓取 Python 包修改后需重新构建，见下一节。

脚本与辅助程序检查的是接口、反馈和模式就绪，不验证相机/TCP 标定精度，
也不声明现有抓取算法已经完成真机验收。


## 5. 抓取预览和执行顺序

更新后先编译一次：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/setup.bash
source ~/ur_ros2_ws/install/setup.bash
source ~/ur_sim_ws/install/setup.bash
cd /home/rob/robot_grasp_ws
colcon build --symlink-install --packages-select grasp_executor
source install/setup.bash
```

按以下顺序，每个持续运行的节点单独开终端：

1. `./run_robot_prepare.sh`，等硬件、MoveIt 和 TF 就绪。
2. 在示教器启动 External Control，运行 `./run_grasp_position_control.sh`。
3. `./run_grasp_executor.sh`，默认只规划，等待目标。
4. `./run_rim_locator.sh`，开始等待本次识别数据。
5. `./run_yolo_vision.sh`，生成本次新目标。

等 `/grasp_executor/status` 报告 `planned`，在 RViz 的 MotionPlanning → Planned Path
查看 `/display_planned_path`：接近、竖直下降、竖直抬升三段。仅显示 approach/grasp
两个 Pose 箭头不代表规划已完成。默认预览不会打开夹爪或发送轨迹 Action。

确认预览后，退出抓取、定位、YOLO 节点，再按上面顺序启动本次任务，第三步改为：

```bash
./run_grasp_executor.sh --execute
```

执行模式也会重新规划并验证全部三段，然后接近 → 打开 → 下降 → 关闭 → 抬升。
节点一次只处理一个任务，完成/失败后保持结果，下一次需重启；重试重新识别定位。
两个模式互斥，抓取节点开始检查后会持有与手柄相同的模式锁，退出后释放。
不要在抓取节点运行时再次执行位置切换脚本。

静止物体的 `max_target_age` 和 `input_timeout` 现为 **600 秒（10 分钟）**。
仍检查原始时间戳、坐标系、未来时间戳，并在规划完成后再次检查有效期；不会自动刷新旧目标时间。
物体、相机和底盘相对位置必须保持不变。已尝试执行的目标写入本地日志，失败也不会自动复用；
纯预览不占用目标执行记录。推荐始终按上述顺序采集新目标。

目前 `table_enabled=false`，没有人为添加未经测量的桌面模型，也不要求必须存在世界障碍物。
MoveIt 自碰撞和已有场景检查保持开启；未建模桌面不会被检测到。
夹爪沿用现有 Open/Close 命令并等待 1 秒；`success` 表示动作流程完成，
状态中的 `grasp_verified=false` 明确表示尚无夹持成功的传感器反馈。


## 6. 一条命令测试三个模块（只预览）

硬件准备完成后，关闭已运行的 YOLO、定位、抓取节点，再运行：

```bash
./run_pipeline_test.sh
```

脚本先检查相机和 MoveIt，再按“抓取等待 → 定位等待 → YOLO”启动；
打印各模块状态，并把日志及 `result.json` 保存到 `debug_output/live_validation/`。
结束后只关闭本次三个项目节点，保留硬件。测试入口不激活运动控制器，也没有执行模式。
目标被拒绝、等待超时会给出失败结果，不会永久等下去。

当前现场的绿色碗在全图缩小到 512 后容易漏检。当前相机视角已验证的命令是：

```bash
./run_pipeline_test.sh --vision-config src/yolo_vision/config/table_roi.example.json
```

该示例限制原图 `[x0,y0,x1,y1]=[320,350,740,550]`，相机或桌面移动后须重新确认区域。
默认 `config.json` 仍使用全图，未自动改为裁剪。裁剪结果会还原到原图像素坐标，
再选取点云，不改变 TF 或目标时间戳。此区域不是桌面碰撞模型。
本次测试结果见 [pipeline_validation_20260921.md](../pipeline_validation_20260921.md)。

## 7. 一条命令执行真实抓取

先启动 `./run_robot_prepare.sh` 并等准备完成，在示教器运行 External Control。
关闭其他预览、YOLO、定位、抓取节点，然后运行：

```bash
./run_pipeline_execute.sh
```

该脚本默认采用本次已验证的 `src/yolo_vision/config/table_roi.example.json`，依次：
检查相机和 MoveIt → 调用位置控制脚本 → 启动抓取执行等待 → 定位等待 → YOLO。
三段规划通过后自动接近、开爪、下降、闭爪、抬升，不再询问确认。
`success` 表示动作流程完成，仍不代表已通过传感器确认抓住物体。

| 区别 | `run_pipeline_test.sh` | `run_pipeline_execute.sh` |
|---|---|---|
| 机械臂/夹爪动作 | 不执行 | 执行 |
| 激活位置控制器 | 不会 | 自动调用位置控制脚本 |
| 需要 External Control | 预览不需要 | 必须已运行 |
| 默认识别区域 | 全图，可传桌面配置 | 当前桌面示例配置 |
| 完成状态 | `planned` | `success` |
| 默认整体等待超时 | 240 秒 | 600 秒 |
| 默认日志 | `debug_output/live_validation/` | `debug_output/live_execution/` |

先使用相同桌面配置预览，再执行：

```bash
./run_pipeline_test.sh --vision-config src/yolo_vision/config/table_roi.example.json
# 预览结束、确认路径后，再运行：
./run_pipeline_execute.sh
```

两个入口均不重复启动硬件，不允许已有项目节点同时运行。完成或失败后只清理本次三个节点，
硬件保留，执行脚本激活的位置控制器保持 active。Ctrl+C 会请求抓取节点取消任务，
等待其停止流程；若必须强制退出，结果标记 `forced_shutdown`，不能据此认定机器人已停止。
不会自动重试或复用失败目标。

可用 `--vision-config /路径/配置.json` 指定区域配置，`--timeout 600` 设置整体等待上限，
`--output-dir /路径` 指定日志目录。整体超时不会覆盖各节点自己的规划、动作或输入超时。
桌面区域配置仅适用于当前视角，不是碰撞模型；首次真机抓取仍需现场验证。

此入口已通过 7 项不连接硬件的编排测试（启动顺序、两种完成判定、取消清理、重复节点拒绝、
控制器准备失败阻断、参数拒绝），以及 Bash/Python 语法检查；本次未运行真实执行入口。
