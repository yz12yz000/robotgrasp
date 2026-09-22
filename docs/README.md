# Robot Grasp Workspace

三个独立 ROS 2 Python 包，第一版针对 **一个碗、一个目标点、一次抓取**。目标环境为 Linux / ROS 2 Humble。当前 Windows 环境只做源码和离线测试，不执行构建或机器人命令。

```text
RGB + 注册有序点云
  -> yolo_vision：一次分割、目标点云
  -> rim_locator：TF、顶部近侧定位
  -> grasp_executor：接近、打开、竖直下降、关闭
```

三个原始设计说明保留。抓取包现已接入用户 Word 文档中的真实位置控制接口，正式代码不再包含模拟抓取后端。

## 三个独立包

| 包 | 职责 | 独立调试 |
|---|---|---|
| yolo_vision | RGB + organized cloud -> object_cloud | 相机/录制输入，或保存的图像、XYZ、mask |
| rim_locator | object_cloud + TF -> rim_point | 保存的目标点云，或已在基座系的 NumPy 点数组 |
| grasp_executor | 单目标点 -> 位置控制器 + hand_control | 纯位姿预览，或现场确认目标点后的实机单次执行 |

每个包有自己的配置、launch、安装声明，不互相导入业务代码。点云编解码器各自保留以维持独立性，检查脚本验证两份实现一致。

## 实机接口与 TF

控制接口来自 `ROS2+UR_v5.docx` 的位置控制部分：

```text
/cartesian_motion_controller/target_frame
geometry_msgs/msg/PoseStamped
header.frame_id = base_link
```

实际坐标系名称来自仓库中的 `frames_2026-09-14_15.25.33(1).pdf`：

```text
base_link -> base -> camera_link -> ... -> camera_color_optical_frame
base_link -> base_link_inertia -> ... -> flange -> tool0
                                           -> gripper_base_link -> grasp_center
```

抓取中心是 **grasp_center**。程序通过控制器参数读取实际 `end_effector_link`，再利用已有工具 TF 换算目标；不会直接把 grasp_center 的 XYZ 当作 tool0 的目标。

详细的启动、标定、反馈和停止机制见 [实机接入说明](docs/hardware_adapter.md)。

现场还需要：已有 YOLO26n-Seg OpenVINO 模型、相机和机器人驱动、对应 TF、已激活的位置控制器，以及真实抓取姿态/偏移。这里没有 mock/real 切换；启动执行节点后就是实机执行。配置中的姿态、偏移和工作空间必须由现场人员自行确认。

## ROS 启动

以下命令仅供现场已有 ROS 环境使用；本次没有执行构建或这些机器人命令。

```bash
ros2 launch yolo_vision yolo_vision.launch.py
ros2 launch rim_locator rim_locator.launch.py
ros2 launch grasp_executor grasp_executor.launch.py config_path:=/absolute/path/grasp.json
```

总启动：

```bash
# 包含实机抓取节点，需完成标定配置
ros2 launch grasp_executor pipeline.launch.py
# 只运行感知和定位
ros2 launch grasp_executor pipeline.launch.py enable_grasp:=false
```

总启动可指定 `yolo_vision_config`、`rim_locator_config`、`grasp_executor_config`。相机、UR 驱动和 TF 由现场独立启动。实机执行器拒绝 `use_sim_time=true`；感知/定位回放可以用 `enable_grasp:=false use_sim_time:=true`。

ROS 依赖在各包的 package.xml 中声明；视觉还需要与现有模型兼容的 Ultralytics/OpenVINO。模型放到 `src/yolo_vision/models/`，或配置外部绝对路径。不会自动下载模型。

## 无 ROS 调试

先在 workspace 根目录设置 Python 搜索路径。

Linux：

```bash
export PYTHONPATH="$PWD/src/yolo_vision:$PWD/src/rim_locator:$PWD/src/grasp_executor${PYTHONPATH:+:$PYTHONPATH}"
```

PowerShell：

```powershell
$env:PYTHONPATH = "$PWD/src/yolo_vision;$PWD/src/rim_locator;$PWD/src/grasp_executor"
```

视觉输入为注册对齐的 H×W×3 XYZ 数组和 H×W 布尔 mask：

```bash
python -m yolo_vision.debug --image rgb.png --xyz organized_xyz.npy --mask mask.npy
# 需要实际模型时改用 --model /absolute/path/openvino_model
```

定位输入为已在 target_frame 下的 N×3 数组；不传文件时使用几何测试点：

```bash
python -m rim_locator.debug
python -m rim_locator.debug --points base_points.npy
```

抓取只预览计算位姿，不模拟动作、不调用硬件：

```bash
python -m grasp_executor.debug --point 0.518 -0.162 0.824
# ROS 包安装后的同一纯计算入口
ros2 run grasp_executor preview_grasp --point 0.518 -0.162 0.824
```

## ROS 下独立测试

- 视觉：只启动相机和视觉包，看 mask、overlay、object_cloud。
- 定位：启动 rim_locator，再用 `ros2 run rim_locator publish_object_cloud` 提供 base_link 下的几何点；也可用 `--points object_points.npy --frame camera_color_optical_frame` 并提供 TF。
- 实机抓取：完成标定和控制器准备后启动 grasp_executor，再用 `ros2 run grasp_executor publish_rim_point --point X Y Z --frame base_link` 发布现场确认的目标。这会触发实际动作；只检查计算请用 preview_grasp。

点云文件发布工具赋当前时间戳，不变换文件坐标；运动相机历史数据应回放包含原始时间戳和 TF 的 rosbag。

## 输出与任务约定

| Topic | 含义 |
|---|---|
| /yolo_vision/object_cloud | 相机坐标系的目标点云，保留采集时间 |
| /rim_locator/rim_point | base_link 下的单目标点，保留观测时间 |
| /yolo_vision/mask、/yolo_vision/overlay | mask 和叠加图 |
| /rim_locator/base_cloud、rim_candidates、local_points | 几何筛选中间结果 |
| /grasp_executor/approach_pose、grasp_pose | grasp_center 的目标位姿 |
| /各包名/status | JSON 状态、失败步骤和原因 |

感知/定位结果使用 RELIABLE / TRANSIENT_LOCAL / Depth 1，完成后节点保持存活。机械臂命令使用 VOLATILE，不为晚加入的控制器保留旧目标命令。

```bash
ros2 topic echo /grasp_executor/status std_msgs/msg/String --qos-reliability reliable --qos-durability transient_local
```

每次启动只锁定一份输入，无抬升、投放、多目标循环或失败重试。去重拒绝、标定失败和预检查失败不会停止机械臂；本实例尝试动作后失败才进入停止流程。

基座 Z 必须向上。顶部定位保留原 MD 的高度下限算法，高处杂点仍可能影响结果，本次没有更换 rim 算法。夹爪命令成功也不等于已通过传感器确认碗夹牢。

## 检查

```bash
python -B tools/check_source.py
python -B -m unittest discover -s tests -v
```

假接口只存在于 tests，不随业务包安装。检查结果见 [verification.md](docs/verification.md)。源码检查和离线测试不能代替现场 ROS、模型与机械臂联调。
