# grasp_executor 第一版设计说明

## 1. 项目定位

`grasp_executor` 是 `robot_grasp_ws` 中负责执行抓取动作的 ROS2 Python package。

第一版只完成一条固定抓取流程：

> 接收 `rim_locator` 发布的 `rim_point`，根据固定偏移计算抓取点，根据固定接近高度计算预抓点，使用已标定的固定朝下姿态，依次执行“移动到预抓点 → 打开夹爪 → 竖直下降到抓取点 → 关闭夹爪”。

第一版不负责视觉识别、点云处理、抓取点搜索，也不增加抬升、搬运、投放等动作。

---

## 2. 整体 Workspace 结构

三个功能模块放在同一个 ROS2 workspace 中：

```text
robot_grasp_ws/
├── src/
│   ├── yolo_vision/
│   │   └── ...
│   │
│   ├── rim_locator/
│   │   └── ...
│   │
│   └── grasp_executor/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── grasp_executor
│       ├── config/
│       │   └── config.json
│       └── grasp_executor/
│           ├── __init__.py
│           ├── main.py
│           ├── grasp.py
│           └── interfaces.py
│
├── build/
├── install/
└── log/
```

其中：

```text
main.py        → ROS2 通信和任务入口
grasp.py       → 抓取流程与位姿计算
interfaces.py  → 对接机械臂运动接口和夹爪命令
```

---

## 3. 输入

订阅：

```text
/rim_locator/rim_point
```

消息类型：

```text
geometry_msgs/msg/PointStamped
```

要求：

```text
header.frame_id = base
```

或现场实际使用的机器人基坐标系名称。

例如：

```text
point.x = 0.518
point.y = -0.162
point.z = 0.824
```

该点作为当前目标的视觉定位点。

---

## 4. 抓取点和接近点定义

### 4.1 抓取点

定义：

```text
抓取点 = base 中的 rim_point + 夹持偏移
```

写成：

```text
grasp_point.x = rim_point.x + offset_x
grasp_point.y = rim_point.y + offset_y
grasp_point.z = rim_point.z + offset_z
```

第一版把偏移放在配置文件中，根据实际夹爪 Grab Center 与目标接触位置标定。

---

### 4.2 接近点

定义：

```text
接近点 = 抓取点 + 竖直向上的接近高度
```

即：

```text
approach_point.x = grasp_point.x
approach_point.y = grasp_point.y
approach_point.z = grasp_point.z + approach_height
```

因此最后一段运动始终是：

```text
沿 base 坐标系 Z 方向竖直向下
```

---

### 4.3 姿态

第一版不实时计算姿态。

直接使用已经标定好的固定朝下姿态：

```text
orientation =
[qx, qy, qz, qw]
```

该固定姿态同时用于：

```text
approach_pose
grasp_pose
```

两者只改变位置，不改变姿态。

---

## 5. 第一版完整流程

```text
接收 /rim_locator/rim_point
        ↓
检查 frame_id 和坐标有效性
        ↓
rim_point + grasp_offset
        ↓
得到 grasp_point
        ↓
grasp_point + approach_height
        ↓
得到 approach_point
        ↓
加入固定朝下姿态
        ↓
approach_pose
grasp_pose
        ↓
移动机械臂到 approach_pose
        ↓
确认到位
        ↓
打开夹爪
        ↓
确认打开命令成功
        ↓
沿竖直方向直线移动到 grasp_pose
        ↓
确认到位
        ↓
关闭夹爪
        ↓
确认关闭命令成功
        ↓
返回本次抓取执行结果
```

第一版到“关闭夹爪”为止。

不加入：

```text
抬升
搬运
投放
重新识别
失败后自动重试
```

---

## 6. Python 文件职责

### `main.py`

负责：

- 初始化 ROS2 node。
- 读取 `config.json`。
- 订阅 `/rim_locator/rim_point`。
- 接收第一条有效 `PointStamped`。
- 检查输入坐标系。
- 调用 `grasp.py` 执行一次抓取。
- 输出成功或失败状态。

第一版采用 one-shot：

> 一次任务只处理一个 `rim_point`。

---

### `grasp.py`

只负责抓取流程和位姿生成。

主要函数可以设计为：

```python
build_grasp_pose(rim_point, config)
build_approach_pose(grasp_pose, config)
execute_grasp(approach_pose, grasp_pose, interfaces)
```

其中：

```text
build_grasp_pose
```

完成：

```text
rim_point + grasp_offset + fixed_orientation
```

`build_approach_pose` 完成：

```text
grasp_pose + approach_height
```

`execute_grasp` 固定执行：

```text
move_to_pose(approach_pose)
        ↓
open_gripper()
        ↓
move_linear(grasp_pose)
        ↓
close_gripper()
```

---

### `interfaces.py`

只负责与现场实际硬件接口连接。

第一版只需要 6 个最小接口：

```text
move_to_pose(pose)
move_linear(pose)
is_pose_reached()
stop_motion()

open_gripper()
close_gripper()
```

`grasp.py` 不直接关心这些接口内部如何实现。

---

## 7. 夹爪控制

当前夹爪已经可以通过：

```bash
source ~/handControl2_ws/install/setup.bash
ros2 run hand_control talker Open 1
```

打开。

关闭命令：

```bash
source ~/handControl2_ws/install/setup.bash
ros2 run hand_control talker Close 1
```

第一版可以在 `interfaces.py` 中直接调用 shell 命令。

例如逻辑：

```text
open_gripper()
    ↓
bash -lc "
source ~/handControl2_ws/install/setup.bash &&
ros2 run hand_control talker Open 1
"
```

关闭同理：

```text
close_gripper()
    ↓
bash -lc "
source ~/handControl2_ws/install/setup.bash &&
ros2 run hand_control talker Close 1
"
```

程序检查：

```text
进程返回码
+
超时
```

判断命令是否正常执行。

第一版不额外修改现有 `hand_control` package。

---

## 8. 机械臂运动接口

当前设计不预设具体机械臂控制实现。

`interfaces.py` 只提供统一接口：

```python
move_to_pose(approach_pose)
move_linear(grasp_pose)
```

其中：

### 到接近点

使用普通位姿运动：

```text
当前位置
   ↓
approach_pose
```

### 最后一段

必须使用直线运动：

```text
approach_pose
      ↓
沿两点连线
      ↓
grasp_pose
```

由于两点：

```text
X 相同
Y 相同
orientation 相同
```

只改变：

```text
Z
```

因此这一段就是竖直向下。

具体运动命令由现场已有机械臂接口接入，不在第一版中重复实现轨迹规划器。

---

## 9. 与 `rim_locator` 的 ROS2 通信

数据链：

```text
rim_locator
        ↓
/rim_locator/rim_point
        ↓
grasp_executor
```

Topic：

```text
/rim_locator/rim_point
```

消息类型：

```text
geometry_msgs/msg/PointStamped
```

建议 QoS：

```text
Reliability: RELIABLE
Durability: TRANSIENT_LOCAL
Depth: 1
```

与 `rim_locator` 输出端保持一致。

这样 `rim_point` 即使先发布，稍后启动的 `grasp_executor` 仍可获得最后一个目标点。

---

## 10. 配置文件

`config/config.json` 第一版建议：

```json
{
  "input_topic": "/rim_locator/rim_point",
  "base_frame": "base",

  "grasp_offset": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  },

  "approach_height": 0.10,

  "fixed_orientation": {
    "x": 0.0,
    "y": 1.0,
    "z": 0.0,
    "w": 0.0
  },

  "motion_timeout": 15.0,
  "gripper_timeout": 5.0
}
```

注意：

```text
fixed_orientation
```

中的四元数这里只表示配置形式。

实际数值必须替换成现场已经标定好的固定朝下姿态，不能直接使用示例值。

`grasp_offset` 同样由实际 Grab Center 与目标接触位置关系确定。

---

## 11. 任务检查

执行前至少检查：

```text
1. rim_point 的 frame_id 是否等于 base_frame
2. x / y / z 是否为有限数值
3. approach_height 是否大于 0
4. fixed_orientation 是否已经配置
5. 机械臂接口是否可用
6. 夹爪控制命令是否可用
```

如果不满足，停止任务，不执行机械臂运动。

---

## 12. 执行状态

第一版建议只保留简单状态：

```text
waiting_target
moving_to_approach
opening_gripper
moving_to_grasp
closing_gripper
success
failed
```

失败时记录：

```text
failed_step
reason
```

例如：

```json
{
  "status": "failed",
  "failed_step": "moving_to_grasp",
  "reason": "motion_timeout"
}
```

---

## 13. 安全与失败处理

第一版遵循：

> 上一步成功后才执行下一步。

如果：

```text
移动超时
运动接口报错
夹爪命令失败
输入数据无效
```

则：

```text
停止当前流程
调用 stop_motion()
不继续下一步
```

第一版不自动重试。

---

## 14. 第一版最终数据链

```text
/rim_locator/rim_point
        ↓
PointStamped in base
        ↓
grasp_offset
        ↓
grasp_point
        ↓
+ approach_height
        ↓
approach_point
        ↓
fixed downward orientation
        ↓
approach_pose + grasp_pose
        ↓
移动到 approach_pose
        ↓
Open 1
        ↓
直线下降到 grasp_pose
        ↓
Close 1
        ↓
抓取流程结束
```

---

## 15. 第一版实现原则

1. `grasp_executor` 与 `yolo_vision`、`rim_locator` 放在同一个 `robot_grasp_ws`，但保持独立 ROS2 package。
2. 第一版只处理一个 `rim_point`。
3. 抓取位置由 `rim_point + grasp_offset` 得到。
4. 接近位置由 `grasp_point + approach_height` 得到。
5. 接近点和抓取点使用同一个固定朝下姿态。
6. 最后一段必须是竖直直线下降。
7. 到达接近点后再打开夹爪。
8. 到达抓取点后再关闭夹爪。
9. 夹爪直接复用现有 `hand_control` 命令。
10. 不在本模块重新计算视觉、点云或抓取点。
11. 不加入抬升、搬运、投放和自动重试。
12. 第一版优先保证：
    - rim_point 能正确接收；
    - approach_pose / grasp_pose 计算正确；
    - 机械臂两段运动顺序正确；
    - Open / Close 命令正确执行。
