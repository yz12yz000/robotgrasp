#!/usr/bin/env python3
"""Startup checks and controller ownership for the three shell entry points.

No motion goal or command publisher is created here. ROS imports stay inside
RosProbe so controller selection and feedback validation can be tested offline.
"""

import argparse
from collections import Counter, deque
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


JOINTS = (
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
)
MOTION_CONTROLLERS = {
    "scaled_joint_trajectory_controller", "joint_trajectory_controller",
    "forward_position_controller", "forward_velocity_controller",
    "cartesian_motion_controller", "cartesian_compliance_controller",
    "cartesian_force_controller", "force_mode_controller",
    "freedrive_mode_controller", "passthrough_trajectory_controller",
}
CONTROLLER_TYPES = {
    "scaled_joint_trajectory_controller": "ur_controllers/ScaledJointTrajectoryController",
    "forward_position_controller": "position_controllers/JointGroupPositionController",
}
BUSY_GOAL_STATES = {1, 2, 3}  # accepted, executing, canceling


def camera_pair_ready(images, clouds, now_ns):
    """Require two fresh, advancing, aligned RGB/cloud pairs, not just topics.

    Callers may append the receipt time; both source and receipt must be fresh.
    """
    def usable(rows):
        return [r for r in rows if r[0] > 0 and
                -0.1 <= (now_ns-r[0])/1e9 <= 10.0 and
                -0.1 <= (now_ns-(r[4] if len(r) > 4 else r[0]))/1e9 <= 10.0
                and r[1:3] == (1280, 800) and r[3]]
    # Pixel indexing requires depth registered to the RGB optical frame.
    pairs = [(a[0], b[0]) for a in usable(images) for b in usable(clouds)
             if a[3] == b[3] and abs(a[0]-b[0]) <= 100_000_000]
    return any(a[0] < b[0] and a[1] < b[1] for a in pairs for b in pairs)


@dataclass
class Controller:
    name: str
    state: str
    type: str
    claimed_interfaces: tuple = ()


def active_motion(controllers):
    active = []
    for controller in controllers:
        if controller.state != "active":
            continue
        if controller.name in MOTION_CONTROLLERS:
            active.append(controller.name)
        elif any(interface.split("/")[0] in JOINTS for interface in controller.claimed_interfaces):
            raise RuntimeError(f"未知控制器占用机械臂关节，拒绝自动切换：{controller.name}")
    return active


def switch_plan(controllers, target):
    selected = next((c for c in controllers if c.name == target), None)
    if selected is None or selected.state not in ("inactive", "active"):
        raise RuntimeError(f"{target} 尚未加载并配置，请先完成 run_robot_prepare.sh")
    if selected.type != CONTROLLER_TYPES[target]:
        raise RuntimeError(f"{target} 类型不符：{selected.type}")
    deactivate = [name for name in active_motion(controllers) if name != target]
    activate = [] if selected.state == "active" else [target]
    return activate, deactivate


def joint_values(message, now_ns):
    stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    age = (now_ns - stamp) / 1e9
    if stamp <= 0 or not -0.2 <= age <= 1.0:
        raise ValueError("/joint_states 时间戳过期或时钟不同步")
    if len(message.name) != len(set(message.name)):
        raise ValueError("/joint_states 包含重复关节名称")
    if len(message.position) != len(message.name) or len(message.velocity) != len(message.name):
        raise ValueError("/joint_states 缺少完整位置/速度反馈")
    indices = [message.name.index(name) for name in JOINTS]
    position = tuple(message.position[i] for i in indices)
    velocity = tuple(message.velocity[i] for i in indices)
    if not all(math.isfinite(v) for v in position + velocity):
        raise ValueError("/joint_states 含无效数值")
    if max(abs(v) for v in velocity) > 0.02:
        raise ValueError("机械臂仍在运动，拒绝切换模式")
    return stamp, position


def position_window_stationary(samples, stamp, position):
    """Check position range over >=100 ms; adjacent 500 Hz deltas amplify noise.

    Reported joint velocity is independently checked by joint_values. Use the
    full range, not endpoint displacement, so back-and-forth motion is visible.
    """
    if samples and stamp <= samples[-1][0]:
        raise ValueError("关节反馈时间戳未递增")
    samples.append((stamp, position))
    cutoff = stamp - 100_000_000
    while len(samples) > 2 and samples[1][0] <= cutoff:
        samples.popleft()
    span = (stamp - samples[0][0]) / 1e9
    if span < 0.1:
        return False
    ranges = [max(values) - min(values) for values in zip(*(p for _, p in samples))]
    if max(ranges) / span > 0.02:
        raise ValueError("关节位置仍在变化，拒绝切换模式")
    return True


class RosProbe:
    def __init__(self, timeout):
        import rclpy
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

        self.ros = rclpy
        self.node = rclpy.create_node("robot_startup_probe")
        self.timeout = timeout
        self.retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   reliability=ReliabilityPolicy.RELIABLE)
        self.messages = {}
        self.subscriptions = []
        self.clients = []
        self.still_since = None
        self.joint_receipt = 0.0
        self.joint_window = deque()
        self.joint_error = "尚未收到 /joint_states"
        self.buffer = None

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.ros.spin_once(self.node, timeout_sec=min(0.05, max(0, deadline - time.monotonic())))

    def wait(self, predicate, reason):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.ros.spin_once(self.node, timeout_sec=0.05)
        raise RuntimeError(f"等待超时：{reason}")

    def call(self, service_type, name, request=None):
        client = self.node.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout):
                raise RuntimeError(f"服务不存在：{name}")
            future = client.call_async(request if request is not None else service_type.Request())
            self.ros.spin_until_future_complete(self.node, future, timeout_sec=self.timeout)
            if not future.done():
                future.cancel()
                raise RuntimeError(f"服务超时（如为切换操作，其实际状态需重新检查）：{name}")
            result = future.result()
            if result is None:
                raise RuntimeError(f"服务无返回：{name}")
            return result
        finally:
            self.node.destroy_client(client)

    def controllers(self):
        from controller_manager_msgs.srv import ListControllers

        response = self.call(ListControllers, "/controller_manager/list_controllers")
        return [Controller(c.name, c.state, c.type, tuple(c.claimed_interfaces)) for c in response.controller]

    def node_names(self):
        names = self.node.get_node_names_and_namespaces()
        return [namespace.rstrip("/") + "/" + name for name, namespace in names]

    def absent(self, basenames):
        existing = [name for name in self.node_names() if name.rsplit("/", 1)[-1] in basenames]
        if existing:
            raise RuntimeError("以下节点已存在，请先退出对应程序，拒绝重复启动/抢占：" + ", ".join(existing))

    def params(self, node, names):
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python

        request = GetParameters.Request(names=names)
        result = self.call(GetParameters, node + "/get_parameters", request)
        if len(result.values) != len(names):
            raise RuntimeError(f"参数响应不完整：{node}")
        return dict(zip(names, (parameter_value_to_python(v) for v in result.values)))

    def action_ready(self, action_type, name):
        from rclpy.action import ActionClient

        client = ActionClient(self.node, action_type, name)
        self.clients.append(client)
        self.wait(client.server_is_ready, f"Action {name}")

    def planning_ready(self):
        from moveit_msgs.action import MoveGroup, ExecuteTrajectory

        params = self.params("/move_group", ["robot_description", "robot_description_semantic", "use_sim_time"])
        if params["use_sim_time"] is not False:
            raise RuntimeError("/move_group 必须使用真实时间 use_sim_time=false")
        for key in ("robot_description", "robot_description_semantic"):
            if not isinstance(params[key], str) or not params[key].strip():
                raise RuntimeError(f"/move_group 缺少 {key}")
        semantic = ET.fromstring(params["robot_description_semantic"])
        chain = semantic.find("./group[@name='ur_manipulator']/chain")
        if chain is None or chain.get("base_link") != "base_link" or chain.get("tip_link") != "tool0":
            raise RuntimeError("MoveIt 规划组必须为 ur_manipulator，链为 base_link -> tool0")
        self.action_ready(MoveGroup, "/move_action")
        self.action_ready(ExecuteTrajectory, "/execute_trajectory")
        for service, kind in (("/plan_kinematic_path", "GetMotionPlan"), ("/compute_ik", "GetPositionIK"),
                              ("/compute_fk", "GetPositionFK"), ("/compute_cartesian_path", "GetCartesianPath"),
                              ("/check_state_validity", "GetStateValidity"), ("/get_planning_scene", "GetPlanningScene")):
            self.wait(lambda: any(name == service and "moveit_msgs/srv/" + kind in types
                                 for name, types in self.node.get_service_names_and_types()),
                      service + " 规划服务")
        return params

    def subscribe(self, message_type, topic, retained=False):
        from rclpy.qos import qos_profile_sensor_data

        subscription = self.node.create_subscription(
            message_type, topic, lambda message: self.messages.__setitem__(topic, message),
            self.retained if retained else qos_profile_sensor_data)
        self.subscriptions.append(subscription)

    def observe_robot(self):
        from action_msgs.msg import GoalStatusArray
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool
        from ur_dashboard_msgs.msg import RobotMode, SafetyMode
        from rclpy.qos import qos_profile_sensor_data
        from tf2_ros import Buffer, TransformListener

        self.buffer = Buffer(node=self.node)
        self.listener = TransformListener(self.buffer, self.node)
        self.subscribe(Bool, "/io_and_status_controller/robot_program_running", True)
        self.subscribe(RobotMode, "/io_and_status_controller/robot_mode", True)
        self.subscribe(SafetyMode, "/io_and_status_controller/safety_mode", True)
        self.subscriptions.append(self.node.create_subscription(JointState, "/joint_states", self.on_joint,
                                                               qos_profile_sensor_data))
        # Observe every currently discovered action status, including trajectory
        # controllers and MoveIt. A canceled-but-not-stopped goal is still busy.
        for name, types in self.node.get_topic_names_and_types():
            if name.endswith("/_action/status") and "action_msgs/msg/GoalStatusArray" in types:
                self.subscribe(GoalStatusArray, name, True)

    def on_joint(self, message):
        now = time.monotonic()
        try:
            stamp, position = joint_values(message, self.node.get_clock().now().nanoseconds)
            if now - self.joint_receipt > 0.5:
                self.still_since = None
                self.joint_window.clear()
            if not position_window_stationary(self.joint_window, stamp, position):
                self.still_since = None
                self.joint_error = "等待至少 100ms 的连续关节位置反馈"
                return
            self.still_since = self.still_since if self.still_since is not None else now
            self.joint_error = ""
        except (ValueError, IndexError) as exc:
            self.still_since = None
            self.joint_error = str(exc)
            self.joint_window.clear()
        finally:
            self.joint_receipt = now

    def robot_ready(self):
        from rclpy.time import Time

        program = self.messages.get("/io_and_status_controller/robot_program_running")
        mode = self.messages.get("/io_and_status_controller/robot_mode")
        safety = self.messages.get("/io_and_status_controller/safety_mode")
        if program is None or not program.data:
            raise RuntimeError("External Control 未运行；请在示教器启动对应程序，本脚本不会自动启动机器人程序")
        if mode is None or mode.mode != 7 or safety is None or safety.mode not in (1, 2):
            raise RuntimeError("机器人尚未处于 RUNNING / NORMAL 或 REDUCED 状态")
        now = time.monotonic()
        if self.still_since is None or now - self.still_since < 0.3 or now - self.joint_receipt > 0.5:
            raise RuntimeError(self.joint_error or "尚无持续静止、实时的六关节反馈")
        for target in ("tool0", "grasp_center"):
            transform = self.buffer.lookup_transform("base_link", target, Time())
            stamp = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
            age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
            p, q = transform.transform.translation, transform.transform.rotation
            if stamp <= 0 or not -0.2 <= age <= 1.0:
                raise RuntimeError(f"base_link -> {target} TF 反馈已过期")
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                raise RuntimeError(f"无效 TF：{target}")
            if abs(math.hypot(q.x, q.y, q.z, q.w) - 1) > 0.01:
                raise RuntimeError(f"无效 TF 四元数：{target}")

    def idle(self):
        self.absent({"grasp_executor", "logitech_f710_servo_twist"})
        for topic, message in self.messages.items():
            if topic.endswith("/_action/status") and any(s.status in BUSY_GOAL_STATES for s in message.status_list):
                raise RuntimeError(f"仍有未完成运动/任务 Action：{topic}")

    def preflight(self, target, joystick=False):
        self.spin_for(1.0)
        duplicates = [name for name, count in Counter(self.node_names()).items()
                      if count > 1 and name in ("/controller_manager", "/move_group", "/robot_state_publisher")]
        if duplicates:
            raise RuntimeError("发现重复关键节点：" + ", ".join(duplicates))
        self.idle()
        if joystick:
            self.absent({"servo_node"})
        switch_plan(self.controllers(), target)
        self.planning_ready()
        self.observe_robot()
        last_reason = ["等待机器人状态"]

        def ready():
            try:
                self.robot_ready()
                return True
            except Exception as exc:
                last_reason[0] = str(exc)
                return False

        try:
            self.wait(ready, "External Control、关节状态和 TF")
        except RuntimeError as exc:
            raise RuntimeError(last_reason[0]) from exc
        # Allow reliable retained action-status discovery before deciding idle.
        self.spin_for(0.5)
        self.idle()
        self.robot_ready()
        if target == "scaled_joint_trajectory_controller":
            from control_msgs.action import FollowJointTrajectory
            self.action_ready(FollowJointTrajectory, "/scaled_joint_trajectory_controller/follow_joint_trajectory")
            if self.node.get_publishers_info_by_topic("/scaled_joint_trajectory_controller/joint_trajectory"):
                raise RuntimeError("轨迹命令 Topic 存在发布者，请先停止外部轨迹发送程序")
        elif self.node.get_publishers_info_by_topic("/forward_position_controller/commands"):
            raise RuntimeError("forward_position_controller 已有命令发布者，请先停止它")
        print("预检通过：驱动、External Control、静止关节、TF、MoveIt 及任务互斥。")

    def trigger(self, name):
        from std_srvs.srv import Trigger

        result = self.call(Trigger, name)
        if not result.success:
            raise RuntimeError(f"{name} 未成功：{result.message}")

    def switch(self, activate, deactivate):
        from controller_manager_msgs.srv import SwitchController

        if not activate and not deactivate:
            print("控制器已经处于目标状态，无需重复切换。")
            return
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = False
        request.timeout.sec = max(1, min(10, int(self.timeout) - 1))
        print(f"切换控制器：activate={activate}, deactivate={deactivate}")
        response = self.call(SwitchController, "/controller_manager/switch_controller", request)
        if not response.ok:
            raise RuntimeError("控制器切换被拒绝；未发送运动目标，请检查控制器/硬件日志")

    def select(self, target, ownership_marker=None):
        if target == "scaled_joint_trajectory_controller" and "/servo_node" in self.node_names():
            self.trigger("/servo_node/stop_servo")
        self.spin_for(0.1)
        self.idle()
        self.robot_ready()
        plan = switch_plan(self.controllers(), target)
        if ownership_marker is not None:
            Path(ownership_marker).write_text("forward_position_controller\n")
        self.switch(*plan)
        controllers = self.controllers()
        if active_motion(controllers) != [target]:
            raise RuntimeError("切换后运动控制器状态不符合预期：" + str(active_motion(controllers)))
        self.spin_for(0.1)
        self.robot_ready()
        print(f"已确认：{target} active，其余机械臂运动控制器 inactive。")

    def servo_config(self, output):
        import yaml
        from ament_index_python.packages import get_package_share_directory
        from rcl_interfaces.srv import ListParameters

        params = self.planning_ready()
        request = ListParameters.Request(prefixes=["robot_description_kinematics", "robot_description_planning"], depth=100)
        names = self.call(ListParameters, "/move_group/list_parameters", request).result.names
        if names:
            params.update(self.params("/move_group", names))
        path = Path(get_package_share_directory("ur_moveit_config")) / "config/ur_servo.yaml"
        servo = yaml.safe_load(path.read_text())
        if (servo.get("command_out_topic") != "/forward_position_controller/commands"
                or servo.get("command_out_type") != "std_msgs/Float64MultiArray"
                or servo.get("publish_joint_positions") is not True
                or servo.get("move_group_name") != "ur_manipulator"):
            raise RuntimeError(f"Servo 配置与手柄模式不匹配：{path}")
        servo["use_gazebo"] = False
        servo["is_primary_planning_scene_monitor"] = False
        params["moveit_servo"] = servo
        Path(output).write_text(yaml.safe_dump({"servo_node": {"ros__parameters": params}}, allow_unicode=True))
        print("已从运行中的 MoveIt 取得相同 URDF/SRDF，生成独立 Servo 参数。")

    def driver_ready(self):
        def ready():
            controllers = self.controllers()
            for target in CONTROLLER_TYPES:
                switch_plan(controllers, target)
            if active_motion(controllers):
                raise RuntimeError("预备阶段检测到 active 运动控制器，拒绝声明预备完成")
            return True

        # Wait for the driver's inactive controller spawner to finish.
        deadline = time.monotonic() + self.timeout
        while True:
            try:
                ready()
                print("UR 控制器已加载：scaled/forward 均 inactive。")
                return
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                self.spin_for(0.2)

    def run(self, command, values):
        if command == "absent":
            self.spin_for(1.5)
            self.absent(set(values))
        elif command == "driver-ready":
            self.driver_ready()
        elif command == "planning-ready":
            self.planning_ready()
            print("MoveIt 模型、规划服务和执行 Action 已就绪（沿用现有规划器配置）。")
        elif command in ("position", "check-position", "joystick", "check-joystick"):
            joystick = command.endswith("joystick")
            target = "forward_position_controller" if joystick else "scaled_joint_trajectory_controller"
            self.preflight(target, joystick)
            if not command.startswith("check-"):
                self.select(target, values[0] if command == "joystick" else None)
        elif command == "servo-config":
            self.servo_config(values[0])
        elif command == "start-servo":
            self.trigger("/servo_node/start_servo")
            from std_msgs.msg import Int8
            self.subscribe(Int8, "/servo_node/status")
            self.wait(lambda: "/servo_node/status" in self.messages, "Servo 状态反馈")
            if self.messages["/servo_node/status"].data != 0:
                raise RuntimeError(f"Servo 未就绪，状态码：{self.messages['/servo_node/status'].data}")
        elif command == "joystick-ready":
            self.wait(lambda: "/logitech_f710_servo_twist" in self.node_names(), "F710 节点")
            # Parameter service responds only after its constructor (including
            # the existing startup controller switch) has finished.
            self.params("/logitech_f710_servo_twist", ["device"])
            self.wait(lambda: bool(self.node.get_publishers_info_by_topic("/servo_node/delta_twist_cmds")),
                      "手柄 Twist 发布接口")
            if active_motion(self.controllers()) != ["forward_position_controller"]:
                raise RuntimeError("手柄启动后控制器状态不正确")
        elif command == "stop-joystick":
            controllers = self.controllers()
            stop = [c.name for c in controllers if c.state == "active" and c.name in
                    {"forward_position_controller", "scaled_joint_trajectory_controller"}]
            self.switch([], stop)
            if any(c.state == "active" and c.name in stop for c in self.controllers()):
                raise RuntimeError("手柄退出后未能确认控制器 inactive，请检查机器人状态")
        elif command == "wait-node":
            self.wait(lambda: any(n.rsplit("/", 1)[-1] == values[0] for n in self.node_names()), values[0])
        elif command == "wait-gripper":
            self.wait(lambda: any(info.node_name == "hand_control_node" for info in
                                 self.node.get_subscriptions_info_by_topic("/handControlCmd")),
                      "hand_control_node 的 /handControlCmd 订阅接口")
        elif command == "wait-camera":
            from sensor_msgs.msg import Image
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            # Startup also runs before the project overlay is sourced.
            sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/yolo_vision'))
            from yolo_vision.sensor_input import sensor_metadata
            images, clouds = deque(maxlen=60), deque(maxlen=60)
            errors = []
            def capture(rows, data):
                try:
                    rows.append((*sensor_metadata(data), self.node.get_clock().now().nanoseconds))
                except ValueError as exc:
                    errors[:] = [str(exc)]
            for kind, topic, rows in ((Image, '/camera/color/image_raw', images),
                                      (Image, '/camera/depth/image_raw', clouds)):
                self.subscriptions.append(self.node.create_subscription(kind, topic,
                    lambda data, rows=rows: capture(rows, data),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True))
            try:
                self.wait(lambda: camera_pair_ready(images, clouds, self.node.get_clock().now().nanoseconds),
                          "相机尚无两组可配对 RGB/对齐深度图（不是物体数量要求）")
            except RuntimeError as exc:
                delta = min((abs(a[0]-b[0])/1e9 for a in images for b in clouds), default=None)
                raise RuntimeError(f"{exc}；收到 RGB={len(images)} 帧，对齐深度={len(clouds)} 帧；"
                                   f"最小时间差={delta}s，要求≤0.1s；"
                                   f"最新 RGB={images[-1][:4] if images else None}，"
                                   f"最新深度={clouds[-1][:4] if clouds else None}；解码错误={errors}") from exc
        elif command == "wait-lidar":
            from sensor_msgs.msg import PointCloud2
            self.subscribe(PointCloud2, "/livox/lidar")
            self.wait(lambda: "/livox/lidar" in self.messages and self.messages["/livox/lidar"].width > 0,
                      "Mid-360 的 /livox/lidar 点云数据；检查雷达供电和 MID360_config.json 的 IP")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30)
    parser.add_argument("command", choices=["absent", "driver-ready", "planning-ready", "position",
                        "check-position", "joystick", "check-joystick", "servo-config", "start-servo",
                        "joystick-ready", "stop-joystick", "wait-node", "wait-gripper", "wait-camera", "wait-lidar"])
    parser.add_argument("values", nargs="*")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout < 1:
        parser.error("timeout must be at least one second")
    if args.command in ("servo-config", "wait-node", "joystick") and len(args.values) != 1:
        parser.error("this command requires exactly one argument")
    import rclpy
    rclpy.init(args=[])
    probe = None
    try:
        probe = RosProbe(args.timeout)
        probe.run(args.command, args.values)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    finally:
        if probe is not None:
            for client in probe.clients:
                client.destroy()
            probe.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
