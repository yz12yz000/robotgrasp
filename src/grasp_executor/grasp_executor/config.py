"""Configuration values in metres and quaternion order x, y, z, w."""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path


def vector(value, keys, name):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError(f"invalid_config:{name}")
    numbers = tuple(value[key] for key in keys)
    if any(isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) for x in numbers):
        raise ValueError(f"invalid_config:{name}")
    return tuple(float(x) for x in numbers)


@dataclass(frozen=True)
class GraspConfig:
    input_topic: str = "/rim_locator/rim_point"
    base_frame: str = "base_link"
    grasp_offset: dict = field(default_factory=lambda: dict(x=0.0, y=0.0, z=0.0))
    approach_height: float = 0.15
    fixed_orientation: dict = field(default_factory=lambda: dict(x=0.0, y=1.0, z=0.0, w=0.0))
    motion_timeout: float = 90.0
    gripper_timeout: float = 5.0
    stop_timeout: float = 5.0
    poll_interval: float = 0.05
    input_timeout: float = 600.0
    max_target_age: float = 600.0
    future_tolerance: float = 0.1
    tool_frame: str = "grasp_center"
    controller_name: str = "scaled_joint_trajectory_controller"
    controller_manager: str = "/controller_manager"
    controller_namespace: str = ""
    controller_tip_frame: str = "auto"
    command_rate: float = 50.0
    approach_speed: float = 0.03
    descend_speed: float = 0.01
    angular_speed: float = 0.15
    position_tolerance: float = 0.002
    orientation_tolerance: float = 0.02
    settle_time: float = 0.25
    feedback_timeout: float = 0.5
    max_tracking_error: float = 0.03
    max_tracking_angle: float = 0.20
    linear_lateral_tolerance: float = 0.005
    hand_control_setup: str = "~/handControl2_ws/install/setup.bash"
    execution_journal: str = "~/.local/state/robot_grasp_ws/executed_targets"
    workspace_min: dict = field(default_factory=lambda: dict(x=-1.5, y=-1.5, z=-0.10))
    workspace_max: dict = field(default_factory=lambda: dict(x=1.5, y=1.5, z=1.5))
    publish_debug: bool = True
    plan_only: bool = True
    move_group: str = "ur_manipulator"
    planning_link: str = "tool0"
    move_group_node: str = "/move_group"
    planning_pipeline: str = ""
    planner_id: str = "RRTConnectkConfigDefault"
    planning_timeout: float = 60.0
    allowed_planning_time: float = 5.0
    velocity_scaling: float = 0.1
    acceleration_scaling: float = 0.1
    cartesian_step: float = 0.005
    jump_threshold: float = 2.0
    max_joint_step: float = 0.25
    validation_joint_step: float = 0.04
    validation_time_step: float = 0.05
    max_validation_samples: int = 5000
    max_joint_velocity: float = 0.3
    max_joint_acceleration: float = 0.3
    start_joint_tolerance: float = 0.01
    gripper_settle_time: float = 1.0
    table_enabled: bool = False
    table_id: str = "grasp_table"
    table_center: dict = field(default_factory=lambda: dict(x=0.6, y=0.0, z=-0.15))
    table_size: dict = field(default_factory=lambda: dict(x=0.8, y=0.8, z=0.05))
    require_collision_world_for_execution: bool = False

    def __post_init__(self):
        for name in ("input_topic", "base_frame", "tool_frame", "hand_control_setup", "execution_journal",
                     "controller_name", "controller_manager", "controller_tip_frame", "move_group",
                     "planning_link", "move_group_node", "planner_id", "table_id"):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"invalid_config:{name}")
        if any(getattr(self, name).startswith("/") for name in ("base_frame", "tool_frame", "controller_tip_frame")):
            raise ValueError("frame_id_must_not_start_with_slash")
        if not isinstance(self.controller_namespace, str) or "/" in self.controller_name:
            raise ValueError("invalid_controller_namespace_or_name")
        for name in ("publish_debug", "plan_only", "table_enabled", "require_collision_world_for_execution"):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"invalid_config:{name}")
        for name in ("approach_height", "motion_timeout", "gripper_timeout", "stop_timeout",
                     "poll_interval", "input_timeout", "max_target_age", "command_rate", "approach_speed",
                     "descend_speed", "angular_speed", "position_tolerance", "orientation_tolerance",
                     "settle_time", "feedback_timeout", "max_tracking_error", "max_tracking_angle",
                     "linear_lateral_tolerance", "planning_timeout", "allowed_planning_time",
                     "velocity_scaling", "acceleration_scaling", "cartesian_step", "jump_threshold",
                     "max_joint_step", "validation_joint_step", "validation_time_step", "max_joint_velocity",
                     "max_joint_acceleration", "start_joint_tolerance", "gripper_settle_time"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"invalid_config:{name}")
        for name in ("future_tolerance",):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError(f"invalid_config:{name}")
        vector(self.grasp_offset, "xyz", "grasp_offset")
        orientation = vector(self.fixed_orientation, "xyzw", "fixed_orientation")
        norm = math.hypot(*orientation)
        if not math.isfinite(norm) or norm < 1e-12 or abs(norm - 1.0) > 0.01:
            raise ValueError("orientation_must_be_unit_quaternion")
        low = vector(self.workspace_min, "xyz", "workspace_min")
        high = vector(self.workspace_max, "xyz", "workspace_max")
        if low[0] >= high[0] or low[1] >= high[1] or low[2] >= high[2]:
            raise ValueError("invalid_workspace_bounds")
        if not isinstance(self.planning_pipeline, str) or self.planning_link.startswith('/'):
            raise ValueError("invalid_planning_configuration")
        if self.velocity_scaling > 1 or self.acceleration_scaling > 1:
            raise ValueError("motion_scaling_must_be_in_0_1")
        if type(self.max_validation_samples) is not int or self.max_validation_samples < 2:
            raise ValueError("invalid_max_validation_samples")
        if self.gripper_settle_time >= self.gripper_timeout:
            raise ValueError("gripper_timeout_must_exceed_settle_time")
        vector(self.table_center, "xyz", "table_center")
        if any(v <= 0 for v in vector(self.table_size, "xyz", "table_size")):
            raise ValueError("invalid_table_size")
        if self.command_rate > 200:
            raise ValueError("command_rate_must_not_exceed_200_hz")
        if self.max_tracking_error <= self.position_tolerance or self.max_tracking_angle <= self.orientation_tolerance:
            raise ValueError("tracking_limits_must_exceed_arrival_tolerances")
        if self.linear_lateral_tolerance < self.position_tolerance:
            raise ValueError("lateral_tolerance_must_cover_arrival_tolerance")

def load_config(path):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return GraspConfig(**json.load(stream))
