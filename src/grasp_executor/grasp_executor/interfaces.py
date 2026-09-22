"""MoveIt arm and existing hand_control command interface."""

import os
from pathlib import Path
import shutil
import signal
import subprocess
from threading import Event, Lock
import time


class ShellGripper:
    def __init__(self, setup_path, settle_time=1.0):
        self.setup_path = str(Path(setup_path).expanduser().resolve())
        self.lock = Lock()
        self.process = None
        self.stopped = Event()
        self.settle_time = settle_time

    @staticmethod
    def _kill(process):
        # Kill the entire private process group: bash / ros2 / talker children.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def _run(self, script, arguments, timeout):
        if os.name != "posix" or shutil.which("bash") is None:
            raise RuntimeError("real_gripper_requires_linux_bash")
        if not Path(self.setup_path).is_file():
            raise RuntimeError("hand_control_setup_missing")
        # Values are positional arguments, NEVER interpolated as shell code.
        with self.lock:
            if self.stopped.is_set():
                raise RuntimeError("cancelled")
            if self.process is not None:
                raise RuntimeError("gripper_command_already_running")
            process = subprocess.Popen(
                ["bash", "-c", script, "hand_control", self.setup_path, *arguments],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                encoding="utf-8", errors="replace",
                start_new_session=True,
            )
            self.process = process
        try:
            try:
                stdout, stderr = process.communicate(timeout=timeout)
            except subprocess.TimeoutExpired as exc:
                self._kill(process)
                process.communicate(timeout=1.0)
                raise TimeoutError("gripper_timeout") from exc
            if process.returncode != 0:
                raise RuntimeError(f"gripper_command_failed:{process.returncode}:{stderr[-1000:]}")
            if self.stopped.is_set():
                raise RuntimeError("cancelled")
            return stdout
        finally:
            try:
                if process.poll() is None:
                    self._kill(process)
                    process.wait(timeout=1.0)
            finally:
                with self.lock:
                    self.process = None

    def check_ready(self, timeout):
        # Listing an executable does not send any command to the gripper.
        output = self._run('source "$1" && ros2 pkg executables hand_control', [], timeout)
        if not any(line.split() == ["hand_control", "talker"] for line in output.splitlines()):
            raise RuntimeError("hand_control_talker_missing")
        return True

    def command(self, command, timeout):
        if command not in ("Open", "Close"):
            raise ValueError("invalid_gripper_command")
        self._run('source "$1" && exec ros2 run hand_control talker "$2" 1', [command], timeout - self.settle_time)
        if self.stopped.wait(self.settle_time):
            raise RuntimeError("cancelled")
        return True

    def stop(self):
        self.stopped.set()
        with self.lock:
            if self.process is not None:
                self._kill(self.process)


class HardwareInterfaces:
    def __init__(self, arm, gripper):
        self.arm = arm
        self.gripper = gripper

    def check_ready(self, timeout):
        start = time.monotonic()
        if self.arm.check_ready(timeout) is not True:
            raise RuntimeError("arm_not_ready")
        if self.arm.config.plan_only:
            return True
        remaining = timeout - (time.monotonic() - start)
        if remaining <= 0:
            raise TimeoutError("interface_readiness_timeout")
        return self.gripper.check_ready(remaining)

    def prepare_plans(self, approach, grasp):
        return self.arm.prepare_plans(approach, grasp)

    def close(self):
        self.arm.close()

    def move_to_pose(self, pose, timeout):
        return self.arm.move_to_pose(pose, timeout)

    def move_linear(self, pose, timeout):
        return self.arm.move_linear(pose, timeout)

    def is_pose_reached(self, timeout):
        return self.arm.is_pose_reached(timeout)

    def stop_motion(self, timeout):
        # Stop the command subprocess, then cancel and stop the robot motion.
        try:
            self.gripper.stop()
        finally:
            result = self.arm.stop_motion(timeout)
        if result is not True:
            raise RuntimeError("arm_stop_failed")
        return True

    def open_gripper(self, timeout):
        return self.gripper.command("Open", timeout)

    def close_gripper(self, timeout):
        return self.gripper.command("Close", timeout)

    def finish(self):
        self.arm.finish()


def create_interfaces(config, node=None):
    if node is None:
        raise ValueError("real_robot_requires_ros_node")
    if node.get_parameter("use_sim_time").value and not config.plan_only:
        raise ValueError("real_robot_requires_use_sim_time_false")
    from .moveit_ros import MoveItArm
    arm = MoveItArm(node, config, node.cancel)
    return HardwareInterfaces(arm, ShellGripper(config.hand_control_setup, config.gripper_settle_time))
