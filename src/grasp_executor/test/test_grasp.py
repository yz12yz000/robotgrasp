from dataclasses import replace
from threading import Event, RLock
from types import SimpleNamespace as NS
import math
import time

import pytest
from rclpy.task import Future
from moveit_msgs.msg import RobotTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectoryPoint

from grasp_executor.config import GraspConfig, load_config
from grasp_executor.grasp import Pose, run_task, validate_target, build_grasp_pose, build_approach_pose
from grasp_executor.cartesian import multiply, rotate, tool_target_to_tip, pose_error
from grasp_executor.trajectory import validate_trajectory, slow_down, quintic, set_seconds, samples, seconds
from grasp_executor.moveit_ros import MoveItArm, JOINTS


def trajectory():
    t = RobotTrajectory()
    t.joint_trajectory.joint_names = list(JOINTS)
    for q, sec in ((0., 0.), (.1, 1.)):
        p = JointTrajectoryPoint(positions=[q]*6, velocities=[0.]*6, accelerations=[0.]*6)
        set_seconds(p.time_from_start, sec)
        t.joint_trajectory.points.append(p)
    return t


class Interfaces:
    def __init__(self, fail=None):
        self.events, self.fail = [], fail

    def event(self, key):
        self.events.append(key)
        if self.fail == key:
            raise RuntimeError('injected_' + key)
        return True

    def check_ready(self, timeout): return self.event('ready')
    def prepare_plans(self, approach, grasp): return self.event('plan_all')
    def move_to_pose(self, pose, timeout): return self.event('approach')
    def move_linear(self, pose, timeout): return self.event('linear')
    def is_pose_reached(self, timeout): return self.event('reached')
    def open_gripper(self, timeout): return self.event('open')
    def close_gripper(self, timeout): return self.event('close')
    def stop_motion(self, timeout): return self.event('stop')
    def finish(self): return self.event('finish')


def task(config, interfaces, now=lambda: 2_000_000_000):
    return run_task((.4, .1, .2), 1_000_000_000, now, interfaces, config)


def test_tool_transform_includes_rotation():
    q = (0., 0., math.sin(.4), math.cos(.4))
    target = Pose((.4, .2, .3), (0., 1., 0., 0.), 'base_link', 'grasp_center')
    offset = ((.02, .03, .09), q)
    tip = tool_target_to_tip(target, offset, 'tool0')
    reconstructed = Pose(tuple(a+b for a,b in zip(tip.position, rotate(tip.orientation, offset[0]))),
                         multiply(tip.orientation, q), 'base_link', 'grasp_center')
    assert pose_error(reconstructed, target) == pytest.approx((0., 0.), abs=1e-7)
    assert tip.position != tuple(a-b for a,b in zip(target.position, offset[0]))


def test_target_expiry_still_enforced():
    c = GraspConfig()
    assert c.max_target_age == c.input_timeout == 600.
    validate_target('base_link', (.4, .1, .2), 1_000_000_000, 601_000_000_000, c)
    for stamp, now, reason in ((1, 601_000_000_000, 'stale'), (10_000_000_000, 1, 'future'), (0, 1, 'invalid')):
        with pytest.raises(ValueError, match=reason):
            validate_target('base_link', (.4, .1, .2), stamp, now, c)
    with pytest.raises(ValueError, match='outside_workspace'):
        build_grasp_pose((.4, .1, -.11), c)


def test_preview_never_moves_or_claims(tmp_path):
    c = GraspConfig(execution_journal=str(tmp_path/'journal'))
    i = Interfaces()
    assert task(c, i).status == 'planned'
    assert i.events == ['ready', 'plan_all']
    assert not (tmp_path/'journal').exists()


def test_execution_orders_and_duplicate_rejected(tmp_path):
    c = GraspConfig(plan_only=False, execution_journal=str(tmp_path))
    i = Interfaces()
    assert task(c, i).status == 'success'
    assert i.events == ['ready', 'plan_all', 'approach', 'reached', 'open', 'linear', 'reached',
                        'close', 'linear', 'reached', 'finish']
    duplicate = Interfaces()
    assert task(c, duplicate).reason == 'target_already_claimed'
    assert duplicate.events == ['ready', 'plan_all']


@pytest.mark.parametrize('step', ['ready', 'plan_all'])
def test_preflight_failure_never_commands(tmp_path, step):
    i = Interfaces(step)
    result = task(GraspConfig(plan_only=False, execution_journal=str(tmp_path)), i)
    assert result.status == 'failed'
    assert set(i.events) <= {'ready', 'plan_all'}
    assert not list(tmp_path.iterdir())


def test_expiry_during_planning_blocks_execution(tmp_path):
    ticks = iter([2_000_000_000, 602_000_000_000])
    i = Interfaces()
    assert task(GraspConfig(plan_only=False, execution_journal=str(tmp_path)), i, lambda: next(ticks)).reason == 'stale_target'
    assert i.events == ['ready', 'plan_all']


@pytest.mark.parametrize('step', ['approach', 'open', 'linear', 'close', 'finish'])
def test_failed_execution_stops_and_keeps_claim(tmp_path, step):
    c = GraspConfig(plan_only=False, execution_journal=str(tmp_path))
    i = Interfaces(step)
    result = task(c, i)
    assert result.status == 'failed'
    assert i.events[-1] == 'stop'
    assert len(list(tmp_path.iterdir())) == 1
    assert task(c, Interfaces()).reason == 'target_already_claimed'


def test_retiming_preserves_quintic_geometry():
    t = trajectory()
    validate_trajectory(t, JOINTS)
    scaled = slow_down(t, 4.)
    for fraction in (0., .1, .5, .9, 1.):
        a = quintic(*t.joint_trajectory.points, fraction)
        b = quintic(*scaled.joint_trajectory.points, fraction)
        assert b[0] == pytest.approx(a[0])
        assert b[1] == pytest.approx([v/4 for v in a[1]])
        assert b[2] == pytest.approx([v/16 for v in a[2]])
    assert seconds(t.joint_trajectory.points[-1].time_from_start) == 1.
    assert seconds(scaled.joint_trajectory.points[-1].time_from_start) == 4.


@pytest.mark.parametrize('defect', ['nan', 'derivatives', 'time', 'joints', 'endpoint', 'empty'])
def test_invalid_trajectory_rejected(defect):
    t = trajectory()
    if defect == 'nan': t.joint_trajectory.points[-1].positions[0] = math.nan
    if defect == 'derivatives': t.joint_trajectory.points[-1].accelerations = []
    if defect == 'time': t.joint_trajectory.points[-1].time_from_start.sec = 0
    if defect == 'joints': t.joint_trajectory.joint_names[0] = 'unexpected'
    if defect == 'endpoint': t.joint_trajectory.points[-1].velocities[0] = .1
    if defect == 'empty': t.joint_trajectory.points = []
    with pytest.raises(ValueError): validate_trajectory(t, JOINTS)


def test_sample_limit():
    with pytest.raises(ValueError, match='sample_limit'):
        samples(trajectory(), GraspConfig(max_validation_samples=2))


def bare_arm():
    a = MoveItArm.__new__(MoveItArm)
    a.config = GraspConfig()
    a.cancel, a.stopped, a.lock = Event(), Event(), RLock()
    a.goal_handle = a.result_future = a.action_error = None
    a.settle_started = None
    a._robot_running = lambda: None
    a._state = lambda **kw: None
    return a


@pytest.mark.parametrize('fraction, code', [(.999, 1), (1., -31), (math.nan, 1)])
def test_partial_cartesian_path_rejected(fraction, code):
    a = bare_arm()
    from geometry_msgs.msg import Pose as RosPose
    a.RosPose = RosPose
    a.tip_to_tool = ((0., 0., .095), (0., 0., 0., 1.))
    def call(typ, name, req, deadline):
        assert req.avoid_collisions
        assert req.link_name == 'tool0'
        return NS(error_code=NS(val=code), fraction=fraction)
    a._call = call
    with pytest.raises(RuntimeError, match='cartesian_path_incomplete'):
        a._linear_plan(RobotState(), build_grasp_pose((.4, .1, .2), a.config), time.monotonic()+1)


def completed(value):
    f = Future()
    f.set_result(value)
    return f


def test_late_goal_acceptance_is_canceled():
    a = bare_arm()
    a.stopped.set()
    canceled = []
    handle = NS(accepted=True, get_result_async=lambda: Future(), cancel_goal_async=lambda: canceled.append(True))
    a._goal_response(completed(handle))
    assert canceled == [True]


def test_action_rejection():
    a = bare_arm()
    a._goal_response(completed(NS(accepted=False)))
    assert a.action_error == 'trajectory_goal_rejected'


@pytest.mark.parametrize('status,code', [(6, 1), (4, -4), (5, -7)])
def test_action_failure_not_arrival(status, code):
    a = bare_arm()
    goal = build_grasp_pose((.4, .1, .2), a.config)
    a.active, a.read_pose = NS(linear=False, goal=goal), lambda: goal
    a.result_future = completed(NS(status=status, result=NS(error_code=NS(val=code))))
    with pytest.raises(RuntimeError, match='trajectory_execution_failed'):
        a.is_pose_reached(.1)


def test_arrival_requires_action_success_and_settling():
    a = bare_arm()
    goal = build_grasp_pose((.4, .1, .2), a.config)
    a.active, a.read_pose = NS(linear=False, goal=goal), lambda: goal
    assert a.is_pose_reached(.1) is False
    a.result_future = completed(NS(status=4, result=NS(error_code=NS(val=1))))
    assert a.is_pose_reached(.1) is False
    a.settle_started = time.monotonic() - a.config.settle_time - .1
    assert a.is_pose_reached(.1) is True


def test_service_wait_timeout_and_cancel():
    a = bare_arm()
    with pytest.raises(TimeoutError): a._wait_future(Future(), time.monotonic()-.1)
    a.cancel.set()
    with pytest.raises(RuntimeError, match='cancelled'): a._wait_future(Future(), time.monotonic()+1)


def test_stop_before_command_does_not_switch_controller():
    a = bare_arm()
    a.commanded, a.goal_future = False, None
    assert a.stop_motion(.1)


def test_stop_waits_for_terminal_and_new_stationary_feedback():
    a = bare_arm()
    a.config = replace(a.config, settle_time=.03)
    a.commanded = True
    cancellations = []
    def state(**kw): a.joint_received = time.monotonic()
    a._state = state
    handle = NS(accepted=True, cancel_goal_async=lambda: (cancellations.append(True) or completed(NS())),
                get_result_async=lambda: completed(NS(status=5)))
    a.goal_future = completed(handle)
    assert a.stop_motion(.3)
    assert cancellations


def test_stop_does_not_use_old_feedback():
    a = bare_arm()
    a.commanded, a.joint_received = True, time.monotonic()-1
    a.goal_future = completed(NS(accepted=False))
    with pytest.raises(RuntimeError, match='stop_feedback_unconfirmed'):
        a.stop_motion(.05)


def test_unconfirmed_cancel_deactivates_but_reports_error():
    a = bare_arm()
    a.config = replace(a.config, settle_time=.01)
    a.commanded, a.goal_future = True, Future()
    calls = []
    def state(**kw): a.joint_received = time.monotonic()
    a._state = state
    def call(typ, name, request, deadline, ignore_cancel=False):
        calls.append(name)
        if name.endswith('/switch_controller'):
            assert request.deactivate_controllers == [a.config.controller_name]
            return NS(ok=True)
        return NS(controller=[NS(name=a.config.controller_name, state='inactive')])
    a._call = call
    with pytest.raises(RuntimeError, match='controller_deactivated_but_action_cancel_unconfirmed'):
        a.stop_motion(.3)
    assert len(calls) == 2


def test_changed_start_rejects_before_action_dispatch():
    from grasp_executor.moveit_ros import Segment
    a = bare_arm()
    a.config = replace(a.config, plan_only=False)
    a.next_segment = 0
    goal = build_grasp_pose((.4,.1,.2), a.config)
    start = RobotState()
    start.joint_state.name, start.joint_state.position = list(JOINTS), [.5]*6
    a._state = lambda **kw: start
    a._controller_ready = lambda deadline: None
    a.plans = [Segment('approach', trajectory(), start, goal, False, [])]
    with pytest.raises(RuntimeError, match='start_state_changed'):
        a._begin(goal, 1., False)


def test_plan_only_rejects_direct_execution():
    a = bare_arm()
    with pytest.raises(RuntimeError, match='plan_only_execution_forbidden'):
        a._begin(build_grasp_pose((.4,.1,.2), a.config), 1., False)


def test_approach_ik_seed_and_joint_goal():
    from geometry_msgs.msg import Pose as RosPose
    a = bare_arm()
    a.RosPose = RosPose
    a.tip_to_tool = ((.02, 0., .095), (0.,0.,0.,1.))
    start = RobotState()
    start.joint_state.name, start.joint_state.position = list(JOINTS), [.1]*6
    solved = RobotState()
    solved.joint_state.name, solved.joint_state.position = list(JOINTS), [.2]*6
    calls = []
    def call(kind, name, request, deadline):
        calls.append(name)
        if name == '/compute_ik':
            assert request.ik_request.robot_state == start
            assert request.ik_request.avoid_collisions is True
            assert request.ik_request.ik_link_name == 'tool0'
            return NS(error_code=NS(val=1), solution=solved)
        goal = request.motion_plan_request.goal_constraints[0]
        assert [j.position for j in goal.joint_constraints] == [.2]*6
        assert request.motion_plan_request.start_state == start
        return NS(motion_plan_response=NS(error_code=NS(val=1), trajectory=trajectory()))
    a._call = call
    a._pose_plan(start, build_grasp_pose((.4,.1,.2), a.config), time.monotonic()+1)
    assert calls == ['/compute_ik', '/plan_kinematic_path']


def test_ik_failure_does_not_plan_motion():
    from geometry_msgs.msg import Pose as RosPose
    a = bare_arm()
    a.RosPose = RosPose
    a.tip_to_tool = ((0.,0.,0.), (0.,0.,0.,1.))
    def call(kind, name, request, deadline):
        assert name == '/compute_ik'
        return NS(error_code=NS(val=-31))
    a._call = call
    with pytest.raises(RuntimeError, match='approach_ik_failed:-31'):
        a._pose_plan(RobotState(), build_grasp_pose((.4,.1,.2), a.config), time.monotonic()+1)
