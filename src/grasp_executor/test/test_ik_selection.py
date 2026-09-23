"""Live joint winding regression without requesting hardware motion."""
from dataclasses import replace
import math
from types import SimpleNamespace as NS
import time

import numpy as np
import pytest
from geometry_msgs.msg import Pose as RosPose
from moveit_msgs.msg import RobotState

from grasp_executor.config import GraspConfig
from grasp_executor.grasp import build_grasp_pose
from grasp_executor.ik_angles import nearest_equivalent_angles
from grasp_executor.moveit_ros import JOINTS
from test_grasp import bare_arm, trajectory


def state(values):
    result = RobotState()
    result.joint_state.name, result.joint_state.position = list(JOINTS), list(values)
    return result


def arm():
    result = bare_arm()
    result.RosPose = RosPose
    result.tip_to_tool = ((0., 0., 0.), (0., 0., 0., 1.))
    result.joint_limits = {j: (-math.tau, math.tau, .3) for j in JOINTS}
    result.node = NS(get_logger=lambda: NS(info=lambda m: None, warning=lambda m: None))
    return result


def test_recorded_motor_turns_are_removed_without_changing_fk_angles():
    reference = dict(zip(JOINTS, [4.11785, -1.74999, -1.54150, -1.44591, 1.57243, -3.74340]))
    raw = dict(zip(JOINTS, [-2.94848, -2.28048, -1.05798, 4.66789, 1.46964, -.38930]))
    limits = {j: (-math.tau, math.tau, .3) for j in JOINTS}
    selected = nearest_equivalent_angles(raw, reference, limits)
    assert abs(selected['shoulder_pan_joint']-reference['shoulder_pan_joint']) < .8
    assert abs(selected['wrist_1_joint']-reference['wrist_1_joint']) < .18
    # Wrist 3 cannot move another -2*pi: that would violate the URDF limit.
    assert selected['wrist_3_joint'] == raw['wrist_3_joint']
    for joint in JOINTS:
        assert -math.tau <= selected[joint] <= math.tau
        assert math.sin(selected[joint]) == pytest.approx(math.sin(raw[joint]))
        assert math.cos(selected[joint]) == pytest.approx(math.cos(raw[joint]))
    assert raw['shoulder_pan_joint'] == -2.94848  # Caller data is unchanged.


def test_soft_limit_prevents_the_recorded_wrong_endpoint():
    import xml.etree.ElementTree as ET
    from grasp_executor.ik_angles import revolute_bounds
    joint = ET.fromstring('<joint type="revolute"><limit lower="-6.283185307" upper="6.283185307" velocity="3.14"/>'
                          '<safety_controller soft_lower_limit="-6.133185307" soft_upper_limit="6.133185307"/></joint>')
    limits = revolute_bounds(joint)
    assert limits == (-6.133185307, 6.133185307, 3.14)
    result = nearest_equivalent_angles({'wrist_3_joint': .070430694}, {'wrist_3_joint': -3.74340}, {'wrist_3_joint': limits})
    assert result['wrist_3_joint'] == pytest.approx(.070430694)
    # The geometrically equivalent -6.2127546 is beyond MoveIt's safety bound;
    # selecting it would cause the planner to clamp/change the endpoint.
    assert result['wrist_3_joint'] != pytest.approx(.070430694-math.tau)


@pytest.mark.parametrize('raw,reference,limits', [(2.,0.,(-1.,1.)), (float('nan'),0.,(-1.,1.)),
    (0.,float('inf'),(-1.,1.)), (0.,0.,(1.,-1.))])
def test_invalid_or_no_bounded_equivalent_is_rejected(raw, reference, limits):
    with pytest.raises(ValueError):
        nearest_equivalent_angles({'j':raw}, {'j':reference}, {'j':limits})


def test_multiple_ik_branches_rank_by_measured_travel_and_deduplicate():
    a = arm()
    a.config = replace(a.config, ik_attempts=3)
    start = state([4., -1., -1., -1., 1., -3.])
    close = [3.5, -1.1, -1.1, -1.1, 1.1, -2.9]
    far = [1., 1., 1., 1., 3., 0.]
    wrapped = [close[0]-math.tau, *close[1:]]
    solutions = iter([far, wrapped, close])
    calls = []
    def call(kind, name, request, deadline):
        calls.append(name)
        if name == '/compute_ik':
            assert request.ik_request.avoid_collisions
            return NS(error_code=NS(val=1), solution=state(next(solutions)))
        if name == '/check_state_validity':
            assert request.group_name == ''
            return NS(valid=True, contacts=[])
        actual = [c.position for c in request.motion_plan_request.goal_constraints[0].joint_constraints]
        np.testing.assert_allclose(actual, close, atol=1e-12)
        assert request.motion_plan_request.start_state == start
        return NS(motion_plan_response=NS(error_code=NS(val=1), trajectory=trajectory()))
    a._call = call
    a._pose_plan(start, build_grasp_pose((.4, .1, .2), a.config), time.monotonic()+2)
    assert calls.count('/compute_ik') == 3
    assert calls.count('/check_state_validity') == 2
    assert calls.count('/plan_kinematic_path') == 1


def test_rejected_path_tries_next_branch_without_bypassing_validation():
    a = arm()
    a._ik_solutions = lambda *args: [dict(positions=[.1]*6,max_delta=.1), dict(positions=[.2]*6,max_delta=.2)]
    attempts = []
    a._call = lambda *args: NS(motion_plan_response=NS(error_code=NS(val=1), trajectory=trajectory()))
    def validate(path):
        attempts.append(path)
        if len(attempts) == 1:
            raise RuntimeError('motion_timeout_too_short_for_trajectory:approach')
        return 'validated'
    assert a._pose_plan(state([0.]*6), build_grasp_pose((.4,.1,.2), a.config), time.monotonic()+2, validate) == 'validated'
    assert len(attempts) == 2
    def cancelled(path):
        raise RuntimeError('cancelled')
    with pytest.raises(RuntimeError, match='cancelled'):
        a._pose_plan(state([0.]*6), build_grasp_pose((.4,.1,.2), a.config), time.monotonic()+2, cancelled)


def test_all_ik_goals_in_collision_never_call_motion_planner():
    a = arm()
    calls = []
    def call(kind, name, request, deadline):
        calls.append(name)
        if name == '/compute_ik':
            return NS(error_code=NS(val=1), solution=state([.2]*6))
        assert name == '/check_state_validity'
        return NS(valid=False, contacts=[NS(contact_body_1='forearm_link', contact_body_2='gripper_base_link')])
    a._call = call
    with pytest.raises(RuntimeError, match='approach_goal_collision:forearm_link/gripper_base_link'):
        a._pose_plan(state([0.]*6), build_grasp_pose((.4,.1,.2), a.config), time.monotonic()+2)
    assert '/plan_kinematic_path' not in calls


def test_infeasible_descent_retries_approach_and_only_commits_complete_triplet():
    from grasp_executor.moveit_ros import Segment
    from grasp_executor.grasp import build_approach_pose
    a = arm()
    start = state([0.]*6)
    a._scene = a._valid = lambda *args: None
    a._state = lambda **kwargs: start
    a.preview_state = None
    a.model_id = 'test'
    displayed = []
    a.display = NS(publish=displayed.append)
    a._ik_solutions = lambda *args: [dict(positions=[.1]*6,max_delta=.1), dict(positions=[.2]*6,max_delta=.2)]
    planned = []
    def call(kind,name,request,deadline):
        assert name == '/plan_kinematic_path'
        value = request.motion_plan_request.goal_constraints[0].joint_constraints[0].position
        planned.append(value)
        path = trajectory();path.joint_trajectory.points[-1].positions = [value]*6
        return NS(motion_plan_response=NS(error_code=NS(val=1),trajectory=path))
    a._call = call
    def linear(initial,goal,deadline):
        value = initial.joint_state.position[0]
        if value == .1:
            raise RuntimeError('cartesian_path_incomplete:fraction=0.03125,code=1')
        path=trajectory()
        path.joint_trajectory.points[0].positions=[value]*6
        path.joint_trajectory.points[-1].positions=[value+.1]*6
        return path
    a._linear_plan = linear
    a._validate_and_time = lambda name,path,initial,goal,linear,deadline: Segment(name,path,initial,goal,linear,[])
    grasp=build_grasp_pose((.4,.1,.2),a.config)
    a.prepare_plans(build_approach_pose(grasp,a.config),grasp)
    assert planned == [.1,.2]
    assert [segment.name for segment in a.plans] == ['approach','descend','lift']
    assert list(a.plans[0].trajectory.joint_trajectory.points[-1].positions) == [.2]*6
    assert len(displayed)==1 and len(displayed[0].trajectory)==3
    assert a.preview_state.joint_state.position == pytest.approx([.4]*6)


@pytest.mark.parametrize('values', [dict(ik_attempts=x) for x in (0,17,True,1.5)] +
    [dict(ik_timeout=x) for x in (0,-1,float('nan'),True,6)] + [dict(max_ik_plans=0)])
def test_ik_search_config_is_bounded(values):
    with pytest.raises(ValueError, match='invalid_config'):
        GraspConfig(**values)
