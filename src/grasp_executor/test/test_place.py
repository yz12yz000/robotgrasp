from pathlib import Path
from types import SimpleNamespace as NS
import math
import pytest

from grasp_executor.config import GraspConfig, load_config
from grasp_executor.grasp import Pose, run_batch_task
from grasp_executor.moveit_ros import MoveItArm


class FakeInterfaces:
    def __init__(self):
        self.events = []

    def check_ready(self, timeout):
        self.events.append('ready')
        return True

    def prepare_plans(self, *poses):
        self.events.append(('plan', len(poses)))
        return True

    def move_to_pose(self, pose, timeout):
        self.events.append('move')
        return True

    def move_linear(self, pose, timeout):
        self.events.append('linear')
        return True

    def is_pose_reached(self, timeout):
        self.events.append('reached')
        return True

    def open_gripper(self, timeout):
        self.events.append('open')
        return True

    def close_gripper(self, timeout):
        self.events.append('close')
        return True

    def finish(self):
        self.events.append('finish')

    def stop_motion(self, timeout):
        self.events.append('stop')
        return True


def test_batch_executes_release_after_dwell(tmp_path):
    config = GraspConfig(plan_only=False, place_dwell_time=.01,
                         execution_journal=str(tmp_path))
    pose = Pose((.4, .1, .3), (0., 1., 0., 0.), 'base_link', 'grasp_center')
    interfaces = FakeInterfaces()
    result = run_batch_task([pose], 1_000_000_000, lambda: 2_000_000_000,
                            interfaces, config)
    assert result.status == 'success'
    # First open is before descent; second open is after place dwell/arrival.
    assert interfaces.events.count('open') == 2
    assert interfaces.events[-2:] == ['reached', 'finish']
    assert any(event == ('plan', 5) for event in interfaces.events)
    assert list(Path(tmp_path).glob('*.json'))


def test_fixed_tcp_reference_converts_to_saved_grasp_center():
    config = load_config(Path(__file__).parents[1] / 'config' / 'place_config.json')

    def transform(translation, rotation):
        return NS(transform=NS(translation=NS(x=translation[0], y=translation[1], z=translation[2]),
                               rotation=NS(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])))

    class Buffer:
        def lookup_transform(self, target, source, stamp):
            if (target, source) == ('base_link', 'base'):
                return transform((0., 0., 0.), (0., 0., 1., 0.))
            if (target, source) == ('tool0', 'grasp_center'):
                return transform((.033, .001089906, .166452628), (0., 0., 0., 1.))
            raise AssertionError((target, source))

    arm = MoveItArm.__new__(MoveItArm)
    arm.config, arm.buffer = config, Buffer()
    pose = arm.resolve_place_pose()
    expected = tuple(config.place_ready_position[k] for k in 'xyz')
    assert pose.position == pytest.approx(expected, abs=3e-4)
    assert math.isclose(abs(sum(a*b for a, b in zip(pose.orientation,
                                                     tuple(config.place_ready_orientation[k] for k in 'xyzw')))),
                        1.0, abs_tol=3e-4)


class RecordingInterfaces(FakeInterfaces):
    def __init__(self, fail_motion=0):
        super().__init__()
        self.moves, self.plans, self.fail_motion = [], [], fail_motion
    def prepare_plans(self, *poses):
        self.plans.append(poses)
        return super().prepare_plans(*poses)
    def move_to_pose(self, pose, timeout):
        self.moves.append(pose)
        if len(self.moves) == self.fail_motion:
            raise RuntimeError('injected_motion_failure')
        return super().move_to_pose(pose, timeout)
    def move_linear(self, pose, timeout):
        self.moves.append(pose)
        if len(self.moves) == self.fail_motion:
            raise RuntimeError('injected_motion_failure')
        return super().move_linear(pose, timeout)


def test_two_objects_order_preserved_orientation_drop_release_return(tmp_path):
    from dataclasses import replace
    c = GraspConfig(plan_only=False, place_dwell_time=.001, execution_journal=str(tmp_path))
    near = Pose((.3,.1,.2),(1.,0.,0.,0.),'base_link','grasp_center')
    far = replace(near, position=(.5,.1,.2), orientation=(0.,1.,0.,0.))
    i = RecordingInterfaces()
    result = run_batch_task([far,near], 1_000_000_000, lambda:2_000_000_000, i,c)
    assert result.status == 'success' and result.completed_count == result.object_count == 2
    assert len(i.moves) == 12 and len(i.plans) == 2
    for offset, target in ((0,near),(6,far)):
        ap, gp, lift, ready, drop, retreat = i.moves[offset:offset+6]
        assert gp == target and ap == lift
        assert ap.orientation == gp.orientation
        assert ap.position[2]-gp.position[2] == pytest.approx(c.approach_height)
        assert ready == retreat
        assert drop.position[:2] == ready.position[:2]
        assert ready.position[2]-drop.position[2] == pytest.approx(.2)
        assert ready.orientation == drop.orientation
    assert i.events.count('open') == 4 and i.events.count('close') == 2
    again = RecordingInterfaces()
    duplicate = run_batch_task([near,far], 1_000_000_000, lambda:2_000_000_000, again,c)
    assert duplicate.reason == 'batch_already_claimed' and not again.moves


@pytest.mark.parametrize('failure', range(1,7))
def test_failure_stops_batch_without_release_retry_or_second_object(tmp_path, failure):
    c = GraspConfig(plan_only=False, place_dwell_time=.001, execution_journal=str(tmp_path))
    p = Pose((.3,.1,.2),(1.,0.,0.,0.),'base_link','grasp_center')
    i = RecordingInterfaces(failure)
    result = run_batch_task([p,p], 1_000_000_000, lambda:2_000_000_000, i,c)
    assert result.status == 'failed' and result.object_index == result.completed_count == 0
    assert len(i.plans) == 1 and len(i.moves) == failure
    assert i.events[-1] == 'stop'
    # Before retreat, a failed motion must never issue the release command.
    assert i.events.count('open') == (0 if failure == 1 else 2 if failure == 6 else 1)


@pytest.mark.parametrize('stamp, now, reason', [(0,2,'invalid_target_timestamp'),
    (4_000_000_000,2_000_000_000,'future'), (1,700_000_000_000,'stale')])
def test_bad_batch_time_never_plans(stamp, now, reason):
    p = Pose((.3,.1,.2),(1.,0.,0.,0.),'base_link','grasp_center')
    i = RecordingInterfaces()
    r = run_batch_task([p],stamp,lambda:now,i,GraspConfig())
    assert reason in r.reason and not i.plans and not i.moves


def test_one_second_dwell_occurs_after_arrival_before_release(monkeypatch):
    from grasp_executor import grasp as g
    clock = [0.]
    class Cancel:
        def is_set(self): return False
        def wait(self, seconds): clock[0] += seconds; return False
    monkeypatch.setattr(g.time, 'monotonic', lambda:clock[0])
    c = GraspConfig()
    p = Pose((.3,.1,.2),(1.,0.,0.,0.),'base_link','grasp_center')
    ap, ready = g.build_approach_pose(p,c), g.build_place_ready_pose(c)
    states = {}
    r = g.execute_grasp_place(ap,p,ready,g.build_place_drop_pose(ready,c),
        RecordingInterfaces(),c,lambda state:states.update({state:clock[0]}),Cancel())
    assert r.status == 'success'
    assert states['releasing_gripper']-states['holding_before_release'] == pytest.approx(1.)


@pytest.mark.parametrize('count', [1, 2])
@pytest.mark.parametrize('plan_only', [False, True])
def test_batch_offset_once_in_preview_plans_and_motion(tmp_path, count, plan_only):
    from dataclasses import replace
    from grasp_executor.grasp import build_place_ready_pose, build_place_drop_pose, build_grasp_pose
    config = GraspConfig(plan_only=plan_only, place_dwell_time=.001, execution_journal=str(tmp_path),
                         grasp_offset=dict(x=.01,y=0.,z=.02))
    pose = Pose((.3,.1,.2),(1.,0.,0.,0.),'base_link','grasp_center')
    originals = [pose, replace(pose, position=(.5,.1,.2))][:count]
    saved = list(originals)
    interfaces, previews = RecordingInterfaces(), []
    result = run_batch_task(originals, 1_000_000_000, lambda:2_000_000_000, interfaces,
                            config, on_poses=lambda *p:previews.append(p))
    assert result.status == ('planned' if plan_only else 'success')
    assert originals == saved and len(previews) == count and len(interfaces.plans) == count
    ready = build_place_ready_pose(config)
    for index, (ap, gp, pr, pd) in enumerate(previews):
        assert gp.position == pytest.approx((saved[index].position[0]+.01,.1,.22))
        assert gp.orientation == saved[index].orientation
        assert ap.position == pytest.approx((gp.position[0],.1,.22+config.approach_height))
        assert ap.orientation == gp.orientation
        assert (pr,pd) == (ready,build_place_drop_pose(ready,config))
        assert interfaces.plans[index] == (ap,gp,pr,pd,pr)
        if not plan_only:
            assert interfaces.moves[index*6:index*6+6] == [ap,gp,ap,pr,pd,pr]
    old = build_grasp_pose(pose.position, config)
    assert previews[0][1].position == old.position
    assert not plan_only or not interfaces.moves


@pytest.mark.parametrize('offset', [dict(x=2.,y=0.,z=0.), dict(x=0.,y=0.,z=1.2)])
def test_offset_or_approach_outside_workspace_rejected_before_planning(offset):
    config = GraspConfig(grasp_offset=offset)
    pose = Pose((.3,.1,.2),(1.,0.,0.,0.),'base_link','grasp_center')
    interfaces = RecordingInterfaces()
    result = run_batch_task([pose], 1_000_000_000, lambda:2_000_000_000, interfaces, config)
    assert result.failed_step == 'preflight' and result.reason == 'pose_outside_workspace'
    assert not interfaces.plans and not interfaces.moves and not interfaces.events


def test_offset_changes_execution_order_but_not_journal_identity(tmp_path):
    from dataclasses import replace
    import json
    config = GraspConfig(plan_only=False, place_dwell_time=.001, execution_journal=str(tmp_path))
    near = Pose((.3,.1,.2),(1.,0.,0.,0.),'base_link','grasp_center')
    far = replace(near, position=(-.4,.1,.2))
    shifted = replace(config, grasp_offset=dict(x=.2,y=0.,z=.02))
    interfaces = RecordingInterfaces()
    first = run_batch_task([far,near], 1_000_000_000, lambda:2_000_000_000, interfaces, shifted)
    assert first.status == 'success'
    assert interfaces.plans[0][1].position == pytest.approx((-.2,.1,.22))
    journal = list(tmp_path.glob('*.json'))
    assert len(journal) == 1
    payload = json.loads(journal[0].read_text())
    assert payload['poses'][0]['position'] == list(near.position)
    assert payload['poses'][1]['position'] == list(far.position)
    again = RecordingInterfaces()
    second = run_batch_task([near,far], 1_000_000_000, lambda:2_000_000_000, again, config)
    assert second.reason == 'batch_already_claimed' and not again.moves


def test_old_zero_offset_journal_blocks_nonzero_offset(tmp_path):
    from grasp_executor.grasp import claim_batch
    pose = Pose((.3,.1,.2),(1.,0.,0.,0.),'base_link','grasp_center')
    claim_batch(tmp_path, [pose], 1_000_000_000)
    config = GraspConfig(plan_only=False, execution_journal=str(tmp_path), grasp_offset=dict(x=.01,y=0.,z=.02))
    interfaces = RecordingInterfaces()
    result = run_batch_task([pose], 1_000_000_000, lambda:2_000_000_000, interfaces, config)
    assert result.reason == 'batch_already_claimed' and not interfaces.moves
    assert len(list(tmp_path.glob('*.json'))) == 1


@pytest.mark.parametrize('plan_only',[False,True])
def test_zero_targets_does_not_touch_any_interface(plan_only):
    class Forbidden:
        def __getattr__(self,name):
            pytest.fail('zero-target batch must not access interface: '+name)
    def forbidden(): pytest.fail('zero-target batch must not request a robot clock')
    result=run_batch_task([],0,forbidden,Forbidden(),GraspConfig(plan_only=plan_only))
    assert result.status=='no_targets' and result.object_count==result.completed_count==0


def test_three_objects_execute_all_from_near_to_far(tmp_path):
    config=GraspConfig(plan_only=False,place_dwell_time=.001,execution_journal=str(tmp_path))
    targets=[Pose((x,.1,.2),(1.,0.,0.,0.),'base_link','grasp_center') for x in (.5,.3,.4)]
    interfaces=RecordingInterfaces()
    result=run_batch_task(targets,1_000_000_000,lambda:2_000_000_000,interfaces,config)
    assert result.status=='success' and result.completed_count==result.object_count==3
    assert len(interfaces.moves)==18 and len(interfaces.plans)==3
    assert [p[1] for p in interfaces.plans]==[targets[1],targets[2],targets[0]]
    assert interfaces.events.count('open')==6 and interfaces.events.count('close')==3
