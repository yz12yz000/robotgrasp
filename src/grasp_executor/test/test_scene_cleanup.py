"""Migration only removes our two retired objects and preserves other geometry."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest
from moveit_msgs.msg import (PlanningScene, CollisionObject, AttachedCollisionObject,
                            AllowedCollisionEntry)
from moveit_msgs.srv import ApplyPlanningScene

from grasp_executor.config import load_config
from grasp_executor.scene_cleanup import removal_diff, clear_legacy_objects


def legacy_scene():
    scene = PlanningScene()
    scene.world.collision_objects = [CollisionObject(id='grasp_table'), CollisionObject(id='other_obstacle')]
    scene.robot_state.attached_collision_objects = [
        AttachedCollisionObject(link_name='gripper_base_link', object=CollisionObject(id='grasp_gripper_envelope')),
        AttachedCollisionObject(link_name='tool0', object=CollisionObject(id='held_object'))]
    acm = scene.allowed_collision_matrix
    acm.entry_names = ['link_a', 'grasp_table', 'link_b', 'grasp_gripper_envelope']
    acm.entry_values = [AllowedCollisionEntry(enabled=row) for row in (
        [False, False, True, False], [False]*4, [True, False, False, False], [False]*4)]
    acm.default_entry_names = ['grasp_table', 'other_obstacle']
    acm.default_entry_values = [False, False]
    return scene


def test_removal_preserves_other_objects_and_robot_matrix():
    current = legacy_scene()
    original = deepcopy(current)
    diff = removal_diff(current)
    assert current == original
    assert diff.is_diff and diff.robot_state.is_diff
    assert [(o.id, o.operation) for o in diff.world.collision_objects] == [
        ('grasp_gripper_envelope', CollisionObject.REMOVE), ('grasp_table', CollisionObject.REMOVE)]
    assert [(o.link_name, o.object.id, o.object.operation) for o in diff.robot_state.attached_collision_objects] == [
        ('gripper_base_link', 'grasp_gripper_envelope', CollisionObject.REMOVE)]
    acm = diff.allowed_collision_matrix
    assert acm.entry_names == ['link_a', 'link_b']
    assert [list(row.enabled) for row in acm.entry_values] == [[False, True], [True, False]]
    assert acm.default_entry_names == ['other_obstacle']


def test_clean_scene_needs_no_apply():
    calls = []
    def call(service, name, request, deadline):
        calls.append(name)
        assert service is not ApplyPlanningScene
        return SimpleNamespace(scene=PlanningScene())
    clear_legacy_objects(call, 1.)
    clear_legacy_objects(call, 1.)
    assert calls == ['/get_planning_scene'] * 2


@pytest.mark.parametrize('apply_ok, expected', [(False, 'cleanup_failed'), (True, 'still_present')])
def test_failed_removal_or_readback_stops_planning(apply_ok, expected):
    def call(service, name, request, deadline):
        if service is ApplyPlanningScene:
            return SimpleNamespace(success=apply_ok)
        return SimpleNamespace(scene=legacy_scene())
    with pytest.raises(RuntimeError, match=expected):
        clear_legacy_objects(call, 1.)


@pytest.mark.parametrize('name', ['config.json', 'place_config.json'])
def test_production_configs_have_no_table_dependency(name):
    config = load_config(Path(__file__).parents[1] / 'config' / name)
    assert not any(key.startswith('table_') for key in vars(config))
    assert 'require_collision_world_for_execution' not in vars(config)
