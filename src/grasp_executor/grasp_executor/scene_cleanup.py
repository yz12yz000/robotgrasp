"""Remove only the retired desktop and extra gripper envelope from MoveIt.

No geometry is loaded. Robot URDF geometry, SRDF allowances, OctoMap and
unrelated scene objects are untouched. Kept for migration of existing scenes.
"""
from copy import deepcopy

LEGACY_IDS = frozenset(('grasp_table', 'grasp_gripper_envelope'))


def scene_request():
    from moveit_msgs.msg import PlanningSceneComponents as C
    from moveit_msgs.srv import GetPlanningScene
    request = GetPlanningScene.Request()
    request.components.components = (C.WORLD_OBJECT_NAMES | C.ROBOT_STATE_ATTACHED_OBJECTS
                                     | C.ALLOWED_COLLISION_MATRIX)
    return request


def has_legacy_objects(scene):
    acm = scene.allowed_collision_matrix
    return (any(obj.id in LEGACY_IDS for obj in scene.world.collision_objects)
            or any(obj.object.id in LEGACY_IDS for obj in scene.robot_state.attached_collision_objects)
            or bool(LEGACY_IDS.intersection(acm.entry_names))
            or bool(LEGACY_IDS.intersection(acm.default_entry_names)))


def removal_diff(current):
    from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
    scene = PlanningScene(is_diff=True)
    scene.robot_state.is_diff = True
    # MoveIt detaches bodies into the world before processing world removals.
    # Include attached IDs here too, so a detached envelope cannot remain as
    # a stationary obstacle at the old gripper position.
    ids = ({obj.id for obj in current.world.collision_objects}
           | {obj.object.id for obj in current.robot_state.attached_collision_objects}) & LEGACY_IDS
    scene.world.collision_objects = [
        CollisionObject(id=name, operation=CollisionObject.REMOVE) for name in sorted(ids)]
    scene.robot_state.attached_collision_objects = [
        AttachedCollisionObject(link_name=obj.link_name,
                                object=CollisionObject(id=obj.object.id, operation=CollisionObject.REMOVE))
        for obj in current.robot_state.attached_collision_objects if obj.object.id in LEGACY_IDS]
    acm = deepcopy(current.allowed_collision_matrix)
    keep = [i for i, name in enumerate(acm.entry_names) if name not in LEGACY_IDS]
    acm.entry_names = [acm.entry_names[i] for i in keep]
    rows = [acm.entry_values[i] for i in keep]
    for row in rows:
        row.enabled = [row.enabled[i] for i in keep]
    acm.entry_values = rows
    defaults = [(name, value) for name, value in zip(acm.default_entry_names, acm.default_entry_values)
                if name not in LEGACY_IDS]
    acm.default_entry_names = [name for name, _ in defaults]
    acm.default_entry_values = [value for _, value in defaults]
    scene.allowed_collision_matrix = acm
    return scene


def clear_legacy_objects(call, deadline):
    """Apply a targeted removal and verify it; call follows MoveItArm._call."""
    from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
    scene = call(GetPlanningScene, '/get_planning_scene', scene_request(), deadline).scene
    if has_legacy_objects(scene):
        result = call(ApplyPlanningScene, '/apply_planning_scene',
                      ApplyPlanningScene.Request(scene=removal_diff(scene)), deadline)
        if not result.success:
            raise RuntimeError('legacy_scene_cleanup_failed')
        scene = call(GetPlanningScene, '/get_planning_scene', scene_request(), deadline).scene
        if has_legacy_objects(scene):
            raise RuntimeError('legacy_scene_objects_still_present')
    return scene
