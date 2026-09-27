"""ROS-free policies for bounded, fresh-observation pick/place rounds."""
from copy import deepcopy
from dataclasses import dataclass
import math


GEOMETRY_REASONS = frozenset({
    'empty_cloud', 'insufficient_points', 'insufficient_pointcloud',
    'rim_not_found:insufficient_candidates', 'rim_not_found:insufficient_local_points',
    'pose_not_found:ambiguous_components', 'pose_not_found:normal_direction_ambiguous',
    'pose_not_found:insufficient_wall_points', 'pose_not_found:degenerate_wall_neighborhood',
    'pose_not_found:wall_normal_out_of_range', 'pose_not_found:insufficient_wall_span',
    'pose_not_found:wall_fit_residual', 'pose_not_found:anchor_not_on_wall',
    'pose_not_found:unstable_wall_normal', 'pose_not_found:uncorroborated_wall_normal',
    'pose_not_found:no_supported_contact', 'pose_not_found:closing_axis_parallel_to_approach',
})


@dataclass(frozen=True)
class RoundLimits:
    max_reobservations: int = 2
    max_observation_rounds: int = 30
    max_completed_objects: int = 20

    def __post_init__(self):
        for name in ('max_reobservations', 'max_observation_rounds', 'max_completed_objects'):
            value = getattr(self, name)
            if type(value) is not int or value < (0 if name == 'max_reobservations' else 1):
                raise ValueError('invalid_limit:' + name)


@dataclass
class RoundBudget:
    deadline: float
    limits: RoundLimits
    observation_count: int = 0
    completed_count: int = 0
    reobservations_without_progress: int = 0
    pending_failure: bool = False
    unresolved_rejections: bool = False

    def __post_init__(self):
        if not math.isfinite(self.deadline):
            raise ValueError('invalid_deadline')

    def begin_round(self, now):
        if now >= self.deadline:
            raise TimeoutError('pipeline_timeout')
        if self.observation_count >= self.limits.max_observation_rounds:
            raise RuntimeError('max_observation_rounds_reached')
        if self.completed_count >= self.limits.max_completed_objects:
            raise RuntimeError('max_completed_objects_reached')
        self.observation_count += 1
        return self.observation_count

    def retry_observation(self, now):
        self.pending_failure = True
        if now >= self.deadline or self.reobservations_without_progress >= self.limits.max_reobservations:
            return False
        self.reobservations_without_progress += 1
        return True

    def complete_object(self, has_rejections=False):
        self.completed_count += 1
        self.reobservations_without_progress = 0
        prior_failure = self.pending_failure
        self.pending_failure = False
        # Without cross-frame identity matching, an unrelated success cannot
        # prove an earlier rejected object was handled. Keep this uncertainty.
        self.unresolved_rejections = self.unresolved_rejections or prior_failure or bool(has_rejections)

    def empty_result(self):
        if self.pending_failure or self.unresolved_rejections:
            return False, 'partial' if self.completed_count else 'failed', 'remaining_unknown'
        return True, 'completed' if self.completed_count else 'no_targets', ''


def recoverable_failure(statuses):
    """Only fully returned pre-motion feasibility/geometry failures may reobserve."""
    executor = statuses.get('grasp_executor', {})
    if executor.get('command_attempted') is True:
        return False
    failures = [(name, value) for name, value in statuses.items()
                if value.get('status') in ('failed', 'candidates_exhausted')]
    if not failures:
        return False
    for name, value in failures:
        reason = value.get('reason', '')
        if name == 'grasp_executor':
            if value.get('status') != 'candidates_exhausted' or value.get('command_attempted') is not False:
                return False
        elif name == 'yolo_vision':
            if reason != 'no_valid_depth_for_detected_targets':
                return False
        elif name == 'rim_locator':
            rejected = value.get('rejected', [])
            if (reason != 'pose_not_found:no_valid_object' or not rejected
                    or any(item.get('reason') not in GEOMETRY_REASONS for item in rejected)):
                return False
        else:
            return False
    return True


def select_candidate_groups(batch, offset, execute):
    """Preserve stable object distance ordering; never score/reorder candidates."""
    selected = deepcopy(batch)
    groups = selected['objects']
    # Locator already provides the historical distance/object-id ordering.
    groups.sort(key=lambda group: math.hypot(
        group['candidates'][0]['position'][0] + offset['x'],
        group['candidates'][0]['position'][1] + offset['y']))
    if execute:
        selected['objects'] = groups[:1]
        selected['rejected'] = []  # The original observation remains in the round journal.
    return selected


def context_matches(value, task_id, observation_id):
    return (isinstance(value, dict) and value.get('task_id') == task_id
            and value.get('observation_id') == observation_id)
