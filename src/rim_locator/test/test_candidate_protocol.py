"""Malformed grouped targets fail before any consumer can use their poses."""
from copy import deepcopy
import json
import math

import pytest

from rim_locator.candidate_protocol import (MAX_MESSAGE_BYTES, decode_candidate_batch,
                                            encode_candidate_batch, validate_candidate_batch)


def batch():
    return {
        'schema_version': 1, 'task_id': 'task', 'observation_id': 'task/round_0001',
        'source_stamp_ns': 123_456_789, 'frame_id': 'base_link', 'tool_frame': 'grasp_center',
        'objects': [{'object_id': 7, 'distance': math.hypot(.4, .1), 'candidates': [
            {'candidate_id': 0, 'position': [.4, .1, .2], 'orientation': [0., 1., 0., 0.],
             'geometry': {'method': 'supported_wall_search', 'wall_count': 25}}]}],
        'rejected': [{'object_id': 9, 'reason': 'pose_not_found:insufficient_wall_points',
                      'geometry': {'searched_contacts': 4}}],
    }


def test_round_trip_preserves_evidence_and_returns_independent_values():
    value = batch()
    encoded = encode_candidate_batch(value)
    result = decode_candidate_batch(encoded, expected_task_id='task',
                                    expected_observation_id='task/round_0001',
                                    expected_frame_id='base_link', expected_tool_frame='grasp_center',
                                    source_not_before_ns=value['source_stamp_ns'] - 1)
    assert result == value
    result['objects'][0]['candidates'][0]['geometry']['wall_count'] = 999
    assert value['objects'][0]['candidates'][0]['geometry']['wall_count'] == 25


def test_all_rejected_is_valid_and_retains_observation_context():
    value = batch()
    value['objects'] = []
    result = decode_candidate_batch(encode_candidate_batch(value))
    assert not result['objects'] and result['rejected']
    assert result['source_stamp_ns'] == value['source_stamp_ns']
    assert result['observation_id'] == 'task/round_0001'


def test_legacy_standalone_context_requires_both_or_neither():
    value = batch()
    value['task_id'] = value['observation_id'] = ''
    assert validate_candidate_batch(value)['task_id'] == ''
    value['task_id'] = 'task'
    with pytest.raises(ValueError, match='incomplete_context'):
        validate_candidate_batch(value)


@pytest.mark.parametrize('field,value', [
    ('schema_version', 2), ('schema_version', True), ('source_stamp_ns', True),
    ('source_stamp_ns', 0), ('source_stamp_ns', -1), ('source_stamp_ns', 1.0),
    ('source_stamp_ns', 2**63), ('frame_id', '/base_link'), ('frame_id', 'base link'),
    ('tool_frame', ''), ('task_id', '   '), ('tool_frame', '/grasp_center'), ('objects', {}), ('rejected', None),
])
def test_invalid_top_level_fields(field, value):
    message = batch()
    message[field] = value
    with pytest.raises(ValueError, match='invalid_candidate_batch'):
        validate_candidate_batch(message)


@pytest.mark.parametrize('kwargs', [
    {'expected_task_id': 'other'}, {'expected_observation_id': 'task/round_0002'},
    {'expected_frame_id': 'base'}, {'expected_tool_frame': 'tool0'},
    {'source_not_before_ns': 123_456_789}, {'source_not_before_ns': 123_456_790}, {'source_not_before_ns': True},
])
def test_context_and_capture_boundary_are_enforced(kwargs):
    with pytest.raises(ValueError, match='invalid_candidate_batch'):
        decode_candidate_batch(encode_candidate_batch(batch()), **kwargs)


@pytest.mark.parametrize('mutation', [
    lambda b: b.pop('rejected'),
    lambda b: b.update(extra='unsupported'),
    lambda b: b['objects'].append(deepcopy(b['objects'][0])),
    lambda b: b['rejected'].append(deepcopy(b['rejected'][0])),
    lambda b: b['rejected'][0].update(object_id=7),
    lambda b: b['objects'][0].update(object_id=True),
    lambda b: b['objects'][0].update(object_id=-1),
    lambda b: b['objects'][0].update(distance=.01),
    lambda b: b['objects'][0].update(distance=True),
    lambda b: b['objects'][0].update(candidates=[]),
    lambda b: b['objects'][0]['candidates'].append(deepcopy(b['objects'][0]['candidates'][0])),
    lambda b: b['objects'][0]['candidates'][0].update(candidate_id=True),
    lambda b: b['objects'][0]['candidates'][0].update(candidate_id=1),
    lambda b: b['objects'][0]['candidates'][0].update(position=[.4, .1]),
    lambda b: b['objects'][0]['candidates'][0].update(position=[True, .1, .2]),
    lambda b: b['objects'][0]['candidates'][0].update(orientation=[0., 0., 0., 0.]),
    lambda b: b['objects'][0]['candidates'][0].update(orientation=[0., 2., 0., 0.]),
    lambda b: b['objects'][0]['candidates'][0].update(geometry=[]),
    lambda b: b['objects'][0]['candidates'][0]['geometry'].update(residual=float('nan')),
    lambda b: b['objects'][0]['candidates'][0]['geometry'].update(residual=float('inf')),
    lambda b: b['rejected'][0].update(reason=''),
])
def test_invalid_groups_and_nested_data_are_rejected(mutation):
    value = batch()
    mutation(value)
    with pytest.raises(ValueError, match='invalid_candidate_batch'):
        validate_candidate_batch(value)


@pytest.mark.parametrize('marker', ['schema_version', 'candidate_id'])
def test_duplicate_keys_rejected_even_when_values_agree(marker):
    encoded = encode_candidate_batch(batch())
    value = 1 if marker == 'schema_version' else 0
    encoded = encoded.replace(f'"{marker}":{value}', f'"{marker}":{value},"{marker}":{value}', 1)
    with pytest.raises(ValueError, match='duplicate_json_key'):
        decode_candidate_batch(encoded)


@pytest.mark.parametrize('constant', ['NaN', 'Infinity', '-Infinity'])
def test_nonstandard_json_numbers_are_rejected(constant):
    encoded = encode_candidate_batch(batch()).replace('"wall_count":25', '"wall_count":' + constant)
    with pytest.raises(ValueError, match='nonfinite_number'):
        decode_candidate_batch(encoded)


def test_quantity_and_size_limits_are_hard_errors():
    value = batch()
    value['objects'][0]['candidates'] *= 33
    with pytest.raises(ValueError, match='candidate_limit'):
        validate_candidate_batch(value)
    value = batch()
    value['rejected'] = [{'object_id': index + 1000, 'reason': 'geometry'} for index in range(256)]
    with pytest.raises(ValueError, match='object_limit'):
        validate_candidate_batch(value)
    value = batch()
    value['objects'][0]['candidates'][0]['geometry']['note'] = 'x' * MAX_MESSAGE_BYTES
    with pytest.raises(ValueError, match='message_size'):
        validate_candidate_batch(value)
    with pytest.raises(ValueError, match='message_size'):
        decode_candidate_batch(' ' * (MAX_MESSAGE_BYTES + 1))


def test_hard_limit_accepts_exactly_32_candidates_without_truncation():
    value = batch()
    prototype = value['objects'][0]['candidates'][0]
    value['objects'][0]['candidates'] = [dict(deepcopy(prototype), candidate_id=index) for index in range(32)]
    assert len(decode_candidate_batch(encode_candidate_batch(value))['objects'][0]['candidates']) == 32


def test_deep_or_non_json_diagnostics_cannot_bypass_validation():
    value = batch()
    nested = {}
    cursor = nested
    for _ in range(18):
        cursor['next'] = {}
        cursor = cursor['next']
    value['objects'][0]['candidates'][0]['geometry'] = nested
    with pytest.raises(ValueError, match='nesting_limit'):
        validate_candidate_batch(value)
    value['objects'][0]['candidates'][0]['geometry'] = {'not_json': {1, 2}}
    with pytest.raises(ValueError, match='json_value'):
        validate_candidate_batch(value)
