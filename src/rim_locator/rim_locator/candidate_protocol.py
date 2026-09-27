"""Versioned, ROS-free candidate-group protocol shared by all three consumers.

Only the standard library is imported here.  IDs identify an observation and
its local objects; they are not a persistent object tracker.  Quality evidence
is transported verbatim and never used to reorder candidates.
"""

import json
import math


SCHEMA_VERSION = 1
MAX_MESSAGE_BYTES = 1024 * 1024
MAX_OBJECTS = 256
MAX_CANDIDATES = 32
_MAX_INTEGER = 2**63 - 1
_BATCH_KEYS = {'schema_version', 'task_id', 'observation_id', 'source_stamp_ns',
               'frame_id', 'tool_frame', 'objects', 'rejected'}


def _invalid(reason):
    raise ValueError('invalid_candidate_batch:' + reason)


def _keys(value, required, optional=()):
    if type(value) is not dict or not required <= value.keys() or value.keys() - required - set(optional):
        _invalid('fields')


def _integer(value, name, minimum=0, maximum=_MAX_INTEGER):
    if type(value) is not int or not minimum <= value <= maximum:
        _invalid(name)
    return value


def _number(value, name):
    if type(value) not in (int, float):
        _invalid(name)
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        _invalid(name)
    return value


def _string(value, name, allow_empty=False):
    if type(value) is not str or len(value) > 512 or (not value.strip() and not (allow_empty and value == "")):
        _invalid(name)
    return value


def _frame(value, name):
    _string(value, name)
    if value.startswith('/') or any(char.isspace() for char in value):
        _invalid(name)


def _json_values(value, depth=0):
    """Reject non-JSON values and nonfinite numbers even inside diagnostics."""
    if depth > 16:
        _invalid('nesting_limit')
    if value is None or type(value) is bool:
        return
    if type(value) is str:
        if len(value) > MAX_MESSAGE_BYTES:
            _invalid('message_size')
        return
    if type(value) in (int, float):
        _number(value, 'nonfinite_number')
        return
    if type(value) is list:
        if len(value) > MAX_MESSAGE_BYTES:
            _invalid('message_size')
        for item in value:
            _json_values(item, depth + 1)
        return
    if type(value) is dict:
        if len(value) > MAX_MESSAGE_BYTES:
            _invalid('message_size')
        for key, item in value.items():
            if type(key) is not str:
                _invalid('json_key')
            _json_values(key, depth + 1)
            _json_values(item, depth + 1)
        return
    _invalid('json_value')


def _dump(value):
    try:
        encoded = json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
        if len(encoded.encode('utf-8')) > MAX_MESSAGE_BYTES:
            _invalid('message_size')
    except (TypeError, OverflowError, RecursionError, UnicodeError) as exc:
        raise ValueError('invalid_candidate_batch:json_encoding') from exc
    return encoded


def validate_candidate_batch(value, *, expected_task_id=None, expected_observation_id=None,
                             expected_frame_id=None, expected_tool_frame=None,
                             source_not_before_ns=0):
    """Validate a v1 batch and return an independent JSON-shaped copy.

    Empty task/observation strings retain standalone-node compatibility.  A
    task runner passes its nonempty expected identity to enforce round scope.
    Caller-specific target age and workspace checks remain at the executor.
    """
    _keys(value, _BATCH_KEYS)
    if type(value['schema_version']) is not int or value['schema_version'] != SCHEMA_VERSION:
        _invalid('schema_version')
    for name in ('task_id', 'observation_id'):
        _string(value[name], name, allow_empty=True)
    if bool(value['task_id']) != bool(value['observation_id']):
        _invalid('incomplete_context')
    _frame(value['frame_id'], 'frame_id')
    _frame(value['tool_frame'], 'tool_frame')
    for name, expected in (('task_id', expected_task_id), ('observation_id', expected_observation_id),
                           ('frame_id', expected_frame_id), ('tool_frame', expected_tool_frame)):
        if expected is not None and value[name] != expected:
            _invalid(name + '_mismatch')
    stamp = _integer(value['source_stamp_ns'], 'source_stamp_ns', minimum=1)
    _integer(source_not_before_ns, 'source_not_before_ns')
    if source_not_before_ns and stamp <= source_not_before_ns:
        _invalid('source_before_boundary')
    if type(value['objects']) is not list or type(value['rejected']) is not list:
        _invalid('objects_or_rejected')
    if len(value['objects']) + len(value['rejected']) > MAX_OBJECTS:
        _invalid('object_limit')
    seen = set()
    for group in value['objects']:
        _keys(group, {'object_id', 'distance', 'candidates'})
        object_id = _integer(group['object_id'], 'object_id', maximum=2**32 - 1)
        if object_id in seen:
            _invalid('duplicate_object_id')
        seen.add(object_id)
        candidates = group['candidates']
        if type(candidates) is not list or not 1 <= len(candidates) <= MAX_CANDIDATES:
            _invalid('candidate_limit')
        for index, candidate in enumerate(candidates):
            _keys(candidate, {'candidate_id', 'position', 'orientation', 'geometry'})
            if _integer(candidate['candidate_id'], 'candidate_id') != index:
                _invalid('candidate_id_order')
            for name, length in (('position', 3), ('orientation', 4)):
                vector = candidate[name]
                if type(vector) is not list or len(vector) != length:
                    _invalid(name + '_shape')
                for number in vector:
                    _number(number, name)
            if not math.isclose(math.hypot(*candidate['orientation']), 1., rel_tol=0., abs_tol=.02):
                _invalid('orientation_not_unit')
            if type(candidate['geometry']) is not dict:
                _invalid('geometry')
        distance = _number(group['distance'], 'distance')
        expected_distance = math.hypot(*candidates[0]['position'][:2])
        if distance < 0 or not math.isclose(distance, expected_distance, rel_tol=1e-6, abs_tol=1e-6):
            _invalid('distance_mismatch')
    for rejected in value['rejected']:
        _keys(rejected, {'object_id', 'reason'}, {'geometry', 'z'})
        object_id = _integer(rejected['object_id'], 'object_id', maximum=2**32 - 1)
        if object_id in seen:
            _invalid('duplicate_object_id')
        seen.add(object_id)
        _string(rejected['reason'], 'rejection_reason')
        if 'geometry' in rejected and type(rejected['geometry']) is not dict:
            _invalid('geometry')
        if 'z' in rejected:
            _number(rejected['z'], 'rejected_z')
    _json_values(value)
    return json.loads(_dump(value))


def _unique_pairs(pairs):
    value = {}
    for key, item in pairs:
        if key in value:
            _invalid('duplicate_json_key')
        value[key] = item
    return value


def decode_candidate_batch(text, **kwargs):
    """Decode with duplicate-key and NaN/Infinity rejection before validation."""
    if type(text) is not str:
        _invalid('json_text')
    try:
        if len(text.encode('utf-8')) > MAX_MESSAGE_BYTES:
            _invalid('message_size')
        value = json.loads(text, object_pairs_hook=_unique_pairs,
                           parse_constant=lambda value: _invalid('nonfinite_number'))
    except (json.JSONDecodeError, RecursionError, UnicodeError) as exc:
        raise ValueError('invalid_candidate_batch:json_text') from exc
    return validate_candidate_batch(value, **kwargs)


def encode_candidate_batch(value):
    """Emit exactly the validated protocol; JSON NaN extensions are forbidden."""
    return _dump(validate_candidate_batch(value))
