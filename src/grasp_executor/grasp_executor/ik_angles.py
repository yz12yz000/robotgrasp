"""Choose equivalent rotary IK angles near measured joints, within URDF limits."""
import math


def revolute_bounds(joint):
    """Match MoveIt's URDF safety limits instead of using hard stops only."""
    if joint is None or joint.get('type') != 'revolute':
        raise ValueError('expected_revolute_ur_joint')
    limit = joint.find('limit')
    if limit is None:
        raise ValueError('joint_limit_missing')
    low, high, velocity = (float(limit.get(k)) for k in ('lower', 'upper', 'velocity'))
    safety = joint.find('safety_controller')
    if safety is not None:
        low = max(low, float(safety.get('soft_lower_limit', low)))
        high = min(high, float(safety.get('soft_upper_limit', high)))
    if not all(math.isfinite(v) for v in (low, high, velocity)) or low >= high or velocity <= 0:
        raise ValueError('invalid_joint_limits')
    return low, high, velocity


def nearest_equivalent_angles(solution, reference, limits):
    adjusted = {}
    for name, value in solution.items():
        low, high = limits[name][:2]
        start = reference[name]
        if not all(math.isfinite(v) for v in (value, start, low, high)) or low >= high:
            raise ValueError('invalid_ik_joint_limits:' + name)
        first = math.ceil((low-value)/math.tau)
        last = math.floor((high-value)/math.tau)
        if first > last:
            raise ValueError('ik_joint_outside_limits:' + name)
        turns = min(last, max(first, round((start-value)/math.tau)))
        adjusted[name] = value + turns*math.tau
    return adjusted
