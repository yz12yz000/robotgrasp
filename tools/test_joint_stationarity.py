from collections import deque
import math
import pytest
from robot_control import position_window_stationary


def test_stationary_encoder_noise_at_500hz():
    samples=deque()
    outcomes=[]
    for i in range(301):
        outcomes.append(position_window_stationary(samples, 1_000_000_000+i*2_000_000,
                                                  (0.0001*(-1)**i,)*6))
    assert outcomes[-1] and not outcomes[0]
    assert len(samples)<=52


@pytest.mark.parametrize('trajectory', [lambda t:.03*t, lambda t:.003*math.sin(2*math.pi*10*t)])
def test_motion_and_returning_oscillation_rejected(trajectory):
    samples=deque()
    with pytest.raises(ValueError,match='位置仍在变化'):
        for i in range(101):
            position_window_stationary(samples,1_000_000_000+i*2_000_000,(trajectory(i*.002),)*6)


@pytest.mark.parametrize('stamp',[1_000_000_000,999_000_000])
def test_duplicate_or_backward_time_rejected(stamp):
    samples=deque([(1_000_000_000,(0.,)*6)])
    with pytest.raises(ValueError,match='未递增'):
        position_window_stationary(samples,stamp,(0.,)*6)
