"""Serialize local ROS integration fixtures before DDS discovery/initialization."""
import fcntl
import os
from pathlib import Path


class IsolatedRosTestLock:
    """A kernel-held lock closes the race between independent empty-domain probes.

    Test domains must still be empty and localhost-only. This lock only keeps
    this user's test fixtures from starting their probes simultaneously.
    """
    def __init__(self, path=None):
        path = path or Path('/tmp') / f'robot-grasp-{os.getuid()}-isolated-ros-tests.lock'
        fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(fd)
            raise RuntimeError('another_isolated_ros_test_is_running')
        self.fd = fd

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
