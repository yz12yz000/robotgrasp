"""Linux process identity for stopping a task when its supervisor disappears."""
from pathlib import Path


def process_start_ticks(pid):
    if type(pid) is not int or pid <= 0:
        raise ValueError('invalid_supervisor_pid')
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    if fields[0] in ('Z', 'X'):
        raise ProcessLookupError('supervisor_exited')
    return int(fields[19])  # Linux /proc stat field 22; comm may contain spaces.


def supervisor_alive(pid, start_ticks):
    if pid == 0 and start_ticks == 0:
        return True  # Standalone legacy entry has no external supervisor.
    try:
        return process_start_ticks(pid) == start_ticks
    except (OSError, ValueError, IndexError):
        return False
