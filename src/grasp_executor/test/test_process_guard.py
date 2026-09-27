"""Supervisor loss stops the existing worker; no ROS graph or robot is used."""
import json
import os
from queue import Queue
import subprocess
import sys
from threading import Event
from types import SimpleNamespace as NS

import pytest

from grasp_executor.main import GraspNode
from grasp_executor.process_guard import process_start_ticks, supervisor_alive
from test_candidate_execution import Candidates, run


def test_pid_identity_detects_exit_and_reuse():
    assert supervisor_alive(0, 0)
    pid = os.getpid()
    ticks = process_start_ticks(pid)
    assert supervisor_alive(pid, ticks)
    assert not supervisor_alive(pid, ticks + 1)
    child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
    try:
        ticks = process_start_ticks(child.pid)
        assert supervisor_alive(child.pid, ticks)
        child.kill()
        child.wait(timeout=3)
        assert not supervisor_alive(child.pid, ticks)
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=3)


def fake_node(worker=None):
    reports = []
    node = NS(supervisor_pid=os.getpid(), supervisor_start_ticks=process_start_ticks(os.getpid()) + 1,
              supervisor_lost=False, deadline_expired=False, worker=worker, cancel=Event(),
              state='waiting_target', events=Queue(), task_deadline_monotonic_ns=0,
              wait_started=0., config=NS(input_timeout=30.))
    def report(status, **details):
        node.state = status
        reports.append(dict(status=status, **details))
    node.report = report
    node.check_supervisor = lambda: GraspNode.check_supervisor(node)
    return node, reports


def test_lost_supervisor_prevents_input_and_fails_waiting_node():
    node, reports = fake_node()
    GraspNode.on_candidate_groups(node, NS(data='must_not_be_decoded'))
    assert node.cancel.is_set() and node.supervisor_lost
    assert reports == [dict(status='failed', failed_step='waiting_target', reason='supervisor_lost',
                            command_attempted=False)]


def test_steady_timer_cancels_active_worker_without_claiming_it_has_stopped():
    node, reports = fake_node(worker=object())
    node.state = 'moving_to_grasp'
    GraspNode.tick(node)
    assert node.cancel.is_set() and node.supervisor_lost and reports == []


def test_supervisor_loss_during_execution_does_not_release_or_start_next_segment(tmp_path):
    node, _ = fake_node(worker=object())
    interface = Candidates()
    def state(event):
        if event['status'] == 'moving_to_place_ready':
            GraspNode.check_supervisor(node)
    result = run(tmp_path, interface, cancel=node.cancel, on_state=state)
    assert result.status == 'failed' and result.reason == 'cancelled'
    assert result.failed_step == 'moving_to_place_ready'
    assert len(interface.moves) == 3
    assert interface.events.count('open') == interface.events.count('close') == 1
    assert interface.events.count('stop') == 1
    assert result.completed_count == 0 and result.command_attempted


def test_failure_keeps_specific_supervisor_reason():
    payloads = []
    node = NS(state='moving', task_id='task', observation_id='round', supervisor_lost=True,
              deadline_expired=False, config=NS(plan_only=False),
              status_pub=NS(publish=lambda msg: payloads.append(json.loads(msg.data))),
              get_logger=lambda: NS(info=lambda msg: None))
    GraspNode.report(node, 'failed', reason='cancelled', stop_error='stopped')
    assert payloads[0]['reason'] == 'supervisor_lost'
    assert payloads[0]['stop_error'] == 'stopped'
