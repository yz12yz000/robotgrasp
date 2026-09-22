"""Test orchestration with fake ROS/processes; never open a ROS context."""
import signal
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest
import test_live_pipeline as runner


@pytest.mark.parametrize('execute,abort', [(False,False),(True,False),(True,True)])
def test_pipeline_modes_and_cleanup(monkeypatch,tmp_path,execute,abort):
    import json
    clock=[0.]
    subscriptions={}
    processes=[]
    calls=[]
    phase=[0]
    def status(name,value):
        subscriptions['/'+name+'/status'](NS(data=json.dumps({'status':value})))
    class Node:
        def __init__(self,*args):pass
        def get_node_names(self):return []
        def create_subscription(self,typ,topic,callback,qos): subscriptions[topic]=callback;return callback
        def destroy_node(self):pass
    class Process:
        def __init__(self,cmd,**kwargs):
            self.cmd=cmd;self.pid=1000+len(processes);self.returncode=None
            processes.append(self);calls.append(cmd)
            if cmd[0].endswith('run_grasp_executor.sh'):status('grasp_executor','waiting_target')
            if cmd[0].endswith('run_rim_locator.sh'):status('rim_locator','waiting_input')
        def poll(self):return self.returncode
        def wait(self,timeout):
            if self.returncode is None:raise subprocess.TimeoutExpired(self.cmd,timeout)
            return self.returncode
    def spin(node,timeout_sec):
        clock[0]+=.1
        if len(processes)==3 and phase[0]==0:
            status('yolo_vision','success');status('rim_locator','success');status('grasp_executor','planned')
            point=NS(time_from_start=NS(sec=1,nanosec=0))
            subscriptions['/display_planned_path'](NS(trajectory=[NS(joint_trajectory=NS(points=[point]))]*3))
            phase[0]=1
        elif len(processes)==3 and phase[0]==1 and execute:
            phase[0]=2
            if abort:signal.getsignal(signal.SIGINT)(signal.SIGINT,None)
            else:status('grasp_executor','success')
    def kill(pid,sig):
        assert sig==signal.SIGINT
        next(p for p in processes if p.pid==pid).returncode=0
    monkeypatch.setattr(runner,'Node',Node)
    monkeypatch.setattr(runner.rclpy,'init',lambda **kw:None)
    monkeypatch.setattr(runner.rclpy,'shutdown',lambda:None)
    monkeypatch.setattr(runner.rclpy,'spin_once',spin)
    monkeypatch.setattr(runner.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(runner.subprocess,'Popen',Process)
    monkeypatch.setattr(runner.subprocess,'run',lambda cmd,**kw:calls.append(cmd))
    monkeypatch.setattr(runner.os,'killpg',kill)
    monkeypatch.setattr(sys,'argv',['test','--output-dir',str(tmp_path)])
    assert runner.main(execute=execute)==(1 if abort else 0)
    commands=[(c[0].split('/')[-1],c[1:]) for c in calls]
    if execute:
        assert commands[0][0]=='run_grasp_position_control.sh'
        commands=commands[1:]
    assert commands[0]==('run_grasp_executor.sh',['--execute' if execute else '--plan-only'])
    assert [c[0] for c in commands]==['run_grasp_executor.sh','run_rim_locator.sh','run_yolo_vision.sh']
    result=json.loads((tmp_path/'result.json').read_text())
    assert result['mode']==('execute' if execute else 'plan_only')
    assert result['passed']==(not abort)
    assert all(p.poll()==0 for p in processes)


def test_preview_cli_cannot_enable_execution(monkeypatch):
    monkeypatch.setattr(sys,'argv',['test','--execute'])
    monkeypatch.setattr(runner.rclpy,'init',lambda **kw:pytest.fail('ROS must not start'))
    with pytest.raises(SystemExit) as error:runner.main()
    assert error.value.code==2


def test_invalid_timeout_rejected_before_ros(monkeypatch):
    monkeypatch.setattr(sys,'argv',['test','--timeout','nan'])
    monkeypatch.setattr(runner.rclpy,'init',lambda **kw:pytest.fail('ROS must not start'))
    with pytest.raises(SystemExit):runner.main(execute=True)


@pytest.mark.parametrize('busy', [True,False])
def test_busy_or_controller_failure_never_starts_pipeline(monkeypatch,tmp_path,busy):
    clock=[0.]
    class Node:
        def __init__(self,*args):pass
        def get_node_names(self):return ['grasp_executor'] if busy else []
        def create_subscription(self,*args):return None
        def destroy_node(self):pass
    monkeypatch.setattr(runner,'Node',Node)
    monkeypatch.setattr(runner.rclpy,'init',lambda **kw:None)
    monkeypatch.setattr(runner.rclpy,'shutdown',lambda:None)
    monkeypatch.setattr(runner.rclpy,'spin_once',lambda *a,**kw:clock.__setitem__(0,clock[0]+.1))
    monkeypatch.setattr(runner.time,'monotonic',lambda:clock[0])
    def failed_controller(cmd,**kwargs):
        if busy:pytest.fail('Must not switch with existing grasp node')
        raise subprocess.CalledProcessError(1,cmd)
    monkeypatch.setattr(runner.subprocess,'run',failed_controller)
    monkeypatch.setattr(runner.subprocess,'Popen',lambda *a,**kw:pytest.fail('Must not launch motion nodes'))
    monkeypatch.setattr(sys,'argv',['test','--output-dir',str(tmp_path)])
    assert runner.main(execute=True)==1
