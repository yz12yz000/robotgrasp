"""Offline automatic-count runner tests; no device or real subprocess opened."""
import json
import signal
import sys
from pathlib import Path
from dataclasses import asdict
from types import SimpleNamespace as NS
import pytest
import test_live_place_pipeline as runner


def run_fake(monkeypatch, tmp_path, count, execute=False, new_run=False,
             previews=None, fault=None, rejected=False, intermediate=False):
    clock, subscriptions, processes, commands = [0.], {}, [], []
    observed, finished = [False], [False]
    root = Path(runner.__file__).resolve().parents[1]
    sources = [root/f'src/{name}/config/place_config.json' for name in ('yolo_vision','rim_locator','grasp_executor')]
    # An old false flag must never discard all but one observed target.
    override=tmp_path/'vision.json'
    override.write_text('{"multi_instance":false}')
    originals = {p:p.read_bytes() for p in sources+[override]}
    monkeypatch.chdir(root/'New_run' if new_run else root)
    from os.path import relpath
    relative = relpath(override, Path.cwd())
    def status(name, value, **details):
        subscriptions['/'+name+'/status'](NS(data=json.dumps({'status':value,**details})))
    class Node:
        def __init__(self,*a): pass
        def get_node_names(self): return []
        def create_subscription(self,kind,topic,callback,qos): subscriptions[topic]=callback
        def destroy_node(self): pass
    class Process:
        def __init__(self,cmd,**kw):
            self.pid=1000+len(processes); self.code=None
            processes.append(self); commands.append(cmd)
            if cmd[0].endswith('run_grasp_executor.sh'):
                assert observed[0] and count > 0
                status('grasp_executor','waiting_target')
            if cmd[0].endswith('run_rim_locator.sh'): status('rim_locator','waiting_input')
        def poll(self): return self.code
        def wait(self,timeout): assert self.code==0
    def spin(node,timeout_sec):
        clock[0]+=.1
        # Delayed observation also verifies downstream wait exceeds the old 60 s.
        if len(processes)==2 and not observed[0] and clock[0]>=70:
            observed[0]=True
            if fault:
                status('yolo_vision','failed',reason=fault)
            elif count==0:
                status('yolo_vision','no_targets',detected_count=0,object_count=0)
            else:
                status('yolo_vision','success',detected_count=count+(1 if rejected else 0),object_count=count,
                       rejected=[{'object_id':count,'reason':'insufficient_pointcloud'}] if rejected else [])
                subscriptions['/rim_locator/grasp_poses'](NS(header=NS(frame_id='base_link',stamp=NS(sec=1,nanosec=0)),poses=list(range(count))))
                status('rim_locator','success',object_count=count)
        if len(processes)==3 and not finished[0]:
            if intermediate:
                for _ in range(count*6):
                    subscriptions['/display_planned_path'](NS(trajectory=[NS(joint_trajectory=NS(points=[1,2]))]))
            for _ in range(count if previews is None else previews):
                subscriptions['/display_planned_path'](NS(trajectory=[NS(joint_trajectory=NS(points=[1,2]))]*6))
            status('grasp_executor','success' if execute else 'planned',completed_count=count if execute else 0)
            finished[0]=True
    def kill(pid,sig):
        assert sig==signal.SIGINT
        next(p for p in processes if p.pid==pid).code=0
    def control(cmd,**kw):
        assert observed[0] and count>0 and not fault
        commands.append(cmd)
    monkeypatch.setattr(runner,'Node',Node)
    monkeypatch.setattr(runner.rclpy,'init',lambda:None)
    monkeypatch.setattr(runner.rclpy,'shutdown',lambda:None)
    monkeypatch.setattr(runner.rclpy,'spin_once',spin)
    monkeypatch.setattr(runner.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(runner.subprocess,'Popen',Process)
    monkeypatch.setattr(runner.subprocess,'run',control)
    monkeypatch.setattr(runner.os,'killpg',kill)
    output=tmp_path/'output'
    monkeypatch.setattr(sys,'argv',['test','--output-dir',str(output),'--vision-config',relative,
                                  '--timeout','120']+(['--execute'] if execute else []))
    code=runner.main()
    result=json.loads((output/'result.json').read_text())
    assert {p:p.read_bytes() for p in originals} == originals
    assert result['selection']=='all_detected' and 'multi_instance' not in result
    assert result['mode']==('execute' if execute else 'plan_only')
    assert result['vision_config_sources']['override']==str(override)
    for name, loader, source in [('rim',runner.load_rim_config,sources[1]),
                                  ('grasp',runner.load_grasp_config,sources[2])]:
        path=Path(result['config_snapshots'][name])
        assert path.is_absolute()
        expected=asdict(loader(source));expected['input_timeout']=max(expected['input_timeout'],155.)
        assert json.loads(path.read_text())==expected
    vision_path=Path(result['effective_vision_config'])
    vision=json.loads(vision_path.read_text())
    assert 'multi_instance' not in vision
    assert vision['inference_roi'] is None  # Full view by default.
    assert f'config_path:={vision_path}' in next(c for c in commands if c[0].endswith('run_yolo_vision.sh'))
    return code,result,commands


@pytest.mark.parametrize('count',[0,1,2,3])
@pytest.mark.parametrize('execute',[False,True])
@pytest.mark.parametrize('new_run',[False,True])
def test_observed_count_controls_whole_batch(monkeypatch,tmp_path,count,execute,new_run):
    code,result,commands=run_fake(monkeypatch,tmp_path,count,execute,new_run)
    assert code==0 and result['passed']
    assert result['outcome']==('completed' if count else 'no_targets')
    assert result['detected_count']==result['target_count']==count
    assert result['completed_count']==(count if execute else 0)
    assert len(result['preview_batches'])==count
    executors=[c for c in commands if c[0].endswith('run_grasp_executor.sh')]
    assert len(executors)==bool(count)
    assert any(c[0].endswith('run_grasp_position_control.sh') for c in commands)==(execute and count>0)
    assert commands[0][0].endswith('run_rim_locator.sh')
    assert commands[1][0].endswith('run_yolo_vision.sh')
    if count:
        assert executors[0][1]==('--execute' if execute else '--plan-only')


def test_missing_preview_is_not_success(monkeypatch,tmp_path):
    code,result,_=run_fake(monkeypatch,tmp_path,3,previews=2,intermediate=True)
    assert code==1 and not result['passed']
    assert result['error']=='preview_batch_mismatch'


@pytest.mark.parametrize('execute',[False,True])
def test_moveit_single_segment_displays_do_not_invalidate_complete_batches(monkeypatch,tmp_path,execute):
    code,result,_=run_fake(monkeypatch,tmp_path,2,execute=execute,intermediate=True)
    assert code==0 and result['passed']
    assert len(result['preview_batches'])==2
    assert len(result['intermediate_previews'])==12


def test_rejected_target_is_reported_as_partial(monkeypatch,tmp_path):
    code,result,_=run_fake(monkeypatch,tmp_path,2,execute=True,rejected=True)
    assert code==1 and result['outcome']=='partial'
    assert result['detected_count']==3 and result['completed_count']==2
    assert result['error']=='rejected_targets'


@pytest.mark.parametrize('reason',['input_timeout','no_valid_depth_for_detected_targets'])
def test_camera_or_depth_failure_never_starts_execution(monkeypatch,tmp_path,reason):
    code,result,commands=run_fake(monkeypatch,tmp_path,0,execute=True,fault=reason)
    assert code==1 and result['outcome']=='failed'
    assert result['failures']['yolo_vision']['reason']==reason
    assert len(commands)==2


@pytest.mark.parametrize('fault',['single','multi','string_bool','unknown','bad_json','missing','timeout'])
def test_invalid_config_fails_before_ros_or_processes(monkeypatch,tmp_path,fault):
    def forbidden(*a,**kw): pytest.fail('invalid input must fail before ROS or processes')
    monkeypatch.setattr(runner.rclpy,'init',forbidden)
    monkeypatch.setattr(runner.subprocess,'Popen',forbidden)
    monkeypatch.setattr(runner.subprocess,'run',forbidden)
    config=tmp_path/'override.json'
    if fault=='string_bool': config.write_text('{"multi_instance":"false"}')
    if fault=='unknown': config.write_text('{"unknown":true}')
    if fault=='bad_json': config.write_text('{')
    args=['test','--output-dir',str(tmp_path/'out'),'--execute']
    args += (['--'+fault] if fault in ('single','multi') else ['--timeout','nan'] if fault=='timeout'
             else ['--vision-config',str(config)])
    monkeypatch.setattr(sys,'argv',args)
    with pytest.raises(SystemExit) as error: runner.main()
    assert error.value.code==2 and not (tmp_path/'out').exists()


@pytest.mark.parametrize('symlink',[False,True])
def test_snapshot_cannot_overwrite_source_config(monkeypatch,tmp_path,symlink):
    output=tmp_path/'out';output.mkdir()
    config=tmp_path/'source.json' if symlink else output/'effective_vision_config.json'
    original='{"multi_instance":false}';config.write_text(original)
    if symlink: (output/'effective_vision_config.json').symlink_to(config)
    def forbidden(*a,**kw): pytest.fail('snapshot collision must fail before ROS or processes')
    monkeypatch.setattr(runner.rclpy,'init',forbidden)
    monkeypatch.setattr(runner.subprocess,'Popen',forbidden)
    monkeypatch.setattr(runner.subprocess,'run',forbidden)
    monkeypatch.setattr(sys,'argv',['test','--output-dir',str(output),'--vision-config',str(config)])
    with pytest.raises(SystemExit) as error: runner.main()
    assert error.value.code==2 and config.read_text()==original
    assert not (output/'effective_rim_config.json').exists()
