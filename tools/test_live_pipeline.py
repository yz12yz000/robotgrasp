#!/usr/bin/env python3
"""Run the three real nodes once against existing hardware, PLANNING ONLY.

Does not start hardware, switch controllers or execute robot/gripper commands.
Rejects existing pipeline nodes; stops only its own process groups on exit.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import argparse
import math
from threading import Event

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from moveit_msgs.msg import DisplayTrajectory


def main(execute=False):
    parser = argparse.ArgumentParser(description=("执行一次真实抓取，规划通过后自动动作。" if execute else __doc__))
    parser.add_argument('--output-dir', default=('debug_output/live_execution/' if execute else 'debug_output/live_validation/') + time.strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--timeout', type=float, default=600. if execute else 240.)
    parser.add_argument('--vision-config', default=str(Path(__file__).resolve().parents[1]/'src/yolo_vision/config/table_roi.example.json') if execute else None, help='Optional YOLO JSON, e.g. table_roi.example.json')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout 必须为有限正数")
    if args.vision_config and not Path(args.vision_config).is_file():
        parser.error("视觉配置文件不存在")
    root = Path(__file__).resolve().parents[1]
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    interrupted = Event()
    previous_signals = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous_signals:
        signal.signal(sig, lambda signum, frame: interrupted.set())
    node = Node('live_pipeline_execution' if execute else 'live_pipeline_preview_test')
    processes, logs, statuses, history, subscriptions = [], [], {}, [], []
    result = {'mode':'execute' if execute else 'plan_only', 'hardware_motion_requested':execute}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def spin_for(duration):
        end = time.monotonic()+duration
        while time.monotonic()<end:
            if interrupted.is_set(): raise RuntimeError("interrupted")
            rclpy.spin_once(node, timeout_sec=.05)
    def start(script, *arguments):
        log = (output/(script+'.log')).open('w')
        logs.append(log)
        processes.append(subprocess.Popen([str(root/script), *arguments], cwd=root,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
    def on_status(key, message):
        value = json.loads(message.data)
        if statuses.get(key) != value:
            print(key, json.dumps(value,ensure_ascii=False), flush=True)
            history.append({'node':key,'status':value})
        statuses[key] = value
    def target(message):
        result['target'] = {'frame':message.header.frame_id,'xyz':[message.point.x,message.point.y,message.point.z],
                            'stamp':message.header.stamp.sec+message.header.stamp.nanosec/1e9}
    def preview(message):
        result['preview_segments'] = [{'points':len(t.joint_trajectory.points),
             'duration':t.joint_trajectory.points[-1].time_from_start.sec +
                        t.joint_trajectory.points[-1].time_from_start.nanosec/1e9}
             for t in message.trajectory if t.joint_trajectory.points]
    try:
        spin_for(1.5)
        busy = set(node.get_node_names()) & {'grasp_executor','rim_locator','yolo_vision'}
        if busy: raise RuntimeError('先关闭已运行的项目节点：' + ', '.join(sorted(busy)))
        for key in ('grasp_executor','rim_locator','yolo_vision'):
            subscriptions.append(node.create_subscription(String,'/'+key+'/status',lambda m,k=key:on_status(k,m),qos))
        subscriptions.append(node.create_subscription(PointStamped,'/rim_locator/rim_point',target,qos))
        subscriptions.append(node.create_subscription(DisplayTrajectory,'/display_planned_path',preview,qos))
        if execute:
            print('真实抓取：检查并启用位置控制器；识别和规划通过后会自动动作。', flush=True)
            subprocess.run([str(root/'run_grasp_position_control.sh')], cwd=root, check=True, timeout=240)
        if interrupted.is_set(): raise RuntimeError('interrupted')
        start('run_grasp_executor.sh','--execute' if execute else '--plan-only')
        start('run_rim_locator.sh')
        ready_until = time.monotonic()+25
        while statuses.get('grasp_executor',{}).get('status') != 'waiting_target' or statuses.get('rim_locator',{}).get('status') != 'waiting_input':
            if time.monotonic()>ready_until:raise TimeoutError('pipeline_startup_timeout')
            if any(s.get('status')=='failed' for s in statuses.values()):raise RuntimeError('pipeline_startup_failed')
            spin_for(.1)
        start('run_yolo_vision.sh', *(['config_path:='+str(Path(args.vision_config).resolve())]
                                    if args.vision_config else []))
        deadline=time.monotonic()+args.timeout
        while time.monotonic()<deadline:
            spin_for(.1)
            if any(s.get('status')=='failed' for s in statuses.values()):break
            grasp = statuses.get('grasp_executor',{})
            if grasp.get('failed_step') == 'validating' and grasp.get('reason'):
                result['target_rejected'] = grasp['reason']
                break
            if statuses.get('grasp_executor',{}).get('status') == ('success' if execute else 'planned'):break
            if any(p.poll() is not None for p in processes):raise RuntimeError('child_process_exited')
        else:result['timeout']=True
        spin_for(.5)
        result['passed'] = (statuses.get('yolo_vision',{}).get('status')=='success' and
                            statuses.get('rim_locator',{}).get('status')=='success' and
                            statuses.get('grasp_executor',{}).get('status')==('success' if execute else 'planned') and
                            len(result.get('preview_segments',[]))==3)
    except Exception as exc:
        result['error']=str(exc)
        result['passed']=False
    finally:
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:
                end = time.monotonic()+20
                while p.poll() is None and time.monotonic()<end:
                    rclpy.spin_once(node, timeout_sec=.05)
                p.wait(timeout=.1)
            except subprocess.TimeoutExpired:
                result["forced_shutdown"] = True
                result["passed"] = False
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        result['statuses'],result['history']=statuses,history
        (output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
        for sig, handler in previous_signals.items(): signal.signal(sig, handler)
    print('Result:',output/'result.json',flush=True)
    return 0 if result.get('passed') else 1


if __name__ == '__main__':raise SystemExit(main())
