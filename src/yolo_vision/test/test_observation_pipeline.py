"""0/1/2/3 observations through real cloud, localization and batch execution code."""
from dataclasses import replace
from types import SimpleNamespace as NS
import numpy as np
import pytest
import time
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from rclpy.serialization import serialize_message
from yolo_vision import pointcloud
from yolo_vision.sensor_input import SerializedSensor
from rim_locator.main import RimNode
from rim_locator.config import RimConfig
from grasp_executor.main import GraspNode
from grasp_executor.config import GraspConfig
from grasp_executor.grasp import run_batch_task
from test_retry import FakeNode


class Arm:
    def __init__(self): self.plans,self.moves,self.gripper=[],[],[]
    def check_ready(self,timeout): return True
    def prepare_plans(self,*poses): self.plans.append(poses);return True
    def move_to_pose(self,pose,timeout): self.moves.append(pose);return True
    def move_linear(self,pose,timeout): self.moves.append(pose);return True
    def is_pose_reached(self,timeout): return True
    def open_gripper(self,timeout): self.gripper.append('Open');return True
    def close_gripper(self,timeout): self.gripper.append('Close');return True
    def stop_motion(self,timeout): pytest.fail('unexpected motion failure')


@pytest.mark.parametrize('count',[0,1,2,3])
@pytest.mark.parametrize('execute',[False,True])
def test_observe_localize_grasp_and_place_all(monkeypatch,tmp_path,count,execute):
    real_monotonic = time.monotonic
    theta,depth=np.meshgrid(np.linspace(0,2*np.pi,240,endpoint=False),np.linspace(0,.05,40))
    centers=[.65,.35,.50][:count]
    clouds=[np.stack((cx+.085*np.cos(theta),.1+.085*np.sin(theta),.3-depth),axis=-1) for cx in centers]
    xyz=np.concatenate(clouds,axis=1) if count else np.zeros((40,240,3))
    targets=[]
    for index in range(count):
        mask=np.zeros(xyz.shape[:2],dtype=bool);mask[:,index*240:(index+1)*240]=True
        targets.append(NS(mask=mask,confidence=.9-index*.1))
    node=FakeNode(monkeypatch,[targets])
    node.config=replace(node.config,expected_height=xyz.shape[0],expected_width=xyz.shape[1],min_points=30)
    node.bridge=NS(imgmsg_to_cv2=lambda *a,**kw:np.zeros((*xyz.shape[:2],3),dtype=np.uint8))
    # Use real PointCloud2 decoding here, not the callback-unit-test stub.
    monkeypatch.setattr('yolo_vision.main.read_xyz',pointcloud.read_xyz)
    header=Header(stamp=Time(sec=99,nanosec=42),frame_id='base_link')
    cloud=pointcloud.make_cloud(header,xyz.reshape(-1,3))
    cloud.height,cloud.width=xyz.shape[:2];cloud.row_step=cloud.width*cloud.point_step
    image=Image(header=header,height=cloud.height,width=cloud.width)
    node.on_pair(SerializedSensor(serialize_message(image),Image),
                 SerializedSensor(serialize_message(cloud),type(cloud)))
    monkeypatch.setattr(time, 'monotonic', real_monotonic)
    assert node.reports[-1]['detected_count']==count
    arrays=[]
    class Locator:
        on_cloud=RimNode.on_cloud
        finish=RimNode.finish
        def report(self,state,**kw): self.state=state;self.result=kw
    locator=Locator();locator.state='waiting_input';locator.config=RimConfig(orientation_mode='wall_normal',publish_debug=False)
    locator.poses_publisher=NS(publish=arrays.append);locator.publisher=NS(publish=lambda m:None)
    if count:
        assert node.state=='success' and len(node.messages)==1
        locator.on_cloud(node.messages[0])
        assert locator.state=='success'
        assert len(arrays)==1 and arrays[0].header.stamp==header.stamp
        assert len(arrays[0].poses)==count
    else:
        assert node.state=='no_targets' and not node.messages
    config=GraspConfig(plan_only=not execute,place_dwell_time=.001,execution_journal=str(tmp_path))
    poses=[GraspNode.grasp_pose_from_ros(p,config) for p in arrays[0].poses] if count else []
    arm=Arm()
    outcome=run_batch_task(poses,99_000_000_042,lambda:100_000_000_000,arm,config)
    assert outcome.status==('success' if execute else 'planned') if count else outcome.status=='no_targets'
    assert outcome.object_count==count and outcome.completed_count==(count if execute else 0)
    assert len(arm.plans)==count and len(arm.moves)==(6*count if execute else 0)
    positions=[plan[1].position for plan in arm.plans]
    assert [np.hypot(*p[:2]) for p in positions]==sorted(np.hypot(*p[:2]) for p in positions)
    assert arm.gripper==['Open','Close','Open']*count if execute else not arm.gripper
    if count==3: assert locator.result['ordered_object_ids']==[1,2,0]
