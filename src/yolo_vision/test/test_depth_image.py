"""Depth units, pixel correspondence and calibration guards."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from dataclasses import replace
from yolo_vision.pointcloud import read_aligned_depth_xyz, extract_object_points
from test_retry import FakeNode,result,pair


def fixtures(encoding='16UC1',big=False,padded=False):
    header=Header(frame_id='cam',stamp=Time(sec=99))
    dtype=('>' if big else '<')+('u2' if encoding=='16UC1' else 'f4')
    row=np.array([[1000,2000,0],[3000,4000,5000]],dtype=dtype)
    if encoding=='32FC1': row/=1000
    data=b''.join(r.tobytes()+(b'\x00'*4 if padded else b'') for r in row)
    depth=Image(header=header,height=2,width=3,encoding=encoding,is_bigendian=int(big),
                step=3*row.dtype.itemsize+(4 if padded else 0),data=data)
    info=CameraInfo(header=header,height=2,width=3,k=[2.,0.,1.,0.,4.,.5,0.,0.,1.],d=[0.]*5)
    return depth,info


@pytest.mark.parametrize('encoding',['16UC1','32FC1'])
@pytest.mark.parametrize('big',[False,True])
@pytest.mark.parametrize('padded',[False,True])
def test_projection_preserves_original_pixels_and_metres(encoding,big,padded):
    depth,info=fixtures(encoding,big,padded)
    xyz=read_aligned_depth_xyz(depth,info)
    np.testing.assert_allclose(xyz,[[[-.5,-.125,1.],[0.,-.25,2.],[0.,0.,0.]],
                                    [[-1.5,.375,3.],[0.,.5,4.],[2.5,.625,5.]]],atol=1e-7)
    mask=np.zeros((2,3),dtype=bool);mask[1,2]=True
    np.testing.assert_allclose(extract_object_points(xyz,mask,1),[[2.5,.625,5.]])


@pytest.mark.parametrize('fault,reason',[
    ('missing','depth_camera_info_missing'),('frame','frame_mismatch'),('size','dimension_mismatch'),
    ('focal','invalid_depth_intrinsics'),('nan','invalid_depth_intrinsics'),('skew','invalid_depth_intrinsics'),
    ('distortion','depth_must_be_rectified'),('encoding','unsupported_depth_encoding'),
    ('stride','invalid_depth_buffer'),('truncated','invalid_depth_buffer')])
def test_bad_depth_or_calibration_rejected(fault,reason):
    depth,info=fixtures()
    if fault=='missing': info=None
    if fault=='frame': info.header=Header(frame_id='other')
    if fault=='size': info.width=4
    if fault=='focal': info.k[0]=0.
    if fault=='nan': info.k[0]=float('nan')
    if fault=='skew': info.k[1]=1.
    if fault=='distortion': info.d[0]=.1
    if fault=='encoding': depth.encoding='mono16'
    if fault=='stride': depth.step=1
    if fault=='truncated': depth.data=depth.data[:-1]
    with pytest.raises(ValueError,match=reason): read_aligned_depth_xyz(depth,info)


def test_registered_depth_callback_uses_acquisition_stamp(monkeypatch):
    node=FakeNode(monkeypatch,[result(np.ones((2,3),dtype=bool))])
    node.config=replace(node.config,expected_height=2,expected_width=3,depth_image_topic='/depth')
    depth,info=fixtures()
    image=NS(header=depth.header,width=3,height=2)
    node.on_pair(image,depth,info)
    assert node.state=='success' and len(node.messages)==1
    assert node.messages[0].header==depth.header
    assert node.messages[0].width==5  # The zero depth pixel is excluded.


def test_old_intrinsics_stamp_cannot_be_used(monkeypatch):
    node=FakeNode(monkeypatch,[result()])
    node.config=replace(node.config,expected_height=2,expected_width=3,depth_image_topic='/depth')
    depth,info=fixtures();info.header=Header(frame_id='cam',stamp=Time(sec=98))
    node.on_pair(NS(header=depth.header,width=3,height=2),depth,info)
    assert node.state=='failed' and node.reports[-1]['reason']=='depth_camera_info_not_synchronized'
    assert node.calls==0
