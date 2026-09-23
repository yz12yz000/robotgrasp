"""Actual ROS serialization, without creating a ROS graph."""
import struct
from types import SimpleNamespace as NS
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
from rclpy.serialization import serialize_message
from yolo_vision.sensor_input import sensor_metadata, SerializedSensor, raw_subscriber


@pytest.mark.parametrize('kind',[Image,PointCloud2])
@pytest.mark.parametrize('frame',['a','camera_color_optical_frame','base_link','相机'])
def test_cdr_header_matches_ros_deserialization(kind,frame):
    message=kind(header=Header(stamp=Time(sec=123,nanosec=456),frame_id=frame),height=800,width=1280)
    data=serialize_message(message)
    assert sensor_metadata(data)==(123_000_000_456,1280,800,frame)
    wrapped=SerializedSensor(data,kind)
    assert wrapped.header==message.header and wrapped.width==1280 and wrapped.height==800
    assert wrapped.decode()==message


def test_big_endian_header():
    data=b'\x00\x00\x00\x00'+struct.pack('>iII',123,456,4)+b'cam\x00'+struct.pack('>II',800,1280)
    assert sensor_metadata(data)==(123_000_000_456,1280,800,'cam')


@pytest.mark.parametrize('data',[b'',b'\x00'*19,b'\x00\x03'+b'\x00'*30,
    b'\x00\x01\x00\x00'+struct.pack('<iII',1,0,100)+b'\x00'*4])
def test_malformed_cdr_is_rejected(data):
    with pytest.raises(ValueError,match='invalid_sensor_cdr_header'): sensor_metadata(data)


def test_raw_callbacks_do_not_decode_payload_and_stop_after_observation(monkeypatch):
    callbacks,results,errors={},{'messages':[]},[]
    def create(kind,topic,callback,qos,**kw):
        assert kw=={'raw':True}
        callbacks[topic]=callback
        return object()
    active=[True]
    node=NS(create_subscription=create)
    subscriber=raw_subscriber(node,Image,'/image',object(),lambda:active[0],errors.append)
    subscriber.registerCallback(results['messages'].append)
    def forbidden(*args): pytest.fail('unselected sensor data must not be fully deserialized')
    monkeypatch.setattr('rclpy.serialization.deserialize_message',forbidden)
    data=serialize_message(Image(header=Header(stamp=Time(sec=1),frame_id='cam'),width=1280,height=800))
    callbacks['/image'](data)
    assert len(results['messages'])==1 and not errors
    active[0]=False
    callbacks['/image'](b'bad')
    assert len(results['messages'])==1 and not errors
    active[0]=True
    callbacks['/image'](b'bad')
    assert str(errors[0])=='invalid_sensor_cdr_header'
