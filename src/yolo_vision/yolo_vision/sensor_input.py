"""Read only CDR sensor headers until a synchronized pair is selected.

Converting every multi-megabyte uint8[] into a ROS Python message can starve
the other subscription in a single-threaded executor. Header inspection is
constant-size; full deserialization happens for the selected observation only.
"""
import struct


def sensor_metadata(data):
    """Return stamp_ns, width, height, frame for ROS Image/PointCloud2 CDR1."""
    if len(data) < 20 or data[:2] not in (b'\x00\x00', b'\x00\x01'):
        raise ValueError('invalid_sensor_cdr_header')
    endian = '<' if data[1] else '>'
    sec, nanosec, size = struct.unpack_from(endian + 'iII', data, 4)
    if size < 1 or size > len(data) - 16 or nanosec >= 1_000_000_000:
        raise ValueError('invalid_sensor_cdr_header')
    end = 16 + size
    dimensions = (end + 3) & ~3
    if data[end-1] != 0 or dimensions + 8 > len(data):
        raise ValueError('invalid_sensor_cdr_header')
    frame = bytes(data[16:end-1]).decode('utf-8')
    height, width = struct.unpack_from(endian + 'II', data, dimensions)
    return sec * 1_000_000_000 + nanosec, width, height, frame


class SerializedSensor:
    def __init__(self, data, message_type):
        from builtin_interfaces.msg import Time
        from std_msgs.msg import Header
        stamp, self.width, self.height, frame = sensor_metadata(data)
        self.header = Header(stamp=Time(sec=stamp // 1_000_000_000,
                                       nanosec=stamp % 1_000_000_000), frame_id=frame)
        self.data, self.message_type = data, message_type

    def decode(self):
        from rclpy.serialization import deserialize_message
        return deserialize_message(self.data, self.message_type)


def raw_subscriber(node, message_type, topic, qos, accept, on_error):
    from message_filters import SimpleFilter
    output = SimpleFilter()

    def receive(data):
        if not accept():
            return
        try:
            message = SerializedSensor(data, message_type)
        except (ValueError, struct.error) as exc:
            on_error(exc)
            return
        output.signalMessage(message)

    output.subscription = node.create_subscription(message_type, topic, receive, qos, raw=True)
    return output
