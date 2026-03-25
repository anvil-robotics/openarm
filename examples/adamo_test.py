"""ROS 2 node that bridges Adamo XR tracking data to /head_pose."""

from __future__ import annotations

import struct
import threading

import adamo
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def decode_cdr_envelope(data: bytes) -> tuple[str, str, bytes] | None:
    """Decode CDR-wrapped ROS message.

    Wire format: [u32be topic_len][topic_utf8][u32be type_len][type_utf8][cdr_payload]

    Returns (topic, type_name, cdr_payload) or None if not CDR-encoded.
    """
    if len(data) < 8:
        return None
    try:
        offset = 0
        topic_len = struct.unpack(">I", data[offset : offset + 4])[0]
        offset += 4
        if offset + topic_len > len(data):
            return None
        topic = data[offset : offset + topic_len].decode("utf-8")
        offset += topic_len

        type_len = struct.unpack(">I", data[offset : offset + 4])[0]
        offset += 4
        if offset + type_len > len(data):
            return None
        type_name = data[offset : offset + type_len].decode("utf-8")
        offset += type_len

        cdr_payload = data[offset:]
        return (topic, type_name, cdr_payload)
    except Exception:
        return None


def _skip_cdr_header(cdr: bytes) -> int:
    """Skip CDR header (4 bytes) + ROS Header (stamp + frame_id), return offset."""
    off = 4
    off += 8  # stamp (sec + nanosec)
    fid_len = struct.unpack_from("<I", cdr, off)[0]
    off += 4 + fid_len
    off = (off + 3) & ~3  # align to 4 bytes
    return off


def decode_pose_stamped(cdr: bytes) -> dict | None:
    """Decode geometry_msgs/msg/PoseStamped from CDR (little-endian).

    Returns dict with position [x,y,z] and quaternion [x,y,z,w] (ROS order).
    """
    try:
        off = _skip_cdr_header(cdr)
        if off + 56 > len(cdr):
            return None
        values = struct.unpack_from("<7d", cdr, off)
        return {
            "position": list(values[0:3]),
            "quaternion": list(values[3:7]),
        }
    except Exception:
        return None


def decode_joy(cdr: bytes) -> dict | None:
    """Decode sensor_msgs/msg/Joy from CDR (little-endian).

    Returns dict with axes (list[float]) and buttons (list[int]).
    """
    try:
        off = _skip_cdr_header(cdr)
        n_axes = struct.unpack_from("<I", cdr, off)[0]
        off += 4
        if n_axes > 100:
            return None
        axes = list(struct.unpack_from(f"<{n_axes}f", cdr, off))
        off += n_axes * 4
        n_buttons = struct.unpack_from("<I", cdr, off)[0]
        off += 4
        if n_buttons > 100:
            return None
        buttons = list(struct.unpack_from(f"<{n_buttons}i", cdr, off))
        return {"axes": axes, "buttons": buttons}
    except Exception:
        return None


class AdamoBridge(Node):
    def __init__(self) -> None:
        super().__init__("adamo_bridge")

        self.declare_parameter("api_key", "ak_z7JUjjesgc9WwolEMatSsV0JxKREvuw9")
        self.declare_parameter("adamo_topic", "orin/control/cdr/xr_tracking")

        self._pub = self.create_publisher(PoseStamped, "/head_pose", 10)

        api_key = self.get_parameter("api_key").get_parameter_value().string_value
        adamo_topic = self.get_parameter("adamo_topic").get_parameter_value().string_value

        self._session = adamo.connect(api_key=api_key)
        self._sub = self._session.subscribe(adamo_topic)

        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"Listening on adamo topic '{adamo_topic}', publishing to /head_pose"
        )

    def _recv_loop(self) -> None:
        for sample in self._sub:
            if not rclpy.ok():
                break
            result = decode_cdr_envelope(sample.payload)
            if not result:
                continue
            topic, _type_name, cdr_payload = result
            if topic == "/head_pose":
                pose = decode_pose_stamped(cdr_payload)
                if pose is None:
                    continue
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "world"
                msg.pose.position.x = pose["position"][0]
                msg.pose.position.y = pose["position"][1]
                msg.pose.position.z = pose["position"][2]
                msg.pose.orientation.x = pose["quaternion"][0]
                msg.pose.orientation.y = pose["quaternion"][1]
                msg.pose.orientation.z = pose["quaternion"][2]
                msg.pose.orientation.w = pose["quaternion"][3]

                # Remap axes: negate roll, swap pitch and yaw
                # Original quaternion: [x, y, z, w]
                # Negate x to flip roll, swap y<->z to swap pitch<->yaw
                
                self._pub.publish(msg)

    def destroy_node(self) -> None:
        self._sub.close()
        self._session.close()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AdamoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
