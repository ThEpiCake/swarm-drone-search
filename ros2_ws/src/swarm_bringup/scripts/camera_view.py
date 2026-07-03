#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


class CameraView(Node):
    def __init__(self):
        super().__init__("camera_view")

        self.topic = self.declare_parameter("topic", "/px4_0/camera/image_raw").value
        self.window_name = self.declare_parameter("window_name", "Drone Camera").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self._on_image,
            qos,
        )

        self.last_frame_time = self.get_clock().now()
        self.create_timer(0.1, self._pump_gui)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(f"camera_view subscribed to {self.topic}")

    def _on_image(self, msg: Image):
        frame = self._msg_to_bgr(msg)
        if frame is None:
            return

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)
        self.last_frame_time = self.get_clock().now()

    def _pump_gui(self):
        cv2.waitKey(1)

    def _msg_to_bgr(self, msg: Image):
        if msg.height == 0 or msg.width == 0:
            return None

        data = np.frombuffer(msg.data, dtype=np.uint8)

        if msg.encoding == "rgb8":
            image = data.reshape((msg.height, msg.width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if msg.encoding == "bgr8":
            return data.reshape((msg.height, msg.width, 3))

        if msg.encoding == "mono8":
            gray = data.reshape((msg.height, msg.width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if msg.encoding == "32FC1":
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            finite = depth[np.isfinite(depth)]
            if finite.size == 0:
                normalized = np.zeros((msg.height, msg.width), dtype=np.uint8)
            else:
                near = float(np.min(finite))
                far = float(np.max(finite))
                if math.isclose(near, far):
                    normalized = np.zeros((msg.height, msg.width), dtype=np.uint8)
                else:
                    clipped = np.clip(depth, near, far)
                    normalized = ((clipped - near) / (far - near) * 255.0).astype(np.uint8)
                    normalized[~np.isfinite(depth)] = 0
            return cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)

        self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}", throttle_duration_sec=5.0)
        return None


def main():
    rclpy.init()
    node = CameraView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
