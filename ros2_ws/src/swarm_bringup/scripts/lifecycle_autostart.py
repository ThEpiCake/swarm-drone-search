#!/usr/bin/env python3
"""Configure and activate one lifecycle node reliably from launch."""

import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class LifecycleAutostart(Node):
    def __init__(self):
        super().__init__("lifecycle_autostart")
        self.declare_parameter("target_node", "/px4_0/slam_toolbox")
        self.declare_parameter("timeout_s", 45.0)
        self.declare_parameter("retry_period_s", 0.5)

        self.target_node = str(self.get_parameter("target_node").value).rstrip("/")
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.retry_period_s = float(self.get_parameter("retry_period_s").value)
        self.get_state = self.create_client(GetState, f"{self.target_node}/get_state")
        self.change_state = self.create_client(ChangeState, f"{self.target_node}/change_state")

    def run(self) -> int:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            if self.get_state.wait_for_service(timeout_sec=self.retry_period_s) and \
                    self.change_state.wait_for_service(timeout_sec=self.retry_period_s):
                break
        else:
            self.get_logger().error(f"{self.target_node}: lifecycle services not available")
            return 1

        if not self._ensure_state("inactive", Transition.TRANSITION_CONFIGURE):
            return 1
        if not self._ensure_state("active", Transition.TRANSITION_ACTIVATE):
            return 1
        self.get_logger().info(f"{self.target_node}: active")
        return 0

    def _state_label(self) -> str | None:
        future = self.get_state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.result() is None:
            return None
        return future.result().current_state.label

    def _transition(self, transition_id: int) -> bool:
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change_state.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result() is not None and bool(future.result().success)

    def _ensure_state(self, wanted: str, transition_id: int) -> bool:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            state = self._state_label()
            self.get_logger().info(f"{self.target_node}: lifecycle state={state}")
            if state == "active" or state == wanted:
                return True
            if state in ("unconfigured", "inactive"):
                self._transition(transition_id)
            time.sleep(self.retry_period_s)
        self.get_logger().error(f"{self.target_node}: did not reach {wanted}")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleAutostart()
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
