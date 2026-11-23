#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw (rad) to geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def make_linear_points(
    start: Tuple[float, float],
    goal: Tuple[float, float],
    step: float
) -> List[Tuple[float, float]]:
    sx, sy = start
    gx, gy = goal
    dx = gx - sx
    dy = gy - sy
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        return [(sx, sy)]

    n = max(1, int(dist / step))
    pts = []
    for i in range(n + 1):
        t = i / n
        pts.append((sx + t * dx, sy + t * dy))
    return pts


class LinearPathSender(Node):
    def __init__(self):
        super().__init__("linear_path_sender")

        
        self.declare_parameter("action_name", "/follow_path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("start_x", -2.28)
        self.declare_parameter("start_y", -3.83)
        self.declare_parameter("goal_x", -2.28)
        self.declare_parameter("goal_y", 5.83)
        self.declare_parameter("step", 0.2)

        self.action_name = self.get_parameter("action_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.start = (
            float(self.get_parameter("start_x").value),
            float(self.get_parameter("start_y").value),
        )
        self.goal = (
            float(self.get_parameter("goal_x").value),
            float(self.get_parameter("goal_y").value),
        )
        self.step = float(self.get_parameter("step").value)

        self.client = ActionClient(self, FollowPath, self.action_name)

    def build_path(self) -> Path:
        path = Path()
        now = self.get_clock().now().to_msg()
        path.header.stamp = now
        path.header.frame_id = self.frame_id

        pts = make_linear_points(self.start, self.goal, self.step)
        yaw = math.atan2(self.goal[1] - self.start[1], self.goal[0] - self.start[0])
        q = yaw_to_quat(yaw)

        for (x, y) in pts:
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            ps.pose.orientation = q
            path.poses.append(ps)

        return path

    def send(self):
        self.get_logger().info(f"Waiting for action server: {self.action_name} ...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Action server not available. "
                "Check controller_server is running and the action name is correct."
            )
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = self.build_path()

        # Optional fields exist depending on Nav2 version; Jazzy typically has 'controller_id' and 'goal_checker_id'
        # You can set them if you want to force a specific controller/goal-checker plugin by ID:
        # goal_msg.controller_id = "FollowPath"  # example (must match your controller_server config)
        # goal_msg.goal_checker_id = "general_goal_checker"

        self.get_logger().info(
            f"Sending linear path: start={self.start} goal={self.goal} "
            f"step={self.step} frame={self.frame_id} points={len(goal_msg.path.poses)}"
        )

        send_future = self.client.send_goal_async(goal_msg, feedback_callback=self.on_feedback)
        send_future.add_done_callback(self.on_goal_response)

    def on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        
        self.get_logger().debug("Received feedback.")

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller_server.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        
        self.get_logger().info(f"Result received: {result}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = LinearPathSender()
    node.send()
    rclpy.spin(node)


if __name__ == "__main__":
    main()