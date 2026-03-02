#!/usr/bin/env python3
"""ROS2 node to spawn litter models (stones, coke) in Gazebo.

This module contains the same logic as the script in the `scripts/` folder
but packaged as a Python module so that it can be installed and launched
with `ros2 run virtual_maize_field spawn_litter`.
"""

import os
import random
import time

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import Trigger

# Try to import SpawnEntity from common gazebo packages: prefer `gazebo_msgs`,
# fall back to `ros_gz_msgs` (used with ros_gz_sim / Ignition/Gazebo).
SpawnEntity = None
try:
    from gazebo_msgs.srv import SpawnEntity  # classic gazebo_ros
    SpawnEntity = SpawnEntity
except Exception:
    try:
        from ros_gz_msgs.srv import SpawnEntity  # ros_gz (Ignition/Gazebo)
        SpawnEntity = SpawnEntity
    except Exception:
        SpawnEntity = None


def quaternion_from_yaw(yaw: float) -> Quaternion:
    import math
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


class SpawnLitterNode(Node):
    def __init__(self):
        super().__init__('spawn_litter_node')

        # parameters
        self.declare_parameter('models', ['stone_01', 'stone_02', 'coke_can'])
        self.declare_parameter('count', 5)
        self.declare_parameter('zone_x_min', 1.0)
        self.declare_parameter('zone_x_max', 5.0)
        self.declare_parameter('zone_y_min', -1.0)
        self.declare_parameter('zone_y_max', 1.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('z_min', 0.2)
        self.declare_parameter('random_yaw', True)
        self.declare_parameter('autostart', True)
        self.declare_parameter('package_name', 'virtual_maize_field')

        self.models = self.get_parameter('models').get_parameter_value().string_array_value
        self.count = self.get_parameter('count').value
        self.zone_x_min = self.get_parameter('zone_x_min').value
        self.zone_x_max = self.get_parameter('zone_x_max').value
        self.zone_y_min = self.get_parameter('zone_y_min').value
        self.zone_y_max = self.get_parameter('zone_y_max').value
        self.z = self.get_parameter('z').value
        self.z_min = self.get_parameter('z_min').value
        self.random_yaw = self.get_parameter('random_yaw').value
        self.autostart = self.get_parameter('autostart').value
        self.package_name = self.get_parameter('package_name').get_parameter_value().string_value

        # robot pose (updated from /odom)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # subscribe to odom to get robot position
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # spawn service client
        if SpawnEntity is None:
            self.get_logger().warn('gazebo_msgs.srv.SpawnEntity not available; service calls will fail until dependency is present')
            self.client = None
        else:
            self.client = self.create_client(SpawnEntity, '/spawn_entity')
            # wait for service
            self.get_logger().info('Waiting for /spawn_entity service...')
            if not self.client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning('/spawn_entity service not available yet')

        # trigger service to spawn on demand
        self.create_service(Trigger, '/spawn_litter/spawn', self.handle_spawn_request)

        # book-keeping
        self.spawned = []

        if self.autostart:
            # we may start before Gazebo, so use a repeated timer to wait for service
            self.get_logger().info('Autostart enabled: will spawn automatically when possible')
            # timer callback checks periodically until spawn_objects succeeds
            self._autostart_timer = self.create_timer(1.0, self._autostart_cb)

    def _autostart_cb(self):
        # attempt spawn until successful, then cancel timer
        if hasattr(self, '_autostart_done') and self._autostart_done:
            return
        self.get_logger().info('Autostart: trying to spawn')
        ok = self.spawn_objects()
        if ok:
            self._autostart_done = True
            self.get_logger().info('Autostart spawn succeeded, stopping timer')
            if hasattr(self, '_autostart_timer'):
                self._autostart_timer.cancel()
        else:
            self.get_logger().info('Autostart spawn failed, will retry')

    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # yaw from quaternion
        import math
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot_yaw = yaw

    def handle_spawn_request(self, request, response):
        self.get_logger().info('Spawn service called')
        ok = self.spawn_objects()
        response.success = ok
        response.message = 'spawned' if ok else 'failed'
        return response

    def spawn_objects(self) -> bool:
        # if we have a service client, use it; otherwise we'll attempt CLI fallback
        if self.client is None:
            self.get_logger().warning('Spawn client not available, will use ros_gz_sim CLI fallback')
            use_cli = True
        else:
            use_cli = False

        # prepare models content map
        model_xml_cache = {}
        pkg_share = None
        try:
            pkg_share = get_package_share_directory(self.package_name)
        except Exception as e:
            self.get_logger().error(f'Cannot find package share for {self.package_name}: {e}')
            return False

        for model in self.models:
            model_path = os.path.join(pkg_share, 'models', model, 'model.sdf')
            if not os.path.exists(model_path):
                self.get_logger().warning(f'Model sdf not found: {model_path}')
                continue
            with open(model_path, 'r') as f:
                model_xml_cache[model] = f.read()

        if len(model_xml_cache) == 0:
            self.get_logger().error('No model XML available to spawn')
            return False

        # generate random points in the robot-relative rectangle
        spawned_any = False
        for i in range(self.count):
            rx = random.uniform(self.zone_x_min, self.zone_x_max)
            ry = random.uniform(self.zone_y_min, self.zone_y_max)

            # rotate/translate relative to robot yaw/position
            import math
            cos_t = math.cos(self.robot_yaw)
            sin_t = math.sin(self.robot_yaw)
            wx = self.robot_x + (rx * cos_t - ry * sin_t)
            wy = self.robot_y + (rx * sin_t + ry * cos_t)
            # Keep spawned objects above a minimum altitude to avoid being buried in terrain.
            wz = max(self.z, self.z_min)

            model_name = random.choice(list(model_xml_cache.keys()))
            xml = model_xml_cache[model_name]

            # unique name for either method
            unique_name = f"{model_name}_{int(time.time()*1000) % 100000}_{i}"

            if not use_cli and SpawnEntity is not None:
                # build request
                req = SpawnEntity.Request()
                req.name = unique_name
                req.xml = xml
                # initial_pose
                p = Pose()
                p.position = Point(x=wx, y=wy, z=wz)
                yaw = random.uniform(-3.14159, 3.14159) if self.random_yaw else 0.0
                p.orientation = quaternion_from_yaw(yaw)
                req.initial_pose = p

                # call service
                fut = self.client.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                if fut.done() and fut.result() is not None:
                    res = fut.result()
                    self.get_logger().info(f"Spawned {req.name} model={model_name} (service)")
                    self.spawned.append(req.name)
                    spawned_any = True
                    continue
                else:
                    self.get_logger().warning(f'Failed to spawn {req.name} via service, will try CLI fallback')
                    use_cli = True

            # fallback: call ros_gz_sim create command with model file
            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-file', os.path.join(pkg_share, 'models', model_name, 'model.sdf'),
                '-name', unique_name,
                '-x', str(wx),
                '-y', str(wy),
                '-z', str(wz),
            ]
            if self.random_yaw:
                cmd += ['-Y', str(random.uniform(-3.14159, 3.14159))]
            import subprocess
            self.get_logger().info(f'Executing CLI spawn: {cmd}')
            try:
                subprocess.run(cmd, check=True)
                self.get_logger().info(f"Spawned {unique_name} model={model_name} (cli)")
                self.spawned.append(unique_name)
                spawned_any = True
            except subprocess.CalledProcessError as e:
                self.get_logger().warning(f'CLI spawn failed: {e}')

        return spawned_any


def main(args=None):
    rclpy.init(args=args)
    node = SpawnLitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
