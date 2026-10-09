"""NavigateToPose client used by waypoint and lecture orchestration."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

from sztu_slam.message_validation import MessageFreshness
from .nav_geometry import localization_valid


def spin_once(node):
    executor = node.executor
    if executor is not None and node in executor.get_nodes():
        time.sleep(0.02)
    else:
        rclpy.spin_once(node, timeout_sec=0.05)


class Nav2Navigator(Node):
    def __init__(self, **kwargs):
        super().__init__('sztu_nav2_navigator', **kwargs)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        self.freshness = MessageFreshness(2.0)
        self.pose = None
        self.goal_handle = None
        self.is_navigating = False
        self.last_result = None
        self.generation = 0
        self.stop_publisher = self.create_publisher(Bool, '/sztu/navigation_stop', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.pose_callback, qos_profile_sensor_data)

    def pose_callback(self, message):
        self.pose = None
        if (message.header.frame_id == 'map' and localization_valid(message)
                and self.freshness.accept(message.header.stamp,
                                          self.get_clock().now().nanoseconds,
                                          time.monotonic())):
            self.pose = message.pose.pose

    def wait_for_pose(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            age = self.freshness.age(self.get_clock().now().nanoseconds, time.monotonic())
            if self.pose is not None and age is not None and age <= 2.0:
                return True
            spin_once(self)
        return False

    def navigate_to(self, x, y, yaw=None, timeout=120.0):
        if not all(math.isfinite(value) for value in (x, y, yaw or 0.0)):
            return False
        if (self.is_navigating or not self.wait_for_pose()
                or not self.wait_until_active() or not self.client.wait_for_server(5.0)):
            self.last_result = 'not_ready'
            return False
        self.generation += 1
        generation = self.generation
        self.is_navigating = True
        self.last_result = None
        self.stop_publisher.publish(Bool(data=False))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin((yaw or 0.0) / 2)
        goal.pose.pose.orientation.w = math.cos((yaw or 0.0) / 2)
        deadline = time.monotonic() + timeout
        pending = self.client.send_goal_async(goal)

        def cancel_late(future):
            handle = future.result()
            if handle is not None and handle.accepted and generation != self.generation:
                handle.cancel_goal_async()

        pending.add_done_callback(cancel_late)
        while rclpy.ok() and self.is_navigating and time.monotonic() < deadline:
            spin_once(self)
            if pending.done():
                break
        if (not pending.done() or not self.is_navigating
                or time.monotonic() >= deadline):
            self.cancel_navigation()
            return False
        self.goal_handle = pending.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self.last_result = 'rejected'
            self.is_navigating = False
            self.stop_publisher.publish(Bool(data=True))
            return False
        result = self.goal_handle.get_result_async()
        while rclpy.ok() and self.is_navigating and time.monotonic() < deadline:
            spin_once(self)
            if result.done():
                success = result.result().status == GoalStatus.STATUS_SUCCEEDED
                self.last_result = 'arrived' if success else 'failed'
                self.is_navigating = False
                self.stop_publisher.publish(Bool(data=True))
                self.goal_handle = None
                return success
        self.cancel_navigation()
        return False

    def wait_until_active(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if not self.lifecycle.wait_for_service(timeout_sec=0.2):
                continue
            pending = self.lifecycle.call_async(GetState.Request())
            while not pending.done() and rclpy.ok() and time.monotonic() < deadline:
                spin_once(self)
            if pending.done() and pending.result().current_state.id == 3:
                return True
            time.sleep(0.1)
        return False

    def cancel_navigation(self):
        self.generation += 1
        self.stop_publisher.publish(Bool(data=True))
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.is_navigating = False
        self.last_result = 'canceled'

    def stop(self):
        self.cancel_navigation()

    def destroy_node(self):
        self.client.destroy()
        return super().destroy_node()
