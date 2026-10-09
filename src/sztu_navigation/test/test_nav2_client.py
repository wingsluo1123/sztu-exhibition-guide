import math
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sztu_navigation.nav2_navigator import Nav2Navigator


@pytest.fixture
def action_environment():
    rclpy.init()
    server_node = Node('test_nav2_action_server')
    navigator = Nav2Navigator()
    poses = server_node.create_publisher(
        PoseWithCovarianceStamped, '/amcl_pose', qos_profile_sensor_data)

    def publish_pose():
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = server_node.get_clock().now().to_msg()
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = 0.01
        poses.publish(message)

    server_node.create_timer(0.1, publish_pose)
    requests = []
    mode = {'abort': False, 'hold': False, 'active': True}

    def lifecycle_callback(request, response):
        response.current_state = State(id=3 if mode['active'] else 2)
        return response

    server_node.create_service(GetState, '/bt_navigator/get_state', lifecycle_callback)

    def execute(handle):
        requests.append(handle.request)
        while mode['hold'] and rclpy.ok():
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.01)
        if mode['abort']:
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    from rclpy.callback_groups import ReentrantCallbackGroup
    action = ActionServer(server_node, NavigateToPose, '/navigate_to_pose', execute,
                          cancel_callback=lambda unused: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server_node)
    executor.add_node(navigator)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield navigator, mode, requests
    mode['hold'] = False
    navigator.stop()
    time.sleep(0.1)
    executor.shutdown()
    thread.join(timeout=2)
    action.destroy()
    navigator.destroy_node()
    server_node.destroy_node()
    rclpy.shutdown()


def test_success_is_map_goal_and_final_yaw(action_environment):
    navigator, mode, requests = action_environment
    assert navigator.navigate_to(2.0, 3.0, 0.5, timeout=5)
    assert navigator.last_result == 'arrived'
    assert requests[-1].pose.header.frame_id == 'map'
    assert requests[-1].pose.pose.position.x == 2.0
    assert requests[-1].pose.pose.orientation.z == pytest.approx(math.sin(0.25))


def test_aborted_goal_does_not_start_lecture(action_environment):
    navigator, mode, requests = action_environment
    mode['abort'] = True
    assert not navigator.navigate_to(2.0, 3.0, timeout=5)
    assert navigator.last_result == 'failed'


def test_timeout_cancels_outstanding_goal(action_environment):
    navigator, mode, requests = action_environment
    mode['hold'] = True
    assert not navigator.navigate_to(2.0, 3.0, timeout=1)
    assert navigator.last_result == 'canceled'
    assert not navigator.is_navigating
