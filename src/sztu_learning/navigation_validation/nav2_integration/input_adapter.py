"""Fresh atomic localization -> namespaced Nav2 TF/odometry, no motion API."""
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from navigation_observation import decode_observation
from goal_navigation.input_readiness import MAP_HASH, MAX_SOURCE_AGE
from planar_math import pose, velocity


class InputAdapter(Node):
    def __init__(self):
        super().__init__('learning_nav2_input_adapter')
        self.last = None
        self.tf_pub = self.create_publisher(TFMessage, '/learning_nav2/tf', 10)
        self.odom_pub = self.create_publisher(Odometry, '/learning_nav2/odom', 10)
        self.status_pub = self.create_publisher(String, '/learning_nav2/input_status', 1)
        self.create_subscription(String, '/learning/nav/observation', self.receive, 1)

    def receive(self, message):
        try:
            status, local, map_matrix = decode_observation(message.data)
            epoch = self.get_clock().now().nanoseconds*1e-9
            if (not status['valid'] or status.get('map_manifest_sha256') != MAP_HASH
                    or not -.05 <= epoch-status['stamp'] <= MAX_SOURCE_AGE):
                raise ValueError('invalid/stale localization')
            seconds = status['stamp']
            if self.last is not None and seconds <= self.last[0]:
                return
            if self.last is None:
                self.last = (seconds, local)
                return  # No measured velocity on the first observation.
            previous = self.last
            self.last = (seconds, local)
            twist = velocity(previous[1], local, seconds-previous[0])
            sec = math.floor(seconds); ns = round((seconds-sec)*1e9)
            if ns >= 1000000000: sec += 1; ns = 0
            odom = Odometry()
            odom.header.stamp.sec, odom.header.stamp.nanosec = sec, ns
            odom.header.frame_id, odom.child_frame_id = 'learning_nav_odom', 'learning_nav_base'
            odom.pose.pose.position.x, odom.pose.pose.position.y = local[:2]
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(local[2]/2), math.cos(local[2]/2)
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = twist
            odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = 1e6
            transforms = []
            for parent, child, value in [('learning_map', 'learning_nav_odom', pose(map_matrix)),
                                         ('learning_nav_odom', 'learning_nav_base', local)]:
                t = TransformStamped(); t.header.stamp = odom.header.stamp
                t.header.frame_id, t.child_frame_id = parent, child
                t.transform.translation.x, t.transform.translation.y = map(float, value[:2])
                t.transform.rotation.z, t.transform.rotation.w = math.sin(value[2]/2), math.cos(value[2]/2)
                transforms.append(t)
            self.tf_pub.publish(TFMessage(transforms=transforms)); self.odom_pub.publish(odom)
            self.status_pub.publish(String(data=json.dumps(dict(valid=True, stamp=seconds))))
        except (ValueError, TypeError, KeyError, AttributeError) as error:
            self.last = None
            self.status_pub.publish(String(data=json.dumps(dict(valid=False, reason=str(error)))))


def main():
    rclpy.init(); node = InputAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
