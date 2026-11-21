"""Read-only SSH display: robot domain0 to isolated computer domain42."""
import argparse
import base64
import copy
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from build_rviz_map import sha


def fresh(sent_epoch, source_age, now, offset, uncertainty):
    age = now - offset - sent_epoch
    return uncertainty <= .15 and -.05 - uncertainty <= age and age + source_age + uncertainty <= .6


def clock_offset(local_sent, local_received, robot_echo):
    return (local_sent + local_received) / 2. - robot_echo


def read_lines(stream, destination):
    for line in stream:
        try:
            value = json.loads(line)
        except (ValueError, TypeError):
            continue
        if destination.full():
            try:
                destination.get_nowait()
            except queue.Empty:
                pass
        destination.put(value)


class RobotStream(Node):
    def __init__(self):
        super().__init__('learning_online_display_export')
        self.latest = {}
        self.pings = queue.Queue(maxsize=16)
        threading.Thread(target=read_lines, args=(sys.stdin, self.pings), daemon=True).start()
        for name, topic, message_type in [
                ('status', '/learning/localization/status', String),
                ('pose', '/learning/localization/robot_base_pose', PoseStamped),
                ('cloud', '/learning/localization/endpoints', PointCloud2)]:
            self.create_subscription(message_type, topic, lambda message, key=name: self.receive(key, message), qos_profile_sensor_data)
        self.create_timer(.2, self.send)

    def receive(self, name, message):
        self.latest[name] = (time.monotonic(), base64.b64encode(serialize_message(message)).decode())

    def send(self):
        while not self.pings.empty():
            value = self.pings.get_nowait()
            if isinstance(value, dict) and isinstance(value.get('ping'), (int, float)):
                print(json.dumps({'pong': value['ping'], 'robot_epoch': time.time()}), flush=True)
        now = time.monotonic()
        print(json.dumps({'sent_epoch': time.time(), 'messages': {
            name: {'source_age': now - received, 'data': data} for name, (received, data) in self.latest.items()}}), flush=True)


class ComputerDisplay(Node):
    def __init__(self, args):
        super().__init__('learning_online_display_import')
        root = Path(__file__).resolve().parent
        self.map_hash = sha(root / 'maps/room_7x3_20261120_map01/manifest.json')
        command = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                   '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2', args.robot,
                   'source ~/.aima/env/bashrc >/dev/null 2>&1; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1; '
                   'python3 ~/sztu_learning/live_release01/online_display_bridge.py --robot-stream']
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.incoming = queue.Queue(maxsize=32)
        threading.Thread(target=read_lines, args=(self.process.stdout, self.incoming), daemon=True).start()
        self.offset, self.uncertainty, self.best_rtt = None, 1., float('inf')
        self.last_sync, self.next_ping, self.last_good = 0., 0., None
        self.last_packet = None
        self.last_pose_stamp = None
        self.path = PathMessage()
        self.path.header.frame_id = 'learning_map'
        self.pose_publisher = self.create_publisher(PoseStamped, '/learning/localization/robot_base_pose', 1)
        self.path_publisher = self.create_publisher(PathMessage, '/learning/localization/robot_base_path', 1)
        self.cloud_publisher = self.create_publisher(PointCloud2, '/learning/localization/endpoints', 1)
        self.marker_publisher = self.create_publisher(Marker, '/learning/localization/current_marker', 1)
        self.status_publisher = self.create_publisher(String, '/learning/localization/status', 1)
        self.create_timer(.05, self.tick)

    def clear(self, reason, diagnostics=None):
        marker = Marker()
        marker.header.frame_id = 'learning_map'
        marker.ns, marker.id, marker.action = 'current_localization', 0, Marker.DELETE
        self.marker_publisher.publish(marker)
        self.status_publisher.publish(String(data=json.dumps({'valid': False, 'state': 'lost', 'reason': reason, 'mode': 'live_robot_display', 'diagnostics': diagnostics})))

    def tick(self):
        now = time.monotonic()
        if now >= self.next_ping and self.process.poll() is None:
            try:
                self.process.stdin.write(json.dumps({'ping': time.time()}) + '\n')
                self.process.stdin.flush()
            except (BrokenPipeError, OSError):
                self.clear('ssh_disconnected')
            self.next_ping = now + (.25 if self.offset is None else 3.)
        snapshot = None
        while not self.incoming.empty():
            value = self.incoming.get_nowait()
            if 'pong' in value:
                local = time.time()
                rtt = local - value['pong']
                if 0. <= rtt <= .3 and (rtt < self.best_rtt or now - self.last_sync > 15.):
                    self.best_rtt, self.uncertainty = rtt, rtt / 2.
                    self.offset = clock_offset(value['pong'], local, value['robot_epoch'])
                    self.last_sync = now
            elif 'messages' in value:
                snapshot = value
        if snapshot and self.offset is not None and now - self.last_sync <= 30.:
            self.last_packet = now
            try:
                self.show(snapshot)
            except (ValueError, KeyError, TypeError, RuntimeError) as error:
                self.clear('invalid_display_packet:' + str(error))
        if self.last_packet is None or now - self.last_packet > .6 or self.process.poll() is not None:
            self.clear('waiting_for_fresh_live_localization_or_ssh_disconnected')

    def show(self, packet):
        messages = packet['messages']
        for key in ['status', 'pose']:
            if not fresh(packet['sent_epoch'], messages[key]['source_age'], time.time(), self.offset, self.uncertainty):
                self.clear('stale_live_display_packet', {'message': key, 'packet_age_s': time.time() - self.offset - packet['sent_epoch'],
                                                        'source_age_s': messages[key]['source_age'], 'clock_offset_s': self.offset,
                                                        'clock_uncertainty_s': self.uncertainty})
                return
        status = json.loads(deserialize_message(base64.b64decode(messages['status']['data']), String).data)
        if status.get('map_manifest_sha256') != self.map_hash or not status.get('valid') or status.get('mode') != 'live_robot':
            self.clear('robot_localization_invalid_or_map_mismatch:' + status.get('reason', 'unknown'))
            return
        pose = deserialize_message(base64.b64decode(messages['pose']['data']), PoseStamped)
        stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
        if pose.header.frame_id != 'learning_map' or not fresh(stamp, 0., time.time(), self.offset, self.uncertainty):
            self.clear('stale_robot_pose')
            return
        self.last_good = time.monotonic()
        status['display_clock_offset_s'], status['display_clock_uncertainty_s'] = self.offset, self.uncertainty
        self.status_publisher.publish(String(data=json.dumps(status)))
        if self.last_pose_stamp is None or stamp > self.last_pose_stamp:
            self.pose_publisher.publish(pose)
            self.path.header = pose.header
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-6000:]
            self.path_publisher.publish(self.path)
            self.last_pose_stamp = stamp
        marker = Marker(header=pose.header)
        marker.ns, marker.id, marker.type, marker.action = 'current_localization', 0, Marker.ARROW, Marker.ADD
        marker.pose = copy.deepcopy(pose.pose)
        marker.pose.position.z = .04
        marker.scale.x, marker.scale.y, marker.scale.z = .35, .07, .07
        marker.color.r, marker.color.g, marker.color.a = 1., .65, 1.
        self.marker_publisher.publish(marker)
        if 'cloud' in messages and fresh(packet['sent_epoch'], messages['cloud']['source_age'], time.time(), self.offset, self.uncertainty):
            self.cloud_publisher.publish(deserialize_message(base64.b64decode(messages['cloud']['data']), PointCloud2))

    def close(self):
        try:
            if self.context.ok():
                self.clear('display_closed')
        finally:
            self.process.terminate()
            try:
                self.process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-stream', action='store_true')
    parser.add_argument('--robot', default='agi@192.168.224.19')
    args = parser.parse_args()
    if args.robot_stream:
        if os.environ.get('ROS_DOMAIN_ID') != '0':
            raise RuntimeError('Robot exporter requires domain0')
    elif os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Computer importer requires domain42/localhost')
    rclpy.init()
    node = RobotStream() if args.robot_stream else ComputerDisplay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if isinstance(node, ComputerDisplay):
            node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
