"""SSH RViz transport: computer domain42, robot domain0, optional one goal.

No motion API. Goals require a fresh robot waiting session and valid same-map
localization. No reconnect or goal replay. Default mode only displays.
"""
import argparse
import base64
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shlex
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message, deserialize_message
from std_msgs.msg import String
from visualization_msgs.msg import Marker


TOPICS = {'pose': ('/learning/localization/robot_base_pose', PoseStamped),
          'status': ('/learning/nav/localization_status', String),
          'mission': ('/learning/nav/mission_state', String),
          'path': ('/learning/nav/planned_path', RosPath),
          'accepted': ('/learning/nav/accepted_goal', PoseStamped)}


def read_lines(stream, destination):
    for line in stream:
        if len(line) > 1000000:
            continue
        try:
            value = json.loads(line)
        except ValueError:
            continue
        try:
            destination.put_nowait(value)
        except queue.Full:
            pass


def goal_values(message):
    p, q = message.pose.position, message.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    if (message.header.frame_id != 'learning_map' or not all(math.isfinite(v) for v in values)
            or abs(p.z) > .01 or abs(q.x) > 1e-6 or abs(q.y) > 1e-6
            or abs(q.z*q.z+q.w*q.w-1) > .001):
        raise ValueError('Invalid planar learning_map goal')
    return values


def decode(encoded, message_type):
    return deserialize_message(base64.b64decode(encoded, validate=True), message_type)


class RobotRelay(Node):
    def __init__(self, args, input_stream=None):
        super().__init__('learning_rviz_robot_relay')
        self.allow, self.used = args.forward_one_goal, False
        self.map_hash = args.map_hash
        self.latest = {}
        self.incoming = queue.Queue(maxsize=16)
        threading.Thread(target=read_lines, args=(sys.stdin if input_stream is None else input_stream, self.incoming), daemon=True).start()
        for key, (topic, cls) in TOPICS.items():
            qos = (QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if key in ('path', 'accepted') else qos_profile_sensor_data)
            self.create_subscription(cls, topic, lambda m, k=key: self.receive(k, m), qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/learning/nav/goal', 1)
        self.create_timer(.1, self.tick)

    def receive(self, key, message):
        self.latest[key] = (time.monotonic(), message)

    def waiting(self):
        now = time.monotonic()
        values = {}
        for key in ('status', 'mission'):
            received, message = self.latest.get(key, (-1e9, None))
            if now-received > .5 or message is None:
                return None
            values[key] = json.loads(message.data)
        s, m = values['status'], values['mission']
        if (not s.get('valid') or m.get('state') != 'waiting_for_goal'
                or any(v.get('map_manifest_sha256') != self.map_hash for v in (s, m))):
            return None
        return m['mission_id']

    def tick(self):
        while not self.incoming.empty():
            packet = self.incoming.get_nowait()
            if not isinstance(packet, dict):
                continue
            if 'ping' in packet:
                print(json.dumps(dict(pong=packet['ping'], robot_epoch=time.time())), flush=True)
            if 'goal' in packet:
                accepted = False
                try:
                    waiting = self.waiting()
                    # Deadline is expressed in robot time using the ping estimate.
                    if (not self.allow or self.used or not waiting or packet.get('mission_id') != waiting
                            or not 0 <= packet['expires_robot_epoch']-time.time() <= 1.
                            or self.goal_pub.get_subscription_count() != 1):
                        raise ValueError('Goal relay is not ready for this session')
                    message = decode(packet['goal'], PoseStamped)
                    goal_values(message)
                    message.header.stamp = self.get_clock().now().to_msg()
                    self.used = True
                    self.goal_pub.publish(message)
                    accepted = True
                except (ValueError, KeyError, TypeError, RuntimeError):
                    pass
                print(json.dumps(dict(goal_forwarded=accepted, mission_id=packet.get('mission_id'))), flush=True)
        now = time.monotonic()
        print(json.dumps(dict(sent_epoch=time.time(), map_hash=self.map_hash,
                              waiting=self.waiting(), used=self.used,
                              messages={k: dict(age=now-t, data=base64.b64encode(serialize_message(m)).decode())
                                        for k, (t, m) in self.latest.items()})), flush=True)


class ComputerRelay(Node):
    def __init__(self, args):
        super().__init__('learning_rviz_computer_relay')
        self.args = args
        self.selection_grid = None
        if getattr(args, 'validate_map_directory', None):
            from grid_route import GridRoute
            self.selection_grid = GridRoute.load(args.validate_map_directory, .40)
            if self.selection_grid.manifest_hash != args.map_hash:
                raise ValueError('Selection map identity mismatch')
        remote_script = shlex.quote(args.remote_script)
        remote = ('source ~/.aima/env/bashrc >/dev/null 2>&1; '
                  'export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0; '
                  f'exec python3 {remote_script} --robot-stream --map-hash {shlex.quote(args.map_hash)}'
                  + (' --forward-one-goal' if args.forward_one_goal else ''))
        self.process = subprocess.Popen(['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5',
                                         '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2',
                                         args.robot, remote], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        text=True, bufsize=1)
        self.incoming = queue.Queue(maxsize=32)
        threading.Thread(target=read_lines, args=(self.process.stdout, self.incoming), daemon=True).start()
        self.offset, self.rtt, self.sync, self.packet_received = None, 1., 0., 0.
        self.next_ping, self.packet, self.used = 0., None, False
        self.relay_publishers = {}
        for key in ('path', 'accepted'):
            topic, cls = TOPICS[key]
            self.relay_publishers[key] = self.create_publisher(cls, topic, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.marker = self.create_publisher(Marker, '/learning/nav/current_marker', 20)
        self.status = self.create_publisher(String, '/learning/nav/bridge_status', 1)
        self.create_subscription(PoseStamped, '/learning/nav/rviz_goal', self.goal, 1)
        self.last = {}
        self.suggestion = None
        self.create_timer(.05, self.tick)

    def send(self, value):
        self.process.stdin.write(json.dumps(value)+'\n')
        self.process.stdin.flush()

    def fresh(self):
        return (self.packet is not None and self.offset is not None and self.rtt <= .3
                and time.monotonic()-self.sync < 15 and time.monotonic()-self.packet_received < .5
                and -.05 <= time.time()-self.offset-self.packet['sent_epoch'] < .4
                and self.packet['map_hash'] == self.args.map_hash)

    def goal(self, message):
        try:
            goal_values(message)
            if self.selection_grid is not None:
                xy = [message.pose.position.x, message.pose.position.y]
                if not self.selection_grid.allowed[self.selection_grid.cell(xy)]:
                    raise ValueError('目标处于障碍物、未知区域或净空不足的位置；请重新选点')
            if not self.args.forward_one_goal or self.used or not self.fresh() or not self.packet.get('waiting'):
                raise ValueError('Not armed or robot is not waiting; goal discarded')
            if getattr(self.args,'suggest_relative_goal',None):
                if self.suggestion is None or self.suggestion[0]!=self.packet['waiting']:
                    raise ValueError('Wait for the yellow goal marker')
                if math.hypot(message.pose.position.x-self.suggestion[1],message.pose.position.y-self.suggestion[2])>.20:
                    raise ValueError('Click the YELLOW CENTER within its ring; outside click discarded locally, still waiting')
            self.send(dict(goal=base64.b64encode(serialize_message(message)).decode(),
                           mission_id=self.packet['waiting'], expires_robot_epoch=time.time()-self.offset+.7))
            self.used = True
            self.get_logger().info('One RViz goal sent; awaiting robot acknowledgement. No retry.')
        except (ValueError, BrokenPipeError, OSError) as error:
            self.get_logger().warning(str(error))

    def show_suggestion(self, pose):
        waiting = self.packet.get('waiting')
        relative = getattr(self.args, 'suggest_relative_goal', None)
        if not waiting or not relative:
            for ident in range(4):
                marker = Marker();marker.header.frame_id = 'learning_map'
                marker.ns, marker.id, marker.action = 'suggested_goal', ident, Marker.DELETE
                self.marker.publish(marker)
            return
        if self.suggestion is None or self.suggestion[0] != waiting:
            p,q = pose.pose.position,pose.pose.orientation
            yaw = 2*math.atan2(q.z,q.w);forward,left = relative
            self.suggestion = (waiting, p.x+math.cos(yaw)*forward-math.sin(yaw)*left,
                               p.y+math.sin(yaw)*forward+math.cos(yaw)*left,
                               yaw+math.atan2(left,forward))
        _,x,y,yaw = self.suggestion
        for ident in range(4):
            marker = Marker();marker.header.frame_id = 'learning_map'
            marker.ns,marker.id,marker.action = 'suggested_goal',ident,Marker.ADD
            marker.pose.position.x,marker.pose.position.y = x,y
            marker.pose.position.z = .05 if ident < 2 else .25
            marker.pose.orientation.z,marker.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
            marker.color.r,marker.color.g,marker.color.a = 1.,.8,1.
            marker.lifetime.nanosec = 500000000
            if ident == 0:
                marker.type = Marker.SPHERE
                marker.scale.x = marker.scale.y = marker.scale.z = .12
            elif ident == 1:
                marker.type = Marker.ARROW
                marker.scale.x,marker.scale.y,marker.scale.z = .32,.06,.06
            elif ident == 2:
                marker.type = Marker.TEXT_VIEW_FACING
                marker.scale.z,marker.text = .15,'Click yellow center'
            else:
                marker.type=Marker.LINE_STRIP;marker.scale.x=.015
                # Ring uses the same selection radius as local admission.
                marker.pose.orientation.z=0.;marker.pose.orientation.w=1.
                marker.points=[Point(x=.20*math.cos(i*2*math.pi/64),
                                     y=.20*math.sin(i*2*math.pi/64),z=0.) for i in range(65)]
            self.marker.publish(marker)

    def tick(self):
        now = time.monotonic()
        try:
            if now >= self.next_ping and self.process.poll() is None:
                self.send(dict(ping=time.time()))
                self.next_ping = now+2
            while not self.incoming.empty():
                packet = self.incoming.get_nowait()
                if 'pong' in packet:
                    rtt = time.time()-packet['pong']
                    if 0 <= rtt <= .3:
                        self.rtt, self.offset = rtt, (time.time()+packet['pong'])/2-packet['robot_epoch']
                        self.sync = now
                elif 'messages' in packet:
                    self.packet, self.packet_received = packet, now
                elif 'goal_forwarded' in packet:
                    self.get_logger().info('Robot goal acknowledgement: '+json.dumps(packet))
            valid = False
            if self.fresh():
                messages = self.packet['messages']
                if all(k in messages and messages[k]['age'] < .4 for k in ('status', 'pose')):
                    status = json.loads(decode(messages['status']['data'], String).data)
                    valid = status.get('valid', False) and status.get('map_manifest_sha256') == self.args.map_hash
                if valid:
                    pose = decode(messages['pose']['data'], PoseStamped)
                    marker = Marker(header=pose.header, pose=pose.pose)
                    marker.ns, marker.id, marker.type, marker.action = 'navigation_current', 0, Marker.ARROW, Marker.ADD
                    marker.scale.x, marker.scale.y, marker.scale.z = .35, .07, .07
                    marker.color.g, marker.color.a = 1., 1.
                    marker.lifetime.sec, marker.lifetime.nanosec = 0, 500000000
                    self.marker.publish(marker)
                    self.show_suggestion(pose)
                for key, pub in self.relay_publishers.items():
                    if key in messages and messages[key]['data'] != self.last.get(key):
                        pub.publish(decode(messages[key]['data'], TOPICS[key][1]))
                        self.last[key] = messages[key]['data']
            if not valid:
                marker = Marker(); marker.header.frame_id = 'learning_map'
                marker.ns, marker.id, marker.action = 'navigation_current', 0, Marker.DELETE
                self.marker.publish(marker)
            self.status.publish(String(data=json.dumps(dict(valid=bool(valid),
                waiting=self.packet.get('waiting') if self.fresh() else None,
                forward_one_goal=self.args.forward_one_goal, goal_sent=self.used))))
        except (ValueError, KeyError, TypeError, RuntimeError, BrokenPipeError, OSError) as error:
            self.get_logger().warning('Bridge packet discarded: '+str(error))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-stream', action='store_true')
    parser.add_argument('--forward-one-goal', action='store_true')
    parser.add_argument('--robot', default='agi@192.168.224.19')
    parser.add_argument('--remote-script', default='/home/agi/sztu_learning/navigation_validation/rviz_release04/goal_navigation/rviz_bridge.py')
    parser.add_argument('--map-hash')
    parser.add_argument('--suggest-relative-goal',type=float,nargs=2)
    parser.add_argument('--selection-mode', choices=('map', 'trial'), default='map',
                        help='Map-wide selection, or the legacy front-right trial marker')
    parser.add_argument('--validate-map-directory', type=Path,
                        help='Computer-side saved-map clearance check before consuming a click')
    args = parser.parse_args()
    if args.selection_mode == 'trial' and args.suggest_relative_goal is None:
        args.suggest_relative_goal = [.6, -.4]
    if args.robot_stream:
        if os.environ.get('ROS_DOMAIN_ID') != '0':
            raise RuntimeError('Robot relay requires domain0')
        path = Path(os.environ.get('LEARNING_LOCALIZATION_ROOT', '/home/agi/sztu_learning/live_release01'))/'maps/room_7x3_20261120_map01/manifest.json'
    else:
        if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Computer relay requires domain42/localhost')
        path = Path(__file__).resolve().parents[2]/'previous_learning/maps/room_7x3_20261120_map01/manifest.json'
    expected = hashlib.sha256(path.read_bytes()).hexdigest()
    if args.map_hash is not None and args.map_hash != expected:
        raise RuntimeError('Map identity mismatch')
    args.map_hash = expected
    rclpy.init()
    node = RobotRelay(args) if args.robot_stream else ComputerRelay(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if isinstance(node, ComputerRelay):
            node.process.terminate()
            node.process.wait(timeout=5)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
