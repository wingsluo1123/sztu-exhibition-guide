#!/usr/bin/env python3
"""Run the new project's real estimators against a local bag, never motor APIs."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from lifecycle_msgs.srv import GetState

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_BAG = ROOT.parent/'previous_learning/bags/room_7x3_20261120_154812'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, default=DEFAULT_BAG)
    parser.add_argument('--name', default='mapping01')
    parser.add_argument('--localize', action='store_true')
    parser.add_argument('--octomap', action='store_true')
    parser.add_argument('--record-output', action='store_true')
    parser.add_argument('--integrated-launch', action='store_true')
    parser.add_argument('--rate', type=float, default=0.7)
    parser.add_argument('--localization-cloud', default='/lio/cloud_registered_lidar')
    parser.add_argument('--localization-config', type=Path, default=ROOT/'config/localization.yaml')
    args = parser.parse_args()
    if args.integrated_launch and not args.localize:
        parser.error('--integrated-launch requires --localize')
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Offline runs require ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1')
    logs = ROOT/'evidence'/args.name
    logs.mkdir(exist_ok=False)
    output = ROOT/'maps'/args.name
    output.mkdir(exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('trial_replay_observer', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    counts = {}
    stamps = {}
    poses = {'odom': [], 'localization': []}
    map_messages = []
    children, handles = {}, []
    report = {'bag': str(args.bag.resolve()), 'rate': args.rate, 'physical_motor_bridge_started': False,
              'status_scope':'process liveness and nonempty interfaces; see analysis.json for localization quality'}
    if args.localize:
        report.update(localization_cloud=args.localization_cloud,
                      localization_config=str(args.localization_config.resolve()))
    reliable = QoSProfile(depth=500, reliability=ReliabilityPolicy.RELIABLE)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

    def receive(name, msg):
        counts[name] = counts.get(name, 0)+1
        stamps.setdefault(name,set()).add((msg.header.stamp.sec,msg.header.stamp.nanosec))
        if name in poses:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            poses[name].append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        if name == 'map':
            map_messages[:] = [msg]

    subscriptions = []
    map_odom_stamps = []
    def receive_tf(message):
        for tf in message.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                map_odom_stamps.append(tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
    subscriptions.append(node.create_subscription(TFMessage, '/tf', receive_tf, 100))
    for name, kind, topic, qos in [
        ('rectified_cloud', PointCloud2, '/lidar/points_rectified', reliable),
        ('registered', PointCloud2, '/lio/cloud_registered', reliable),
        ('odom', Odometry, '/odom', reliable),
        ('localization', PoseWithCovarianceStamped, '/pcl_pose', latched),
        ('map', OccupancyGrid, '/projected_map', latched),
    ]:
        subscriptions.append(node.create_subscription(kind, topic, lambda m,k=name:receive(k,m), qos))

    def start(name, command):
        h=(logs/(name+'.log')).open('x'); handles.append(h)
        children[name]=subprocess.Popen(command, cwd=ROOT, stdout=h, stderr=subprocess.STDOUT, start_new_session=True)

    def spin_for(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=0.05)
            failed={n:p.returncode for n,p in children.items() if p.poll() is not None and n!='player'}
            if failed: raise RuntimeError('Pipeline child exited: '+str(failed))

    try:
        if args.integrated_launch:
            start('bringup',['ros2','launch',str(ROOT/'launch/trial.launch.py'),'nav2:=true'])
        else:
            start('sensors',['ros2','launch',str(ROOT/'launch/sensors.launch.py')])
            start('kinematics',[sys.executable,str(ROOT/'scripts/kinematic_odom.py'),'--ros-args','-p','use_sim_time:=true'])
            start('lio',['ros2','run','spark_fast_lio','spark_lio_mapping','--ros-args',
                         '--params-file',str(ROOT/'config'/('odometry.yaml' if args.localize else 'mapping.yaml')),
                         '-p','use_sim_time:=true','-r','__ns:=/lio','-r','lidar:=/lidar/points_rectified','-r','imu:=/imu/rectified'])
        if args.record_output:
            start('recorder',['ros2','bag','record','--use-sim-time','-o',str(output/'registered_bag'),
                              '/lio/cloud_registered_lidar','/lio/cloud_registered','/odom','/tf','/tf_static'])
        if args.octomap:
            start('octomap',['ros2','run','octomap_server','octomap_server_node','--ros-args',
                            '--params-file',str(ROOT/'config/octomap.yaml'),'-r','cloud_in:=/lio/cloud_registered_lidar'])
        if args.localize and not args.integrated_launch:
            start('localizer',['ros2','run','lidar_localization_ros2','lidar_localization_node',
                              '--ros-args','--params-file',str(args.localization_config),
                              '-p','use_sim_time:=true','-r','cloud:='+args.localization_cloud,
                              '-r','map:=/pcd_map_input'])
        spin_for(4.0)
        if args.localize and not args.integrated_launch:
            for transition in ['configure','activate']:
                result=subprocess.run(['ros2','lifecycle','set','/lidar_localization',transition],capture_output=True,text=True,timeout=30)
                (logs/('lifecycle_'+transition+'.log')).write_text(result.stdout+result.stderr)
                if result.returncode or 'successful' not in result.stdout:raise RuntimeError('Localizer lifecycle '+transition+' failed')
        command=['ros2','bag','play',str(args.bag),'--clock','100','--rate',str(args.rate),'--delay','2',
                 '--disable-keyboard-controls','--wait-for-all-acked','5000',
                 '--qos-profile-overrides-path',str(ROOT/'config/playback_qos.yaml')]
        report['playback_command']=command
        start('player',command)
        started=time.monotonic();next_update=started
        while children['player'].poll() is None:
            spin_for(0.1)
            if time.monotonic()>=next_update:
                print(json.dumps({'elapsed':round(time.monotonic()-started),'counts':counts,'last_odom':poses['odom'][-1][1:4] if poses['odom'] else None}),flush=True)
                next_update=time.monotonic()+15
            if time.monotonic()-started>650:raise RuntimeError('Offline replay exceeded 650 seconds')
        if children['player'].returncode:raise RuntimeError('Bag playback failed')
        spin_for(4)
        report['wall_sec']=time.monotonic()-started
        if not poses['odom'] or not counts.get('registered'):raise RuntimeError('Missing estimator output')
        if args.localize and not poses['localization']:raise RuntimeError('No NDT localization output')
        if args.localize and not map_odom_stamps:raise RuntimeError('No NDT map-to-odom TF output')
        if args.integrated_launch:
            report['lifecycle']={}
            for name in ['lidar_localization','map_server','planner_server','controller_server',
                         'behavior_server','bt_navigator','waypoint_follower']:
                client=node.create_client(GetState,'/'+name+'/get_state')
                future=client.call_async(GetState.Request());deadline=time.monotonic()+5
                while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
                if not future.done():raise RuntimeError('Lifecycle service timeout: '+name)
                state=future.result().current_state
                report['lifecycle'][name]=state.label
                if state.id!=3:raise RuntimeError(name+' is not active')
                node.destroy_client(client)
            report['isolated_velocity_publishers']=node.count_publishers('/trial/nav_cmd_vel')
            report['unisolated_velocity_publishers']=node.count_publishers('/cmd_vel')
            if report['unisolated_velocity_publishers']:raise RuntimeError('Unisolated command output')
        if args.octomap:
            if not map_messages:raise RuntimeError('No OctoMap projection received')
            # map_saver_cli itself uses wall time; the input map retains its original frame.
            saver=subprocess.Popen(['ros2','run','nav2_map_server','map_saver_cli','-f',str(output/'map'),
                                    '--ros-args','-p','save_map_timeout:=10.0','-r','map:=/projected_map'],
                                   stdout=(logs/'map_saver.log').open('w'),stderr=subprocess.STDOUT)
            deadline=time.monotonic()+20
            while saver.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=0.05)
            if saver.poll() is None:saver.terminate();saver.wait(timeout=5)
            if saver.returncode:raise RuntimeError('Map saver failed')
        report['status']='passed'
    except Exception as error:
        report.update(status='failed',error=repr(error))
    finally:
        for name,process in reversed(list(children.items())):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        for h in handles:h.close()
        report['counts']=counts
        report['unique_header_counts']={name:len(values) for name,values in stamps.items()}
        report['exit_codes']={n:p.returncode for n,p in children.items()}
        report['map_odom_tf']={'count':len(map_odom_stamps),
                               'first':min(map_odom_stamps) if map_odom_stamps else None,
                               'last':max(map_odom_stamps) if map_odom_stamps else None}
        for name, rows in poses.items():
            if rows:
                np.savetxt(output/(name+'.csv'),np.array(rows),delimiter=',',header='stamp,x,y,z,qx,qy,qz,qw',comments='')
                report[name]={'samples':len(rows),'first':rows[0],'last':rows[-1],
                              'xyz_min':np.min(np.array(rows)[:,1:4],axis=0).tolist(),
                              'xyz_max':np.max(np.array(rows)[:,1:4],axis=0).tolist()}
        if map_messages:
            msg=map_messages[-1];data=np.array(msg.data,dtype=np.int8).reshape(msg.info.height,msg.info.width)
            np.save(output/'occupancy.npy',data)
            report['map']={'frame':msg.header.frame_id,'width':msg.info.width,'height':msg.info.height,'resolution':msg.info.resolution,
                           'unknown':int((data<0).sum()),'occupied':int((data==100).sum()),'free':int((data==0).sum())}
        (logs/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        node.destroy_node();rclpy.try_shutdown()
    print(json.dumps(report,ensure_ascii=False),flush=True)
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':
    raise SystemExit(main())
