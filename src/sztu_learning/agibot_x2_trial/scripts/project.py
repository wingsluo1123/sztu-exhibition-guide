#!/usr/bin/env python3
"""OctoMap projection of saved per-scan clouds with their recorded sensor origins.

The floor transform comes from PCL RANSAC on this exact map run. Replaying scan
origins avoids pretending that an accumulated PCD was observed from one origin.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from PIL import Image
import yaml

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--name',default='mapping02');args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='42' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    out=ROOT/'maps'/args.name;logs=ROOT/'evidence'/(args.name+'_projection');logs.mkdir(exist_ok=False)
    tf=json.loads((out/'map.pcd.transform.json').read_text());m=np.array(tf['matrix']);q=Rotation.from_matrix(m[:3,:3]).as_quat()
    config=yaml.safe_load((ROOT/'config/octomap.yaml').read_text())
    config['/**']['ros__parameters'].update(occupancy_min_z=0.25,occupancy_max_z=1.85,pointcloud_min_z=-0.2,pointcloud_max_z=2.5)
    (logs/'octomap.yaml').write_text(yaml.safe_dump(config))
    rclpy.init();node=rclpy.create_node('trial_projection_observer');maps=[]
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub=node.create_subscription(OccupancyGrid,'/projected_map',lambda msg:maps.append(msg) if not maps else maps.__setitem__(0,msg),qos)
    children={};handles=[];report={'source_run':args.name,'algorithm':'ROS Humble octomap_server','motor_api':False}
    def start(name,command):
        h=(logs/(name+'.log')).open('x');handles.append(h)
        children[name]=subprocess.Popen(command,stdout=h,stderr=subprocess.STDOUT,start_new_session=True)
    def spin(sec):
        end=time.monotonic()+sec
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    try:
        start('level_tf',['ros2','run','tf2_ros','static_transform_publisher',
                         '--x',str(m[0,3]),'--y',str(m[1,3]),'--z',str(m[2,3]),
                         '--qx',str(q[0]),'--qy',str(q[1]),'--qz',str(q[2]),'--qw',str(q[3]),
                         '--frame-id','map','--child-frame-id','mapping_odom'])
        start('octomap',['ros2','run','octomap_server','octomap_server_node','--ros-args',
                         '--params-file',str(logs/'octomap.yaml'),'-r','cloud_in:=/lio/cloud_registered_lidar'])
        spin(3)
        start('player',['ros2','bag','play',str(out/'registered_bag'),'--clock','100','--rate','1.0',
                        '--disable-keyboard-controls','--delay','2','--topics','/tf','/tf_static','/lio/cloud_registered_lidar'])
        deadline=time.monotonic()+300
        while children['player'].poll() is None:
            spin(.1)
            if children['octomap'].poll() is not None:raise RuntimeError('Octomap exited')
            if time.monotonic()>deadline:raise RuntimeError('Projection replay timed out')
        if children['player'].returncode:raise RuntimeError('Projection playback failed')
        spin(3)
        if not maps:raise RuntimeError('No projected map')
        start('saver',['ros2','run','nav2_map_server','map_saver_cli','-f',str(out/'map'),
                       '--ros-args','-p','save_map_timeout:=10.0','-r','map:=/projected_map'])
        deadline=time.monotonic()+20
        while children['saver'].poll() is None and time.monotonic()<deadline:spin(.1)
        if children['saver'].poll()!=0:raise RuntimeError('Map saver failed')
        grid=maps[0];data=np.array(grid.data,dtype=np.int8).reshape(grid.info.height,grid.info.width)
        np.save(out/'occupancy.npy',data)
        report.update(status='passed',width=grid.info.width,height=grid.info.height,resolution=grid.info.resolution,
                      occupied=int((data==100).sum()),free=int((data==0).sum()),unknown=int((data<0).sum()))
        image=Image.open(out/'map.pgm');image.resize((image.width*2,image.height*2)).save(out/'preview.png')
    except Exception as error:report.update(status='failed',error=repr(error))
    finally:
        for name,p in reversed(list(children.items())):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        report['exit_codes']={k:v.returncode for k,v in children.items()}
        for h in handles:h.close()
        node.destroy_node();rclpy.try_shutdown();(logs/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
