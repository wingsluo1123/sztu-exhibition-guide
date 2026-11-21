#!/usr/bin/env python3
"""Exercise upstream Nav2 on the generated map with explicitly synthetic feedback.

This tests actions, planners, controllers and final pose checking. It does not
test NDT, gait response, real stopping distance or real obstacle detection.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.ndimage import distance_transform_edt
from scipy.spatial.transform import Rotation
import yaml

ROOT=Path(__file__).resolve().parents[1]


class Plant(Node):
    def __init__(self, pose, obstacles, events):
        super().__init__('trial_software_robot')
        self.pose=list(pose);self.command=(0.0,0.0);self.v=self.w=0.0
        self.last=time.monotonic();self.received=0.;self.nonzero=0;self.events=events
        self.obstacles=obstacles;self.counter=0
        self.odom=self.create_publisher(Odometry,'/odom',10)
        self.cloud=self.create_publisher(PointCloud2,'/lidar/points_rectified',10)
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        identity=TransformStamped();identity.header.frame_id='map';identity.child_frame_id='odom';identity.transform.rotation.w=1.0
        self.static.sendTransform(identity)
        self.create_subscription(Twist,'/cmd_vel',self.receive,10)
        self.create_timer(0.02,self.tick)
        self.navigate=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.plan=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')

    def receive(self,message):
        values=[message.linear.x,message.angular.z]
        if not all(math.isfinite(v) for v in values):raise RuntimeError('Nonfinite Nav2 command')
        self.command=values;self.received=time.monotonic();self.nonzero+=int(any(values))

    def tick(self):
        now=time.monotonic();dt=min(.1,now-self.last);self.last=now
        v,w=self.command if now-self.received<.5 else (0.,0.)
        self.v+=(v-self.v)*min(1.,dt/.3);self.w+=(w-self.w)*min(1.,dt/.3)
        x,y,yaw=self.pose
        self.pose=[x+self.v*math.cos(yaw)*dt,y+self.v*math.sin(yaw)*dt,math.remainder(yaw+self.w*dt,2*math.pi)]
        x,y,yaw=self.pose
        message=Odometry();message.header.frame_id='odom';message.child_frame_id='base_link'
        message.header.stamp=self.get_clock().now().to_msg()
        message.pose.pose.position.x=x;message.pose.pose.position.y=y;message.pose.pose.position.z=.65
        message.pose.pose.orientation.z=math.sin(yaw/2);message.pose.pose.orientation.w=math.cos(yaw/2)
        message.twist.twist.linear.x=self.v;message.twist.twist.angular.z=self.w
        self.odom.publish(message)
        tf=TransformStamped();tf.header=message.header;tf.child_frame_id='base_link'
        tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.translation.z=.65
        tf.transform.rotation=message.pose.pose.orientation;self.tf.sendTransform(tf)
        self.counter+=1
        if self.counter%5==0:
            # Synthetic scan from this test map, NOT actual sensor observations.
            delta=self.obstacles-np.array([x,y]);delta=delta[np.linalg.norm(delta,axis=1)<5.]
            rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
            xy=delta@rotation
            xyz=np.column_stack([xy,np.full(len(xy),.3)]).astype('<f4')
            cloud=PointCloud2();cloud.header.stamp=message.header.stamp;cloud.header.frame_id='base_link'
            cloud.height=1;cloud.width=len(xyz);cloud.point_step=12;cloud.row_step=len(xyz)*12
            cloud.fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(['x','y','z'])]
            cloud.is_dense=True;cloud.data=xyz.tobytes();self.cloud.publish(cloud)
            self.events.write(json.dumps({'pose':self.pose,'command':list(self.command)})+'\n')


def wait(node,future,timeout):
    deadline=time.monotonic()+timeout
    while not future.done():
        if time.monotonic()>deadline:raise TimeoutError('Software verification deadline')
        rclpy.spin_once(node,timeout_sec=.01)
    return future.result()


def pose_message(node,x,y,yaw):
    msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=node.get_clock().now().to_msg()
    msg.pose.position.x=float(x);msg.pose.position.y=float(y)
    msg.pose.orientation.z=math.sin(yaw/2);msg.pose.orientation.w=math.cos(yaw/2)
    return msg


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--name',default='nav2_check01');args=parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID')=='42' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    out=ROOT/'evidence'/args.name;out.mkdir(exist_ok=False)
    folder=ROOT/'maps/mapping02';config=yaml.safe_load((folder/'map.yaml').read_text())
    grid=np.load(folder/'occupancy.npy');resolution=config['resolution'];origin=np.array(config['origin'][:2])
    clearance=distance_transform_edt(grid==0)*resolution
    tf=np.array(json.loads((folder/'map.pcd.transform.json').read_text())['matrix'])
    row=np.loadtxt(folder/'odom.csv',delimiter=',',skiprows=1)[0]
    initial=tf@np.r_[row[1:4],1.]
    rotation=tf[:3,:3]@Rotation.from_quat(row[4:8]).as_matrix()
    yaw=Rotation.from_matrix(rotation).as_euler('xyz')[2]
    indices=np.argwhere(clearance>.60);points=origin+(indices[:,::-1]+.5)*resolution
    distances=np.linalg.norm(points-initial[:2],axis=1)
    options=points[(distances>.8)&(distances<1.4)]
    if not len(options):raise RuntimeError('No software goal with 0.6m map clearance')
    target=options[np.argmin(np.linalg.norm(options-(initial[:2]+[1.,.4]),axis=1))]
    angle=math.atan2(target[1]-initial[1],target[0]-initial[0])
    occupied=np.argwhere(grid==100);obstacles=origin+(occupied[:,::-1]+.5)*resolution
    rclpy.init();events=(out/'plant.jsonl').open('x');node=Plant([*initial[:2],yaw],obstacles,events)
    report={'synthetic_feedback':True,'physical_robot_tested':False,'ndt_tested_by_this_script':False,
            'initial_pose':node.pose.copy(),'goal_xy':target.tolist(),'results':[]}
    log=(out/'nav2.log').open('x')
    process=subprocess.Popen(['ros2','launch','my_nav2_pointcloud_pkg','nav2_bringup.launch.py',
                              'map:='+str(folder/'map.yaml'),'params_file:='+str(ROOT/'config/nav2.yaml'),
                              'use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+45
        while not node.navigate.server_is_ready():
            if time.monotonic()>deadline or process.poll() is not None:raise RuntimeError('Nav2 startup failed')
            rclpy.spin_once(node,timeout_sec=.05)
        report['lifecycle']={}
        for name in ['map_server','planner_server','controller_server','behavior_server','bt_navigator','waypoint_follower']:
            client=node.create_client(GetState,'/'+name+'/get_state')
            while True:
                value=wait(node,client.call_async(GetState.Request()),5).current_state
                if value.id==3:break
                if time.monotonic()>deadline:raise RuntimeError(name+' not active')
                rclpy.spin_once(node,timeout_sec=.1)
            report['lifecycle'][name]=value.label;node.destroy_client(client)
        client=node.create_client(GetParameters,'/controller_server/get_parameters')
        names=['FollowPath.plugin','FollowPath.desired_linear_vel','FollowPath.rotate_to_heading_angular_vel','general_goal_checker.stateful']
        values=wait(node,client.call_async(GetParameters.Request(names=names)),5).values
        report['loaded_parameters']={n:{'type':v.type,'string':v.string_value,'double':v.double_value,'bool':v.bool_value}for n,v in zip(names,values)}
        for target_yaw in [angle,angle+.7]:
            goal=NavigateToPose.Goal();goal.pose=pose_message(node,*target,target_yaw)
            handle=wait(node,node.navigate.send_goal_async(goal),10)
            if not handle.accepted:raise RuntimeError('Goal rejected')
            result=wait(node,handle.get_result_async(),100)
            until=time.monotonic()+2
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.01)
            distance=math.dist(node.pose[:2],target);error=abs(math.remainder(node.pose[2]-target_yaw,2*math.pi))
            report['results'].append({'status':result.status,'goal_yaw':target_yaw,'final_pose':node.pose.copy(),'xy_error_m':distance,'yaw_error_rad':error})
            if result.status!=4 or distance>.20 or error>.20:raise RuntimeError('Task outcome outside configured tolerances')
        report['nonzero_commands']=node.nonzero;report['status']='passed'
    except Exception as error:report.update(status='failed',error=repr(error))
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
        report['nav2_exit_code']=process.returncode
        log.close();events.close();node.destroy_node();rclpy.try_shutdown()
        (out/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
    return 0 if report['status']=='passed' else 1


if __name__=='__main__':raise SystemExit(main())
