"""Offline map viewer overlay; publishes saved estimated trajectory, not a robot TF."""
import argparse
import csv
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--map-directory',type=Path,required=True)
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args)
    node=rclpy.create_node('learning_saved_trajectory')
    cfg=yaml.safe_load((args.map_directory/'effective_config.yaml').read_text())
    metadata=yaml.safe_load((args.map_directory/'map.yaml').read_text())
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher=node.create_publisher(PathMessage,'/learning/map_trajectory',qos)
    message=PathMessage()
    message.header.frame_id=cfg['frame_id']
    message.header.stamp=node.get_clock().now().to_msg()
    with (args.map_directory/'trajectory.csv').open() as stream:
        for row in csv.DictReader(stream):
            pose=PoseStamped()
            pose.header=message.header
            pose.pose.position.x=float(row['x']);pose.pose.position.y=float(row['y'])
            pose.pose.position.z=.03  # flatten the saved trajectory for a 2-D overlay only
            for axis in ['x','y','z','w']:
                setattr(pose.pose.orientation,axis,float(row['q'+axis]))
            message.poses.append(pose)
    # This static transform describes the image's actual lower-left origin,
    # not a fabricated current robot pose. It also registers the map TF frame.
    broadcaster=StaticTransformBroadcaster(node)
    transform=TransformStamped()
    transform.header=message.header
    transform.child_frame_id=cfg['frame_id']+'_grid_origin'
    transform.transform.translation.x=float(metadata['origin'][0])
    transform.transform.translation.y=float(metadata['origin'][1])
    transform.transform.rotation.w=1.
    broadcaster.sendTransform(transform)
    publisher.publish(message)
    node.get_logger().info(f'Published {len(message.poses)} saved poses; display only, no robot control')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
