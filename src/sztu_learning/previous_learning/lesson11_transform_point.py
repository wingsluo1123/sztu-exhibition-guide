"""转换一个假设雷达点；不连接 ROS 网络，不代表真机外参。"""

from math import sin, cos, pi
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_geometry_msgs import do_transform_point

transform = TransformStamped()
transform.header.frame_id = 'demo_base'
transform.child_frame_id = 'demo_lidar'
transform.transform.translation.x = 1.0
transform.transform.translation.y = 0.5
transform.transform.rotation.z = sin(pi / 4)
transform.transform.rotation.w = cos(pi / 4)

point = PointStamped()
point.header.frame_id = 'demo_lidar'
point.point.x = 2.0

point_in_base = do_transform_point(point, transform)
print(f'input: frame={point.header.frame_id}, '
      f'xyz=({point.point.x:.3f}, {point.point.y:.3f}, {point.point.z:.3f})')
print(f'output: frame={point_in_base.header.frame_id}, '
      f'xyz=({point_in_base.point.x:.3f}, {point_in_base.point.y:.3f}, '
      f'{point_in_base.point.z:.3f})')
