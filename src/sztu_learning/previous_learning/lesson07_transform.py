"""认识变换消息：全部是假设值，不发布 TF，不代表真机外参。"""

from geometry_msgs.msg import TransformStamped

transform = TransformStamped()
transform.header.frame_id = 'demo_base'
transform.child_frame_id = 'demo_lidar'
transform.transform.translation.x = 1.0
transform.transform.rotation.w = 1.0

print(transform)
