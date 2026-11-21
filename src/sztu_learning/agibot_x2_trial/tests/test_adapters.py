import importlib.util
from pathlib import Path
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.parameter import Parameter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'upstream/src/py_examples'))
sys.path.insert(0,str(ROOT/'scripts'))
from py_examples.pcl_rectifier import PclRectifier
from kinematic_odom import base_pose, matrix
from scipy.spatial.transform import Rotation


def test_rectifier_preserves_acquisition_times_fields_and_padded_rows():
    rclpy.init()
    node = PclRectifier()
    try:
        captured=[]
        class Publisher:
            def publish(self,message):captured.append(message)
        node.pub=Publisher()
        message=PointCloud2()
        message.header.stamp.sec=10
        message.header.stamp.nanosec=123
        message.header.frame_id='lidar_chest_front'
        message.height=2;message.width=2;message.point_step=32;message.row_step=72
        message.fields=[PointField(name=k,offset=offset,datatype=kind,count=1) for k,offset,kind in
                        [('x',0,7),('y',4,7),('z',8,7),('intensity',16,7),('ring',20,4),('timestamp',24,8)]]
        raw=bytearray(message.row_step*message.height)
        dtype=np.dtype({'names':['x','y','z','intensity','ring','timestamp'],
                        'formats':['<f4','<f4','<f4','<f4','<u2','<f8'],
                        'offsets':[0,4,8,16,20,24],'itemsize':32})
        points=np.ndarray((2,2),dtype=dtype,buffer=raw,strides=(72,32))
        points['x']=1;points['y']=2;points['z']=3
        points['intensity']=42;points['ring']=[[0,31],[63,95]]
        times=np.array([[332000,30000000],[60000000,95000000]],dtype=float)
        points['timestamp']=times;message.data=bytes(raw)
        node.cb(message)
        out=captured[0]
        output_dtype=np.dtype({'names':['x','y','z','ring','timestamp','time'],
                               'formats':['<f4','<f4','<f4','<u2','<f8','<f4'],
                               'offsets':[0,4,8,20,24,32],'itemsize':36})
        value=np.frombuffer(out.data,dtype=output_dtype)
        np.testing.assert_allclose(np.column_stack([value[k] for k in ['x','y','z']]),[[1,3,-2]]*4,atol=1e-6)
        np.testing.assert_array_equal(value['ring'],[0,31,63,95])
        absolute_output=out.header.stamp.sec+out.header.stamp.nanosec*1e-9+value['time'].astype(float)
        absolute_input=10+123e-9+times.reshape(-1)*1e-9
        np.testing.assert_allclose(absolute_output,absolute_input,rtol=0,atol=5e-9)
        assert out.header.frame_id=='lidar_rectified'
        assert message.header.frame_id=='lidar_chest_front'
        assert out.row_step==72 and len(out.data)==144
    finally:
        node.destroy_node();rclpy.shutdown()


def test_base_reference_point_composition_with_dynamic_chest():
    # Known base motion composed with two different chest poses must recover the
    # same robot base, instead of reporting chest sway as base translation.
    expected=matrix([1,2,0.4],Rotation.from_euler('z',0.7).as_quat())
    for pitch in [-0.15,0.15]:
        base_from_imu=matrix([0.1,0,0.4],Rotation.from_euler('y',pitch).as_quat())
        np.testing.assert_allclose(base_pose(expected@base_from_imu,base_from_imu),expected,atol=1e-12)
