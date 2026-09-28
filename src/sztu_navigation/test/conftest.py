#!/usr/bin/env python3
"""
pytest 夹具：没装 AimDK SDK 时也让这个包的测试跑得起来。

和 sztu_interaction/test/conftest.py 同一套机制，见 sztu_interaction.testing。
这个包原先不需要它 —— test_control.py 和 test_waypoints.py 测的都是纯函数，
不碰 aimdk_msgs。test_locomotion_response.py 要 import locomotion，而
locomotion 在模块层就 import aimdk_msgs，所以这里必须先把替身装上，
否则缺 SDK 的机器上会在 collection 阶段就报 ModuleNotFoundError。

替身里 CommonState 的取值是照抄真包的，所以判读测试拿它当输入是有效的。
"""

from sztu_interaction.testing import aimdk_msgs_available, install_fake_aimdk_msgs

USING_FAKE_AIMDK = install_fake_aimdk_msgs()
HAVE_REAL_AIMDK = aimdk_msgs_available()
