#!/usr/bin/env python3
"""
pytest 夹具：没装 AimDK SDK 时也让这个包的测试跑得起来

sztu_guide 的 bridge_server / guide_manager 在模块顶层就 import 了
sztu_interaction 的控制器，而那些控制器顶层又 import aimdk_msgs。开发机上
SDK 不一定装了，于是连"标签表有没有对齐"这种纯字典断言都会 ImportError。

这里在收集用例之前注入一个假 aimdk_msgs（仅当真包缺席）。装了真 SDK 之后
什么都不做，测试照样用真接口跑。
"""

from sztu_interaction.testing import aimdk_msgs_available, install_fake_aimdk_msgs

# pytest 会先 import conftest 再收集用例，所以放在模块层就够早。
USING_FAKE_AIMDK = install_fake_aimdk_msgs()

# 给用例读的标记：需要真服务的测试可以据此 skip。
HAVE_REAL_AIMDK = aimdk_msgs_available()
