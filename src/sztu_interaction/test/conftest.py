#!/usr/bin/env python3
"""
pytest 夹具：没装 AimDK SDK 时也让这个包的测试跑得起来

见 sztu_interaction.testing 的说明。真包在就什么都不做。
"""

from sztu_interaction.testing import aimdk_msgs_available, install_fake_aimdk_msgs

USING_FAKE_AIMDK = install_fake_aimdk_msgs()
HAVE_REAL_AIMDK = aimdk_msgs_available()
