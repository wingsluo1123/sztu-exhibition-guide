#!/usr/bin/env python3
"""
AimDK 走跑速度控制器 —— 兼容转发。

实现已移到 ``sztu_navigation.locomotion``：走跑控制同时被遥控
（``sztu_guide.bridge_server``）和自主导航（``sztu_navigation.navigation_manager``）
使用，放在"交互"包里层次不对。这里保留再导出，现有的
``from sztu_interaction.locomotion_controller import LocomotionController``
不用改。

新代码请直接 ``from sztu_navigation.locomotion import LocomotionController``。
"""

from sztu_navigation.locomotion import (  # noqa: F401
    LOCOMOTION_QOS,
    LocomotionController,
)

__all__ = ['LocomotionController', 'LOCOMOTION_QOS']
