#!/usr/bin/env python3
"""
建图工具节点

提供交互式建图功能
"""

import sys
import rclpy
from sztu_slam.slam_manager import SlamManager


def main(args=None):
    rclpy.init(args=args)
    slam_manager = SlamManager()

    print("\n" + "="*60)
    print("        SZTU 展厅建图工具")
    print("="*60)
    print("\n操作说明:")
    print("  1 - 开始建图")
    print("  2 - 停止建图并保存")
    print("  q - 退出")
    print("="*60 + "\n")

    try:
        while rclpy.ok():
            # 处理 ROS 回调
            rclpy.spin_once(slam_manager, timeout_sec=0.1)

            # 非阻塞输入检查
            import select
            if select.select([sys.stdin], [], [], 0.1)[0]:
                choice = input("\n请选择操作: ").strip()

                if choice == '1':
                    slam_manager.start_mapping()
                    print("Mapping started，请控制机器人移动以扫描环境")

                elif choice == '2':
                    map_name = input("请输入地图名称: ").strip()
                    if not map_name:
                        stamp = int(rclpy.clock.Clock().now().nanoseconds / 1e9)
                        map_name = f"exhibition_map_{stamp}"

                    if slam_manager.stop_mapping(map_name):
                        print(f"Map saved为: {map_name}")
                    else:
                        print("Save failed")

                elif choice.lower() == 'q':
                    print("👋 退出建图工具")
                    break

                else:
                    print("Invalid input, please enter 1, 2, or q")

    except KeyboardInterrupt:
        print("\n\n👋 用户中断")
    finally:
        slam_manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
