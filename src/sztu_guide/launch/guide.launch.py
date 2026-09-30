#!/usr/bin/env python3
"""Launch 文件：启动讲解系统的各个子系统（**不含 guide_manager**）

**这个 launch 和 `ros2 run sztu_guide guide_manager` 是二选一，不要同时跑。**
guide_manager 在进程内自己 new 了 SlamManager / NavigationManager /
TtsController / EmotionController / GestureController 全套（见
guide_manager.py:58-71）。两边一起起，上面每个节点都会有两个实例，其中
**两个 NavigationManager 会用同一个名字 sdk_sztu_nav、同一个优先级 45
去注册 AimDK 走跑输入源，并且同时下发速度** —— 这是实走时最危险的一种
重复，机器人会收到两路互相打架的速度指令。

**同一个陷阱还有第二个入口，而且它默认开机就在跑：`remote.launch.py`
也起一个独立的 navigation_manager，而 `sztu-bridge.service`（systemd
用户级单元，Restart=always）启动的就是它。** 真机上实测到过：
sztu-bridge 已经跑着一个 navigation_manager（**没带 dry_run**），
这时候再 `ros2 run sztu_guide guide_manager` 就正好凑齐上面那两路。
所以实走前先停掉它：

    systemctl --user stop sztu-bridge      # 走完再 start 回去

判断有没有重的办法（方括号是必需的，不是花哨）：

    pgrep -af 'navigation_[m]anager'

`ros2 node list | grep -c navigation_manager` 和 `pgrep -cf navigation_manager`
这两种写法都实测过，**都会骗人**：前者会把 ros2 daemon 里同名的陈旧发现条目
一起数进去（只有一个进程时数出 2），后者会匹配到发起这条命令的 shell 自己
（一个都没跑时数出 1）。详见 ROADMAP「这个自检我先前给错了两版」。

所以实走只用一条命令：
    source ~/sztu/setup.sh
    ros2 run sztu_guide guide_manager
空跑（不下发速度，只看日志）：
    ros2 run sztu_guide guide_manager --ros-args -p dry_run:=true -p navigation_timeout:=8.0
（那句 source 不能省，而且**不等于** `source install/setup.bash`：
 guide_manager 一路 import 到 `sztu_slam.slam_manager`，它要厂商的
 `aimdk_msgs`，而那个包既不在 /opt/ros 里也不在本工作区里，是从
 `~/aimdk/src/aimdk_msgs/prebuilt_aarch64` 靠 PYTHONPATH /
 LD_LIBRARY_PATH / AMENT_PREFIX_PATH 三个变量挂上去的 —— setup.sh 干的
 就是这件事。只 source 前两个的话报的是 `ModuleNotFoundError:
 No module named 'aimdk_msgs'`，看着像包没装，其实是环境没配全。
 终端 1 的 run_lio_live.sh 不用管，它自己 source。）
（实测过：不带节点名的 -p 会透进进程内自建的每个子节点，包括
 navigation_manager。注意 dry_run 只拦底盘，胳膊和嘴照常动。
 navigation_timeout 要一起压短：dry_run 下机器人不动、位姿不变，
 每个点位都会走满默认的 120 秒才报超时 —— 7 个点 14 分钟，看着像全线失败，
 其实是空跑的必然结果。）

这个 launch 留给"只想起子系统、自己写上层逻辑"的场景。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明参数
    language_arg = DeclareLaunchArgument(
        'language',
        default_value='zh_CN',
        description='Language for the tour (zh_CN or en_US)'
    )
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='true 时导航照常算控制量但不下发速度，机器人不动'
    )

    # SLAM 管理器
    slam_node = Node(
        package='sztu_slam',
        executable='slam_manager',
        name='slam_manager',
        output='screen'
    )

    # 位姿监控
    pose_monitor_node = Node(
        package='sztu_slam',
        executable='pose_monitor',
        name='pose_monitor',
        output='screen'
    )

    # 导航管理器
    navigation_node = Node(
        package='sztu_navigation',
        executable='navigation_manager',
        name='navigation_manager',
        output='screen',
        parameters=[{'dry_run': LaunchConfiguration('dry_run')}]
    )

    # TTS 控制器
    tts_node = Node(
        package='sztu_interaction',
        executable='tts_controller',
        name='tts_controller',
        output='screen'
    )

    # 表情控制器
    emotion_node = Node(
        package='sztu_interaction',
        executable='emotion_controller',
        name='emotion_controller',
        output='screen'
    )

    # 手势控制器
    gesture_node = Node(
        package='sztu_interaction',
        executable='gesture_controller',
        name='gesture_controller',
        output='screen'
    )

    # 讲解员核心不在这里起：guide_manager 的 main() 是交互式菜单（要在终端
    # 里按 1/2 选语言），由 launch 托管的话拿不到 stdin。而且它自建全套子系统
    # ——见本文件开头，两边同时起会出现两个 NavigationManager 抢同一个走跑
    # 输入源。实走请只用 ros2 run sztu_guide guide_manager。

    return LaunchDescription([
        language_arg,
        dry_run_arg,
        slam_node,
        pose_monitor_node,
        navigation_node,
        tts_node,
        emotion_node,
        gesture_node,
    ])
