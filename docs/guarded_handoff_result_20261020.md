# 受保护步态与阻尼记录：2026-10-20

本次单次授权已执行。操作者确认“出现步态，期间按键，已停住并保持阻尼”，
尚未恢复站立。该现场观察与原始阻尼状态一致；但软件早于阻尼切换停止
发送前进指令，不能将本次标为持续SDK指令下的阻尼抢停验证通过。

以硬件话题读回首条SDK非零速度为0秒：

| 相对时间 | 原始记录 |
|---|---|
| 0～0.219s | 共12条SDK前进指令，0.10m/s，侧向/角速度零 |
| 0.0255s | MC状态选中SDK输入源 |
| 0.2266s | STAND_DEFAULT / status=200 / fsm=4 / body=1 |
| 0.2385s | 首条随后零速读回 |
| 0.3261s | LOCOMOTION_DEFAULT / status=100 / fsm=4 / body=1 |
| 0.4265s | LOCOMOTION_DEFAULT / status=100 / fsm=5 / body=1 |
| 4.1269s | DAMPING_DEFAULT首次出现，其后fsm=6、body=4 |

程序退出原因为`mc_state_invalid`，`wire_test_passed=false`，
`hardware_stop_verified=false`。首个异常字段由原始轨迹及当时guard推断为
动作状态200；旧版未保存回调ROS时间和首故障快照，不能据此重新验证当时
每条消息的源年龄。保存全部原文件，不把失败记录改成通过。

SDK实际消息契约在`/agibot/software/common/share/aimdk_msgs/msg/`：
`McActionStatus`定义100=RUNNING、200=TRANSITION；`McFsmState`定义
3=MOVING、4=SAFE、5=SPECIAL、6=TEST；`McBodyPoseStatus`定义1=STAND、
4=SIT、5=SQUAT。不能因为现场报告正在走路，就把任意SPECIAL状态放入普通
行走允许集；也不能把TEST/SQUAT当作阻尼枚举。现有排除和锁停行为保持。

本次软件变更只增加首次故障字段、检测时间和触发观测的保存；后续健康或
阻尼消息不能覆盖首次故障。终端摘要移除冗长的控制源查询数组，完整数组
仍写入JSON。加入实机模式轨迹回归和TRANSITION隔离域集成用例。

代码`b638e4e`已推送云端、真机快进同步；两机navigation包编译成功，未
重启现有导航/定位服务。开发机相关62项测试通过。机器人真实SDK隔离域42
测试为22通过、1失败、15未选：正常流程在零速阶段出现269ms回读间隔，
触发既有250ms的`own_wire_lost`看门狗；MC故障回归及TRANSITION用例通过。
正常流程单独复查1通过、21未选，原失败保留，不据此宣布时序问题已解决。
没有放宽任何看门狗或状态允许集。

测试后的只读6秒记录收到37条MC状态、33条里程计，末帧保持阻尼描述、
fsm=6、body=4；没有收到硬件速度消息，这不等于验证了硬件速度为零。
另外4秒窗口收到160条全零cmd_vel。live及三个覆盖标志均false，竞争桥接
inactive。没有注册控制源、发送速度、重启定位或恢复站立。

原始文件归档于两机`~/.local/state/sztu-nav-diagnosis/20261020_guarded_handoff/`，
模式摘要和哈希见
[诊断JSON](../maps/classroom/diagnostics/20261020/guarded_handoff_summary.json)。
未记录按键时间沿，也没有独立距离读数；不能报告按键响应延时或20cm
停车距离通过。

后续先核实本机实际运控的站立/行走切换及SPECIAL状态语义，设计零速等待
切换、限定状态组合的策略并完成隔离验证，再提出新的有限实物试验。
本记录不产生重试授权。地面短程实走仍需近身/身体扫掠观测、停止距离与
有效地图证据；深度候选诊断流不能直接视为连续避障覆盖。
