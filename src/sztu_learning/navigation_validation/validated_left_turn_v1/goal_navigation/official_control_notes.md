# 本轮核对的官方控制约定

2026-11-21读取AimDK_X2 1.1.0在线文档：

- 用户提供的[运动模式切换](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/modeswitch.html)。v0.8.0以上稳定站立/走跑会内部自动切换；不因机器人尚未迈步反复请求切换模式。SetMcAction使用action_desc，新版本current_action不再使用。
- [走跑控制](https://x2-aimdk.agibot.com/zh-cn/latest/Interface/control_mod/locomotion.html)。消息McLocomotionVelocity，话题/aima/mc/locomotion/velocity，BEST_EFFORT+VOLATILE，推荐50Hz；前进正、左移正、逆时针角速度正，单位m/s和rad/s。需注册自定义source，不使用内置源冒充。
- 速度接口给目标速度，实际响应有步态/固件相关误差；需要自研定位反馈和闭环到点控制。文档推荐的厂商导航选装包不属于本项目已购能力。
- 官方起步门限仅为大致值，不能替代实测：前进约0.1m/s，角速度约0.03–0.05rad/s。当前本项目采用已实走的0.15m/s；首次转动0.70rad/s依据ROADMAP 2026-09-30真机记录和本次用户边界，不能声称官网要求0.70。
- 超过200ms未收到新指令会清零；本程序另有0.20s指令超时和独立看门狗。这个机制不等同于已实测停车距离。
- 遥控器活动时可能抢占SDK输入；保留遥控器优先级。回读消息、MC选中source、实际运动是三种不同证据。
- 官网要求避免短时间批量启动节点（建议不超过2个/秒），程序运行于开发计算单元，不部署到运控PC1。

本轮实际非零转动仅约0.19秒就中止，用户观察主要晃动，不能据此推断转向符号错误或角速度不足。已发现控制端单次只取一条回调，存在消息排队问题；修正为有时间预算的批量处理，并保持原新鲜度上限。后续需重新做独立现场试验。
