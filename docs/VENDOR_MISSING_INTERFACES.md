# 开发机上缺的厂商接口（以及怎么自己补出来）

## 问题

开发机上挂的 `aimdk_msgs`（`~/aimdk/install`）比运控二进制里带的那份**旧**。
运控自己带了一整套 `libaimdk_msgs__rosidl_*`，里面有 **59 个服务 / 115 个消息**；
`~/aimdk` 和 `~/agi-aimdk` 两个 checkout 的 `interface/` 里都只有一部分。

表现：运控注册了某个服务，`ros2 service list` 也看得到，但是

```
$ ros2 service call /aimdk_5Fmsgs/srv/GetMcLocomotionSpeedStatus \
    aimdk_msgs/srv/GetMcLocomotionSpeedStatus "{}"
The passed service type is invalid
```

—— 本地没有这个类型的定义。厂商发的包里只有编译产物，没有 `.srv` 源文件，
也没有 Python 绑定（`find` 过，一个 `*cpython*` 都没有）。

## 怎么反推

`tools/dump_vendor_idl.py`。rosidl 的 introspection typesupport 库里存着完整的
运行期反射信息（字段名、类型、嵌套结构），用 ctypes 把那些结构体走一遍就能
按 `.msg` 的写法打印出来。

```bash
source /opt/ros/humble/setup.bash          # 别挂 ~/aimdk/install，见工具里的坑 1
MC=~/mc-x86-v1.0.0-20260522
LD_LIBRARY_PATH="$MC/lib:$MC/mc3th/lib:$LD_LIBRARY_PATH" \
  tools/dump_vendor_idl.py \
    --lib "$MC/lib/libaimdk_msgs__rosidl_typesupport_introspection_c.so" --list
```

反推出来的定义**不是权威的**：字段名、类型、顺序是准的（够用来通信），
但默认值和常量（`uint8 FOO=1` 这类）拿不到。真要准确还是得找厂商要源文件。

## 怎么用上

厂商的 `aimdk_msgs` 是从 `interface/` 目录 `GLOB_RECURSE` 出来再
`rosidl_generate_interfaces` 的，所以把补出来的文件放进去重新生成就行。
**别改 `~/aimdk`**，拷一份出来：

```bash
rm -rf /tmp/aimdk_ws && mkdir -p /tmp/aimdk_ws/src
cp -a ~/aimdk/src/aimdk_msgs /tmp/aimdk_ws/src/
# 把下面的 .msg / .srv 写进 /tmp/aimdk_ws/src/aimdk_msgs/interface/robot/mc/motion/
cd /tmp/aimdk_ws
colcon build --packages-select aimdk_msgs \
  --cmake-args -DUSE_PREBUILT_PACKAGE=OFF -DBUILD_TESTING=OFF
source /tmp/aimdk_ws/install/setup.bash    # 用这个**代替** ~/aimdk/install
```

生成出来的包名/类型名和运控那份一致（`aimdk_msgs/srv/...`），Humble 按
类型名匹配，所以能通上。**字段顺序就是序列化顺序，一个都不能调** ——
类型名对得上、字节对不上的话，调用会静默地读到垃圾。

## 2026-09-29 补的这几个

当时在查"仿真里机器人一步都不走"，怀疑速度档位被设成了 `none_speed`
（`speed_state.yaml` 里那一档三个方向的上下界全是 0）。要确认就得调这两个服务。

> 结论先说：**怀疑错了。** 实测 `current_speed_status.speed_mode.value == 1`
> （`low_speed`，角速度上下界 ±1.0），指令根本没被夹。真正的原因是我们自己的
> `turn_probe.py` 漏了喂狗 —— `LocomotionController.set_velocity` 只登记目标值，
> `tick()` 里有 0.25 秒看门狗，过期就发零。详见 `tools/turn_probe.py` 的坑 4。
>
> 留着这几个定义是因为它们本身有用（要限速的时候就得调 Set 那个），
> 而且反推这条路以后还会用到。

放在 `interface/robot/mc/motion/` 下。

### `msg/McLocomotionSpeedMode.msg`

```
# 走跑速度档位。0=none(上下界全 0) 1=low 2=medium 3=high 4=step
# 见 mc_param/src/robot/<型号>/speed_state.yaml
int32 value
```

### `msg/McLocomotionSpeedBounds.msg`

```
float64 min_value
float64 max_value
```

### `msg/McLocomotionSpeedStatus.msg`

```
McLocomotionSpeedMode speed_mode

McLocomotionSpeedBounds forward_bounds
McLocomotionSpeedBounds lateral_bounds
McLocomotionSpeedBounds angular_bounds
```

### `srv/SetMcLocomotionSpeedMode.srv`

```
# Request 部分
RequestHeader header

McLocomotionSpeedMode speed_mode
---

CommonResponse response
```

### `srv/GetMcLocomotionSpeedStatus.srv`

```
# Request 部分
CommonRequest request
---

ResponseHeader header

McLocomotionSpeedStatus current_speed_status
McLocomotionSpeedStatus none_speed_status
McLocomotionSpeedStatus low_speed_status
McLocomotionSpeedStatus medium_speed_status
McLocomotionSpeedStatus high_speed_status
McLocomotionSpeedStatus step_speed_status
```

反推的定义验证过：`GetMcLocomotionSpeedStatus` 的响应能正常反序列化，
六个档位的上下界和 `speed_state.yaml` 里写的**逐字对得上** ——
这说明字段布局是对的，不是碰巧能解出来。

## 顺带记一条：X2 的速度档位表

`~/mc-x86-v1.0.0-20260522/bin/mc_param/src/robot/lx2501_3_t2d5/speed_state.yaml`，
运控实测默认在 `low_speed`（配置文件里**没有**任何地方声明默认档位，
但实测是 1 而不是 0）。

| 档位 | mode | forward | lateral | angular |
|---|---|---|---|---|
| none | 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| low | 1 | -1.0 / 0.8 | ±1.0 | ±1.0 |
| medium | 2 | -1.0 / 1.2 | ±1.0 | ±1.0 |
| high | 3 | -1.0 / 1.8 | ±1.0 | ±1.0 |
| step | 4 | ±2.0 | ±1.0 | ±1.0 |

注意 `low_speed` 的角速度上界是 1.0，而 `control.py` 的 `max_angular_speed`
现在是 0.8 —— 还有余量。仿真里实测最小有效原地转速是 0.70 rad/s
（`tools/turn_probe.py`），离 1.0 不远，所以**没多少下调空间**。
