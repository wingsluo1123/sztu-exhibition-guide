# 开机自启：北向桥接服务

让机器人一开机就自动跑起 `remote.launch.py`（SLAM + 导航 + `bridge_server`，
TCP 8888），不必再 ssh 上去手动执行 `restart_bridge.sh`。

实现方式是一个 systemd 系统服务 `sztu-bridge.service`。

| 文件 | 作用 |
|------|------|
| `bridge_boot.sh` | 服务入口：等网络/AimDK 就绪 → `source setup.sh` → `exec ros2 launch` |
| `sztu-bridge.service.in` | systemd 单元模板，`__占位符__` 由安装脚本替换 |
| `install_autostart.sh` | 安装/卸载脚本，在机器人上执行一次 |

> ⚠️ 这套东西要装在**机器人的开发计算单元**（X2 Ultra 为 `10.0.1.41`）上，
> 也就是平时 ssh 进去跑 `restart_bridge.sh` 的那台。
> 远程开发机/仿真机不需要装（要装也能用，前提是有 systemd——WSL 旧版没有）。

---

## 安装

在机器人上，确认工作空间已经能手动跑通（`./restart_bridge.sh` 正常），然后：

```bash
cd ~/sztu
git pull
sudo ./deploy/install_autostart.sh --now
```

- 不加 `--now` 只设置开机自启，不立即启动。
- 想先看生成的单元文件：`./deploy/install_autostart.sh --dry-run`。
- 服务默认以**执行 sudo 前的登录用户**身份运行（如 `sudo` 前是 `agibot`，
  服务就跑在 `agibot` 下，HOME 指向它，`~/aimdk` 才能被 `setup.sh` 找到）。
  要指定别的用户：`--user <名字>`。

装完重启机器人验证：

```bash
sudo reboot
# 起来后
systemctl status sztu-bridge
ss -tlnp | grep 8888
```

## 日常操作

```bash
systemctl status sztu-bridge              # 状态
journalctl -u sztu-bridge -f              # 实时日志（等价于原来前台的输出）
journalctl -u sztu-bridge -b              # 本次开机以来的全部日志
sudo systemctl restart sztu-bridge        # 重启
sudo systemctl stop sztu-bridge           # 临时停（下次开机仍自启）
sudo systemctl disable --now sztu-bridge  # 取消自启并停止
```

装了服务之后，`./restart_bridge.sh` 会自动转成 `systemctl restart sztu-bridge`，
不会再出现「手动实例和服务实例互相抢 8888 端口」的情况。

需要前台调试（看彩色输出、按 Ctrl+C 停）时：

```bash
sudo systemctl stop sztu-bridge
SZTU_NO_SYSTEMD=1 ./restart_bridge.sh ws_enabled:=true
```

## 参数调整

改 `/etc/default/sztu-bridge`（安装脚本会生成一份模板），
然后 `sudo systemctl restart sztu-bridge`。

| 变量 | 默认 | 说明 |
|------|------|------|
| `ROS_DOMAIN_ID` | 安装时自动带入 | 必须与机器人侧一致，见下面的坑 |
| `RMW_IMPLEMENTATION` | 安装时自动带入 | DDS 实现，需与机器人侧一致 |
| `SZTU_LAUNCH_ARGS` | 空 | 追加给 `remote.launch.py`，如 `ws_enabled:=true ws_port:=8080` |
| `SZTU_TCP_PORT` | `8888` | 北向 TCP 端口 |
| `SZTU_PEER_IP` | 空 | 设了就等这个 IP ping 通再启动；空=不等 |
| `SZTU_WAIT_NET` | `120` | 等 `SZTU_PEER_IP` 的秒数 |
| `SZTU_WAIT_AIMDK` | `90` | 等 AimDK ROS 服务出现的秒数，`0`=不等 |

> ⚠️ **systemd 不加载 `~/.bashrc`。**
> 手动 ssh 上去跑的时候，`ROS_DOMAIN_ID` 这类变量是 shell 配置给的；
> 开机自启时它们全都不存在，域 ID 会回落到 0——服务看着正常在跑，
> 却收不到机器人的任何话题，这种故障最难排。
> 所以安装脚本会自动把 `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` 从当前环境或
> 目标用户的 `~/.bashrc` 里读出来，写进 `/etc/default/sztu-bridge`。
> 以后在 bashrc 里改了域 ID，记得同步改这个文件。

## 启动时序是怎么处理的

开机瞬间网络和 AimDK 底层服务往往还没好，所以 `bridge_boot.sh` 按这个顺序把关：

1. 等 `network-online.target`（systemd 层面）；
2. 若配了 `SZTU_PEER_IP`，等它 ping 通；
3. 检查 8888 端口没被占——被占就退出，**不强杀**，避免踢掉别人手动跑的实例；
4. `source setup.sh`，校验工作空间已编译且 `import aimdk_msgs` 成功；
5. 等 `/aimdk_5Fmsgs/srv/*` 服务出现（等不到也照常启动，各节点调用时会自己重试）；
6. `exec ros2 launch`。

任何一步不满足就以非 0 退出，`Restart=always` + `RestartSec=10` 会每 10 秒重试一次，
且 `StartLimitIntervalSec=0` 保证 systemd 不会因重试次数多而放弃。
因此「AimDK 比本服务晚起来几分钟」这种情况能自愈。

停止用 `SIGINT` 而不是默认的 `SIGTERM`，因为 `ros2 launch` 只对 SIGINT 做优雅关闭
（先让各节点 `shutdown()`），否则容易留下僵留节点占着端口。

## 故障排查

**服务反复重启**

```bash
journalctl -u sztu-bridge -n 80 --no-pager
```

按日志里的 `!!` 行定位，常见三种：

| 日志 | 原因 | 处理 |
|------|------|------|
| `工作空间尚未编译` | 缺 `install/setup.bash` | `cd ~/sztu && colcon build --symlink-install` |
| `AimDK 消息库未加载` | `~/aimdk` 不在服务用户的 HOME 下 | 确认单元里的 `User=`/`HOME=` 与实际装 AimDK 的用户一致 |
| `端口 8888 已被占用` | 有人手动跑着 bridge | 停掉手动实例，服务会在 10 秒内自己接上 |
| `不可达，退出等 systemd 重试` | 网口 IP 还没配好 | 检查静态 IP（`10.0.1.2/24`）与网线 |

**服务 active 但客户端收不到数据 / 机器人话题一个都没有**

九成是 `ROS_DOMAIN_ID` 没带进服务（systemd 不读 `~/.bashrc`）。对比一下：

```bash
# 手动 ssh 会话里的值
echo $ROS_DOMAIN_ID
# 服务里的值
sudo systemctl show sztu-bridge -p Environment
journalctl -u sztu-bridge -b | grep 'ROS_DOMAIN_ID'
```

不一致就改 `/etc/default/sztu-bridge` 再 `sudo systemctl restart sztu-bridge`。

**改了单元模板之后**：重新跑一次 `sudo ./deploy/install_autostart.sh` 即可，
它会覆盖 `/etc/systemd/system/sztu-bridge.service` 并 `daemon-reload`。
`/etc/default/sztu-bridge` 已存在时不会被覆盖。

**完全卸载**

```bash
sudo ./deploy/install_autostart.sh --uninstall
```
