# 开机自启：北向桥接服务

让机器人一开机就自动跑起 `remote.launch.py`（SLAM + 导航 + `bridge_server`，
TCP 8888），不必再 ssh 上去手动执行 `restart_bridge.sh`。

提供三种方式，按机器人上拿到的权限选：

| 方式 | 前提 | 装法 |
|------|------|------|
| systemd 系统服务（最好） | 能 sudo 执行任意命令 | `sudo ./deploy/install_autostart.sh --now` |
| systemd 用户服务 | 无需 root，但要能开 linger | `./deploy/install_autostart_nosudo.sh --now` |
| cron `@reboot` + 守护壳 | 零权限，永远可用 | 同上，自动退回这种 |

`install_autostart_nosudo.sh` 会自己探测 linger 能不能开，开不了就退回 cron。
**拿不准就直接用它**，它不需要任何特权。

| 文件 | 作用 |
|------|------|
| `bridge_boot.sh` | 启动入口：等网络/AimDK 就绪 → `source setup.sh` → `exec ros2 launch` |
| `bridge_supervise.sh` | cron 方式的守护壳：自动重启 + 日志轮转 + 启停控制 |
| `sztu-bridge.service.in` | systemd 系统单元模板 |
| `sztu-bridge.user.service.in` | systemd 用户单元模板 |
| `install_autostart.sh` | 需要 root 的安装脚本 |
| `install_autostart_nosudo.sh` | 不需要 root 的安装脚本 |
| `_common.sh` | 两个安装脚本共用的环境变量探测 |

> ⚠️ 这套东西要装在**机器人的开发计算单元**（X2 Ultra 为 `10.0.1.41`）上，
> 也就是平时 ssh 进去跑 `restart_bridge.sh` 的那台。
> 远程开发机/仿真机不需要装（要装也能用，前提是有 systemd——WSL 旧版没有）。

三种方式不会共存：装任意一种时，脚本会把另外两种的残留清掉，
避免开机起两个实例抢 8888 端口。

---

## 方式一：systemd 系统服务（需要 root）

在机器人上，确认工作空间已经能手动跑通（`./restart_bridge.sh` 正常），然后：

```bash
cd ~/sztu
git pull
sudo ./deploy/install_autostart.sh --now
```

- 不加 `--now` 只设置开机自启，不立即启动。
- 想先看生成的单元文件：`./deploy/install_autostart.sh --dry-run`。
- 服务默认以**执行 sudo 前的登录用户**身份运行（如 `sudo` 前是 `agi`，
  服务就跑在 `agi` 下，HOME 指向它，`~/aimdk` 才能被 `setup.sh` 找到）。
  要指定别的用户：`--user <名字>`。

> 如果报 `user xxx is not allowed to execute ... as root`，说明这台机器的 sudo
> 是命令白名单（厂商常见配置）。先 `sudo -l` 看看白名单里有什么；
> 没有可用的口子就用方式二。

装完重启机器人验证：

```bash
sudo reboot
# 起来后
systemctl status sztu-bridge
ss -tlnp | grep 8888
```

## 方式二：不需要 root

```bash
cd ~/sztu
git pull
./deploy/install_autostart_nosudo.sh --now   # 注意：不要加 sudo
```

脚本最后会打印它选了哪种方式和对应的常用命令。

- **systemd 用户服务**：能开 linger 时用这个，有 `Restart=always` 和
  `journalctl --user`，体验与系统服务基本一致。
  linger 的作用是「开机就拉起用户实例，不用等人登录」——没有它，
  用户服务只会在 ssh 登录时启动，不算开机自启。
- **cron `@reboot` + 守护壳**：linger 开不了时的兜底，零权限。
  cron 没有自动重启也没有 journald，这两件事由 `bridge_supervise.sh` 自己做：
  单实例锁、退出后 10 秒重试、日志写到 `log/bridge/`（保留最近 10 份，
  `latest.log` 指向本次开机）。
  想直接指定这种：加 `--cron`。

卸载（两种一起清）：`./deploy/install_autostart_nosudo.sh --uninstall`

## 日常操作

**系统服务**（方式一）：

```bash
systemctl status sztu-bridge              # 状态
journalctl -u sztu-bridge -f              # 实时日志（等价于原来前台的输出）
journalctl -u sztu-bridge -b              # 本次开机以来的全部日志
sudo systemctl restart sztu-bridge        # 重启
sudo systemctl stop sztu-bridge           # 临时停（下次开机仍自启）
sudo systemctl disable --now sztu-bridge  # 取消自启并停止
```

**用户服务**（方式二之一）：把上面的 `systemctl` 换成 `systemctl --user`、
`journalctl -u` 换成 `journalctl --user -u`，而且都不需要 sudo。

**cron 守护壳**（方式二之二）：

```bash
./deploy/bridge_supervise.sh --status     # 状态（守护循环 / 桥接进程 / 端口）
./deploy/bridge_supervise.sh --log        # 实时日志
./deploy/bridge_supervise.sh --restart    # 重启桥接（守护循环 10s 内拉起新的）
./deploy/bridge_supervise.sh --stop       # 停掉（下次开机仍自启）
```

装了自启之后，`./restart_bridge.sh` 会自动识别是哪种方式并转发过去，
不会再出现「手动实例和自启实例互相抢 8888 端口」的情况。

需要前台调试（看彩色输出、按 Ctrl+C 停）时，先停掉自启实例，然后：

```bash
SZTU_NO_SYSTEMD=1 ./restart_bridge.sh ws_enabled:=true
```

## 参数调整

| 方式 | 配置文件 | 改完怎么生效 |
|------|---------|-------------|
| 系统服务 | `/etc/default/sztu-bridge` | `sudo systemctl restart sztu-bridge` |
| 用户服务 | `~/.config/sztu-bridge.env` | `systemctl --user restart sztu-bridge` |
| cron 守护壳 | `~/.config/sztu-bridge.env` | `./deploy/bridge_supervise.sh --restart` |

安装脚本会生成一份带注释的模板；文件已存在时不覆盖。

| 变量 | 默认 | 说明 |
|------|------|------|
| `ROS_DOMAIN_ID` | 安装时自动带入 | 必须与机器人侧一致，见下面的坑 |
| `RMW_IMPLEMENTATION` | 安装时自动带入 | DDS 实现，需与机器人侧一致 |
| `SZTU_LAUNCH_ARGS` | 空 | 追加给 `remote.launch.py`，如 `ws_enabled:=true ws_port:=8080` |
| `SZTU_TCP_PORT` | `8888` | 北向 TCP 端口 |
| `SZTU_PEER_IP` | 空 | 设了就等这个 IP ping 通再启动；空=不等 |
| `SZTU_WAIT_NET` | `120` | 等 `SZTU_PEER_IP` 的秒数 |
| `SZTU_WAIT_AIMDK` | `90` | 等 AimDK ROS 服务出现的秒数，`0`=不等 |
| `SZTU_RETRY_SEC` | `10` | 仅 cron 方式：退出后多久重试 |
| `SZTU_KEEP_LOGS` | `10` | 仅 cron 方式：`log/bridge/` 保留几份日志 |

> ⚠️ **systemd 和 cron 都不加载 `~/.bashrc`。**
> 手动 ssh 上去跑的时候，`ROS_DOMAIN_ID` 这类变量是 shell 配置给的；
> 开机自启时它们全都不存在，域 ID 会回落到 0——服务看着正常在跑，
> 却收不到机器人的任何话题，这种故障最难排。
> 所以安装脚本会自动把 `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` 从当前环境或
> 目标用户的 `~/.bashrc` 里读出来，写进上表对应的配置文件。
> 以后在 bashrc 里改了域 ID，记得同步改这个文件。

## 启动时序是怎么处理的

开机瞬间网络和 AimDK 底层服务往往还没好，所以 `bridge_boot.sh` 按这个顺序把关：

1. 等 `network-online.target`（systemd 层面）；
2. 若配了 `SZTU_PEER_IP`，等它 ping 通；
3. 检查 8888 端口没被占——被占就退出，**不强杀**，避免踢掉别人手动跑的实例；
4. `source setup.sh`，校验工作空间已编译且 `import aimdk_msgs` 成功；
5. 等 `/aimdk_5Fmsgs/srv/*` 服务出现（等不到也照常启动，各节点调用时会自己重试）；
6. `exec ros2 launch`。

任何一步不满足就以非 0 退出，然后每 10 秒重试一次
（systemd 靠 `Restart=always` + `RestartSec=10` + `StartLimitIntervalSec=0`；
cron 方式靠 `bridge_supervise.sh` 的循环）。
因此「AimDK 比本服务晚起来几分钟」这种情况能自愈。

停止用 `SIGINT` 而不是默认的 `SIGTERM`，因为 `ros2 launch` 只对 SIGINT 做优雅关闭
（先让各节点 `shutdown()`），否则容易留下僵留节点占着端口。
守护壳里为此开了 `set -m`：非交互 shell 用 `&` 起的后台任务会继承
`SIGINT=SIG_IGN`（POSIX 规定），而被忽略的信号在 bash 里无法再 trap 回来，
不开 job control 的话 `ros2 launch` 根本收不到 SIGINT，只能被硬杀。

## 故障排查

**服务反复重启**

```bash
journalctl -u sztu-bridge -n 80 --no-pager      # 系统服务
journalctl --user -u sztu-bridge -n 80          # 用户服务
./deploy/bridge_supervise.sh --log              # cron 守护壳
```

按日志里的 `!!` 行定位，常见几种：

| 日志 | 原因 | 处理 |
|------|------|------|
| `工作空间尚未编译` | 缺 `install/setup.bash` | `cd ~/sztu && colcon build --symlink-install` |
| `AimDK 消息库未加载` | `~/aimdk` 不在运行用户的 HOME 下 | 确认自启用的用户与实际装 AimDK 的用户一致 |
| `端口 8888 已被占用` | 有人手动跑着 bridge | 停掉手动实例，10 秒内会自己接上 |
| `不可达，退出等 systemd 重试` | 网口 IP 还没配好 | 检查静态 IP（`10.0.1.2/24`）与网线 |

**服务 active 但客户端收不到数据 / 机器人话题一个都没有**

九成是 `ROS_DOMAIN_ID` 没带进自启环境（systemd / cron 都不读 `~/.bashrc`）。
对比一下：

```bash
# 手动 ssh 会话里的值
echo $ROS_DOMAIN_ID
# 自启环境里的值（日志每次启动都会打印）
journalctl -u sztu-bridge -b | grep ROS_DOMAIN_ID     # systemd
grep ROS_DOMAIN_ID log/bridge/latest.log              # cron
```

不一致就改对应的配置文件（见「参数调整」）再重启。

**cron 方式开机后没起来**

```bash
grep CRON /var/log/syslog | tail          # 看 @reboot 有没有被触发
crontab -l                                # 确认条目还在
cat log/bridge/cron.log                   # 守护壳自身的输出
```

机器人厂商如果把 cron 服务关了（`systemctl is-enabled cron`），这条路就不通，
只能回到方式一想办法拿 root。

**改了单元模板之后**：重新跑一次对应的安装脚本即可，它会覆盖单元文件并
`daemon-reload`。配置文件（`/etc/default/sztu-bridge`、`~/.config/sztu-bridge.env`）
已存在时不会被覆盖。

**完全卸载**

```bash
sudo ./deploy/install_autostart.sh --uninstall        # 方式一
./deploy/install_autostart_nosudo.sh --uninstall      # 方式二（两种都清）
```
