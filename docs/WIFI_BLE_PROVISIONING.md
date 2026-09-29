# BLE Wi-Fi 配网服务

`remote.launch.py` 会启动 `sztu_interaction` 包中的 BLE GATT 配网服务。服务发现名默认为 `SZTU-WiFi`，可通过 `SZTU_BLE_NAME` 修改。客户端连接后发现服务并订阅状态通知；配网写特征使用普通 BLE 写入，不要求配对。

## GATT 协议

| 项目 | UUID | 属性 |
|------|------|------|
| 配网服务 | `7d2ea28a-f7bd-485a-bd9d-92ad6ecfe93e` | Primary |
| 配网请求 | `7d2ea28b-f7bd-485a-bd9d-92ad6ecfe93e` | Write |
| 配网状态 | `7d2ea28c-f7bd-485a-bd9d-92ad6ecfe93e` | Read, Notify |

客户端向请求特征写入 UTF-8 JSON，例如：

```json
{"ssid":"Office-WiFi","password":"your-password"}
```

状态特征返回 JSON 通知：

```json
{"status":"connecting"}
{"status":"connected","ip":"192.168.1.42"}
```

失败时返回 `{"status":"error","error":"..."}`；错误码包括 `invalid_request`、`request_too_large`、`busy`、`aima_unavailable`、`connection_timeout` 和 `connection_failed`。请求最大 512 字节，SSID 最大 32 个 UTF-8 字节，密码最大 128 个 UTF-8 字节。

配网通过设备自带的 `aima` Wi-Fi 接口完成，不直接调用 `nmcli`。密码不会写入服务日志或命令行参数；由于写入不要求配对或链路加密，附近设备可能监听或提交配网请求，请仅在可信的近距离环境使用。

## 部署

系统需要 BlueZ、Python Bluezero 和设备自带的 `aima` 命令。修改后重新编译工作空间，再重启现有桥接服务：

```bash
cd ~/sztu
colcon build --symlink-install --packages-select sztu_interaction sztu_guide
source install/setup.bash
systemctl --user restart sztu-bridge
```

也可以单独运行服务验证：

```bash
ros2 run sztu_interaction wifi_provisioning_server
```