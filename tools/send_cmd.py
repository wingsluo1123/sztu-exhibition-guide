#!/usr/bin/env python3
"""从开发机命令行给机器人上位机发北向指令，并等回 ack。

为什么不直接用 nc
-----------------
`docs/APP_HANDOFF.md` 原来写的是 `nc <机器人IP> 8888` 手工发。实测这招会
**静默丢掉 ack**：`printf '引导右\\r\\n' | nc <ip> 8888` 里 printf 一结束
stdin 就 EOF，nc 立刻关连接；而手势、转手、模式切换都是慢路径，ack 要几秒
后才回，那时 socket 已经没了。表现是"命令发出去了，什么都没看见"，
很容易误判成服务端没响应。

2026-10-01 在仿真上复现过：同一条 `引导右`，用 printf 管道什么都收不到，
把连接按住 18 秒才看到 `ERR:引导右`。

所以这个脚本按住连接等 ack，并且把超时和"本来就没有 ack"区分开
（`JOY:` 是高频流式指令，协议规定不回 ack，见 docs/REMOTE_PROTOCOL.md）。

用法
----
    export SZTU_ROBOT_IP=192.168.43.x        # 机器人上位机，不是运控计算单元
    tools/send_cmd.py 挥手
    tools/send_cmd.py --host 192.168.43.x STAND 引导右
    tools/send_cmd.py STOP                   # 急停，单独发

一条连接按顺序发多条，和 App 的长连接行为一致。任何一条回 ERR，退出码非 0。

⚠ 这是**真机**。`挥手` / `引导右` 这类指令会让真实手臂动，`JOY:` 会让它走。
   发之前确认现场有人、周围没人没障碍。急停是 `STOP`。
"""

import argparse
import os
import socket
import sys

DEFAULT_PORT = 8888
DEFAULT_TIMEOUT = 20.0

# 协议规定不回 ack 的指令前缀（高频流式，回了会刷屏）。
# 见 docs/REMOTE_PROTOCOL.md：「`JOY:` 和 `STRAFE:` 为高频流式指令，不回复 ack」。
# 漏一个的后果是这个脚本等满超时然后谎报失败。
NO_ACK_PREFIXES = ('JOY:', 'STRAFE:')


def parse_args():
    parser = argparse.ArgumentParser(
        description='给机器人上位机发北向指令并等 ack',
        epilog='机器人 IP 用 --host 或环境变量 SZTU_ROBOT_IP 指定。')
    parser.add_argument('commands', nargs='+', metavar='CMD',
                        help='要发的指令，例如 挥手 / 引导右 / STAND / STOP')
    parser.add_argument('--host', default=os.environ.get('SZTU_ROBOT_IP'),
                        help='机器人上位机 IP（默认取 $SZTU_ROBOT_IP）')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--timeout', type=float, default=DEFAULT_TIMEOUT,
                        help=f'等单条 ack 的秒数，默认 {DEFAULT_TIMEOUT:.0f}。'
                             '引导手势约 6 s、转手约 4 s，别调太小')
    parser.add_argument('--no-wait', action='store_true',
                        help='发完就走，不等 ack（流式 JOY 用）')
    args = parser.parse_args()

    if not args.host:
        parser.error('没给机器人 IP：用 --host，或 export SZTU_ROBOT_IP=<ip>')
    return args


def recv_line(sock, buffer):
    """读一行 ack。返回 (行文本 or None, 剩余缓冲)。None 表示超时/对端关闭。"""
    while b'\n' not in buffer:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            return None, buffer
        if not chunk:
            return None, buffer
        buffer += chunk

    line, buffer = buffer.split(b'\n', 1)
    return line.decode('utf-8', errors='replace').strip(), buffer


def main():
    args = parse_args()

    print(f'→ {args.host}:{args.port}', file=sys.stderr)

    try:
        sock = socket.create_connection((args.host, args.port), timeout=10.0)
    except OSError as exc:
        print(f'✗ 连不上 {args.host}:{args.port} —— {exc}', file=sys.stderr)
        print('  上位机上的 bridge_server 起了吗？在机器人上查：'
              'ss -tlnp | grep 8888', file=sys.stderr)
        return 2

    sock.settimeout(args.timeout)
    buffer = b''
    failed = 0

    try:
        for command in args.commands:
            sock.sendall((command + '\r\n').encode('utf-8'))

            if args.no_wait or command.upper().startswith(NO_ACK_PREFIXES):
                reason = '协议规定不回 ack' if not args.no_wait else '--no-wait'
                print(f'{command}  → (不等 ack: {reason})')
                continue

            ack, buffer = recv_line(sock, buffer)
            if ack is None:
                print(f'{command}  → ✗ {args.timeout:.0f} s 内没有 ack'
                      f'（连接可能被对端关了，或动作比超时更久）')
                failed += 1
                continue

            print(f'{command}  → {ack}')
            if ack.startswith('ERR'):
                failed += 1
    finally:
        sock.close()

    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
