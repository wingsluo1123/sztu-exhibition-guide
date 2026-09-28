#!/usr/bin/env python3
r"""
Northbound bridge server

Bridges the HarmonyOS northbound client (Robot_APP) to the ROS 2 side.

The client (see Robot_APP/entry/src/main/ets/pages/*) connects over TCP
(default port 8888, UTF-8 lines terminated by \r\n) or WebSocket, and sends
line-based string commands. This node hosts the server, parses those commands,
and dispatches them to the existing controllers:

  - JOY / STOP           -> AimDK McLocomotionVelocity (50 Hz, fast path)
  - STAND / WALK / HOIST -> AimDK SetMcAction motion-mode switch (slow path)
  - expression labels     -> EmotionController.show_emotion
  - gesture/action labels -> GestureController.perform_gesture
  - hand-turn labels      -> GestureController.turn_hand (wrist roll)
  - SAY:<text>            -> TtsController.speak

Design notes:
  - 命令文本怎么解析、中文标签往哪路由，全在 sztu_guide.protocol 里，那边不
    依赖 ROS，可以单测。本文件只负责 socket 收发和把解析结果落到控制器上。
  - Fast commands (JOY/STOP) are handled directly on the socket thread because
    updating the target velocity is lock-protected and must stay low-latency;
    the actual McLocomotionVelocity goes out on the 50 Hz timer.
  - 其余命令（表情/手势/TTS/模式切换/MODE 查询）一律压进线程安全队列，由主
    ROS 线程的定时器回调执行。这样阻塞的 TTS 不会拖住遥控，而且所有 service
    调用都留在单线程上 —— MODE 查询曾经是在 socket 线程上直接调 service 的，
    违反了这条约定，现在也走队列。
  - The server replies "OK:<cmd>" / "ERR:<cmd>:<reason>" so the client log
    (which prints "服务端:") shows an acknowledgement.
"""

import queue
import socket
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sztu_interaction.tts_controller import TtsController
from sztu_interaction.emotion_controller import EmotionController
from sztu_interaction.gesture_controller import GestureController
from sztu_interaction.locomotion_controller import LocomotionController
from sztu_interaction.motion_mode_controller import MotionModeController

from . import protocol
from .protocol import (
    EXPRESSION_LABELS,
    GESTURE_LABELS,
    HAND_TURN_LABELS,
    MOTION_MODE_LABELS,
    Kind,
)

# 标签表搬到 protocol.py 了，这里重新导出，保持 from bridge_server import ... 可用。
__all__ = [
    'BridgeServer', 'main',
    'EXPRESSION_LABELS', 'GESTURE_LABELS', 'HAND_TURN_LABELS', 'MOTION_MODE_LABELS',
]


class BridgeServer(Node):
    """Hosts the northbound TCP/WebSocket server and dispatches commands."""

    def __init__(self):
        super().__init__('bridge_server')

        # Parameters
        self.declare_parameter('tcp_port', 8888)
        self.declare_parameter('ws_enabled', False)
        self.declare_parameter('ws_port', 8080)
        self.declare_parameter('cmd_vel_topic', '/aima/mc/locomotion/velocity')
        self.declare_parameter('locomotion_source', 'sdk_sztu_app')
        self.declare_parameter('locomotion_priority', 55)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.8)

        self.tcp_port = self.get_parameter('tcp_port').value
        self.ws_enabled = self.get_parameter('ws_enabled').value
        self.ws_port = self.get_parameter('ws_port').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        locomotion_source = self.get_parameter('locomotion_source').value
        locomotion_priority = self.get_parameter('locomotion_priority').value

        self.get_logger().info('=' * 60)
        self.get_logger().info('   Northbound Bridge Server')
        self.get_logger().info('=' * 60)

        # AimDK 走跑控制：使用 McLocomotionVelocity 和已注册输入源。
        self.locomotion = LocomotionController(
            topic=cmd_vel_topic,
            source=locomotion_source,
            priority=int(locomotion_priority),
        )
        # Registration result was previously discarded. If it fails, every
        # teleop command is silently dropped by MC and the operator sees a
        # robot that simply ignores the joystick -- worth a log line.
        if not self.locomotion.register_input_source():
            self.get_logger().error(
                'Locomotion input source registration FAILED -- MC will '
                'discard all velocity commands. Teleop will not move the robot.')
        elif not self.locomotion.registration_confirmed:
            self.get_logger().warn(
                'Locomotion input source not confirmed by MC (empty response). '
                'Teleop is enabled anyway; if the robot does not move, check '
                'the MC log for "Input Source is not enabled" first.')

        # Interaction controllers (in-process; do NOT also start the standalone
        # tts/emotion/gesture nodes, or node names will collide).
        self.get_logger().info('Initializing interaction controllers...')
        self.tts = TtsController()
        self.emotion = EmotionController()
        self.gesture = GestureController()
        self.motion_mode = MotionModeController()

        # Command queue for slow commands, drained on the main thread.
        self.cmd_queue: 'queue.Queue[tuple]' = queue.Queue()
        self.create_timer(0.05, self.drain_queue)  # 20 Hz
        self.create_timer(0.02, self.locomotion.tick)  # AimDK 推荐 50 Hz

        # Track connected client writers so we can push acks back.
        # NOTE: keep the "_connections" name. Naming this "_clients" would shadow
        # rclpy Node._clients (the service-client list) and crash the executor
        # with "'socket' object has no attribute 'handle'" once a peer connects.
        self._connections_lock = threading.Lock()
        self._connections = set()

        # Start servers in background daemon threads.
        self._stop = threading.Event()
        threading.Thread(target=self._run_tcp_server, daemon=True).start()
        if self.ws_enabled:
            threading.Thread(target=self._run_ws_server, daemon=True).start()

        self.get_logger().info('Bridge server ready')
        self.get_logger().info('=' * 60)

    # --- TCP server ---------------------------------------------------------
    def _run_tcp_server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind(('0.0.0.0', self.tcp_port))
            srv.listen(4)
        except OSError as e:
            self.get_logger().error(f'TCP bind failed on port {self.tcp_port}: {e}')
            return
        self.get_logger().info(f'TCP server listening on 0.0.0.0:{self.tcp_port}')

        while not self._stop.is_set():
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            self.get_logger().info(f'Client connected: {addr[0]}:{addr[1]}')
            threading.Thread(
                target=self._handle_tcp_client, args=(conn, addr), daemon=True
            ).start()

    def _handle_tcp_client(self, conn: socket.socket, addr):
        conn.settimeout(None)
        with self._connections_lock:
            self._connections.add(conn)
        buffer = b''
        try:
            while not self._stop.is_set():
                data = conn.recv(4096)
                if not data:
                    break
                buffer += data
                lines, buffer = protocol.split_lines(buffer)
                for text in lines:
                    self._on_command(text, conn)
        except OSError:
            pass
        finally:
            with self._connections_lock:
                self._connections.discard(conn)
            try:
                conn.close()
            except OSError:
                pass
            self.get_logger().info(f'Client disconnected: {addr[0]}:{addr[1]}')

    def _reply(self, conn, text: str):
        """Send a line back to a specific client (best effort)."""
        if conn is None:
            return
        try:
            conn.sendall((text + '\r\n').encode('utf-8'))
        except OSError:
            pass

    # --- WebSocket server (optional) ---------------------------------------
    def _run_ws_server(self):
        try:
            import asyncio
            import websockets
        except ImportError:
            self.get_logger().warn(
                'ws_enabled=true but the "websockets" package is not installed; '
                'WebSocket server disabled. Install with: pip install websockets'
            )
            return

        async def handler(ws):
            peer = getattr(ws, 'remote_address', ('?', 0))
            self.get_logger().info(f'WS client connected: {peer}')
            try:
                async for message in ws:
                    text = str(message).strip()
                    if text:
                        # WS acks are sent inline via a small shim.
                        self._on_command(text, _WsConnShim(ws, asyncio.get_event_loop()))
            except Exception:
                pass
            finally:
                self.get_logger().info(f'WS client disconnected: {peer}')

        async def main():
            async with websockets.serve(handler, '0.0.0.0', self.ws_port):
                self.get_logger().info(
                    f'WebSocket server listening on 0.0.0.0:{self.ws_port}')
                await asyncio.Future()  # run forever

        try:
            asyncio.run(main())
        except Exception as e:
            self.get_logger().error(f'WebSocket server error: {e}')

    # --- Command dispatch ---------------------------------------------------
    def _on_command(self, text: str, conn):
        """Route a single command line. Fast path runs here; slow path queues."""
        self.get_logger().info(f'CMD: {text}')

        cmd = protocol.parse(text)

        # 快路径：socket 线程只更新目标速度；控制器以 50 Hz 发布 AimDK 消息。
        if cmd.kind == Kind.STOP:
            self.locomotion.stop()
            self._reply(conn, 'OK:STOP')
            return

        if cmd.kind == Kind.JOY:
            if cmd.error:
                self._reply(conn, f'ERR:JOY:{cmd.error}')
                return
            forward, angular = protocol.joy_to_twist(
                cmd.joy_x, cmd.joy_y, self.max_linear, self.max_angular)
            if not self.locomotion.set_velocity(forward, 0.0, angular):
                self._reply(conn, 'ERR:JOY:input_source_unavailable')
            # 正常 JOY 不回 ack，避免高频刷屏。
            return

        # 其余全部进队列，在 ROS 主线程上执行（含 MODE 查询：它要调 service）
        self.cmd_queue.put((cmd, conn))

    def drain_queue(self):
        """Timer callback (main thread): execute slow commands."""
        while True:
            try:
                cmd, conn = self.cmd_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._dispatch_slow(cmd, conn)
            except Exception as e:  # never let one command kill the timer
                self.get_logger().error(f'Command failed "{cmd.raw}": {e}')
                self._reply(conn, f'ERR:{cmd.raw}:{e}')

    def _dispatch_slow(self, cmd, conn):
        """在 ROS 主线程上执行一条已解析的命令。"""
        if cmd.kind == Kind.MODE_QUERY:
            ok, mode, status = self.motion_mode.get_mode()
            self._reply(conn, f'OK:MODE:{mode}:{status}' if ok else 'ERR:MODE')
            return

        if cmd.kind == Kind.SAY:
            ok = self.tts.speak(cmd.value)
            self._reply(conn, f'{"OK" if ok else "ERR"}:SAY')
            return

        if cmd.kind == Kind.MODE_SWITCH:
            # 稳定站立和走跑都走安全过渡，禁止从零力矩/阻尼直接切入。
            if cmd.value == 'stand':
                ok = self.motion_mode.enter_stand_sequence()
            elif cmd.value == 'locomotion':
                ok = self.motion_mode.enter_locomotion_sequence()
            else:
                ok = self.motion_mode.set_mode(cmd.value)
            self._reply(conn, f'{"OK" if ok else "ERR"}:{cmd.raw}')
            return

        if cmd.kind == Kind.EMOTION:
            ok = self.emotion.show_emotion(cmd.value)
            self._reply(conn, f'{"OK" if ok else "ERR"}:{cmd.raw}')
            return

        if cmd.kind == Kind.GESTURE:
            ok = self.gesture.perform_gesture(cmd.value, interrupt=True)
            self._reply(conn, f'{"OK" if ok else "ERR"}:{cmd.raw}')
            return

        if cmd.kind == Kind.HAND_TURN:
            # 腕部插值是阻塞的，要好几秒 —— 只能在慢路径上做
            ok = self.gesture.turn_hand(cmd.value)
            self._reply(conn, f'{"OK" if ok else "ERR"}:{cmd.raw}')
            return

        # 认不出来的标签当预设动作名透传
        self.get_logger().warn(f'Unknown command, forwarding as gesture: {cmd.raw}')
        ok = self.gesture.perform_gesture(cmd.value or cmd.raw)
        self._reply(conn, f'{"OK" if ok else "ERR"}:{cmd.raw}')

    def shutdown(self):
        self._stop.set()
        self.locomotion.stop()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C / SIGTERM: rclpy raises ExternalShutdownException instead of
        # KeyboardInterrupt once the signal handler has shut the context down.
        pass
    finally:
        # When rclpy handled the signal itself the context is already down and
        # publishing the stop twist would raise RCLError, so guard on rclpy.ok().
        if rclpy.ok():
            node.shutdown()
            node.destroy_node()
            rclpy.shutdown()


class _WsConnShim:
    """Adapts a WebSocket to the ._reply(conn, text) interface used by TCP."""

    def __init__(self, ws, loop):
        self._ws = ws
        self._loop = loop

    def sendall(self, data: bytes):
        text = data.decode('utf-8', errors='replace')
        try:
            import asyncio
            asyncio.run_coroutine_threadsafe(self._ws.send(text), self._loop)
        except Exception:
            pass


if __name__ == '__main__':
    main()
