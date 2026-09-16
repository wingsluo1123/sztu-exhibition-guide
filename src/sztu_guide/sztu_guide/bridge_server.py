#!/usr/bin/env python3
"""
Northbound bridge server

Bridges the HarmonyOS northbound client (Robot_APP) to the ROS 2 side.

The client (see Robot_APP/entry/src/main/ets/pages/*) connects over TCP
(default port 8888, UTF-8 lines terminated by \\r\\n) or WebSocket, and sends
line-based string commands. This node hosts the server, parses those commands,
and dispatches them to the existing controllers:

  - JOY / STOP           -> Twist on /aima/mc/locomotion/velocity (fast path)
  - STAND / WALK / RUN    -> posture / gait command (placeholder, see TODO)
  - expression labels     -> EmotionController.show_emotion
  - gesture/action labels -> GestureController.perform_gesture
  - hand-turn labels      -> GestureController.turn_hand (wrist roll)
  - SAY:<text>            -> TtsController.speak

Design notes:
  - Fast commands (JOY/STOP) are handled directly on the socket thread because
    publishing a Twist is thread-safe and must stay low-latency.
  - Slow commands (expression/gesture/TTS) are pushed onto a thread-safe queue
    and executed on the main ROS thread inside a timer callback, so a blocking
    TTS call never stalls teleop and all service calls stay single-threaded.
  - The server replies "OK:<cmd>" / "ERR:<cmd>:<reason>" so the client log
    (which prints "服务端:") shows an acknowledgement.
"""

import os
import sys
import queue
import socket
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Ensure sibling packages are importable when run from the install space.
sys.path.append(os.path.expanduser('~/sztu/install/sztu_interaction/lib/python3.10/site-packages'))

from sztu_interaction.tts_controller import TtsController
from sztu_interaction.emotion_controller import EmotionController
from sztu_interaction.gesture_controller import GestureController


# --- Command label maps -----------------------------------------------------
# The client sends UI-facing Chinese labels with no type prefix, so the bridge
# decides routing by set membership. Adjust these to match the real AimDK
# preset-motion / emoji catalogue as it is confirmed on the robot.

# Expression labels (from Index.ets `expressions`) -> AimDK emoji semantic name.
# Unknown labels fall through to EmotionController, which passes them straight
# to PlayEmoji as emoji_name.
EXPRESSION_LABELS = {
    '眨眼': 'blink',
    '平静1': 'neutral',
    '平静2': 'neutral',
    '游戏': 'happy',
    '可爱1': 'happy',
    '可爱2': 'happy',
    '可爱3': 'happy',
    '可爱4': 'happy',
    '闭眼': 'blink',
    '睁眼': 'neutral',
    '无聊': 'neutral',
    '告警': 'surprised',
    '睡眠': 'neutral',
    '开心': 'happy',
    '开心+': 'happy',
    '开心++': 'happy',
    '伤心': 'sad',
    '同情': 'sad',
}

# Gesture / action labels (from Index.ets gesture/lively/upperLimb/head lists).
# Value is the gesture name handed to GestureController.perform_gesture; the
# controller maps known names to AimDK preset motions and passes through others.
GESTURE_LABELS = {
    # gestureActions
    '拍手': 'clap',
    '左手点赞': 'thumbs_up_left',
    '右手点赞': 'thumbs_up_right',
    '双手点赞': 'thumbs_up_both',
    '左手比耶': 'yeah_left',
    '右手比耶': 'yeah_right',
    '作揖': 'bow_salute',
    '左手碰拳': 'fist_bump_left',
    '右手碰拳': 'fist_bump_right',
    # headActions
    '点头': 'nod',
    '摇头': 'shake_head',
    '低头': 'lower_head',
    # commonly used upper-limb / lively actions
    '左手挥手': 'wave_left',
    '右手挥手': 'wave_right',
    '敬礼': 'salute',
    '拥抱': 'hug',
    '思考': 'thinking',
}

# Hand-turn labels -> which hand GestureController.turn_hand rotates. These do
# not use the preset-motion service: the preset catalogue has no wrist-roll
# entry, so the wrist joint is interpolated on /aima/hal/joint/arm/command.
HAND_TURN_LABELS = {
    '转手': 'right',
    '左手转手': 'left',
    '右手转手': 'right',
    '双手转手': 'both',
}


class BridgeServer(Node):
    """Hosts the northbound TCP/WebSocket server and dispatches commands."""

    def __init__(self):
        super().__init__('bridge_server')

        # Parameters
        self.declare_parameter('tcp_port', 8888)
        self.declare_parameter('ws_enabled', False)
        self.declare_parameter('ws_port', 8080)
        self.declare_parameter('cmd_vel_topic', '/aima/mc/locomotion/velocity')
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.8)

        self.tcp_port = self.get_parameter('tcp_port').value
        self.ws_enabled = self.get_parameter('ws_enabled').value
        self.ws_port = self.get_parameter('ws_port').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.get_logger().info('=' * 60)
        self.get_logger().info('   Northbound Bridge Server')
        self.get_logger().info('=' * 60)

        # Teleop publisher (fast path)
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Interaction controllers (in-process; do NOT also start the standalone
        # tts/emotion/gesture nodes, or node names will collide).
        self.get_logger().info('Initializing interaction controllers...')
        self.tts = TtsController()
        self.emotion = EmotionController()
        self.gesture = GestureController()

        # Command queue for slow commands, drained on the main thread.
        self.cmd_queue: 'queue.Queue[tuple]' = queue.Queue()
        self.create_timer(0.05, self.drain_queue)  # 20 Hz

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
                # Client frames messages with \r\n; also tolerate bare \n.
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    text = line.decode('utf-8', errors='replace').strip()
                    if text:
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

        upper = text.upper()

        # Fast path: teleop (publish directly, thread-safe, low latency)
        if upper == 'STOP':
            self._publish_twist(0.0, 0.0)
            self._reply(conn, 'OK:STOP')
            return

        if upper.startswith('JOY:'):
            self._handle_joy(text[4:], conn)
            return

        if upper in ('STAND', 'WALK', 'RUN'):
            # TODO: map to real AimDK posture/gait service when available.
            self.get_logger().info(f'Posture/gait command (placeholder): {upper}')
            self._reply(conn, f'OK:{upper}')
            return

        # Slow path: enqueue for main-thread dispatch.
        self.cmd_queue.put((text, conn))

    def _handle_joy(self, payload: str, conn):
        try:
            parts = payload.split(',')
            x = float(parts[0])
            y = float(parts[1])
        except (ValueError, IndexError):
            self._reply(conn, f'ERR:JOY:bad_payload')
            return
        # Client sends x,y in -100..100. y is forward/back, x is turn.
        linear = max(-1.0, min(1.0, y / 100.0)) * self.max_linear
        angular = -max(-1.0, min(1.0, x / 100.0)) * self.max_angular
        self._publish_twist(linear, angular)
        # No ack on JOY: it streams at high rate and would flood the log.

    def _publish_twist(self, linear: float, angular: float):
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd)

    def drain_queue(self):
        """Timer callback (main thread): execute slow commands."""
        while True:
            try:
                text, conn = self.cmd_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._dispatch_slow(text, conn)
            except Exception as e:  # never let one command kill the timer
                self.get_logger().error(f'Command failed "{text}": {e}')
                self._reply(conn, f'ERR:{text}:{e}')

    def _dispatch_slow(self, text: str, conn):
        # SAY:<text> -> TTS
        if text.upper().startswith('SAY:'):
            spoken = text[4:]
            ok = self.tts.speak(spoken)
            self._reply(conn, f'{"OK" if ok else "ERR"}:SAY')
            return

        # Expression label -> emotion controller
        if text in EXPRESSION_LABELS:
            ok = self.emotion.show_emotion(EXPRESSION_LABELS[text])
            self._reply(conn, f'{"OK" if ok else "ERR"}:{text}')
            return

        # Gesture / action label -> gesture controller
        if text in GESTURE_LABELS:
            ok = self.gesture.perform_gesture(GESTURE_LABELS[text])
            self._reply(conn, f'{"OK" if ok else "ERR"}:{text}')
            return

        # Hand-turn label -> wrist rotation (blocking, a couple of seconds)
        if text in HAND_TURN_LABELS:
            ok = self.gesture.turn_hand(HAND_TURN_LABELS[text])
            self._reply(conn, f'{"OK" if ok else "ERR"}:{text}')
            return

        # Unknown label: try it as a raw gesture (preset motion passthrough).
        self.get_logger().warn(f'Unknown command, forwarding as gesture: {text}')
        ok = self.gesture.perform_gesture(text)
        self._reply(conn, f'{"OK" if ok else "ERR"}:{text}')

    def shutdown(self):
        self._stop.set()
        self._publish_twist(0.0, 0.0)


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
