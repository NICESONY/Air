# web_bridge.py — PubAirSensor → WebSocket broadcast (websockets >= 12)
import asyncio
import threading
import json

import rclpy
from rclpy.node import Node
from msgs.msg import PubAirSensor

import websockets

# 상단 임포트에 추가
import requests  # ← 추가



'''

debugging: 웹소켓 클라이언트로 접속해 메시지 수신 확인
# 웹소켓 클라이언트 예시

python3 -m pip install websockets

python3 - << 'PY'
import asyncio, websockets
async def main():
    async with websockets.connect("ws://localhost:8765/") as ws:
        print("connected")
        while True: print(await ws.recv())
asyncio.run(main())
PY

'''


class SensorWSBridge(Node):
    __slots__ = (
        'topic', 'ws_host', 'ws_port', 'ws_path',
        'sub', 'ws_clients', 'loop', 'loop_thread', 'ws_server',
        'server_url'
    )

    def __init__(self):
        super().__init__('sensor_ws_bridge')

        # params
        self.declare_parameter('server_url', 'http://127.0.0.1:8000/ingest')  # DB 수집(옵션)
        self.declare_parameter('topic', '/gas_sensor')
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8765)
        self.declare_parameter('ws_path', '/')

        self.server_url = self.get_parameter('server_url').value
        self.topic   = self.get_parameter('topic').value
        self.ws_host = self.get_parameter('ws_host').value
        self.ws_port = int(self.get_parameter('ws_port').value)
        self.ws_path = self.get_parameter('ws_path').value

        # ROS sub (센서 → 웹으로 브로드캐스트)
        self.sub = self.create_subscription(PubAirSensor, self.topic, self._cb_sensor, 10)

        # WS server(loop in separate thread)
        self.ws_clients = set()
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.loop_thread.start()
        self._start_ws_server()

        self.get_logger().info(f"[WS] listening on ws://{self.ws_host}:{self.ws_port}{self.ws_path}")

    # ---------- WS loop ----------
    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    @staticmethod
    def _norm(p: str) -> str:
        if not p: return '/'
        p = p.strip()
        if not p.startswith('/'): p = '/' + p
        if p != '/' and p.endswith('/'): p = p[:-1]
        return p

    def _start_ws_server(self):
        async def handler(websocket):
            req_path = getattr(websocket, "path", "/")
            if self.ws_path not in ("", None) and self._norm(req_path) != self._norm(self.ws_path):
                await websocket.close(code=1008, reason="invalid path")
                return

            self.ws_clients.add(websocket)
            self.get_logger().info(f"[WS] connected ({len(self.ws_clients)} clients)")
            try:
                # ★ 클라이언트에서 오는 모든 프레임을 그대로 팬아웃(버튼 등)
                async for text in websocket:
                    self.get_logger().info(f"[WS] RX from client: {text}")
                    await self._broadcast(text)
            finally:
                self.ws_clients.discard(websocket)
                self.get_logger().info(f"[WS] disconnected ({len(self.ws_clients)} clients)")

        async def start():
            return await websockets.serve(
                handler, self.ws_host, self.ws_port,
                ping_interval=20, ping_timeout=20
            )
        fut = asyncio.run_coroutine_threadsafe(start(), self.loop)
        self.ws_server = fut.result()

    async def _broadcast(self, text: str):
        dead = []
        for ws in list(self.ws_clients):
            try:
                await ws.send(text)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.ws_clients.discard(ws)

    # ---------- 센서 콜백: 브로드캐스트 + DB POST(옵션) ----------
    def _cb_sensor(self, msg: PubAirSensor):
        # WS로 보낼 페이로드(프론트/구독자용) — type 추가
        ws_payload = {
            "type": "sensor",
            "lpg": int(msg.lpg),
            "co": int(msg.co),
            "smoke": int(msg.smoke),
            "raw": msg.raw,
            "timestamp": float(msg.timestamp),
        }
        asyncio.run_coroutine_threadsafe(self._broadcast(json.dumps(ws_payload)), self.loop)

        # DB로 보낼 페이로드(기존 스키마: type 제외)
        if self.server_url:
            try:
                ingest_payload = {
                    "lpg": int(msg.lpg),
                    "co": int(msg.co),
                    "smoke": int(msg.smoke),
                    "raw": msg.raw,
                    "timestamp": float(msg.timestamp),
                }
                requests.post(self.server_url, json=ingest_payload, timeout=0.5)
            except Exception as e:
                self.get_logger().warn(f"/ingest POST failed: {e}")

    # ---------- 종료 ----------
    async def _shutdown_ws(self):
        try:
            if getattr(self, 'ws_server', None):
                self.ws_server.close()
                await self.ws_server.wait_closed()
        except Exception:
            pass

    def destroy_node(self):
        try:
            if self.loop.is_running():
                asyncio.run_coroutine_threadsafe(self._shutdown_ws(), self.loop).result(timeout=2)
                self.loop.call_soon_threadsafe(self.loop.stop)
                self.loop_thread.join(timeout=2)
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorWSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass

if __name__ == "__main__":
    main()