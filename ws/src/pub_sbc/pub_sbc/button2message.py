#!/usr/bin/env python3
import os, json, time, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from websocket import WebSocketApp  # pip install websocket-client

class WSCmdBridge(Node):
    def __init__(self, url: str):
        super().__init__('ws_cmd_bridge')
        self.url = url
        self.pub_cmd = self.create_publisher(Int32, 'vent_cmd', 10)
        self.pub_raw = self.create_publisher(String, 'vent_raw', 10)
        self._ws = None                 # ★ 추가: 현재 소켓 보관
        self._stop = False              # ★ 추가: 종료 플래그
        self.t = threading.Thread(target=self._run_ws, daemon=True)
        self.t.start()

    def _run_ws(self):
        while rclpy.ok() and not self._stop:
            try:
                self.get_logger().info(f"[WS] connecting {self.url}")
                # ★ 매 회차 새 WebSocketApp 생성 (재연결 안정성 ↑)
                self._ws = WebSocketApp(
                    self.url,
                    on_open=self._on_open,
                    on_message=self._on_msg,
                    on_close=self._on_close,
                    on_error=self._on_err
                )
                self._ws.run_forever(ping_interval=20, ping_timeout=10)
            except Exception as e:
                self.get_logger().warning(f"[WS] error: {e}")
            time.sleep(1.0)

    def _on_open(self, ws):
        self.get_logger().info("[WS] opened")

    def _on_close(self, *a, **k):
        self.get_logger().info("[WS] closed")

    def _on_err(self, ws, err):
        self.get_logger().warning(f"[WS] err: {err}")

    def _on_msg(self, ws, msg: str):
        self.pub_raw.publish(String(data=msg))
        try:
            obj = json.loads(msg)
            if isinstance(obj, dict) and obj.get('type') == 'vent' and 'value' in obj:
                v = int(obj['value'])
                self.get_logger().info(f"[VENT] received value={v}")
                self.pub_cmd.publish(Int32(data=v))
        except Exception as e:
            self.get_logger().debug(f"JSON parse ignore: {e}")

    def destroy_node(self):
        # ★ 종료 시 소켓 닫아서 run_forever 빠지게
        self._stop = True
        try:
            if self._ws:
                self._ws.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    url = os.environ.get('WS_URL', 'ws://127.0.0.1:8765/')
    node = WSCmdBridge(url)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
