#!/usr/bin/env python3
import json, threading, time, os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from websocket import WebSocketApp

class WSToROS(Node):
    def __init__(self, url):
        super().__init__('ws_to_ros')
        self.pub_int = self.create_publisher(Int32, 'vent_cmd', 10)
        self.pub_raw = self.create_publisher(String, 'vent_raw', 10)
        self.url = url
        self.ws = WebSocketApp(url, on_message=self.on_message, on_close=self.on_close)
        self.t = threading.Thread(target=self._run_ws, daemon=True)
        self.t.start()

    def _run_ws(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f'Connecting {self.url}')
                self.ws.run_forever()
            except Exception as e:
                self.get_logger().warn(f'WS error: {e}')
            time.sleep(1.0)

    def on_message(self, ws, msg):
        self.pub_raw.publish(String(data=msg))
        try:
            obj = json.loads(msg)
            if isinstance(obj, dict) and 'value' in obj:
                self.pub_int.publish(Int32(data=int(obj['value'])))
        except Exception:
            pass

    def on_close(self, *args, **kwargs):
        self.get_logger().info('WS closed')

def main():
    rclpy.init()
    url = os.environ.get('WS_URL', 'ws://127.0.0.1:8765/ws')
    node = WSToROS(url)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
