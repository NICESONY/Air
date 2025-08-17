#!/usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32




'''

# 1 보내기 → 아두이노는 "VENT_START"를 받음
ros2 topic pub -1 /vent_cmd std_msgs/Int32 "data: 1"

# 0 보내기 → "VENT_STOP"
ros2 topic pub -1 /vent_cmd std_msgs/Int32 "data: 0"


'''


class VentCmdToSerial(Node):
    def __init__(self):
        super().__init__('vent_cmd_to_serial')

        # 아두이노에 맞춰 기본값 9600 / 문자열 명령
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('on_text',  'VENT_START')  # 줄바꿈은 자동으로 붙일게요
        self.declare_parameter('off_text', 'VENT_STOP')

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.on_text  = self.get_parameter('on_text').value
        self.off_text = self.get_parameter('off_text').value

        self.ser = None
        self.last_sent = None

        self.sub = self.create_subscription(Int32, 'vent_cmd', self._on_cmd, 10)
        self.timer = self.create_timer(1.0, self._ensure_serial)

        self.get_logger().info(f'Open {self.port} @ {self.baud} (1→{self.on_text}, 0→{self.off_text})')

    def _ensure_serial(self):
        if self.ser and self.ser.is_open:
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
            time.sleep(2.0)  # 아두이노 리셋 대기
            self.get_logger().info('Serial opened')
        except Exception as e:
            self.get_logger().warn(f'Serial open failed: {e}')

    def _send_line(self, text: str):
        if not (self.ser and self.ser.is_open):
            self.get_logger().warn('Serial not ready')
            return
        try:
            payload = (text + '\n').encode('ascii', errors='ignore')
            self.ser.write(payload)
            self.ser.flush()
            self.get_logger().info(f'Sent: {text}')
        except Exception as e:
            self.get_logger().warn(f'Write failed: {e}')
            try:
                self.ser.close()
            except Exception:
                pass

    def _on_cmd(self, msg: Int32):
        v = int(msg.data)
        if v == self.last_sent:
            return  # 같은 값 반복 전송 방지 (원하면 지우세요)
        self.last_sent = v

        if v == 1:
            self._send_line(self.on_text)   # 1 → "VENT_START\n"
        else:
            self._send_line(self.off_text)  # 0 또는 그 외 → "VENT_STOP\n"

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = VentCmdToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
