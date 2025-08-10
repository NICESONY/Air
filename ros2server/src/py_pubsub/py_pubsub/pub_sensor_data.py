import re, time, serial
import rclpy
from rclpy.node import Node

# 패키지 이름이 'msg'라서 import 경로가 이렇게 됩니다.
from msgs.msg import PubAirSensor



'''

# 1 보내기 → 아두이노는 "VENT_START"를 받음
ros2 topic pub -1 /vent_cmd std_msgs/Int32 "data: 1"

# 0 보내기 → "VENT_STOP"
ros2 topic pub -1 /vent_cmd std_msgs/Int32 "data: 0"


'''



class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('topic', 'gas_sensor')
        self.declare_parameter('timeout', 0.1)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        topic = self.get_parameter('topic').value
        timeout = float(self.get_parameter('timeout').value)



        self.pub = self.create_publisher(PubAirSensor, topic, 10)
        self.pattern = re.compile(r"LPG:\s*(\d+)\s*ppm\s*CO:\s*(\d+)\s*ppm\s*SMOKE:\s*(\d+)\s*ppm")

        self.get_logger().info(f"Opening serial {port} @ {baud}")
        self.ser = serial.Serial(port, baud, timeout=timeout)

        self.timer = self.create_timer(1.0, self.read_once)  # 1HZ



    def read_once(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        msg = PubAirSensor()
        msg.raw = line
        msg.timestamp = float(time.time())

        m = self.pattern.search(line)
        if m:
            msg.lpg, msg.co, msg.smoke = map(int, m.groups())
        else:
            msg.lpg = msg.co = msg.smoke = -1  # 파싱 실패 표시

        self.pub.publish(msg)
        # self.get_logger().debug(f"pub: {msg}")




def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
