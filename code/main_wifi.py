# main.py
import threading
import asyncio
import logging
import pymysql
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

# ★ 실제 생성된 메시지 타입으로 바꾸세요:
from msgs.msg import PubAirSensor  


logging.basicConfig(level=logging.INFO)
log = logging.getLogger("test")

app = FastAPI()

# MySQL 연결
conn = pymysql.connect(
    host='127.0.0.1', port=3306,
    user='sensor_user', password='111111',
    db='sensor_db', charset='utf8mb4',
    cursorclass=pymysql.cursors.DictCursor,
    autocommit=True
)

latest_reading = {"lpg": None, "co": None, "smoke": None, "raw": None, "timestamp": None}

class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []
    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.active_connections.append(ws)
    def disconnect(self, ws: WebSocket):
        if ws in self.active_connections:
            self.active_connections.remove(ws)
    async def broadcast(self, message: dict):
        for conn_ws in list(self.active_connections):
            try:
                await conn_ws.send_json(message)
            except Exception:
                self.disconnect(conn_ws)

manager = ConnectionManager()

class SensorReading(BaseModel):
    lpg: int
    co: int
    smoke: int
    raw: str
    timestamp: float

# === ROS 구독 노드 ===
class RosBridge(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        super().__init__('fastapi_ros_bridge')
        self.loop = loop
        # 구독할 토픽 이름(원하면 파라미터로 바꿔도 됨)
        topic = '/gas_sensor'
        # ★ 메시지 타입을 실제 것(GasSensor 또는 PubAirSensor)로 맞추세요
        self.sub = self.create_subscription(GasSensor, topic, self.cb, 10)

    def cb(self, msg):
        # ROS 콜백 스레드에서 실행됨
        data = {
            "lpg": int(msg.lpg),
            "co": int(msg.co),
            "smoke": int(msg.smoke),
            "raw": msg.raw,
            "timestamp": float(msg.timestamp),
        }
        latest_reading.update(data)

        # DB insert (요청마다 커서 생성 권장)
        try:
            with conn.cursor() as cur:
                cur.execute(
                    "INSERT INTO sensor (lpg, co, smoke, raw, timestamp) VALUES (%s, %s, %s, %s, %s)",
                    (data["lpg"], data["co"], data["smoke"], data["raw"], data["timestamp"])
                )
        except Exception as e:
            log.exception("DB insert failed: %s", e)

        # WebSocket 브로드캐스트는 메인 asyncio 루프에 투입
        try:
            asyncio.run_coroutine_threadsafe(
                manager.broadcast(dict(data)),  # copy
                self.loop
            )
        except Exception as e:
            self.get_logger().warn(f"broadcast schedule failed: {e}")

# 전역(ROS용)
_ros_executor: SingleThreadedExecutor | None = None
_ros_thread: threading.Thread | None = None
_ros_node: RosBridge | None = None
_main_loop: asyncio.AbstractEventLoop | None = None

# FastAPI startup: rclpy 초기화 + 구독 시작
@app.on_event("startup")
def on_startup():
    global _ros_executor, _ros_thread, _ros_node, _main_loop
    log.info("▶ FastAPI startup: init ROS2 subscriber")
    _main_loop = asyncio.get_event_loop()
    rclpy.init()
    _ros_node = RosBridge(loop=_main_loop)
    _ros_executor = SingleThreadedExecutor()
    _ros_executor.add_node(_ros_node)
    _ros_thread = threading.Thread(target=_ros_executor.spin, daemon=True)
    _ros_thread.start()

# FastAPI shutdown: rclpy 정리
@app.on_event("shutdown")
def on_shutdown():
    global _ros_executor, _ros_thread, _ros_node
    log.info("▶ FastAPI shutdown: stop ROS2")
    try:
        if _ros_executor:
            _ros_executor.shutdown()
    except Exception:
        pass
    try:
        if _ros_node:
            _ros_node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass

# (선택) 환기 명령: 이제는 ROS로도 보낼 수 있어요.
# 원하면 여기서 std_msgs/String 퍼블리셔를 만들어 /ventilate_cmd 토픽에 'VENT_START' 전송하도록 바꿀 수 있음.
# 지금은 REST 엔드포인트만 남겨두고 내부 동작은 추후 ROS publish로 교체 가능.
@app.post("/ventilate")
async def ventilate():
    # TODO: ROS 퍼블리셔로 변경 가능 (예: /ventilate_cmd)
    raise HTTPException(status_code=501, detail="Replace with ROS publisher if needed")

@app.post("/ventilate/stop")
async def ventilate_stop():
    # TODO: ROS 퍼블리셔로 변경 가능
    raise HTTPException(status_code=501, detail="Replace with ROS publisher if needed")

@app.get("/sensor", response_model=SensorReading)
async def get_sensor():
    if latest_reading["lpg"] is None:
        return {"lpg": 0, "co": 0, "smoke": 0, "raw": "", "timestamp": 0.0}
    return latest_reading

@app.get("/history")
async def get_history(limit: int = 100):
    with conn.cursor() as cur:
        cur.execute("SELECT lpg, co, smoke, raw, timestamp FROM sensor ORDER BY id DESC LIMIT %s", (limit,))
        return cur.fetchall()

@app.websocket("/ws/sensor")
async def websocket_endpoint(ws: WebSocket):
    await manager.connect(ws)
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(ws)

app.mount("/", StaticFiles(directory="static", html=True), name="static")
