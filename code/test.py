# test.py

import threading
import time
import asyncio
import re
import serial
import pymysql
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from fastapi.staticfiles import StaticFiles

app = FastAPI()

# ─── 시리얼 포트 설정 ──────────────────────────────────────────
# Windows 환경에서 COM 포트 이름을 실제 사용 중인 포트로 수정하세요.
# 예: SERIAL_PORT = r"\\.\COM3"
SERIAL_PORT = "COM3"
BAUD_RATE    = 9600
# ────────────────────────────────────────────────────────────────

# ─── MySQL 연결 설정 ───────────────────────────────────────────
# 'your_user', 'your_password', 'sensor_db' 등을 실제 환경에 맞게 수정하세요.
conn = pymysql.connect(
    host='127.0.0.1',
    port=3306,
    user='root',
    password='111111',
    db='sensor_db',
    charset='utf8mb4',
    cursorclass=pymysql.cursors.DictCursor,
    autocommit=True
)
cursor = conn.cursor()
# ────────────────────────────────────────────────────────────────

# 최신 센서값 저장용 전역 변수
latest_reading = {
    "lpg": None,
    "co": None,
    "smoke": None,
    "raw": None,
    "timestamp": None,
}

# WebSocket 연결 관리 클래스
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.active_connections.append(ws)

    def disconnect(self, ws: WebSocket):
        self.active_connections.remove(ws)

    async def broadcast(self, message: dict):
        for conn in self.active_connections:
            await conn.send_json(message)

manager = ConnectionManager()

# Pydantic 모델 정의
class SensorReading(BaseModel):
    lpg: int
    co: int
    smoke: int
    raw: str
    timestamp: float

# 시리얼에서 데이터 읽어와 파싱하고 MySQL에 저장, WebSocket 브로드캐스트
def read_serial():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    pattern = re.compile(r"LPG:\s*(\d+)\s*ppm\s*CO:\s*(\d+)\s*ppm\s*SMOKE:\s*(\d+)\s*ppm")
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue

        
        print(f"[Serial raw] {line}")
        timestamp = time.time()
        match = pattern.search(line)
        if match:
            lpg, co, smoke = map(int, match.groups())
            latest_reading.update({
                "lpg": lpg,
                "co": co,
                "smoke": smoke,
                "raw": line,
                "timestamp": timestamp
            })

            # MySQL에 INSERT
            sql = """
                INSERT INTO sensor (lpg, co, smoke, raw, timestamp)
                VALUES (%s, %s, %s, %s, %s)
            """
            cursor.execute(sql, (lpg, co, smoke, line, timestamp))
            # autocommit=True 이므로 별도 commit() 필요 없음

            # WebSocket 브로드캐스트
            asyncio.run(manager.broadcast(latest_reading))

        else:
            # 파싱 실패 시 raw와 timestamp만 업데이트
            latest_reading.update({
                "raw": line,
                "timestamp": timestamp
            })

# 백그라운드 스레드로 시리얼 읽기 시작
threading.Thread(target=read_serial, daemon=True).start()

# ─── REST API: 최신 센서값 조회 ─────────────────────────────────
@app.get("/sensor", response_model=SensorReading)
async def get_sensor():
    if latest_reading["lpg"] is None:
        return {"lpg": 0, "co": 0, "smoke": 0, "raw": "", "timestamp": time.time()}
    return latest_reading

# ─── REST API: 저장된 히스토리 조회 ───────────────────────────────
@app.get("/history")
async def get_history(limit: int = 100):
    cursor.execute(
        "SELECT lpg, co, smoke, raw, timestamp FROM sensor ORDER BY id DESC LIMIT %s",
        (limit,)
    )
    return cursor.fetchall()

# ─── WebSocket: 실시간 센서 스트리밍 ───────────────────────────────
@app.websocket("/ws/sensor")
async def websocket_endpoint(ws: WebSocket):
    await manager.connect(ws)
    try:
        while True:
            # 클라이언트 ping/pong 또는 단순 대기
            await ws.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(ws)


# db에 저장된 데이터 화면에 띄우기

app.mount("/", StaticFiles(directory="static", html=True), name="static")