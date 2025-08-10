# main.py

import threading
import time
import asyncio
import re
import serial
import pymysql
import logging
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from pydantic import BaseModel
from fastapi.staticfiles import StaticFiles




"""
uvicorn main:app --reload --host 0.0.0.0 --port 8000

"""


#  로깅 설정 
logging.basicConfig(level=logging.INFO)
log = logging.getLogger("test")




app = FastAPI()



#  시리얼 포트 설정 
# SERIAL_PORT = "COM3"  # -> window
SERIAL_PORT = "/dev/ttyACM0"   # ← 리눅스용
BAUD_RATE   = 9600
ser = None  # startup 이벤트에서 초기화합니다.



#  MySQL 연결 설정 
# conn = pymysql.connect(
#     host='127.0.0.1',
#     port=3306,
#     user='root',
#     password='111111',
#     db='sensor_db',
#     charset='utf8mb4',
#     cursorclass=pymysql.cursors.DictCursor,
#     autocommit=True
# )


conn = pymysql.connect(
    host='127.0.0.1',
    port=3306,
    user='sensor_user',      # ← 위에서 만든 사용자
    password='111111',
    db='sensor_db',
    charset='utf8mb4',
    cursorclass=pymysql.cursors.DictCursor,
    autocommit=True
) 

cursor = conn.cursor()


# 최신 센서값 저장용 전역 변수
latest_reading = {
    "lpg": None,
    "co": None,
    "smoke": None,
    "raw": None,
    "timestamp": None,
}


# WebSocket 연결 관리 클래스
class ConnectionManager :
    def __init__(self) :
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
    pattern = re.compile(r"LPG:\s*(\d+)\s*ppm\s*CO:\s*(\d+)\s*ppm\s*SMOKE:\s*(\d+)\s*ppm")
    log.info("▶ read_serial thread started")
    while True :
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue

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

            # WebSocket 브로드캐스트
            asyncio.run(manager.broadcast(latest_reading))

        else:
            latest_reading.update({
                "raw": line,
                "timestamp": timestamp
            })

#  FastAPI startup 이벤트 
@app.on_event("startup")
def startup_event() :
    global ser
    log.info("▶ Starting up: opening serial port")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    ser.setDTR(False)
    time.sleep(2)
    threading.Thread(target=read_serial, daemon=True).start()
# 

#  환기 명령 엔드포인트 
@app.post("/ventilate")
async def ventilate() :
    log.info("▶ /ventilate called")
    try:
        ser.write(b'VENT_START\n')
        log.info("▶ VENT_START sent")
        return {"message": "Ventilation started"}
    except Exception as e:
        log.exception("❌ Failed to send VENT_START")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/ventilate/stop")
async def ventilate_stop() :
    log.info("▶ /ventilate/stop called")
    try:
        ser.write(b'VENT_STOP\n')
        log.info("▶ VENT_STOP sent")
        return {"message": "Ventilation stopped"}
    except Exception as e:
        log.exception("❌ Failed to send VENT_STOP")
        raise HTTPException(status_code=500, detail=str(e))
 

#  REST API: 최신 센서값 조회 
@app.get("/sensor", response_model=SensorReading)
async def get_sensor() :
    if latest_reading["lpg"] is None:
        return {"lpg": 0, "co": 0, "smoke": 0, "raw": "", "timestamp": time.time()}
    return latest_reading


#  REST API: 저장된 히스토리 조회 
@app.get("/history")
async def get_history(limit: int = 100) :
    cursor.execute(
        "SELECT lpg, co, smoke, raw, timestamp FROM sensor ORDER BY id DESC LIMIT %s",
        (limit,)
    )
    return cursor.fetchall()


#  WebSocket: 실시간 센서 스트리밍 
@app.websocket("/ws/sensor")
async def websocket_endpoint(ws: WebSocket):
    await manager.connect(ws)
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect :
        manager.disconnect(ws)

# db에 저장된 데이터 화면에 띄우기
app.mount("/", StaticFiles(directory="static", html=True), name="static")
