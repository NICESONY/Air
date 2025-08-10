# main.py — FastAPI server (no ROS import)
import logging
from typing import Optional, Dict, List, Set

import pymysql
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

import json



'''
uvicorn main_wifi:app --reload --host 0.0.0.0 --port 8000
'''


'''

[아두이노] --(USB 시리얼)--> [SerialPublisher 노드]
   └─ /gas_sensor 토픽(PubAirSensor) 으로 publish

[WebBridge 노드]
   └─ /gas_sensor 동일 타입으로 subscribe
   └─ 받은 값을 JSON으로 변환해서 WebSocket 서버(예: ws://<IP>:8765/<path>)로 브로드캐스트
      ※ 여기서 쓰는 건 "websockets" 파이썬 라이브러리입니다. (websocat 툴 아님)

[FastAPI main.py]
   └─ 정적 HTML 서빙(/)


   └─ HTML/JS가 WebSocket으로 WebBridge에 직접 접속해서 실시간 표시


   
--------- logic flow ---------

SerialPublisher가 /gas_sensor로 메시지 발행

WebBridge가 구독해서 JSON으로 변환

WebBridge 안의 웹소켓 서버가 연결된 브라우저들에 JSON을 보내줌

HTML/JS가 그 JSON을 받아 DOM 업데이트
'''

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("web_fastapi")

app = FastAPI()


from pydantic import BaseModel

class IngestPayload(BaseModel):
    lpg: int
    co: int
    smoke: int
    raw: str
    timestamp: float


@app.post("/ingest")
async def ingest(p: IngestPayload):
    conn = get_conn()
    with conn.cursor() as cur:
        cur.execute(
            "INSERT INTO sensor (lpg, co, smoke, raw, timestamp) VALUES (%s,%s,%s,%s,%s)",
            (p.lpg, p.co, p.smoke, p.raw, p.timestamp)
        )
    # 최신 캐시도 갱신(옵션)
    latest_reading.update(p.dict())
    return {"ok": True}


# ===================== DB =====================
DB_CFG = dict(
    host="127.0.0.1",
    port=3306,
    user="sensor_user",
    password="111111",
    db="sensor_db",
    charset="utf8mb4",
    cursorclass=pymysql.cursors.DictCursor,
    autocommit=True,
)
_conn: Optional[pymysql.connections.Connection] = None

def get_conn() -> pymysql.connections.Connection:
    global _conn
    if _conn is None:
        _conn = pymysql.connect(**DB_CFG)
    else:
        try:
            _conn.ping(reconnect=True)
        except Exception:
            _conn = pymysql.connect(**DB_CFG)
    return _conn

# 최신값 캐시(옵션): 없으면 0으로 응답
latest_reading: Dict[str, Optional[float]] = {
    "lpg": None, "co": None, "smoke": None, "raw": None, "timestamp": None
}

class SensorReading(BaseModel):
    lpg: int
    co: int
    smoke: int
    raw: str
    timestamp: float

@app.on_event("startup")
def startup():
    log.info("▶ FastAPI startup: DB connect")
    get_conn()

@app.on_event("shutdown")
def shutdown():
    global _conn
    try:
        if _conn:
            _conn.close()
    except Exception:
        pass

# ============= REST =============
@app.get("/sensor", response_model=SensorReading)
async def get_sensor():
    # 캐시가 없으면 DB에서 최신 1건 가져오기
    if latest_reading["lpg"] is None:
        conn = get_conn()
        with conn.cursor() as cur:
            cur.execute(
                "SELECT lpg, co, smoke, raw, timestamp FROM sensor ORDER BY id DESC LIMIT 1"
            )
            row = cur.fetchone()
            if row:
                return row
        return {"lpg": 0, "co": 0, "smoke": 0, "raw": "", "timestamp": 0.0}
    return latest_reading

@app.get("/history")
async def get_history(limit: int = 100):
    conn = get_conn()
    with conn.cursor() as cur:
        cur.execute(
            "SELECT lpg, co, smoke, raw, timestamp FROM sensor ORDER BY id DESC LIMIT %s",
            (limit,),
        )
        return cur.fetchall()






# --- 추가: 상단 import 근처 --
# 연결된 클라이언트 보관
ws_clients: Set[WebSocket] = set()

async def ws_broadcast_text(text: str):
    dead = []
    for c in list(ws_clients):
        try:
            await c.send_text(text)
        except Exception:
            dead.append(c)
    for c in dead:
        ws_clients.discard(c)

async def ws_broadcast_json(obj):
    await ws_broadcast_text(json.dumps(obj))

# --- 추가: WebSocket 엔드포인트 ---
@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    ws_clients.add(ws)
    try:
        while True:
            msg = await ws.receive_text()
            # 브라우저/ROS 등 어떤 클라이언트가 보낸 메시지든 전체에 팬아웃
            await ws_broadcast_text(msg)
    except WebSocketDisconnect:
        ws_clients.discard(ws)




# ============= 정적 파일 =============
# code/static/index.html 를 / 로 서빙
app.mount("/", StaticFiles(directory="static", html=True), name="static")
