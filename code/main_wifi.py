# main.py — FastAPI server (no ROS import)
import logging
from typing import Optional, Dict, List

import pymysql
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel


'''
uvicorn main_wifi:app --reload --host 0.0.0.0 --port 8000
'''

logging.basicConfig(level=logging.INFO)
log = logging.getLogger("web_fastapi")

app = FastAPI()

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

# ============= 정적 파일 =============
# code/static/index.html 를 / 로 서빙
app.mount("/", StaticFiles(directory="static", html=True), name="static")
