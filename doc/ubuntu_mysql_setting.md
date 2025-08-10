좋아요. 올려준 `main.py` + HTML 기준으로 **Ubuntu VM에서 처음부터 끝까지** 동작하게 만드는 전체 절차를 깔끔하게 정리해 줄게요. (시리얼→FastAPI→MySQL→웹 UI)

---

# 0) 디렉터리 구성

```bash
mkdir -p ~/web_fastapi/code/static
cd ~/web_fastapi/code
# 여기에 main.py 배치
# 올려준 HTML을 ~/web_fastapi/code/static/index.html 로 저장
```

---

# 1) 코드 수정 (리눅스 시리얼 포트)

`main.py`에서 윈도우 포트가 남아있어요. VM에서는 `/dev/ttyACM0`로 바꿔야 합니다.

```python
#  시리얼 포트 설정 
SERIAL_PORT = "/dev/ttyACM0"   # ← 리눅스용
BAUD_RATE   = 9600
```

> `/dev/ttyACM0`가 맞는지 확인: `ls /dev/ttyACM*`

---

# 2) 시리얼 권한

```bash
ls -l /dev/ttyACM0
sudo usermod -aG dialout $USER
# 로그아웃/로그인(또는 재부팅) 후 적용
```

---

# 3) MySQL 서버 설치

```bash
sudo apt update
sudo apt install -y mysql-server
sudo systemctl enable --now mysql
sudo systemctl status mysql
```

## 3-1) DB/계정/테이블 생성

```bash
sudo mysql
```

MySQL 콘솔에서:

```sql
-- DB
CREATE DATABASE sensor_db CHARACTER SET utf8mb4 COLLATE utf8mb4_general_ci;

-- 전용 사용자(로컬 전용)
CREATE USER 'sensor_user'@'localhost' IDENTIFIED BY '111111';
GRANT ALL PRIVILEGES ON sensor_db.* TO 'sensor_user'@'localhost';
FLUSH PRIVILEGES;

-- 테이블
USE sensor_db;
CREATE TABLE sensor (
  id BIGINT PRIMARY KEY AUTO_INCREMENT,
  lpg INT NOT NULL,
  co INT NOT NULL,
  smoke INT NOT NULL,
  raw VARCHAR(255) NOT NULL,
  timestamp DOUBLE NOT NULL,
  created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- (옵션) 인덱스
CREATE INDEX idx_timestamp ON sensor(timestamp);
```

## 3-2) `main.py` DB 접속 정보 맞추기

```python
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
```

---

# 4) Python 패키지 설치

(가상환경을 써도 되고, 우선 시스템에 바로 설치해도 됩니다.)

```bash
python3 -m pip install --upgrade pip
pip install fastapi uvicorn pyserial pymysql
# (옵션) DataFrame 필요하면
pip install pandas SQLAlchemy
```

---

# 5) HTML 배치

올려준 HTML을 `~/web_fastapi/code/static/index.html`로 저장하세요.
지금 `main.py`가 다음을 이미 포함하고 있어서 자동으로 정적 서빙됩니다:

```python
app.mount("/", StaticFiles(directory="static", html=True), name="static")
```

---

# 6) 서버 실행

```bash
cd ~/web_fastapi/code
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

* VM이 **Bridged 네트워크**면, 호스트 브라우저에서 `http://<VM_IP>:8000/` 접속 가능
* NAT면 포트포워딩 필요

> **테스트 엔드포인트**
>
> * 최신값: `http://<VM_IP>:8000/sensor`
> * 히스토리: `http://<VM_IP>:8000/history?limit=50`
> * 환기 시작: `POST http://<VM_IP>:8000/ventilate`
> * 환기 종료: `POST http://<VM_IP>:8000/ventilate/stop`

---

# 7) 빠른 동작 점검 순서

1. **시리얼 연결 확인**

```bash
screen /dev/ttyACM0 9600
# 데이터 보이면 OK. 종료: Ctrl+A, K, Y
```

2. **FastAPI 실행 로그**에 `▶ read_serial thread started` 가 보이는지 확인
3. **프론트 페이지** 접속 → 값이 들어오면 성공
4. **DB 적재 확인**

```bash
mysql -u sensor_user -p sensor_db -e "SELECT * FROM sensor ORDER BY id DESC LIMIT 5\G"
```

---

# 8) (선택) DataFrame으로 DB 로드

두 가지 방법 중 편한 걸로 쓰세요.

## 8-1) REST로 불러서 DataFrame

```python
import pandas as pd
import requests

rows = requests.get("http://127.0.0.1:8000/history?limit=200").json()
df = pd.DataFrame(rows)
print(df.head())
```

## 8-2) MySQL에서 직접 DataFrame

```python
import pandas as pd
from sqlalchemy import create_engine

engine = create_engine("mysql+pymysql://sensor_user:111111@127.0.0.1:3306/sensor_db")
df = pd.read_sql("""
  SELECT id, lpg, co, smoke, raw, timestamp, created_at
  FROM sensor
  ORDER BY id DESC
  LIMIT 200
""", engine)
print(df.head())
```

---

# 9) 흔한 오류 해결

* `ModuleNotFoundError: serial / pymysql / fastapi`
  → `pip install pyserial pymysql fastapi uvicorn`
* `pymysql.err.1045 Access denied`
  → 사용자/비번/호스트 확인 (`sensor_user@localhost`)
* `/dev/ttyACM0 Permission denied`
  → `usermod -aG dialout $USER` 후 재로그인
* 값이 안 올라옴
  → 보드레이트(9600/115200), 케이블, DTR 리셋(코드에 `ser.setDTR(False); time.sleep(2)` 이미 있음), 정규식 패턴 확인
* VM에서 접속은 되는데 호스트에서 안 됨
  → VM 네트워크 **Bridged**로 변경, 방화벽 확인

---

# 10) (옵션) ROS를 끼우고 싶다면

* **방법 A**: ROS 노드가 시리얼 읽고 토픽 발행 → FastAPI가 rosbridge(WebSocket, 9090)로 구독해서 DB+웹 반영
* **방법 B**: 지금 FastAPI 그대로 두고, 향후 ROS 노드가 필요한 추가 토픽만 발행/구독

원하면 바로 **A/B 템플릿 코드**까지 만들어 줄게요.
일단 위 순서대로 진행하면, 지금 HTML/서버 구조로 **VM 안에서 즉시 동작**합니다.
