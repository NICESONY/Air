
## 실행

```bash
# 1) Arduino→Serial→ROS2 퍼블리셔
cd ~/ros2server
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run py_pubsub pub_sensor_data
```

```bash
# 2) local computer(ROS2→WebSocket 브릿지) + websocat server live broadcasting
cd ~/web_fastapi/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run pub_web web_bridge --ros-args -p ws_path:=/
```

```bash
# 3) Web 서버(정적 HTML/REST) + javascript websocat client 
cd ~/web_fastapi/code
uvicorn main_wifi:app --reload --host 0.0.0.0 --port 8000
# (⚠️ 파일명이 main_wifi.py가 맞는지 확인. 파일이 main.py면 main:app 로)
```

```bash
# 4) button message subscriber
cd ~/ros2server
ros2 run py_pubsub sub_button_data 
```


```bash
# 5) button message subscriber
cd ~/web_fastapi/ws
ros2 run pub_sbc button2message
```


### After creating the launch file that includes files #2 and #5, run it.

## 디버깅 (좋음 + 몇 개 추가)

```bash
# 토픽 내용 확인(타입 명시)
cd ~
source /opt/ros/humble/setup.bash
source ~/ros2server/install/setup.bash
ros2 topic echo /gas_sensor msgs/msg/PubAirSensor
```

```bash
# 토픽 정보 자세히 (누가 pub/sub 중인지)
ros2 topic info /gas_sensor --verbose
```

```bash
# WebSocket 수신 테스트
python3 - << 'PY'
import asyncio, websockets
async def main():
    async with websockets.connect("ws://localhost:8765/") as ws:
        print("connected")
        while True:
            print(await ws.recv())
asyncio.run(main())
PY
```

```bash
# 8765 포트 열림 확인
ss -lntp | grep 8765
```

```bash
# 브릿지 파라미터 확인(경로/포트 일치 체크)
ros2 param get /ros_web_bridge ws_path
ros2 param get /ros_web_bridge ws_port
```

```bash
# 남아있는 echo/브릿지 강제 종료(헷갈릴 때)
pkill -f "ros2 topic echo /gas_sensor" || true
pkill -f web_bridge || true
```

## 체크포인트

* 퍼블리셔/브릿지 **타입·토픽 동일**(PubAirSensor, `/gas_sensor`)
* 브라우저 JS의 WebSocket URL이 브릿지와 동일 (`ws://<IP>:8765/`)
* `websockets`는 **12.x 이상** 추천 (서버/클라 모두)

