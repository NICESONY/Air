
# 전체 흐름 (현재 구성: A안)

1. **SerialPublisher 노드**

   * 아두이노 시리얼에서 읽어서 **`/gas_sensor`** 토픽으로 **`msgs/msg/PubAirSensor`** 타입으로 **publish**.

2. **WebBridge 노드(ROS2 → WebSocket 서버)**

   * **같은 토픽/같은 타입**으로 **subscribe**.
   * 받은 메시지를 JSON으로 바꿔 **WebSocket 서버(예: `ws://<호스트>:8765/<경로>` )** 에 **실시간 브로드캐스트**(푸시).

3. **HTML 페이지의 자바스크립트(브라우저)**

   * **WebSocket 클라이언트**로 WebBridge에 연결(`new WebSocket("ws://<호스트>:8765/<경로>")`).
   * 서버가 푸시한 JSON을 받아 화면(LPG/CO/SMOKE, 시간, 알람)을 즉시 갱신.
   * 초기 1회는 `/sensor` REST로 가져와서 비어보이지 않게 함(옵션).

4. **FastAPI 서버(main\_wifi\:app)**

   * **정적 HTML만 서빙**(현재 상태).
   * 데이터 경로에는 관여하지 않음(실시간은 WebBridge ↔ 브라우저 직결).

# 반드시 맞춰야 하는 것

* **토픽 이름 & 메시지 타입**: 퍼블리셔와 브릿지가 **완전히 동일**해야 통신됨.
  (예: `/gas_sensor` + `msgs/msg/PubAirSensor`)
* **WebSocket 주소/경로**: 브릿지의 `ws_port`, `ws_path`와 **HTML/JS의 URL이 동일**해야 함.
  (예: `ws_path=/` → `ws://<호스트>:8765/`, `ws_path=/ws` → `ws://<호스트>:8765/ws`)

# 브로드캐스트 의미

* WebBridge에 붙어 있는 **모든** 웹소켓 클라이언트(브라우저 탭 여러 개, 다른 기기)가 **동시에** 같은 데이터를 푸시로 받음(폴링 불필요).

# 지금 안 하는 것(알아두기)

* **DB 저장은 현재 안 함.**
  원하면 WebBridge 콜백에서 INSERT 하거나, 서버가 수신해 저장하는 구조(B안)로 바꾸면 됨.

# 빠른 점검 포인트

* `ros2 topic echo /gas_sensor msgs/msg/PubAirSensor` 로 발행 확인
* `ros2 param get /ros_web_bridge ws_path` / `ws_port` 확인 → HTML의 WS URL과 일치?
* `python3` 웹소켓 원라이너로 수신 테스트 OK?

이해 정확해! 지금 방식은 “ROS2 → WebSocket → 브라우저” 직결 구조라서 지연이 적고 단순해요. 
DB나 중앙서버 연동이 필요해지면 그때 B안으로 확장하면 됩니다.
