좋지! **DB( MySQL ) 관련 명령어를 “순서대로 + 주석 듬뿍”** 정리했어. 그대로 복붙해서 진행하면 돼.

---

# 0) 서비스 확인 (처음 한 번)

```bash
# MySQL/MariaDB 서비스 상태 확인
sudo systemctl status mysql

# 실행/자동시작 설정(꺼져 있으면)
sudo systemctl enable --now mysql
```

---

# 1) root로 접속해서 DB/계정/권한/테이블 만들기 (처음 한 번)

```bash
# root로 MySQL 쉘 접속
sudo mysql
```

```sql
-- (1) DB 생성
CREATE DATABASE IF NOT EXISTS sensor_db DEFAULT CHARACTER SET utf8mb4;

-- (2) 계정 생성 (필요 시 비번 바꾸세요)
CREATE USER IF NOT EXISTS 'sensor_user'@'localhost' IDENTIFIED BY '111111';

-- (3) 권한 부여
GRANT ALL PRIVILEGES ON sensor_db.* TO 'sensor_user'@'localhost';
FLUSH PRIVILEGES;

-- (4) 사용할 DB 선택
USE sensor_db;

-- (5) 테이블 생성(없으면)
CREATE TABLE IF NOT EXISTS sensor (
  id INT AUTO_INCREMENT PRIMARY KEY,
  lpg INT NOT NULL,
  co INT NOT NULL,
  smoke INT NOT NULL,
  raw VARCHAR(255) NOT NULL,
  timestamp DOUBLE NOT NULL  -- epoch 초(소수 포함)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- 나가기: \q
```

---

# 2) 일반 계정으로 접속 테스트

```bash
# 비번 물어봄
mysql -u sensor_user -p -D sensor_db
```

```sql
-- 현재 테이블 확인
SHOW TABLES;

-- 최근 1건(없으면 0행)
SELECT id,lpg,co,smoke,FROM_UNIXTIME(timestamp) AS ts
FROM sensor ORDER BY id DESC LIMIT 1;

-- 나가기: \q
```

---

# 3) (선택) 세션 타임존을 KST로 맞추고 보기

```bash
mysql -u sensor_user -p -D sensor_db
```

```sql
-- 이 세션에서만 한국시간(+09:00)로 표시
SET time_zone = '+09:00';

-- 최근 5건을 한국시간으로 보기(표시용)
SELECT id,lpg,co,smoke,
  DATE_FORMAT(FROM_UNIXTIME(timestamp), '%Y-%m-%d %H:%i:%s') AS ts_kst
FROM sensor
ORDER BY id DESC
LIMIT 5;

-- 나가기: \q
```

---

# 4) /ingest 엔드포인트 수동 테스트 (서버가 DB에 INSERT 되는지 확인)

```bash
# FastAPI가 8000 포트에서 돌아가고 있어야 함
curl -X POST http://127.0.0.1:8000/ingest \
  -H 'Content-Type: application/json' \
  -d '{"lpg":1,"co":2,"smoke":3,"raw":"test","timestamp":1234567890}'
```

```bash
# DB에 들어갔는지 확인
mysql -u sensor_user -p -D sensor_db -e \
"SELECT id,lpg,co,smoke,FROM_UNIXTIME(timestamp) ts FROM sensor ORDER BY id DESC LIMIT 3;"
```

---

# 5) 1초마다 최신 1건 모니터링(편리)

## 방법 A) 원라이너

```bash
# 매초 최신 1건을 KST로 깔끔 출력
watch -n1 -d 'mysql -N -B -e "SET time_zone='\''+09:00'\''; SELECT id,lpg,co,smoke, DATE_FORMAT(FROM_UNIXTIME(timestamp), '\''%Y-%m-%d %H:%i:%s'\'') ts FROM sensor ORDER BY id DESC LIMIT 1;"'
```

## 방법 B) 스크립트로 보기(따옴표 실수 방지)

```bash
mkdir -p ~/bin
cat > ~/bin/last_sensor.sh <<'BASH'
#!/usr/bin/env bash
mysql -N -B <<'SQL'
SET time_zone = '+09:00';
SELECT
  id, lpg, co, smoke,
  DATE_FORMAT(FROM_UNIXTIME(timestamp), '%Y-%m-%d %H:%i:%s') AS ts_kst
FROM sensor
ORDER BY id DESC
LIMIT 1;
SQL
BASH
chmod +x ~/bin/last_sensor.sh
watch -n1 -d ~/bin/last_sensor.sh
```

---

# 6) 비밀번호 매번 치기 귀찮다면(.my.cnf)

```bash
# 홈 디렉터리에 접속 정보 저장(권한 600 필수)
printf "[client]\nuser=sensor_user\npassword=111111\ndatabase=sensor_db\n" > ~/.my.cnf
chmod 600 ~/.my.cnf

# 이제는 비번 없이:
mysql -e "SELECT NOW();"
watch -n1 -d 'mysql -N -B -e "SET time_zone='\''+09:00'\''; SELECT id,lpg,co,smoke, DATE_FORMAT(FROM_UNIXTIME(timestamp), '\''%Y-%m-%d %H:%i:%s'\'') ts FROM sensor ORDER BY id DESC LIMIT 1;"'
```

---

# 7) 시간 이상해 보일 때 빠른 점검

```bash
# DB 서버 시각 확인
mysql -e "SELECT @@system_time_zone, @@session.time_zone, NOW(), UNIX_TIMESTAMP();"

# OS 시간/타임존/NTP 동기화
date
timedatectl
sudo timedatectl set-ntp true
sudo timedatectl set-timezone Asia/Seoul   # 필요 시
```

---

# 8) 자주 쓰는 조회 모음

```bash
# 최신 5건(KST)
mysql -e "SET time_zone='+09:00'; \
SELECT id,lpg,co,smoke, DATE_FORMAT(FROM_UNIXTIME(timestamp),'%Y-%m-%d %H:%i:%s') ts_kst \
FROM sensor ORDER BY id DESC LIMIT 5;"

# 토탈 카운트
mysql -e "SELECT COUNT(*) AS cnt FROM sensor;"

# 최근 1분치
mysql -e "SELECT COUNT(*) AS last_60s \
FROM sensor WHERE timestamp >= UNIX_TIMESTAMP() - 60;"
```

필요하면 **DROP/초기화**, **인덱스**, **백업/복구** 명령도 이어서 정리해 줄게. 원하는 범위만 말해줘!
