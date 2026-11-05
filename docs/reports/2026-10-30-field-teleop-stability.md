# 현장 원격주행 안정화 — 젯슨 부하, 입력 끊김, Wi-Fi, 시계

2026-10-30 시연 촬영 준비 중 원격주행이 운전 시작 직후 저절로 풀리고 패드가 진동하는 문제를
추적했다. 통합 브랜치 `deploy/jetson-field-20261028`의 젯슨 실행 체크아웃
(`~/power-train-sw-integrated`)과 노트북 콘솔(`.worktrees/jetson-field`)에서 측정했다.

결론부터 적는다.

- 주원인은 노트북 ↔ 로봇 AP 5 GHz 링크였다. 지연이 평균 113~427 ms, 최대 1.3 s까지 튀었다.
  로봇 AP 2.4 GHz로 옮기자 평균 2 ms, 최대 9 ms가 됐다.
- 입력이 0.2 s만 끊겨도 게이트웨이와 차체가 각각 정지를 고정(MOTION_HOLD)해, 짧은 끊김이
  운전 해제로 이어졌다. 통합 compose에서는 끊김 동안 정지만 하고 고정하지 않게 바꿨다.
- 젯슨 전체 부하는 SRT 15 fps로 거의 줄지 않았지만, 영상은 들쭉날쭉한 19 Hz에서 고른
  15 Hz가 됐다.
- 젯슨 RTC가 부팅마다 1970년이 되어 배포한 ROS 코드가 설치본에 반영되지 않는 일이 있었다.

## 1. 젯슨 부하와 SRT 송출 fps

### 변경 (`af0ff62`, 기본값 `594515d`)

`L515_SRT_FPS`(30 또는 15)로 Gateway의 SRT 출력 fps를 고른다. 15면 컬러 프레임을 복사하기
전에 도착 시각 기준으로 한 장씩 건너뛰고(간격 50 ms 미만이면 건너뜀), gst `framerate`와 키프레임
간격을 15로 맞춰 GOP 약 1 s를 유지한다. 카메라 캡처 30 fps, 깊이 10 Hz, ROS 토픽 주기,
1280×720 기하는 그대로다. 의도적 건너뜀은 `skipped`, 백프레셔 덮어쓰기는 기존 `dropped`로 따로
센다. 통합 compose 기본값은 15다(`L515_SRT_FPS=30`으로 되돌린다).

### x86 실경로 시험 (노트북, 합성 30 fps 입력 8 s, 실제 `SrtStreamer` + gst-launch 송수신)

| 설정 | 수신 fps | 건너뜀 / 누락 | x264 프로세스 CPU |
|---|---|---|---|
| 30 · RGB | 30.13 | 0 / 0 | 56.5% |
| 30 · 오버레이 | 30.01 | 0 / 0 | 62.0% |
| 15 · RGB | 15.00 | 120 / 0 | 30.4% |
| 15 · 오버레이 | 15.14 | 120 / 0 | 30.6% |

### 젯슨 실측 (40 s 창, 25 W 모드, 차체·지형 지도·오버레이·로봇팔 스택 가동)

`/proc` CPU 시간 차분, 100% = 코어 1개.

| 항목 | 30 fps | 15 fps |
|---|---|---|
| 전체 사용 (600% 중) | 528% | 516% |
| `gateway_main` | 104.3% | 102.5% |
| gst-launch (x264) | 68.2% | 56.9% |
| `terrain_map` / `terrain_overlay` | 83.8% / 78.3% | 84.3% / 81.3% |
| 로봇팔 `moveit_dynamixel` | 71.2% | 71.8% |
| `chassis` | 38.1% | 38.4% |
| SRT 실제 송출 | 19.13 Hz, 누락 10.80 Hz | 15.01 Hz, 누락 0.05 Hz |

30 fps 설정은 인코더가 따라가지 못해 실제로는 19 Hz를 들쭉날쭉 보냈다. 15 fps는 고르게 15 Hz를
보낸다. 인코더 절감은 코어 약 0.11개분이고, x86 시험의 절반 절감보다 작다. 지형 지도와 오버레이는
두 경우 모두 처리 시간이 100 ms 주기를 넘겨(지도 108~152 ms, 오버레이 104~124 ms) 주기를 계속
놓친다. 다음 부하 대책은 지도·오버레이 주기 낮추기(파라미터만, 이번에 미실시)다.

`powertrain_ros`를 compose로 다시 만들 때 compose가 `powertrain_terrain_map`·
`powertrain_terrain_overlay`를 먼저 정지시켰다(docker events). compose 파일에 두 서비스의
`depends_on`은 없고 원인은 확인하지 못했다. 다시 만든 뒤에는 두 서비스가 떠 있는지 확인한다.

## 2. 입력 끊김이 운전을 해제하던 경로

증상: 운전 시작 1~2 s 뒤 운전이 풀리고 패드가 진동했다. `teleop_command` 로그에
`CONTRACT_VIOLATION: partial record at TCP close`가 수십 초 간격으로 났다(몇 분 사이 5회).

경로 세 개가 겹쳐 있었다.

1. 노트북 `controller_process`는 비차단 소켓 `sendall`이 `BlockingIOError`를 내면(Wi-Fi 정지로
   송신 버퍼가 참) 연결을 끊었다. 레코드 중간에서 끊겨 위 위반이 됐다.
2. 게이트웨이(`remote_input_gateway`)는 입력이 `input_timeout_s` 0.2 s를 넘기면 MOTION_HOLD를
   고정했다. 위반도 MOTION_HOLD로 고정했다.
3. 게이트웨이는 끊긴 순간 0을 한 번만 보내고 조용해지므로, 차체 권한(`chassis/authority.py`)이
   0.3 s 뒤 명령 끊김으로 다시 MOTION_HOLD를 고정했다.

### 변경 (`b6ff0ea`)

- 노트북: 송신 버퍼가 차면 연결을 끊지 않고 남은 레코드를 이어 보낸 뒤 다음 샘플을 보낸다.
  그동안 새 샘플은 건너뛴다. `TCP_USER_TIMEOUT` 5 s는 그대로다.
- 게이트웨이 `latch_stale_input=false`: 입력이 끊기면 0만 내보내고 고정하지 않으며, 새 입력이
  오면 바로 주행을 잇는다. 끊긴 동안 0 명령을 계속 보내 차체 권한도 고정되지 않는다.
  게이트웨이 자체가 MOTION_HOLD이면(원격 E-stop 엣지 등) 지금처럼 조용해져 차체도 고정된다.
- 디코더 `partial_close_violation=false`: TCP 종료에 잘린 레코드는 위반이 아닌 단순 끊김이다.
  재연결 뒤에는 기존대로 중립 입력이 필요하다.
- 기본값은 기존 고정 계약 그대로다. 통합 compose만 `TELEOP_INPUT_TIMEOUT_S`(기본 0.5),
  `TELEOP_LATCH_STALE_INPUT`(false), `TELEOP_PARTIAL_CLOSE_VIOLATION`(false)으로 완화한다.
  0.5 s 동안은 마지막 명령으로 계속 간다(1.5 m/s에서 최대 약 0.75 m).

확인: 젯슨 `ros2 param get /teleop_command`로 0.5 / False / False. 젯슨 쪽 시험 44건, 노트북
컨트롤러 시험 71건 통과(남은 1건은 실행 중인 콘솔의 프로세스 잠금 때문). Codex 검토는 사용자
지시로 생략했다.

콘솔의 1.5 s 길게 누르기와 0.5 s 상태 신선도 조건은 바꾸지 않았다. 링크가 나쁠 때는 누르는
도중 조건이 깜빡여 시작이 취소된다. 완화안(신선도 1 s, 누르는 중 깜빡임 허용, 진동 기준 1.5 s)은
기존 시험 2건과 충돌해 적용하지 않았다.

## 3. Wi-Fi: 5 GHz 혼잡, 2.4 GHz로 해결

로봇 AP GL-SFT1200에는 노트북과 젯슨(유선) 두 기기만 붙어 있었다. 노트북 수신은 약 3.4 Mbit/s로
대역폭 부족은 아니었다.

| 노트북 링크 | 공유기(192.168.8.1) ping | 비고 |
|---|---|---|
| 5 GHz `ZETIN-ROBOT-5G` ch36/80 MHz, 절전 켬 | 평균 113 ms, 최대 445 ms | 젯슨까지 평균 124 / 최대 439 ms |
| 같은 링크, 절전 끔 | 평균 427 ms, 최대 1330 ms | 지연이 주기적이지 않고 계속 높음, tx retries 27,765 |
| 2.4 GHz `ZETIN-ROBOT` ch9 | 평균 2 ms, 최대 9 ms | 젯슨까지 평균 2.7 / 최대 20 ms, 영상 수신 4 Mbit/s |

5 GHz 80 MHz 폭(ch36~48)에 학교 AP 20개가량이 겹쳐 있었다. 공유기까지 이미 지연이 나므로 젯슨
쪽 문제가 아니다. 노트북 NetworkManager 설정: `ZETIN-ROBOT` autoconnect-priority 100, 로봇 AP
프로필 모두 `802-11-wireless.powersave 2`(끔). 절전은 재연결 뒤 다시 켜진 적이 있어
`iw dev <if> get power_save`로 확인한다. 대회장에서는 채널 상황을 다시 잰다.

## 4. 젯슨 시계

젯슨 시계가 부팅마다 1970-01-01이 됐다. 부팅 로그에서 `systemd-timesyncd`가 10.22 s에 저장
시각(10-29)을 복원했지만, 10.27 s에 PMIC RTC `nvvrs-pseq-rtc`가 `rtc0`으로 등록되며 시계를
RTC 값(1970)으로 덮어썼다. 이 RTC는 전원을 끄면 시각을 잃는다(백업 없음으로 판단).

영향: 1970년 시계에서 체크아웃한 소스는 기존 빌드보다 오래돼 보여 `colcon build`가 설치본을
덮어쓰지 않았다(빌드 성공, 설치본에 새 코드 없음). 시계를 노트북에 맞추고 1970년 시각으로 남은
`ros2/src/powertrain_ros` 파일 22개를 `touch`한 뒤 `build/powertrain_ros`를 지우고 다시 빌드해서
반영했다. 시각을 앞으로 맞추는 순간 sudo 인증 캐시가 만료되므로, 그 뒤 sudo는 다시 인증해야 한다.

대책: `scripts/powertrain-clock`과 `scripts/systemd/powertrain-clock-{restore.service,
save.service,save.timer}`. 1분마다(그리고 종료 때) `/var/lib/powertrain/clock-stamp`를
갱신하고, 부팅 때 `rtc0` 등록을 기다렸다가 Docker보다 먼저 그 시각으로 되돌린다. 꺼져 있던
시간만큼 늦지만 뒤로 가지 않는다. 정확한 시각은 배포 때 노트북 시각으로 맞춘다.

설치(젯슨 호스트, 저장소 루트):

```bash
sudo install -Dm755 scripts/powertrain-clock /usr/local/sbin/powertrain-clock
sudo install -Dm644 scripts/systemd/powertrain-clock-restore.service /etc/systemd/system/
sudo install -Dm644 scripts/systemd/powertrain-clock-save.service /etc/systemd/system/
sudo install -Dm644 scripts/systemd/powertrain-clock-save.timer /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now powertrain-clock-restore.service powertrain-clock-save.timer
```

10-30 설치 뒤 두 유닛 enabled·active, 타이머 1분 주기 실행을 확인했다. 11-02 재부팅 로그
(`journalctl -b -o short-monotonic`)에서 10.38 s timesyncd 복원 → 10.40 s `rtc0` 등록 →
12.41 s `powertrain-clock`이 마지막 기록 시각(10-30 18:38)으로 복원하는 순서를 확인했다.
그 부팅에서 젯슨은 인터넷이 되는 `ZETIN_5G`에도 붙어 있어 NTP로 정확한 시각까지 맞춰졌고,
운용 컨테이너 10개가 자동 복귀했다.

## 5. node15(뒤왼쪽) 전류 한계 초과

같은 날 node15에서 세 번 `axis.error 0x40`(MOTOR_FAILED), `motor.error 0x1000`
(CURRENT_LIMIT_VIOLATION), 엔코더 오류 0이 났다. 다른 다섯 축은 오류가 없었다. 첫 번째 전에는
스키드 조향 전환이 기록돼 있었다. 오류가 남아 있으면 차체의 하드웨어 정지 증명이 무효가 되어
주행 해제가 PENDING에 머물고 운전 시작도 막힌다.

조치: 축이 IDLE인 상태에서 CAN `Clear_Errors`(0x18)를 node15에만 보내 지웠다. 차체가 운전 시작 때
보내는 것과 같은 명령이지만, 차체가 버스를 소유한 상태에서 별도 프로세스로 보냈다. 원인
(기계 끌림 또는 node15 `current_lim` 설정)은 확인하지 않았다. 바퀴를 들고 손으로 돌려 비교하고,
USB로 `current_lim`을 읽어 다른 축과 대조하는 것이 다음 순서다.

## 6. 운용 상태와 남은 일

젯슨 재부팅 뒤 운용 컨테이너 10개(`canwatchdog`, `chassis`, `chassis_telemetry`, `control`,
`observability`, `pdist80b_telemetry`, `ros`, `session`, `terrain_map`, `terrain_overlay`)가
unless-stopped로 자동 복귀했고 `can0`이 UP이었다. 로봇팔 컨테이너는 재시작 정책이 `no`이며 팀이
직접 관리한다.

남은 일:

- 지형 지도 2.5 Hz·오버레이 5 Hz 파라미터 A/B (부하 대책).
- node15 전류 초과 원인 확인.
- 콘솔 시작 조건 완화 여부 결정(시험 2건 갱신 필요).
- `powertrain_ros` 재생성 때 지형 서비스가 같이 정지되는 원인.
