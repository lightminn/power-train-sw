# AS5048B 서스펜션 엔코더 배선 확인과 젯슨 부하 재측정

2026-11-02. 젯슨은 `deploy/jetson-field-20261028`(`b245a77`)을 `~/power-train-sw-integrated`에서
실행 중이었다. 노트북과 젯슨 모두 `ZETIN_5G`(젯슨 192.168.50.98)에 붙어 있었고 로봇 AP는 꺼져
있었다. 연결된 장치는 L515, D435i, AS5048B뿐이고 CAN 모터는 연결하지 않았다.

## 1. AS5048B 배선 확인

명령: `PYTHONPATH=motor_control python3 -m suspension_encoders scan`(젯슨 호스트, 읽기 전용),
`i2cdetect -y -r 7`.

| 주소 | 설정상 관절 | 결과 (자석을 손으로 댄 상태) |
|---|---|---|
| 0x40 | `ml_rocker_joint` | 응답. AGC 87~105, 크기 4502~4760, 진단 OCF |
| 0x41 | `mr_rocker_joint` | 응답. AGC 60~84, 크기 4538~4740, 진단 OCF |
| 0x42 | `bl_bogie_joint` | 응답. AGC 66~96, 크기 4511~4762, 진단 OCF |
| 0x43 | `br_bogie_joint` | 응답. AGC 59~86, 크기 4503~4759, 진단 OCF |

- 40핀 헤더 3번(SDA)·5번(SCL)이 `/dev/i2c-7`임을 실물로 확인했다. 설정
  `config/suspension_encoders.json`의 메모를 고쳤다.
- 자석이 없으면 AGC 255·크기 2~11·진단 `CompHigh`이고 원시 각도가 수십 도씩 흔들린다.
  자석을 대면 위 표처럼 바뀐다. 손으로 대고 있었으므로 각도 안정성은 아직 재지 않았다.
- 0x40+0x41, 0x40+0x43을 같은 버스에서 동시에 읽었다. 네 개를 동시에 꽂은 스캔은 아직 없다.
- ⚠️ 0x43 모듈의 3V3이 빠진 채 꽂혀 있는 동안 버스 전체가 멈췄다(`tegra-i2c c250000.i2c: I2C
  transfer timed out`, 다른 주소도 모두 무응답). 3V3을 다시 꽂자 회복됐다. 장착 뒤 커넥터 하나의
  접촉 불량이 네 채널을 모두 끊을 수 있다.
- 관절↔주소 대응, 부호, 영점(`capture-zero`), `verified`는 아직 하지 않았다.

## 2. 통신 지연

콘솔(`~/.local/bin/powertrain-integrated-console`)이 젯슨에 연결되어 영상을 받는 동안
(수신 3.6 Mbit/s) 노트북 → 젯슨 ping 100회: 평균 3.2 ms, 최대 5 ms, 100 ms 초과 0회.
10-30 로봇 AP 5 GHz 링크(평균 113~427 ms)와 달리 이 경로는 지연 문제가 없다.

## 3. 젯슨 부하 (40 s, 25 W 모드)

파워트레인 컨테이너 10개 가동, 콘솔 연결, 로봇팔 꺼짐, CAN 미연결.

| 항목 | CPU (100% = 코어 1개) |
|---|---|
| 전체 | 414% / 600%, 부하 평균 7.8 |
| `gateway_main` | 103.8% |
| `terrain_overlay` / `terrain_map` | 60.1% / 52.6% |
| gst-launch (x264, 15 fps) | 51.3% |
| `chassis` | 33.4% |
| 커널 `pr/ttyTCU0` + journald | 약 30% |
| `chassis_telemetry` / `ops_broker` | 15.4% / 15.4% |

RAM 1.4 / 7.6 GB, 온도 약 52 °C, VDD_IN 7.1 W. SRT 15.09 Hz 누락 0, 컬러 29.9 Hz, 깊이 10 Hz.

이 수치는 실제 운용보다 낮게 나온 부분과 높게 나온 부분이 섞여 있다.

- CAN이 없으면 `can0`이 100 ms마다 bus-off 재시작을 하며 커널 메시지를 초당 약 147줄 낸다
  (`kernel.printk = 6 6 1 7`이라 시리얼 콘솔로도 출력). 이것이 코어 약 0.3개를 쓴다.
- 바퀴 데이터가 없어 지도 화면이 "로버를 1초 정지하세요"였고, 지형 노드 CPU가 10-30
  (84%·81%)보다 낮았다.
- 로봇팔이 빠졌다. 10-30에는 `moveit_dynamixel_bridge`가 약 72%였다.

실제 운용 기준은 10-30 실측(로봇팔·CAN·지형 가동, SRT 15 fps) 516% / 600%이다.

## 4. 부하 대책을 위해 확인한 사실

- 25 W 모드는 CPU 상한을 1344 MHz로 건다(`scaling_max_freq`). 하드웨어 최대는 1728 MHz이고,
  `/etc/nvpmodel.conf`의 MAXN_SUPER는 6코어 그대로 CPU 상한을 없앤다(`MAX_FREQ -1`).
  부하 중 6코어 모두 1344 MHz에서 62~70%였다.
- `/l515/color/image_raw`(bgr8 1280×720, 30 Hz) 구독자는 0이다. 로봇팔은 D435i를 쓴다.
  `/l515/depth/image_rect_raw`는 `terrain_map`과 `terrain_overlay`가 구독한다.

전원 모드 변경, 제어 경로 CPU 우선순위, 지형 주기 절반, 구독자 없는 컬러 발행 생략,
커널 콘솔 로그 수준 같은 대책은 검토 중이며 아직 적용하지 않았다.
