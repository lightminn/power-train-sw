# XL430 마스터–슬레이브 TCP 벤치 가이드

> 상태: 2026-07-22 단축 HIL 완료. 로봇팔 production 명령 경로가 아니다. 실제 로봇팔은
> `ArmCommandAuthority`와 MoveIt Servo의 joint-limit·collision 경로를 사용해야 한다.

## 1. 현재 실측 구성

| 역할 | 장비 | 포트 | ID | 통신 |
|---|---|---|---:|---|
| 마스터 | PC XL430-W250-T | `usb-...-FTAO4U2V...` | 5 | TTL, Protocol 2.0, 1 Mbps |
| 슬레이브 | Jetson XL430-W250-T | `usb-...-FTBEO3M5...` | 2 | TTL, Protocol 2.0, 1 Mbps |

마스터는 torque OFF 상태의 Present Position만 30 Hz로 읽는다. 슬레이브는 연결 첫 프레임을
받은 후 현재 자세를 Goal Position으로 먼저 seed하고 torque를 켠다. 이후 마스터 시작점 대비
상대 tick을 슬레이브 시작점에 더한다. 정상 종료 시 슬레이브 torque를 끄고 기존 Operating Mode,
Profile Velocity/Acceleration, Goal PWM과 모드 변경으로 초기화되는 제어기 gain을 복원한다.

## 2. XL430 단위와 계산

아래 값은 **XL430-W250 전용**이다. 다른 모델은 모델별 e-Manual의 control table과 단위를 다시
확인해야 하며 이 스크립트는 model number 1060이 아니면 실행을 거부한다.

| 항목 | XL430 환산 | 현재 의미 |
|---|---:|---|
| 위치 | 4096 ticks/rev | 1 tick = 0.087890625° |
| Profile Velocity | 0.229 rpm/raw | 1 raw ≈ 1.374°/s |
| Goal PWM | 0.113 %/raw | raw 100 ≈ 11.3%, raw 885 ≈ 100% |
| Present Load | 0.1 %/raw | raw 180 ≈ 18% 추정 부하 |
| Profile Acceleration | 214.577 rev/min²/raw | raw 2 ≈ 42.9°/s² |
| TCP 발행률 | 30 Hz | 약 33.3 ms마다 새 목표 위치 전송 |
| stale stop | 200 ms | 새 frame이 없으면 서버가 torque OFF 후 종료 |

각도 제한을 tick으로 바꾸는 식은 다음과 같다.

```text
ticks = round(degrees × 4096 / 360)
degrees = ticks × 360 / 4096
```

예시:

| 허용 상대각 | `--max-delta-ticks` |
|---:|---:|
| ±5° | 57 |
| ±10° | 114 |
| ±30° | 341 |
| ±90° | 1024 |
| 소프트 상대각 제한 해제 | 4095 |

`4095`는 무한 회전을 뜻하지 않는다. 서버는 Position Control Mode를 사용하며 슬레이브 EEPROM의
Min/Max Position Limit를 읽어 그 범위에서 다시 clamp한다. 현재 모터가 기본값이면 결과 범위는
0~4095다.

Profile Velocity의 대표값:

| raw | rpm | 이론 명령 속도 |
|---:|---:|---:|
| 5 | 1.145 | 약 6.87°/s |
| 10 | 2.29 | 약 13.74°/s |
| 20 | 4.58 | 약 27.48°/s |
| 50 | 11.45 | 약 68.70°/s |

현재 벤치 서버는 `Profile Velocity`를 raw 2~50으로 제한한다. XL430-W250-T 제조사
무부하 속도는 11.1 V에서 57 rpm(342°/s), 12.0 V에서 61 rpm(366°/s)이고 현재 모터의
`Velocity Limit` 실측값은 raw 265(약 60.7 rpm, 364.1°/s)다. 이 값들은 무부하 정격·레지스터
상한이지 그리퍼 조립체의 안전속도가 아니다. 이 벤치에서 제조사 최대속도를 그대로 노출하지 않는
이유다. 모터 종류, 전압, 감속비, 링크 관성, 가동범위가 바뀌면 raw 값을 복사하지 말고 다시
계산·저속 검증해야 한다.

`Profile Velocity=0`은 정지가 아니라 **무한 속도 profile**, `Profile Acceleration=0`은
**무한 최대가속도** 의미라 서버가 거부한다. Velocity-based Profile에서는 Acceleration이
Velocity의 50%를 넘지 않아야 하므로 서버도 `2 × acceleration <= velocity`를 강제한다.
`Velocity Limit(44)`는 공식적으로 Goal Velocity의 상한이지만, 이 벤치는 추가 보수 제한으로
Profile Velocity도 해당 EEPROM 값 이하인지 확인한다.

30 Hz는 “모터가 1초에 30번 순간이동한다”는 뜻이 아니다. 목표 위치가 30 Hz로 갱신될 뿐이며,
실제 축은 Profile Velocity/Acceleration, 전압, 부하, 기구 마찰과 내부 위치제어기에 따라 뒤따른다.
마스터를 더 빨리 움직이면 슬레이브는 최신 목표를 향해 제한 속도로 따라가므로 일시적으로 lag가
생긴다.

## 3. 모터를 바꿀 때 확인할 항목

다음 값은 모터별로 먼저 읽고 실행 인자와 맞춰야 한다.

1. 정확한 모델명과 model number, TTL/RS-485 물리 인터페이스
2. Protocol version, baud rate, 중복되지 않는 ID
3. 현재 Operating Mode와 torque OFF 여부
4. Min/Max Position Limit와 조립된 기구의 실제 충돌 한계
5. Drive Mode(velocity-based/time-based), Velocity Limit 및 안전한 Profile Velocity/Acceleration
6. Goal PWM, Present Load, 온도, 공급전압과 전원 정격
7. 조인트 방향(`--direction 1` 또는 `-1`)과 감속비/링크비
8. U2D2의 `/dev/serial/by-id/...` 고정 경로와 실행 계정의 `dialout` 권한

모델이 같아도 기구 부하와 설치 방향이 다르면 같은 속도·PWM 값을 그대로 복사하지 않는다.
빈 축에서 낮은 PWM과 낮은 속도로 시작하고, 실측 전류·온도·위치 오차를 보고 한 단계씩 올린다.

## 4. 현재 벤치 안전장치와 한계

- 마스터 torque가 켜져 있으면 client는 시작을 거부한다.
- 슬레이브가 이미 다른 프로세스에 의해 torque ON이면 server는 takeover를 거부한다.
- 서버는 XL430 model number 1060만 허용한다.
- 요청 Profile Velocity가 모터의 Velocity Limit를 넘으면 시작을 거부한다.
- 목표는 모터의 EEPROM Min/Max Position Limit 안으로 clamp한다.
- Goal PWM은 출력 상한일 뿐 정확한 토크 제한값이 아니다.
- Present Load 제한은 약 0.2초 간격으로 3회 연속 초과할 때 정지하는 **소프트웨어 감시**다.
  현재 raw 180 설정은 18% 추정 부하다. XL430의 Present Load는 전류 센서 측정값이 아니라
  내부 출력 기반 추정값이므로 실제 전류·토크·힘으로 환산할 수 없고 하드웨어 전류차단을 대신하지
  않는다. 최대 약 0.6초의 검출 지연도 있다.
- TCP frame stale, disconnect, malformed/replay frame, 통신 오류 시 torque OFF를 시도한다.
- TCP는 암호화·상호인증이 없고 `--allowed-client`는 접속 IP만 확인한다. 신뢰된 벤치 Wi-Fi에서만
  사용하며 인터넷이나 공용망에 포트를 노출하지 않는다.
- Jetson 전원 단절, 프로세스 `SIGKILL`, USB/U2D2 고장처럼 cleanup 코드 자체가 실행되지 못하는
  고장은 소프트웨어 torque OFF로 보장할 수 없다. 실험자는 12 V 물리 전원 차단 수단을 손에 둔다.
- 이 벤치는 self-collision, 환경 충돌, joint-space limit, singularity를 계산하지 않는다.
- 둘 이상의 writer를 동시에 실행하지 않는다. 기존 Dynamixel bridge/FSM이 정지된 벤치에서만 쓴다.

## 5. 실행 예시

아래 코드는 PR #22가 merge된 뒤 `extreme-robot`을 갱신한 상태를 기준으로 한다. merge 전에는
운영 중인 dirty Jetson 저장소에서 branch를 바꾸지 말고 별도 clean clone에서만 검증한다.

Jetson에서 빌드한다.

```bash
# Jetson 호스트, SSH 직후 홈 ~ 기준
cd ~/extreme-robot
git pull --ff-only
docker compose up -d ros2
docker exec ros2_humble bash -lc '
  source /opt/ros/humble/setup.bash
  cd /root/ros2_ws
  colcon build --packages-select robot_arm_msgs dynamixel_control
'
```

PC에는 ROS 2가 없어도 된다. PC 저장소에서 전용 venv를 한 번 준비한다.

```bash
# PC, extreme-robot 저장소 루트
python3 -m venv .venv
.venv/bin/pip install 'dynamixel-sdk==4.0.5'
groups | grep dialout
ls -l /dev/serial/by-id/
```

`dialout`이 출력되지 않으면 `sudo usermod -aG dialout "$USER"` 후 로그아웃·로그인을 한 번 한다.
양쪽에서 기존 `position_node`, `moveit_dynamixel_bridge`, `arm_fsm` 또는 다른 진단 스크립트가
같은 U2D2를 점유하지 않는지 먼저 확인한다.

Jetson의 `ros2_humble` 컨테이너에서 슬레이브 서버를 먼저 실행한다.

```bash
docker exec -it ros2_humble bash -lc '
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
exec ros2 run dynamixel_control master_slave_slave \
  --confirm-bench \
  --slave-id 2 \
  --allowed-client 192.168.50.128 \
  --device /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO3M5-if00-port0 \
  --max-delta-ticks 114 \
  --profile-velocity 10 \
  --profile-acceleration 5 \
  --goal-pwm 100 \
  --load-limit-raw 180 \
  --stale-ms 200
'
```

PC에서 마스터 client를 실행한다.

```bash
PYTHONPATH=ros2_ws/src/dynamixel_control \
.venv/bin/python -m dynamixel_control.master_slave_master_client \
  --server 192.168.50.98 \
  --master-id 5 \
  --device /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO4U2V-if00-port0 \
  --rate-hz 30
```

제한 없는 확인이 필요해도 `--max-delta-ticks 4095`만 바꾸고 속도·PWM·전류 감시는 유지한다.
기구의 검증된 충돌 한계를 얻은 뒤에는 4095를 계속 사용하지 말고 해당 관절의 tick 범위로 되돌린다.

## 6. HIL 결과

- 마스터/슬레이브 모두 model number 1060, Protocol 2.0, 1 Mbps를 확인했다.
- 마스터 ID 5는 torque OFF read-only, 슬레이브 ID 2만 서버가 소유하도록 했다.
- 30 Hz 연속 추종에서 ±10° 제한은 114 tick 이동으로 확인했다.
- 상대 제한을 4095로 확대한 시험에서는 마스터 약 +790 tick 이동이 슬레이브 목표·현재 위치에
  연속 반영됐다. 이는 1회 명령 후 종료가 아니라 연결 동안 지속 갱신되는 경로임을 확인한 것이다.
- 시험 중 주소 126 최대 관찰값 raw 약 32는 전류가 아니라 약 3.2%의 추정 부하다.
- 모든 시험 종료 후 슬레이브 torque OFF를 재확인했다.
- 추종 HIL 뒤 공식 control table 대조로 Present Load 명칭과 mode-dependent gain 복원 순서를
  수정했다. 최종 소스는 하드웨어 없이 9개 단위 테스트, 새 파일 ament lint, Jetson 격리
  `colcon build`와 console entry point 기동까지 확인했다. 이 정정분 자체의 모터 재구동은
  아직 하지 않았다.

## 7. 정본

- ROBOTIS XL430-W250 e-Manual:
  <https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/>
- ROBOTIS DYNAMIXEL SDK Python 기본 예제:
  <https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/basic_read_write_tutorial/basic_read_write_tutorial_python/>
