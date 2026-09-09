# corner_module — 코너 모듈 컨트롤러

로커보기 코너 1개(조향 AK45-36 + 구동 BL70200/ODrive)의 협조 제어
라이브러리와 DualSense 데모입니다. 설계는
`docs/specs/2026-05-25-corner-module-controller-design.md`에 있습니다.

## 구성

- `config.py` — `CornerConfig`(한계·워치독·게이트), `clamp`
- `actuator.py` — `Actuator`/`SteerActuator`/`DriveActuator` 인터페이스
- `corner_module.py` — `CornerModule` 상태머신·안전·협조 제어
- `steer_ak40.py` — AK45-36 CAN 조향 드라이버(클래스명은 레거시 AK40)
- `drive_odrive_can.py` — 현재 10모터 차체용 ODrive CAN 구동 드라이버
- `drive_odrive_usb.py` — 무명 보드 선택 때문에 실행이 차단된 레거시 USB 드라이버
- `drive_odrive_usb_axis.py` — registry serial/axis/node로 연결하는 USB 스키드 드라이버; CAN과 모터 소유권 공유
- `fake.py` — 무하드웨어 테스트 더블
- `teleop_dualsense.py` — 레거시 실행은 하드스톱; 순수 입력·안전 헬퍼만 유지

## 단위와 현재 CAN

조향은 출력축 도(°), 구동은 turns/s입니다.
`v = turns/s × 2π × 0.1` 로 m/s를 계산합니다.

실차 차체 경로는 단일 `can0` 500 kbps에서 AK45-36 조향 4개(ID 1–4)와
ODrive 구동 6개(node 11–16)를 개별 제어합니다. 이 README의 단일 코너
텔레옵은 조향 `can0` + 구동 USB 벤치 경로입니다.

## 테스트

```bash
cd /workspace/motor_control
python3 -m pytest corner_module/tests -v
```

## 텔레옵 실행

기존 단일 코너 텔레옵은 이름 없는 첫 USB 보드를 제어하므로 기동 전에 차단한다.
현재 운용은 [통합 운용 정본](../../docs/integrated-operation.md)의
`scripts/robot-start`와 운용 콘솔을 따른다. USB 스키드는 기존 board registry를
사용하며, 풀의 마지막 축이 종료될 때까지 CAN과 동일한 모터 락을 유지한다.
캘리·NVM 정비는 `motor_control/drive/bl70200/bl70200_setup.py`의
`--serial <SERIAL> --axis both --node 11`처럼 대상을 모두 명시한다.

## US-100 안전 상태와 latch

센서 통신은 `BackgroundSafetyMonitor`가 배경에서 실행하므로 50 Hz 제어
루프에서 blocking serial을 호출하지 않습니다.

- `VALID`: 200 mm 미만이면 `estop_required=True`로 컴포넌트 `FAULT`를 latch합니다.
- `INVALID_READING`: UART/MCU liveness는 확인된 정상 응답으로, 자동 정지하지
  않습니다.
- `CHECKING`: 컴포넌트를 fault하지 않고 구동 명령만 0으로 게이팅합니다.
  차체 제어의 `MOTION_HOLD`와 같은 일시 정지 의미입니다.
- `NO_RESPONSE`: 연속 liveness 실패 또는 0.75초 이상 stale한 배경 verdict이며,
  `FAULT`/E-stop을 latch합니다.

멀어진 `VALID`이나 `INVALID_READING`은 latch를 자동으로 풀지 않습니다.
첫 □는 `reset_fault()`로 `IDLE`만 만들고, 다음 □에서만 arm합니다.
활성 hazard 중 reset은 거부되며, ○는 수동 `FAULT`를 latch합니다.

4WS 키네마틱스와 6코너 통합은 `chassis` 패키지가 담당합니다.
