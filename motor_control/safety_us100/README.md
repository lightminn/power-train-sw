# safety_us100 — US-100 충돌방지 안전 모듈

전방 US-100 거리와 UART/MCU liveness를 판정해 구조화된 verdict를
발행합니다. 이 패키지의 `SafetyMonitor`는 모터를 직접 제어하지 않고,
소비자가 `estop_required`와 `status`를 자신의 안전 상태머신에 적용합니다.

## 파일

- `config.py` — 거리 임계, 연속 실패 횟수, UART 설정
- `verdict.py` — `SensorReading`/`Verdict`와 4개 status 상수
- `evaluator.py` — 거리와 status를 E-stop 요청으로 변환
- `safety_monitor.py` — 연속 실패 카운트와 최신 verdict
- `background_monitor.py` — blocking sample을 배경에서 수행하고 stale을 fail-safe로 변환
- `us100.py` — `0x55` 거리 요청과 필요 시 `0x50` liveness 요청
- `fake_sensor.py` — 무하드웨어 테스트 더블
- `demo.py` — 센서 단독 확인
- `teleop_odrive_only.py` — 레거시 ODrive USB 벤치 텔레옵

## Verdict 계약

| status | 의미 | 기본 제어 행동 |
|---|---|---|
| `VALID` | 유효한 거리 | `distance_mm < stop_mm`면 latched `ESTOP`, 아니면 `RUN` |
| `INVALID_READING` | 거리는 무효하지만 UART/MCU liveness 확인 | `RUN`; 기존 latch는 해제하지 않음 |
| `CHECKING` | 시작 중 또는 연속 실패 확정 전 | arm을 유지하고 구동만 0인 `MOTION_HOLD` |
| `NO_RESPONSE` | 연속 liveness 실패 확정 | latched `ESTOP` |

거리가 `stop_mm`와 같을 때는 E-stop이 아닙니다. 배경 작업자가 예외로
샘플을 못 갱신하거나 0.75초 이상 멈추면, 소비자에게
`NO_RESPONSE`, `estop_required=True`를 반환합니다.

`ESTOP`은 hazard가 사라져도 자동 해제되지 않습니다. reset은 `IDLE`로만
이동하며, 모션을 재개하려면 별도 arm 입력이 필요합니다.

## 50 Hz 소비자에서 사용

`SafetyMonitor.tick()`은 serial timeout 동안 blocking될 수 있으므로 50 Hz 루프에서
직접 호출하지 않습니다.

```python
sensor = Us100Sensor(port=cfg.port, baud=cfg.baud)
sensor.open()
monitor = SafetyMonitor(sensor, cfg)
background = BackgroundSafetyMonitor(monitor)
background.start()
try:
    verdict = background.verdict()  # 비차단 스냅샷
    chassis.update_external_safety(
        verdict.status,
        verdict.estop_required,
        verdict.detail,
    )
finally:
    background.close()              # 작업자를 먼저 중지
    sensor.close()                  # 센서는 호출자 소유
```

비-ROS 텔레옵은 위 배경 작업자를 사용합니다. ROS 실행에서는
`powertrain_ros/us100_safety_node.py`가 UART를 독점하고 `/safety_verdict`를
reliable depth 1로 발행합니다.

## 테스트와 데모

```bash
cd /workspace/motor_control
python3 -m pytest safety_us100/tests -v
python3 -m safety_us100.demo
```

실차 차체의 모터 경로는 단일 `can0` 500 kbps에 AK 조향 4개와 ODrive
구동 6개를 공존시키지만, US-100은 별도 UART에서 독립된 충돌 안전
입력으로 동작합니다.
