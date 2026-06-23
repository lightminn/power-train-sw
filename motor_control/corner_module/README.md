# corner_module — 코너 모듈 컨트롤러

로커보기 코너 1개(조향 AK40 + 구동 ODrive 3.6)의 협조 제어 라이브러리 + DualSense 데모.
설계: `docs/specs/2026-05-25-corner-module-controller-design.md`.

## 구성
- `config.py` — CornerConfig(한계·워치독·게이트), clamp
- `actuator.py` — Actuator/SteerActuator/DriveActuator 인터페이스
- `corner_module.py` — CornerModule (상태머신·안전·협조)
- `steer_ak40.py` — AK40(CAN) 조향 드라이버
- `drive_odrive_usb.py` — ODrive(USB) 구동 드라이버 (현재)
- `drive_odrive_can.py` — ODrive(CAN) 구동 (미래 CAN-only 전환 슬롯)
- `fake.py` — 무하드웨어 테스트 더블
- `teleop_dualsense.py` — DualSense 텔레옵 데모

## 단위
조향 = 출력축 도(°), 구동 = turns/s. (m/s 변환: `v = turns/s × 2π × 0.1`)

## 테스트 (x86 dev 컨테이너에서)
```bash
cd /home/light/Defence_Robot/motor_control
python -m pytest corner_module/tests/ -v
```

## 텔레옵 실행 (Jetson, 실모터)
```bash
bash scripts/can_setup.sh            # can0 500kbps 기동 (조향)
# ODrive USB 는 init_odrive.py 로 1회 NVM 셋업 가정
cd /home/light/Defence_Robot/motor_control
python3 -m corner_module.teleop_dualsense   # 패키지 모듈로 실행(직접 .py 실행은 import 깨짐)
# 조향모터 CAN id 가 10이 아니면: AK_MOTOR_ID=<id> python3 -m corner_module.teleop_dualsense
```
□=arm/disarm, ○=estop, 좌스틱 X=조향, RT/LT=전/후진.
DualSense 축/버튼 매핑(HIL 검증): 좌스틱X=axis0, RT=axis4, LT=axis3, □=btn0, ○=btn2.
헤드리스(컨테이너)에서는 SDL 더미 드라이버를 main()이 자동 설정.
US-100 충돌방지(`safety_us100`) 연동: `stop` 판정 시 구동을 0으로 막는다(센서 미연결이면 항상 `stop` → 구동 안 함).

## 미래 (본 라이브러리 범위 밖)
- `drive_odrive_can.py` 구현 (CAN-only 전환)
- 4WS 애커만 키네마틱스 레이어 — 여러 CornerModule 의 소비자
- motor_gui 어댑터 — `state()` dict 를 텔레메트리로 노출
