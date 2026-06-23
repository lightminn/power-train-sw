# 모터 제어 게인 튜닝 가이드 (motor_gui)

날짜: 2026-05-20
대상: `motor_gui` 웹 진단 GUI 로 ODrive 구동 모터의 최적 제어 계수를 찾는 절차.
독자: 이 프로젝트를 처음 보는 엔지니어 / 다른 Claude 세션. **self-contained** 로 작성.

관련: `docs/specs/2026-05-20-motor-gui-design.md`, `motor_gui/README.md`,
도구: `motor_gui/tools/gain_sweep.py`.

---

## 0. 한 줄 요약

ODrive 는 **캐스케이드 P-PI 제어기**다. GUI 의 튜닝 입력칸(또는 `gain_sweep.py`)으로
게인을 바꿔가며 **위치 스텝 응답**을 보고, **정착 오차 / 한계진동(움찔) / 트립** 세 지표로
최적값을 고른다. 잔차가 pos_gain 으로 안 줄면 그건 **코깅**이니 anticogging 캘리로 잡는다.

---

## 1. ODrive 제어 구조 (캐스케이드)

```
input_pos ─►[POS_FILTER: input_filter_bandwidth]─► pos_setpoint
   pos_setpoint ─►[위치 루프 P: pos_gain]─► vel_setpoint
      vel_setpoint ─►[속도 루프 PI: vel_gain(P), vel_integrator_gain(I)]─► torque
         torque ─►[전류 루프 PI: 자동(캘리)]─► 모터
   한계: vel_limit (rev/s), current_lim (A)
```

- **pos_gain** — 위치 오차 → 속도 명령. 높이면 위치 빠릿하지만 너무 높으면 hunting.
  *주의: 마찰/코깅이 있으면 pos_gain 을 올려도 정착 잔차가 안 줄거나 오히려 커진다(코깅).*
- **vel_gain** — 속도 루프의 비례 게인(=속도 강성). 너무 높으면 **불안정→트립**.
- **vel_integrator_gain** — 속도 루프 적분. 정상상태 속도 오차 제거용이지만, 마찰/코깅이
  크면 **와인드업 → 한계진동(수 초 주기로 앞뒤 움찔)** 을 유발. 이 경우 0 이 최선.
- **input_filter_bandwidth** — POS_FILTER 대역폭(Hz). 낮으면 위치 명령이 느릿느릿
  슬루(점대점에 부적합), 높이면 즉답. 비전 추종처럼 부드러운 연속추종엔 낮게(예 2),
  점대점 위치 테스트엔 높게(예 50~75).
- **vel_limit / current_lim** — 안전 한계. 위치 이동 속도는 vel_limit 에 걸린다.

> CAN(CANSimple) 트랙은 `input_filter_bandwidth` 튜닝 명령이 없어 GUI 튜닝칸에서 제외된다.
> 게인 readback 명령도 없어 CAN 은 prefill 이 마지막 전송값/baseline 기준이다.

---

## 2. 측정 지표 3가지 (스텝 응답)

영점(set_origin) 잡고 `pos = step`(예 +2.0 turn) 명령 후 `settle` 초 관찰:

1. **정착 오차** = `mean(마지막 0.5s 위치) − step`. 0 에 가까울수록 좋음.
2. **한계진동 p2p** = `max−min(마지막 3s 위치)`. 크면 "움찔/hunting" (보통 적분기 windup).
3. **트립** = `axis_err != 0` 또는 `current_state != 8(CLOSED_LOOP)`. 게인 과도/불안정.

판정: 오차<0.03 & p2p<0.02 → OK / p2p 큼 → 진동 / 오차만 큼 → 잔차 / 트립.

---

## 3. 안전·운영 원칙 (반드시)

- **read-only 먼저**: 통전 전 `sample()`(vbus/state/pos/temp/err) + config 덤프 + `dump_errors`
  로 연결·상태 확인. 모터 움직이는 명령은 그 다음.
- **모터 자유 회전 확인** 후에만 모션 테스트 (부하·간섭 없이 돌 수 있어야).
- **폐루프 진입 전 `input_pos = 현재 위치`** 로 맞춰 점프(runaway) 방지. (코드가 이미 처리)
- **ODrive USB 는 한 프로세스만 점유**. `gain_sweep.py` 돌리려면 **GUI 서버 먼저 종료**:
  `docker compose -f docker/docker-compose.jetson.yml exec powertrain pkill -f motor_gui.backend.server`
- **스윕은 콤보마다 `clear_errors` + 폐루프 재진입**. 안 그러면 한 콤보가 트립한 뒤
  축이 디스암된 채 남아 이후 콤보가 전부 "안 움직임(−step)" 으로 나온다 (실제 함정이었음).
- E-stop / Ctrl-C 시 IDLE 로 안전 정지.

---

## 4. 두 가지 튜닝 경로

### (A) GUI 라이브 튜닝 — 사람이 직접
1. Jetson 컨테이너에서 서버 기동: `python3 -m motor_gui.backend.server --track usb`
   (CAN: 먼저 `bash scripts/can_setup.sh`, 그 후 `--track can`)
2. 노트북 브라우저 `http://jetson-orin.local:8000`.
3. **폐루프 진입** → (원하는 물리 위치로 둔 뒤) **영점 설정** → 제어 모드 `position`.
4. 튜닝 칸(현재값 prefill 됨)에서 값 바꿔 Enter → **plot 으로 응답 비교**.
5. 목표값 0↔step 왕복하며 정착·움찔 관찰. 만족스러우면 그 값을 baseline 으로 반영(7장).
6. 잔차가 남으면 **anticogging 캘리** 버튼 1회 (폐루프 position 상태에서, 모터가 한 바퀴
   천천히 스윕하며 코깅맵 생성 → 이후 정밀 정착).

### (B) 자동 스윕 — `gain_sweep.py`
정량 비교가 필요할 때. 위 안전원칙대로 서버 종료 + 모터 자유회전 후:
```bash
docker compose -f docker/docker-compose.jetson.yml exec -T powertrain \
  bash -lc "cd /workspace && python3 motor_gui/tools/gain_sweep.py --track usb --step 2.0 --bw 50"
```
- 파일 상단 `COMBOS` 리스트를 그 모터에 맞게 편집.
- 출력 표에서 `OK`(오차·진동 모두 작음) 또는 `잔차`가 가장 작은 조합 선택.
- `--track fake` 로 하드웨어 없이 도구 동작만 검증 가능.

---

## 5. 결과 해석 → 다음 액션

| 관찰 | 해석 | 액션 |
| --- | --- | --- |
| 트립(0x…)이 vel_gain 올릴 때 발생 | 속도 루프 불안정 한계 | vel_gain 을 트립값 아래로 (보수적으로 ~70%) |
| 수 초 주기로 앞뒤 움찔(p2p 큼) | vel_integrator_gain 와인드업 한계진동 | vel_integrator_gain ↓ (0 까지) |
| 잔차가 pos_gain 올려도 안 줄거나 커짐 | **코깅 detent** (P-droop 아님) | fw0.5.1 anticogging 은 불안정(아래 ⚠) — 실용상 잔차 수용 |
| 잔차가 pos_gain 올리면 줄어듦 | 단순 P-droop | pos_gain ↑ (hunting 직전까지) |
| 위치 명령이 느릿느릿 도달 | input_filter_bandwidth 너무 낮음 | bw ↑ (50~75) |

---

## 6. 이 프로젝트 모터별 실측 결과

### SunnySky X2212-13 + TLE5012B (테스트 모터, USB, fw 0.5.1, hw 3.6)
- HW: pole_pairs=7, cpr=16384, R≈0.085Ω, L≈15.5µH, torque_const≈0.04, motor_type=HIGH_CURRENT.
- 스윕 발견:
  - `vel_gain > 0.05` 면 **트립**(0x200 등) → 0.015 유지(거의 안정 한계).
  - `vel_integrator_gain` 0.25 → 3초 주기 움찔 + 잔차 0.21 / **0 → 움찔 소멸 + 잔차 0.077(최소)** /
    0.5 → 트립.
  - `pos_gain` 8→60 올려도 잔차 0.077→0.18 로 **오히려 증가 = 코깅 detent** (anticogging 필요).
  - `input_filter_bandwidth` 2.0(odrive_can_setup 의 비전추종용)은 점대점에 너무 느림 → 50~75.
- **확정 baseline (`DEFAULT_TUNABLES`)**: pos_gain=8, vel_gain=0.015, vel_integrator_gain=0,
  input_filter_bandwidth=50, vel_limit=5, current_lim=10. 잔차 ~0.08turn 은 실용 한계로 수용.

> ⚠️ **anticogging 캘리는 이 fw(0.5.1)에서 쓰지 말 것.** `start_anticogging_calibration` 이
> 불완전하게 끝나며 `controller.config.anticogging.anticogging_enabled=True` + 무효 맵 상태로
> 남아 **폐루프 모션이 완전히 막힘**(명령 줘도 모터 안 움직임, 에러는 안 뜸). GUI 에서 기능 제거함.
> **사고 시 복구**: ODrive 연결 → `axis.controller.config.anticogging.anticogging_enabled=False`,
> `pre_calibrated=False` → `save_configuration()` → `reboot()` → IDLE 대기 후 폐루프 재진입하면
> 정상 복구. (코깅 보상이 꼭 필요하면 fw 업그레이드 + 신중한 별도 검증 후 재시도.)

### BL70200 + 내장 HALL ×3 (실전 구동 모터)
- 아직 motor_gui 로 튜닝 안 함. HALL(pp=5,cpr=30)이라 X2212 와 게인 스케일이 완전히 다름.
  교체 시 아래 7장 절차로 재튜닝 필수. (별도 캘리: `motor_control/drive/bl70200/`)

### AK45-36 (조향, CAN servo)
- ODrive 아님(CubeMars). 별도 프로토콜(`motor_control/steering/ak_control.py`), 본 가이드 범위 밖.

---

## 7. 새 모터로 바꿨을 때 재튜닝 절차

1. **HW 파라미터 셋업**: 그 모터의 캘리 스크립트로 pole_pairs/cpr/encoder mode/motor_type 설정
   + 캘리(`is_calibrated`/`is_ready` True) + NVM 저장. (X2212: `drive/x2212_test/init_odrive.py`
   또는 `odrive_can_setup.py`. BL70200: `drive/bl70200/odrive_calibration.py`.)
2. **read-only 확인**: GUI/스크립트로 sample + config 덤프 + dump_errors. vbus/state/encoder 정상?
3. **보수적 시작**: 그 모터 캘리 스크립트의 게인을 출발점으로.
4. **스윕(자유회전 확인 후)**:
   a. vel_gain 단독으로 키워 **트립 한계** 찾기 → 그 70% 로 고정.
   b. vel_integrator_gain 0 부터 → 움찔 없는 최대 (보통 0).
   c. pos_gain 스윕 → 잔차 최소 & 진동 없는 값. (잔차가 안 줄면 코깅 → 다음)
   d. input_filter_bandwidth 로 점대점 응답 속도 조정(50~75).
5. **anticogging 캘리**(폐루프 position) → 잔차 제거. NVM 저장.
6. **baseline 반영**: `motor_gui/backend/transport/base.py` 의 `DEFAULT_TUNABLES` 를 그 모터
   최적값으로 수정. (현재는 단일 전역 dict = X2212 기준. 여러 모터를 동시에 운용하려면
   트랙/모터별 baseline 으로 분기하도록 확장 필요 — 지금은 USB=X2212 전제.)
7. 이 문서 6장에 그 모터 실측 결과를 추가.

---

## 8. 코드 지도 (어디를 보면 되나)

- 게인/한계/모드/영점/anticogging 적용: `motor_gui/backend/transport/usb_odrive.py`
  (`apply()` 의 set_gain/set_limit/set_mode/set_origin/anticogging), CAN 은 `can_bus.py`.
- baseline 기본값 + UI 메타(tunables/inputs/signal_meta): `transport/base.py`
  (`DEFAULT_TUNABLES`, `ODRIVE_TUNABLES_USB/CAN`, `SIGNAL_META`).
- startup baseline 적용 + prefill: `backend/worker.py` (`_apply_baseline`, `tunables()`).
- 튜닝 입력칸 prefill / 영점 / anticogging 버튼: `frontend/app.js` (`controlPanel`).
- 스윕 도구: `motor_gui/tools/gain_sweep.py` (COMBOS 편집 + `--track`).

## 9. 핵심 함정 (다시 강조)
- 스윕 콤보마다 clear_errors + 폐루프 재진입 안 하면 트립 후 전부 0 으로 나온다.
- ODrive USB 는 한 프로세스 점유 — 서버와 스윕 동시 실행 불가.
- 잔차를 pos_gain 으로만 잡으려 하지 말 것 — 코깅이면 anticogging.
- vel_integrator_gain 은 마찰 큰 모터에서 움찔 주범 — 의심되면 0 부터.
- fw 별 속성 경로 다름(예 fw0.5.1 은 `axis.fet_thermistor`) — transport 가 robust 하게 resolve.
