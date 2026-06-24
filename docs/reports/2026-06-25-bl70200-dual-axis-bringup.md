# BL70200 듀얼축(M0+M1) ODrive 브링업 — 세션 핸드오프

> 작성 2026-06-25 (작업: 2026-06-24 밤~25 새벽). 대상: **다른 Claude 세션이 이 컨텍스트를 그대로 물려받아 이어가기 위한 문서.**
> 한 줄 요약: **하나의 ODrive v3.6(fw 0.5.1)에 동일한 BL70200 인휠모터 2개(M0=axis0, M1=axis1)를 물려, 각각 캘리브레이션 + 위치모드 독립 제어까지 검증 완료.** 둘 다 정상 동작.

---

## 0. 결론 (TL;DR)
- M0·M1 **둘 다 캘리 성공 + 독립 제어 확인**(위치모드 ±1바퀴: M0단독 / M1단독 / 동시 반대방향, 전부 err 0x0).
- 두 모터는 **완전 동일**(같은 BL70200, 같은 결선, 같은 설정, pp=10). 차이 없음.
- 오래 헤맨 진짜 원인 2개: **(A) `calib_scan_omega` 기본값 함정, (B) 캘리 반복실패로 ODrive가 latched 열화상태에 빠져 HALL 카운터 폭주 → 전원 사이클로 해소.**

---

## 1. 환경 / 실행 방법
- 하드웨어: ODrive v3.6, **MKS 56V, fw 0.5.1**, USB로 Jetson 연결, DC **48V**(vbus ≈ 47.7V).
- 모터: BL70200 인휠 BLDC ×2, 내장 HALL ×3, **pp=10 / HALL cpr=60**. M0=axis0, M1=axis1.
- 런타임: **Jetson 컨테이너 `powertrain_jetson` 안에서 python 실행**(host엔 odrive/python-can 없음). repo는 컨테이너 `/workspace`.
- 실행 패턴(랩탑 x86에서 원격):
  ```bash
  sshpass -p "$JETSON_SSH_PASS" ssh jetson 'docker exec -i powertrain_jetson python3 -' < script.py
  ```
  (`jetson` = `~/.ssh/config`의 `jetson-orin.local`, user `zetin`. `$JETSON_SSH_PASS`는 셸 env에 주입됨.)
- odrive 라이브러리 = git `fw-v0.5.6`(=0.5.6) 소스. **enum은 flat 상수 사용**(`AXIS_STATE_*`, `MOTOR_TYPE_*` 등) — 네임스페이스 enum(`AxisState.X`)은 이 빌드에서 IntEnum이 아니라 대입 시 `TypeError`. `current_state` 비교도 flat 정수로.
- 컨테이너 실행 시 매번 `git describe ... non-zero exit status 128` 노이즈 1줄 나옴(무해, 무시).

---

## 2. 현재 하드웨어 상태 (NVM 저장됨)
양축 동일하게 NVM에 저장된 값(공장초기화 후 재설정 완료):

| 항목 | 값 |
|---|---|
| motor_type / pole_pairs / current_lim | HIGH_CURRENT / **10** / 9 A |
| calibration_current / resist_calib_max_V | 8 A / 5 V |
| encoder mode / cpr / bandwidth | HALL / **60** / 30 |
| **calib_scan_distance / calib_scan_omega / calib_range** | 150 / **6.0** / 0.05 |
| controller gains pos/vel/vel_int | 2.0 / 0.06 / 0.2 |
| input_filter_bw / vel_limit / vel_ramp | 2.0 / 50 / 2 |
| control_mode / input_mode | VELOCITY / VEL_RAMP (테스트 땐 POSITION/POS_FILTER로 전환) |
| board UV / OV / brake | 40 / 56 V / 2.0 Ω |
| CAN baud / node (M1 / M0) | 500000 / 11 / 12 |

**RAM에만 있는 것(리부팅 시 소실 → 재설정 필요):**
- 캘리 결과(offset/direction/R/L) — `pre_calibrated=False`라 매 부팅 재캘리.
- `encoder.config.ignore_illegal_hall_state = True` — 이번 세션에서 RAM에만 켬(저장 안 함).

---

## 3. 오늘 발견한 함정 (★중요)

### (A) `calib_scan_omega` 기본값이 OFFSET 캘리를 깨뜨림
- 새로 단 M0은 `calib_scan_omega=12.566`(기본), M1은 원셋업 때 **6.0**이었음.
- 증상: M0만 OFFSET 캘리에서 `encoder.error 0x2 = CPR_POLEPAIRS_MISMATCH`로 15~30s에 조기중단. (HALL polarity 단계는 통과, **offset 단계만** 실패.)
- 원인: omega가 빠르면 오픈루프 락인 스캔에서 카운트 정합이 깨짐. **신규 축은 calib_scan_omega를 6.0으로 맞출 것.**
- ⚠️ `calib_range`(허용오차)를 0.5까지 늘려도 무효였음 — offset 카운트 폭주(아래 B)가 진짜 원인이라 omega/전원이 레버지 range가 아님.

### (B) ★최대 함정: 캘리 반복 실패 → ODrive latched 열화 → HALL 카운터 폭주 → **전원 사이클로만 해소**
- 증상: 캘리 실패를 반복할수록 `encoder.shadow_count` 변동폭이 **실 HALL 전이수의 49→58→231→452배로 단조 증가**(실전이 ~145인데 shadow가 7천~6.5만). ODrive가 "cpr/pp 50배 안 맞음"으로 `0x2`를 띄움.
- **무전원 IDLE에선 shadow_count 변동 0(완전 깨끗)** → 노이즈는 전류 흐를 때(모터 통전)만.
- 시도해서 **다 안 통한 것**: 전류 ↑(15A)·↓(5A), 스캔거리 ↓(30/120), omega ↓(3.0), calib_range ↑(0.5), 공장초기화(`erase_configuration`)+재설정, **소프트 리부팅**.
- **통한 것: 물리 전원 OFF/ON(+잠깐 식힘).** 그 직후 양축 즉시 깨끗 캘리(55s, err 0x0, shadow변동 205 ≈ 실전이 290).
- 해석: 반복 캘리(~12회)의 **열 누적**으로 한계 접점/회로가 누적 열화한 것으로 추정. 소프트 리부팅으론 안 풀리고 **전원 끄고 식혀야** 풀림 → "끄는 행위"보다 **식는 것**이 핵심.
- **운용 규칙**: 캘리 1~2회 실패하면 연사 금지. shadow_count 폭주/CPR mismatch가 파라미터 무관하게 지속되면 **파라미터 만지지 말고 전원부터 내렸다 켤 것.**

### (C) 회전 중 간헐 트립 = `ILLEGAL_HALL_STATE`
- 폐루프 회전 중 `axis.error 0x100 (ENCODER_FAILED)` = `encoder.error 0x10 (ILLEGAL_HALL_STATE)`로 간헐 트립(HALL이 순간 불법상태 000/111 읽음).
- 간헐적 — 에러 클리어 후 재시도하면 됨. **완화: `encoder.config.ignore_illegal_hall_state=True`**(직전 유효상태 유지, 트립 안 함). 적용 후 독립제어 3종 전부 err 0x0.
- 어느 축이 트립할지 고정 아님(둘 다 marginal). **근본 해결은 HW**: HALL 라인 접지 보강 / 필터캡 22~47nF(각 HALL 라인→GND) / 커넥터 단단히.

### (D) 기타 빌드 특성
- `odrv.config.gpioN_mode` **이 보드/펌웨어엔 없음**(AttributeError). GPIO 모드 설정 불가/불필요. 레포 `odrive_diff_drive_test.py`의 gpio9_mode 출력은 이 보드에서 에러남.
- `save_configuration()`은 이 빌드에서 호출 시 reboot(ObjectLostError 던질 수 있음, 정상) → `time.sleep(6~8)` 후 `find_any` 재연결.

---

## 4. 검증된 절차 (재현용)

### 4.1 캘리 (USB, 출력축 자유 필수)
```python
ax.error = ax.motor.error = ax.encoder.error = ax.controller.error = 0
ax.motor.config.calibration_current = 8.0
ax.motor.config.current_lim = 20.0          # 캘리 헤드룸
ax.encoder.config.calib_scan_omega = 6.0    # ★ 필수
ax.encoder.config.calib_scan_distance = 150
ax.encoder.config.calib_range = 0.05
ax.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE   # flat 상수
# current_state != AXIS_STATE_IDLE 동안 대기, timeout >= 120s
# 성공 = motor.is_calibrated and encoder.is_ready, err 0x0, ~55s
ax.motor.config.current_lim = 9.0           # 운용값 복귀
```
- shadow_count 변동폭이 실 hall 전이수와 비슷(수백)하면 정상. 수천~수만이면 폭주 → 전원 사이클.

### 4.2 독립 위치 제어 (한 방향 ≤2바퀴 준수)
```python
for ax in (a0, a1):
    ax.encoder.config.ignore_illegal_hall_state = True
    ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    ax.controller.config.input_mode = INPUT_MODE_POS_FILTER
# 점프 방지: 폐루프 진입 전 input_pos = 현재 pos_estimate
s0, s1 = a0.encoder.pos_estimate, a1.encoder.pos_estimate
a0.controller.input_pos = s0; a1.controller.input_pos = s1
a0.requested_state = a1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL  # state==8 확인
# 이후 input_pos = s + Δ (Δ ≤ 1.0). 끝나면 원위치 복귀 후 AXIS_STATE_IDLE.
```
검증 결과: M0단독(+1, M1고정) ✅ / M1단독(+1, M0고정) ✅ / 동시 M0+1·M1−1 ✅, 전부 err 0x0. 도달 Δ≈0.95(HALL 저해상도 정상상태 오차 ~0.05, Notion 기록과 일치).

---

## 5. 사용자 제약 / 주의
- **"한 방향으로 2바퀴 이상 회전 금지"** — 단, 캘리 스캔(2.5바퀴)은 예외 허용받음. 캘리 외 속도/토크/위치 제어는 ≤2바퀴 준수.
- **"속도제어·토크제어는 사용자 허락 없이 멋대로 하지 말 것."** 위치모드 ±1바퀴는 명시적 허락받아 수행함.

---

## 6. 미해결 / 다음 할 일
1. **HALL 신뢰성 HW 보강**(권장 1순위): 접지/필터캡/커넥터 — 안 하면 실주행 진동·부하에서 `ILLEGAL_HALL_STATE` 재발 가능. `ignore_illegal_hall_state`는 밴드에이드.
2. **영구화 선택**: `ignore_illegal_hall_state`를 NVM 저장하려면 save→리부팅→재캘리 1회. (현재 RAM only.)
3. **레포 정리(미완)**: `motor_control/drive/bl70200/odrive_calibration.py`가 아직 **단일축 + 틀린 pp=5**. 이번 검증값(듀얼축 / pp=10 / calib_scan_omega=6.0 / ignore_illegal_hall_state)으로 갱신 필요. Notion "ODrive(BL70200) 셋업" 페이지도 듀얼축/omega 추가 갱신 필요.
4. 미래: 4WS 코너모듈에서 이 두 구동축을 조향 AK와 단일 CAN으로 협조 제어(메모리 `corner-module-ackermann` 참고).

---

## 7. 참고
- Notion: "ODrive(BL70200) 셋업 — 공장초기화→구동" (3882d27b08d381fcbe3cd0c829687c3a) — 단일축 기준, 본 문서가 듀얼축 확장.
- 프로젝트 메모리: `bl70200-odrive-jetson-bringup.md`(듀얼축 섹션 추가됨).
- 관련 메모리: `jetson-docker-motor-control`, `ak-can-500k-50hz`, `corner-module-ackermann`.
