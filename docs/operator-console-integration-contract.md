# 공학페스티벌 GUI 연동 계약

작성 기준: 2026-10-04
대상: `operator_console` 임무 화면·협조구동 화면

## 1. 이번 구현 범위

GUI는 **시연용 화면이면서 실제 원격 조종 화면**이다. 조종자는 전방/작업 영상을
보고 이동 방향을 결정하고, GUI는 지형·차체·모터·오류·FSM을 한 화면에 모아 그
판단을 돕는다. 명령은 기존 토큰 인증 ops 채널만 사용하며, 텔레메트리 수신 포트는
계속 관측 전용이다.

이번에 코드로 연결된 항목은 다음과 같다.

- 메인 임무 화면: 전방 주행 판단, 능동제어 상태, Roll/Pitch, 6륜 상태와 상대 Iq
- 협조구동 화면: 지형·자세·접지·FSM 카드, 바퀴별 실측/지령/Iq/조향/오류
- 차체 뷰: 마우스 드래그·휠로 360° 회전, 바퀴 클릭 시 해당 바퀴 데이터 확인
- TEST 더미 파이프라인: 정상 → 험지 접근 → 슬립 보조 → 장애물 정지 시나리오 반복
- 데이터 미수신·1초 초과·필드 미제공은 `정보 없음` 또는 `갱신 지연`으로 표시

3D 차체 뷰는 운용 상태 확인을 위한 경량 엔지니어링 뷰이다. 실제 CAD 형상이나
기구학 애니메이션이 필요하면 CAD 메시와 링크각 계약이 확정된 뒤 별도 렌더러로
교체한다.

## 2. 담당별 입력 계약

모든 각도는 전송 시 rad, 화면 표시 시 degree이다. 거리 단위는 m, 전류는 A,
구동 속도는 wheel turns/s이다. 미측정 값은 0으로 채우지 말고 `null` 또는 필드
미제공으로 보낸다.

| 담당/출처 | GUI에 필요한 값 | 현재 v1 필드 | 완료 조건 |
| --- | --- | --- | --- |
| 서연 · L515 지형인지 | 경로 가능 여부, 신뢰도, 오프셋, 방향 오차, 경사, 뱅크, 거칠기, 거부 이유 | `terrain_path_available`, `terrain_confidence`, `terrain_path_offset_m`, `terrain_heading_error_rad`, `terrain_slope_rad`, `terrain_bank_rad`, `terrain_roughness_m`, `terrain_reject_reasons` | 녹화 입력으로 값/부호/주기 대조 |
| 민경 · 상태추정 | 차체 자세, 슬립·끼임 후보 | `roll_rad`, `pitch_rad`, `yaw_rad`, `slip_candidate`, `stuck_candidate` | IMU 정지/기울임 시험과 화면 대조 |
| 도현 · 협조구동 | 바퀴별 명령, 제어 FSM, 감속 상태와 이유 | `wheel_statuses[].command_turns_per_s`, `controller_fsm_state`, `controller_fsm_reasons`, `degradation_state`, `degradation_reasons`, `degradation_speed_scale` | 균일/협조 시험 로그에서 명령 차이 확인 |
| 광민 · CAN/ODrive | 실측 속도, Iq, 조향각, 모드, stale, 축/조향 오류 | `wheel_statuses[].drive_turns_per_s`, `drive_current_a`, `steer_current_a`, `steer_deg`, `mode`, `stale`, `drive_axis_error`, `steer_fault`; `can_state`, `drive_state` | 6륜 순서·부호 확인, 단선/오류 주입 확인 |
| 임무 로직 | 임무/구간 상태와 이유 | `mission_fsm_state`, `mission_fsm_reason`, `section_fsm_section`, `section_fsm_phase`, `section_fsm_notices` | 상태 전이 로그와 화면 대조 |

바퀴 이름과 표시 순서는
`front_left`, `front_right`, `mid_left`, `mid_right`, `rear_left`, `rear_right`로
고정한다. `Iq`는 접지·끼임 판단을 돕는 **상대 부하 지표**이며 절대 하중이나
소비에너지로 표기하지 않는다.

## 3. 전송·신선도·안전

- 차대 통합 텔레메트리: UDP v1, 기본 포트 `5005`, 최대 4096 bytes
- 갱신 권장: 10 Hz 이상. 마지막 유효 프레임이 1초를 넘으면 운용값을 숨긴다.
- sequence는 송신 프로세스가 살아 있는 동안 단조 증가해야 한다.
- GUI는 원시 Point Cloud를 받지 않는다. 임무 화면에는 지형 처리 결과와 영상
  오버레이만 표시하고, 원시 Point Cloud 확인은 RViz/개발 도구가 담당한다.
- 센서/통신 이상 시 송신 측이 값을 임의 유지하지 말고 stale·오류·거부 이유를
  명시한다. 안전 정지는 로봇 측 인터록이 최종 권한을 가진다.
- 좌표계, +회전 방향, 차체 원점은 첫 실데이터 연동 전에 팀이 확정해야 한다.
  확정 전에는 GUI가 받은 수치를 표시할 뿐 좌표 변환을 추정하지 않는다.

## 4. 아직 팀 합의가 필요한 항목

다음 기능은 화면 자리는 준비할 수 있지만 입력 계약이 없어 완료로 간주하지 않는다.

- 장애물별 위치·거리·높이 목록과 영상 위 도형 오버레이
- 실제 지형과 시뮬레이션 통과 조건을 비교한 영역 단위 passability map
- rocker-bogie 링크각·바퀴 접지 여부와 실제 CAD 메시
- 균일구동/협조구동 비교시험의 실측 결과 저장 형식

비교평가 화면은 장식용 점수를 만들지 않는다. 같은 지형·속도·하중 조건에서
통과 성공률, 통과 시간, 슬립 지속시간, 최대 Roll/Pitch, 바퀴별 Iq를 조건당 5회
이상 기록한 뒤 결과를 연결한다.

## 5. 하드웨어 없는 GUI 연동 시험

두 터미널에서 실행한다. 포트 `15003`~`15008`만 사용하며 실제 제어 명령은
전송하지 않는다.

```bash
python -m operator_console.demo_source
```

```bash
/usr/bin/python3 -m operator_console.app \
  --host 127.0.0.1 --d435-port 15002 --l515-port 15000 \
  --metadata-port 15003 --telemetry-port 15004 \
  --chassis-telemetry-port 15005 --arm-telemetry-port 15007 \
  --environment-telemetry-port 15008 --input-source TEST \
  --ops-token-file /tmp/operator-console-no-token
```

카메라 영상은 별도 SRT fixture가 없으므로 연결 대기로 남고, 나머지 카드와
협조구동 시나리오는 계속 갱신된다. `/tmp/operator-console-no-token`은 존재하지
않는 경로를 사용해 모든 조작을 비활성화한다.

## 6. 근거

- [공학페스티벌 준비 회의록](https://www.notion.so/3e62d27b08d3808a9cd1f3595c21648d)
- [통신 GUI·스트리밍·전원 텔레메트리 — 통합 현황](https://www.notion.so/39d2d27b08d3815c907ae8aa338c5fa8)
- [통신 스트리밍 GUI 분석 — 벤치마킹 기록](https://www.notion.so/3962d27b08d3802e9315e5716c03e887)

위 회의록의 핵심 흐름인 `지형인지 → 상태추정 → 6륜 협조 명령 → CAN/ODrive →
GUI·로그`와 “지형·자세·모터·오류·FSM 표시 및 기본 조작”을 최소 구현 기준으로
삼았다.
