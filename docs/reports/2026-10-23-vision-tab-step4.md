# 비전 탭 4단계 — 원시 자세·바퀴·엔코더 인터페이스

브랜치 `agent/vision-tab-20261023`, PR 대상 `feat/l515-terrain-map`.
기존 Graphite GUI의 네 번째 영역에 자세·바퀴 상태를 연결했다.
**표시/선택 인터페이스와 로컬 검증까지 완료**했으며, 장착 보정 합의와 실제 AS5048B 송신은 미확인이다.

## 표시 동작

- 기존 self._chassis_receiver :5005를 공유한다. 수신기·소켓을 추가하지 않는다.
  지도 상태는 동일 지도 worker가 준비한 최신 status를 공유하며 GTK에서 지도 snapshot/계산을 하지 않는다.
- IMU 원시 Roll/Pitch는 수신 rad를 deg로만 바꾼다. 19.5° 등을 차감하거나 영점을 추정하지 않는다.
  '원시'는 장착 보정 미확인이라는 의미이며, 센서 가속도 원시값이 아니라
  `/imu/filtered` 추정 quaternion에서 추출한 Euler 값이다.
- 보정된 차체 자세는 **보정 미확인**이다. 개략도는 미보정 IMU로 기울이지 않는다.
  유효한 조향 피드백만 바퀴 방향으로 그리며, 결측/오래된 조향은 ×로 표시한다.
  단순 개략도·팔레트를 사용하고 기존 3D 모델을 새로 만들지 않았다.
- 여섯 바퀴 steer_deg·drive_turns_per_s의 실제 수신값을 표시한다.
  stale인 바퀴와 없는 필드는 숫자를 숨긴다. 지령을 피드백으로 대체하지 않는다.
  저속 Hall 값도 0으로 바꾸지 않으며 '<0.3 참고' 단서와 전체 설명 툴팁을 제공한다.
- 로커·보기 네 각도와 출처·기준·측정 나이 표시를 마련했다.
  실제 송신이 없는 현재는 '미수신'이며, 값이 있어도 측정 나이가 없으면 '시간 미확인'이다.
- 기준면 기울기와 정지 창의 차체 들림/지형 경사를 구분한다.
  정지 기록은 마지막 측정과 n초 전이라는 표시로만 남기고, 주행 자세나 개략도에 적용하지 않는다.

`vision_attitude.py`는 순수 읽기 모델, `vision_attitude_view.py`는 GTK 표시다.
보이는 동안만 200 ms 주기로 갱신하고, 숨기면 UI 타이머를 중지한다. 데이터 수신은 계속한다.
지도 worker는 상태만 100 ms마다 갱신하고 이미지 생성은 500 ms 주기를 유지한다.
2 Hz 이미지 캐시의 수신 시각으로 자세를 판정하면 지속 수신 중에도 순간적으로 stale이 될 수 있어,
가벼운 상태 캐시와 이미지 결과를 분리했다. 추가 스레드나 소켓은 만들지 않는다.
숨김 시 둘 다 중지하며, session/epoch가 바뀌면 이전 지도 이미지는 새 결과 전까지 비운다.
작은 화면의 네 영역을 유지하기 위해 새 자세 패널에만 10 px 글자를 사용하며,
넓은 화면은 12 px로 표시한다. 기존 페이지의 글자/레이아웃은 바꾸지 않았다.
긴 출처·정의는 툴팁으로 보충하며 1100×680에 네 영역이 모두 표시됨을 검사했다.

## 데이터와 신선도

| 값 | 출처 | 나이 기준·유효성 |
|---|---|---|
| 원시 Roll/Pitch | :5005 roll_rad/pitch_rad | 패킷 수신 후 ≤1 s; 원 측정 시각은 없음 |
| 바퀴 조향·속도 | :5005 wheel_statuses[].steer_deg / drive_turns_per_s | 패킷 ≤1 s 및 바퀴별 stale=false; 단위 ° / rev/s |
| 로커·보기 | 선택 suspension_joints_rad | source age + 수신 후 경과 ≤1 s, 패킷도 최신이어야 함 |
| 기준면 앞/옆 | :5010 plane_tilt | age_ms/1000 + 수신 후 경과 ≤1 s, 상태 링크 ≤0.5 s |
| 정지 기록 | :5010 gravity | 동봉 plane_tilt.age_ms로 관측 시각을 계산; 항상 과거 정지 기록 |

미수신·시간 미확인·유효값·오래됨을 구분한다. 패킷 링크가 오래되면 '수신 끊김'으로 표시하며,
새 패킷을 받아도 개별 필드가 null이면 과거 값을 재사용하지 않는다.
수신된 0은 실제 값으로 표시하지만 없는 값을 0으로 채우지 않는다.
엔코더·지도 age는 생산자가 보낸 age와 콘솔 수신 후 경과의 합이다(편도 네트워크 지연은 별도 미측정).
정지 기록은 한 개만 보관하고 지도 session/epoch가 바뀌면 비운다.
gravity에 독립 시각이 없고 plane_tilt도 없으면 임의로 지금 측정했다고 저장하지 않는다.

## AS5048B 선택 필드 계약 — 송신 구현은 아직 없음

노트북 GUI에 있던 suspension_joints_rad / suspension_age_ms를 재사용하고 출처·기준 선택 필드를 추가했다.
schema_version=1의 선택 확장이므로 예전 패킷에 필드가 없어도 정상 수신된다.

| 필드 | 타입 | 의미 |
|---|---|---|
| suspension_joints_rad | object, 기본 {} | 네 논리 관절 각도, 단위 rad; 누락/null 채널은 미수신 |
| suspension_age_ms | number 또는 null | 전송 시점에 포함된 각도 중 가장 오래된 측정의 나이; 음수·NaN·Infinity·bool 거부 |
| suspension_source | string 또는 null | 실제 생산자/센서 또는 시험 데이터 출처; 자동으로 AS5048B라 가정하지 않음 |
| suspension_frame | string 또는 null | 값의 좌표/영점 기준; 제안값 urdf_joint, 실측 합의 미완료 |

| 관절 키 | UI | 제안 기준 |
|---|---|---|
| ml_rocker_joint | 좌 로커 | URDF 해당 joint 기준 |
| mr_rocker_joint | 우 로커 | URDF 해당 joint 기준 |
| bl_bogie_joint | 좌 보기 | URDF 해당 joint 기준 |
| br_bogie_joint | 우 보기 | URDF 해당 joint 기준 |

제안 규약은 **parent joint frame의 +Y 축 오른손 회전**, 영점은 CAD/URDF reference다.
실제 모델의 네 joint axis=[0,1,0], origin rpy=[0,0,0]을 확인했다.
이는 실물 엔코더의 설치 방향·영점이 확인됐다는 뜻이 아니다.
송신 측에서 AS5048B 원시 카운트를 rad로 변환하고 채널별 방향/영점/랩 처리를 확인해야 한다.
콘솔은 수신 각도에 임의 오프셋·좌우 부호 반전·추정 보간을 하지 않는다.
기존 범위 검사 |rad|≤2π를 보존하며, 미갱신 채널을 새 group age로 재발행하면 안 된다.
독립적으로 오래된 채널은 생략/null 처리하거나 group age를 포함 채널의 최대값으로 보낸다.

계약 예시일 뿐 실제 송신 기록은 아니다:

```json
{
  "schema_version": 1,
  "sequence": 1,
  "suspension_joints_rad": {"ml_rocker_joint": 0.0, "br_bogie_joint": -0.1},
  "suspension_age_ms": 100.0,
  "suspension_source": "fixture-only",
  "suspension_frame": "urdf_joint"
}
```

출처/기준 문자열은 1–80자이며 제어문자는 거부한다. 각도·측정 나이 없는 상태를
출처 문자열만으로 유효 측정이나 보정된 자세로 승격하지 않는다.
실제 송신 노드·I2C 드라이버·펌웨어·Gateway는 이번에 변경하지 않았다.

## 보정 조사 결과와 확인 대기

코드 경로는 L515 IMU → imu_tilt_node → /imu/filtered → chassis_telemetry_sender_node → :5005다.

- imu_tilt_node.opt_to_body는 (z,-x,-y) 축 변환을 한다.
  base_link→l515_link 고정 회전은 rotation.w=1.0이다.
  출력 메시지는 base_link로 표기되지만 실제 장착 회전이 보정됐다는 근거로 삼지 않는다.
- state_estimation._update_tilt는 roll_acc=atan2(ay,az),
  pitch_acc=atan2(-ax,hypot(ay,az))를 사용한다.
  지도 slope.body_tilt는 앞/좌 들림을 asin(up_x), asin(up_y)로 산출하며 Euler 쌍이 아니다.
  같은 축의 순수 pitch라는 제한된 가정에서도 부호가 달라질 수 있다.
- 지도는 선택한 바닥 기준면·정지 창 중력으로 계산한다.
  기준면 값은 차체 자체 기울기가 아니고, gravity도 현재 주행 중 자세가 아니다.
- IMU 원 측정 시각은 :5005에 없으며 지도와 동기화된 자세 쌍을 만들 수 없다.
  따라서 숫자 차이·장착 오프셋을 자동 산출하지 않는다. 화면에 '비교 보류'를 명시한다.

**광민·도현과 아직 확인되지 않은 항목**: 실제 19.5°/15°/20° 중 장착 기준,
quaternion 방향과 좌표계·부호, 영점, 장착 회전을 적용할 위치, 보정 출처/버전,
정지 구간의 시간 정합과 기준면 선택, AS5048B 채널별 물리 영점·방향.
팀에 메시지를 보내거나 확인받은 것처럼 처리하지 않았다.
실기 정지 구간의 두 경로 수치 비교는 미검증이며, 합성 19.5°와 정지 기록 3°를
같은 자세의 차이로 계산하지 않는지만 검증했다.

기존 협조구동/3D의 원시 IMU 표시·기울기 적용은 그대로다. mobility_view.py, rover_3d.py는
수정하지 않았고, 해당 경로가 보정된 자세라는 인수도 하지 않는다.

## 검증

- 신규 자세/파서 검사 30건: 미수신≠측정0, 19.5° 원시값 유지, 필드 결측/null,
  실제 속도/바퀴별 stale, encoder age·부분 수신·메타데이터, NaN/Infinity/bool 거부,
  기준면/정지 기록 구분·epoch 초기화·시간 미확인을 검사했다.
- 기존 3D 상태 검사 포함 38건 통과. 자세+격리 비전 UI 집중 검사 35건도 통과했다(중복 포함).
  상태가 이미지 사이에 갱신되고 숨김 시 중지되는 추가 검사도 포함했다.
- 실제 GTK/SRT/UDP E2E: 새 소켓 없이 같은 차대/지도 수신 경로에서
  미수신→원시 IMU/바퀴→선택 encoder→fresh 패킷의 null/오래된 측정→끊김→복구,
  정지 기록 유지, 숨김 중 UI 중지/수신 지속, 1100×680 배치를 확인했다.
- 미수신을 0/LIVE로 만드는 고장을 메모리에 주입하자 미수신 검사에서 예상 FAIL했다.
- 기존 실제 GTK/WebKit 3D 스모크 PASS: 차체/팔·바퀴 팝업·줌 제한·stale 회귀를 확인했다.
- 최종 전체 콘솔 검사는 **841 passed / 10 failed / 3 skipped**였다.
  실패 10개는 이전 단계의 동일 로봇팔 계약/경계/runtime_smoke 항목이다.
  필수 runtime_smoke도 기존 arm safe-disabled 실패를 유지한다. 전체 green은 아니다.

마지막 상태 캐시/지도 표시 보강 후 비전 UI 5건, 자세 E2E+상태 캐시 갱신 2건도 통과했다.
원시값을 차체에 적용하거나 장착 보정을 완료한 것으로 판정하지 않는다.

- [전체 검사](2026-10-23-vision-tab-step4-evidence/full-tests.log)
- [이전 단계 대비 실패 ID](2026-10-23-vision-tab-step4-evidence/test-comparison.json)
- [집중 검사](2026-10-23-vision-tab-step4-evidence/focused-tests.log)
- [최종 비전 UI](2026-10-23-vision-tab-step4-evidence/vision-ui-tests.log)
- [자세/상태 캐시 E2E](2026-10-23-vision-tab-step4-evidence/status-attitude-e2e.log)
- [미수신을 0으로 만드는 고장 음성 대조](2026-10-23-vision-tab-step4-evidence/missing-zero-negative.log)
- [기존 3D 스모크](2026-10-23-vision-tab-step4-evidence/rover-3d-smoke.log)
- [필수 runtime_smoke](2026-10-23-vision-tab-step4-evidence/runtime-smoke.log)

로그는 줄 끝 공백과 마지막 빈 줄만 정리해 보관했다.

```bash
PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros" \
  GDK_BACKEND=x11 xvfb-run -a /usr/bin/python3 -m pytest operator_console -q
/usr/bin/python3 -m operator_console.runtime_smoke
GDK_BACKEND=x11 xvfb-run -a /usr/bin/python3 -m operator_console.rover_3d_smoke
```

## 화면·증거와 미검증

- [엔코더 미수신 화면](2026-10-23-vision-tab-step4-evidence/missing-encoders.png)
- [선택 필드 합성 수신 화면](2026-10-23-vision-tab-step4-evidence/fixture-encoders.png)
- [끊김 화면](2026-10-23-vision-tab-step4-evidence/stale.png)

세 화면은 모두 TEST/더미 시연이며 실측 화면이 아니다. 각 화면의 상태 JSON도 함께 보관했다.
실물 AS5048B 연결, 실제 장착 보정, 실기 정지 자세 비교, 영상 성능/CPU/메모리의 장시간 인수,
Gateway+L515 10분 운용은 미검증이다. Jetson은 호스트 이름 해석 실패로 접속하지 못했다.

다음은 5단계의 BLAS 실행 경로 통일·글자/최신 프레임·성능 계측이다.
보정 합의·실측 자료가 전달되면 해당 계약을 별도로 확인해야 한다.
PR·push·로봇 배포는 수행하지 않았다.
