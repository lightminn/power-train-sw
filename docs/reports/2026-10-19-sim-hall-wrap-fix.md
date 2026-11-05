# Isaac 시뮬 HallPLL 각도 랩 결함 수정

2026-10-19. 대상: `simulation/jetin_rover_description/tools/manufacturer_models.py`의 `HallPLL`.
2026-10-11 보관한 전달본(`docs/reports/2026-10-11-jetin-isaac-archive.md`)의 시뮬 모델 결함이다.
실차 제어 코드와 Jetson 배포에는 영향이 없다.

## 결함

- Isaac 5.1 articulation은 바퀴 회전 관절각을 [−2π, 2π)로 감아서 보고한다. 지형 지도 기록 run-4138에서 +6.23 rad 다음 값이 −6.26 rad였다.
- `HallPLL.update`는 받은 각에 감속비 5와 60 count/회전을 곱해 Hall 카운트로 바꾼다. 그래서 −4π 점프가 1/120 s 사이 약 −600 count로 들어갔다. 실제 Hall 센서는 카운트를 이어서 세므로 이런 점프를 볼 수 없다.
- PLL 속도 추정은 약 −14 m/s로 튀었다. 실제 속도는 +0.15 m/s였다.
- 호출처는 두 곳이며 둘 다 관절각 원값을 넣는다.
  - `bl70200_controller.py:91`(바퀴 속도 제어 피드백): 모터 PI가 14.3 N·m로 포화돼 바퀴가 6.5 m/s로 헛돌고 로버가 앞으로 튀었다.
  - `motion_feedback.py:36`(ROS `/jetin/wheel_steering_feedback`): 같은 가짜 속도가 바깥으로 나갔다.
- 6바퀴가 같은 시점(t≈7.9–8.1 s)에 2π를 넘어 한꺼번에 일어났다. 무슬립 바퀴 오도메트리는 실제보다 약 0.29 m 더 전진한 것으로 계산했고, 지형 지도에서 언덕 봉우리가 두 겹으로 보였다.

## 수정

- `HallPLL`이 직전 원입력 `raw_angle`과 `turn_offset`을 가진다.
- 재시작(첫 호출 또는 시간 역행)이 아니면 `turn_offset -= 2π·round((raw − raw_prev)/2π)`로 2π 정수배 점프를 흡수하고, 연속각 `raw + turn_offset`을 쓴다. 재시작이면 offset을 0으로 둔다.
- 전제: 호출 1회 사이 실제 바퀴 회전은 π rad보다 작다. 120 Hz에서 377 rad/s, 바퀴 둘레 속도로 약 39 m/s에 해당한다.
- 랩이 없는 입력에서는 수정 전과 비트 단위로 같다.
- 출력 `angle_rad`·`hall_count`는 여러 바퀴를 누적한 연속값이 된다. ODrive `pos_estimate`와 같은 성격이다.
- `/jetin/joint_states`는 원래대로 articulation 참값이고, 바퀴 위치를 참값과 직접 비교하는 검증은 없다. `run_ros2_probe.py:88`은 팔 서보만 검사한다.

| 파일 | sha256 |
|---|---|
| 2026-10-10 전달본 원본 (`config/integrated_delivery_manifest_20261010.json` 기재) | `1af0ac33dda065dcfc400b9dc04aae2d2e4694e2397c3e32cce8e46d7ae5c04d` |
| 이 수정본 | `08f68599221f0afc2d622a7fbb461fed1220c88b935f8d24c074899ae8997ea6` |

전달 매니페스트는 2026-10-10 전달 시점의 역사 기록이라 고치지 않는다. 매니페스트를 읽어 검사하는 코드도 없다.

## 검증

- **새 시험** `simulation/jetin_rover_description/tests/test_hall_pll.py` 6개. Isaac 없이 numpy만 쓴다.
  - 수정 전 코드: 랩 관련 4개 실패, 기존 동작 고정·재시작 2개 통과(음성 대조).
  - 수정 후: 6개 모두 통과.
  - 실행: `python3 -m pytest simulation/jetin_rover_description/tests -q`
- **ws1 Isaac 재기록** (53 s, 0.15 m/s 지형 지도 코스, 같은 기록 하네스):

  | 기록 | 패키지 | 제어기 측정 최소 | 토크 최대 | 헛돎 표본(|참 속도−0.15|>1 m/s) |
  |---|---|---|---|---|
  | 4250 | 원본 | −14.118 m/s | 14.30 N·m | 116 |
  | 4251 | 원본 + 하네스 우회(`--hall-unwrap`) | −0.056 m/s | 3.15 N·m | 0 |
  | 4304 | **수정본**, 우회 없음 | −0.056 m/s | 3.15 N·m | 0 |

  4304는 4251과 참 궤적·바퀴 속도·제어기 측정·토크·깊이 프레임 전체가 비트 단위로 같다. 4304와 4250의 최종 위치 차이는 0.571 m다.
- **교차검증:** Codex 읽기 전용 리뷰에서 새 P1/P2는 없었다.
  - Codex가 반례(π 근처 양방향, 동일 시각 재호출, 여러 랩 뒤 시간 역행)와 음성 대조를 따로 실행했다.
  - P3(수정본 해시를 새 검증 기록에 연결)는 이 문서로 반영했다.
- **기록 하네스 결함:** 기록 하네스(저장소 밖 `power-train-sim/out/2026-10-18-terrain-map-isaac/tools/record_terrain_map_v2.py`)는 Slurm이 배정한 물리 GPU 번호를 `active_gpu`로 넘겼다.
  - 잡 안에서는 배정된 카드만 Vulkan 0번으로 보이고 1번은 AMD 내장 GPU다. 그래서 물리 GPU 1을 배정받으면 렌더 장치를 만들지 못했다(job 4302·4303).
  - 하네스를 항상 0번을 쓰도록 고쳤다. 패키지와는 무관하다.
