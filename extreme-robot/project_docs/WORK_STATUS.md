# 작업 인수인계 지시서

> **대상**: 다음 Claude Code 세션  
> **최종 업데이트**: 2026-07-29 (그리퍼 파지력 실측(2026-07-28) 코드/문서 반영. 아래 최상단 섹션 참고)  
> **기준 문서**: `/home/jo/ros2_ws/CLAUDE.md` (전체 통합 계획)  
> **레포 경로**: `/home/jo/ros2_ws/extreme-robot/`  
> **ROS2 소스**: `extreme-robot/ros2_ws/src/`

---

## 그리퍼 파지력(Ratio) 실측 반영 (2026-07-28 실측 → 2026-07-29 코드 반영)

### 배경
Notion "그리퍼 파지/낙하 감지(effort threshold) 실측·캘리브레이션 절차" 문서 최하단에
`Gripper Load Calibration (2026-07-28)` 실측 표가 추가됨 — 실리콘 테스트 물체 기준으로
`gripper_load_calibration.py`의 `goto R` 명령이 쓰는 **closing ratio**(1.0=기존 계산상
완전 닫힘, 1.10까지 승인된 overtravel 범위)를 0.01~0.02 단위로 올려가며 ID3/ID4 load를
반복 측정한 결과.

| Ratio | ID3 Load | ID4 Load | 결과 |
|---|---|---|---|
| 1.00 | -119 | -50 | 파지력 부족 |
| 1.02 | -130 | -77 | 파지 가능 (약간 미끄러짐) |
| 1.04 | -192 | -131 | 안정적으로 파지 |
| 1.05 | -266 | -175 | 가장 안정적으로 파지 (hwerr=0x00) |

### 한 것
- `gripper_load_calibration.py`에 `RECOMMENDED_GRASP_RATIO = 1.05` 상수 추가 + 실측
  근거 주석. 다음 실기 세션에서 `goto 1.05`로 바로 재현 가능.
- `gripper_presets.py`의 `grasp_effort_thresh`/`drop_effort_thresh` placeholder 주석에
  이번 실측 결과와 잔여 과제(empty/grasp/drop 3그룹 반복측정 필요)를 명시.
- **주의**: 이번 실측은 ratio(닫히는 정도)만 확정한 것이고, `grasp_effort_thresh`/
  `drop_effort_thresh`(전류 임계값) 자체는 Notion 문서 원문에도 "반복 측정을 통해
  추후 최종 결정할 예정"이라고 명시되어 있어 **아직 미확정** — `80.0`/`20.0` 값은
  그대로 두었다. 아래 §547(구 TODO) 참고.

### 남은 것
- [ ] `gripper_load_calibration.py`의 `measure empty/grasp/drop <trial>` × 5회 이상씩
  실행 후 `thresholds` 커맨드로 `grasp_effort_thresh`/`drop_effort_thresh` 최종 확정.
- [ ] 확정되면 `gripper_presets.py`의 `80.0`/`20.0`을 실측값으로 교체.

---

## SRDF Default 충돌행렬 재검증 — 40샘플→3000샘플 (2026-07-24, 젯슨 로컬)

### 배경
2026-07-16(`24f4c4c`)에 등록한 18개 "Default" collision 쌍의 근거가 무작위 자세
40개뿐이라 약하다는 지적(SRDF 파일 내 자체 경고 주석)이 있었음. 실물 하드웨어
없이 이 젯슨에서 바로 진행 가능한 작업이라 우선순위 2번으로 착수.

### 한 것
- **`scripts/sample_collision_matrix.py` 신규 작성** — `/check_state_validity`
  서비스로 `arm_joint_1~5` 리밋 안쪽 무작위 자세를 N개 샘플링해, SRDF에 이미
  등록된 쌍은 제외하고 새로 걸리는 충돌쌍을 "항상 충돌(Default 후보)" vs
  "가끔 충돌(진짜 self-collision, disable 금지)"로 자동 분류하는 재사용 가능한
  스크립트(원래 40샘플 작업은 스크립트 없이 즉석으로 했던 것을 재현 가능한 형태로
  남김). `--samples`/`--seed`/`--srdf` 인자 지원.
- **1차 실행(3000샘플, 원본 SRDF 그대로)**: 기존 미등록 충돌쌍 65개 재확인, 새로
  "항상 충돌"인 쌍은 0개 — 기존 18개 Default 외에 추가로 disable할 쌍 없음.
- **2차 실행(3000샘플, 기존 18개 Default를 SRDF에서 잠깐 제거)**: MoveIt ACM은
  disable된 쌍을 아예 검사하지 않으므로, 기존 18개가 "진짜 항상 충돌"인지
  재검증하려면 SRDF에서 빼고 move_group을 재기동해 다시 검사해야 함. **18개 전부
  3000/3000(100%) 재확인** — 40샘플 근거가 75배 강화됨. 이후 원본 SRDF로 정확히
  복원(`disable_collisions` 18줄 diff 0 확인) + move_group 재기동으로 정상 동작
  재확인(`You can start planning now!`).
- **SRDF 주석 갱신**(`robot_arm_moveit_config/config/robot_arm.srdf` L94-111) —
  3000샘플 재검증 사실 + 갱신된 애매쌍 비율 반영.

### 결과 — 애매한 충돌쌍(disable 절대 금지, 3000샘플 기준)
기존 8쌍(2026-07-16 40샘플 기준 링크 그대로, 비율만 갱신):
```
link_034/link_037   2907/3000 (96.9%)
link_041/link_052   2888/3000 (96.3%)
link_002/link_026   2839/3000 (94.6%)
link_045/link_047   2675/3000 (89.2%)
link_034/link_040   2430/3000 (81.0%)
link_044/link_046   1303/3000 (43.4%)
link_043/link_045    824/3000 (27.5%)
link_041/link_049    107/3000 (3.6%)
```
표본이 커지며 새로 관측된 희귀 충돌쌍 57개(대부분 <2%, base_link/link_031이
극단 관절각에서 걸림 — 예: `link_006/link_031` 1.5%, `base_link/link_039` 1.0%
등). 전체 목록은 `project_docs/collision_matrix_3000samples_2026-07-24.txt`(1차
실행 원본 출력) 참고. 전부 disable 금지 원칙 동일.

### 부가 발견 (이번 작업 범위 밖, 손대지 않음)
`/check_state_validity`로 홈 자세(`arm_joint_1~5` 전부 0)를 직접 검사하면
`valid=False` — 위 애매쌍 중 5개(link_002/026, link_034/037, link_034/040,
link_041/052, link_043/045)가 정확히 이 자세에서 충돌 판정됨. 이 5개는 disable
대상이 아니므로 SRDF로 해결할 문제가 아니고, `24f4c4c` 커밋에서 이미 "RViz로
실제 겹침 여부 확인 필요"로 열어둔 이슈와 동일선상 — **stow 자세 작업(우선순위
1번) 착수 시 홈 자세 자체가 기하학적으로 유효한지도 같이 RViz 육안으로 확인할
것.**

### 검증 범위의 한계
- 여전히 디스플레이 없는 환경이라 정식 MoveIt Setup Assistant GUI
  "Regenerate Default Collision Matrix"(샘플 수천~수만 단위, 이번 3000샘플도
  근사치)는 못 돌림 — 기회 되면 GUI로 재생성해 이 결과를 덮어쓸 것.
- 홈 자세 self-collision 5쌍의 실제 기하 겹침 여부는 육안(RViz) 미확인 상태.

---

## TensorRT 백엔드 도입 + task 오판 버그 수정 + model_presets 5구간 전체 확장 (2026-07-22, 젯슨 실기)

### 이번 세션에서 한 것
- **TensorRT 백엔드 실측 도입**: 호스트 JetPack에 이미 있는 TensorRT 10.3.0을
  컨테이너에 apt로 재설치하는 대신 curated 디렉터리(`~/.tensorrt-libs`,
  `~/.tensorrt-python`)로 만들어 `docker-compose.gpu.yml`에 마운트(cudnn-libs와 동일
  패턴). `perception_node`의 `backend:=trt`로 box(seg) 모델 실제 엔진 빌드(~8분,
  최초 1회) + 실행 성공. **`/detected_objects` 실측: CUDA(pt) 17.1Hz → TensorRT FP16
  25.1~25.5Hz** — 30Hz 목표에 상당히 근접.
- **🐛 발견·수정: TensorRT 엔진의 task 오판으로 마스크가 조용히 사라지는 버그**.
  `.engine` 파일은 task 메타데이터를 보존하지 않아 `YOLO(engine_path)`가 자동으로
  `task='detect'`로 추정 — seg 모델(box)인데도 `r0.masks`가 **에러 없이 None**이 돼
  markerless pose(translation/PCA orientation)가 조용히 깨짐. `model_presets.py`에
  `task` 필드를 추가하고 `perception_node.py`가 `YOLO(resolved, task=preset['task'])`로
  명시 전달하도록 수정 — 수정 후 `box`+`backend:=trt` 조합에서 `/pick_target`
  orientation이 정상 발행됨을 재검증(quaternion z=0.663/w=0.748, 이전 pt 백엔드
  결과와 일치).
- **`model_presets.py`에 구간4/5 placeholder 추가**: `winter_terrain`(구간4
  미션4 노면 주행 — 이 구간이 실제 YOLO 인식이 필요한지 자체가 파워트레인과 미확정)과
  `lead_robot`(미션5 선도차 추종 — 선도차 PID 추종용). 둘 다 모델 파일(.pt) 아직
  없음 — `model_path`/`classes` TODO 상태, 도착하면 `models/`에 배치 + `classes`만
  채우면 바로 동작하는 구조(5구간 전체가 이제 `model_name` 파라미터 하나로 커버됨).
- **`*.onnx`를 `.gitignore`에 추가**: TensorRT 엔진 빌드의 중간 산출물(`.engine`은
  이미 gitignore돼 있었음) — 커밋 방지.

### 검증 범위의 한계
- TensorRT 벤치는 정지 테스트 이미지 1장 기준(`camera_mode=test`) — 실카메라 연속
  프레임 조건에서 25Hz가 유지되는지는 미검증. `winter_terrain`/`lead_robot`은 모델
  파일이 없어 preset 딕셔너리 구조만 검증(실제 로드는 못 함).
- 스트리밍 파이프라인 전체(카메라→추론→인코딩→SRT) e2e FPS 재측정은 여전히 실카메라
  없어 미완료(이전 섹션과 동일 블로커).

---

## CUDA 드라이버 정합 재빌드 검증 + model_presets 4종 실기 검증 (2026-07-22, 젯슨 실기)

### 이번 세션에서 한 것
- **CUDA 픽스 재빌드·실측 확인**: Dockerfile의 Jetson wheel-index torch 교체(직전 세션
  변경, 미빌드 상태였음)를 실제로 `docker compose build` + `up -d`로 반영 후
  `torch.cuda.is_available()` 재확인 — **`True`, `device_name='Orin'`**(주석의 실측
  claim이 실제로 재현됨). GPU 추론 벤치(box seg 모델, 848×480, 30프레임 평균):
  **23.7 fps**(42ms/frame) — 이전 CPU 폴백 상태의 스트리밍 실측(~11~13Hz, 파이프라인
  전체 기준이라 직접 비교는 아니지만 병목이었던 추론 자체는 확실히 풀림)과 비교하면
  큰 개선. **다만 스트리밍 파이프라인 전체(카메라→추론→인코딩→SRT) 재측정은 실제
  카메라가 없어 미완료** — RealSense/Dynamixel 모두 지금 세션엔 호스트에 물리적으로
  연결 안 돼 있었음(`lsusb`로 확인). 다음 하드웨어 세션에서 `run_stream.sh` 등으로
  end-to-end FPS 재측정 필요.
- **`model_presets.py` 4종 전체 실기 검증**(컨테이너 내 `perception_node` 실행,
  `camera_mode=test` + `debug_frame.png`): `box`/`traffic_light`/`iff`/`vision_marker`
  전부 `ros2 run`으로 기동해 (1) preset별 올바른 모델 파일 로드, (2) `model.names`가
  `model_presets.py`의 `classes`와 일치, (3) `/detected_objects` 정상 발행, (4)
  `box`(유일한 pick 대상 preset)는 `/pick_target`까지 정상 발행(box-segmentation,
  conf 0.93) — 관찰 전용 3종은 의도대로 `/pick_target` 미발행 확인. `colcon build`
  (전체 6개 패키지) 회귀 없음.
- **검증 범위의 한계**: 실제 RealSense 카메라 프레임으로는 아직 미검증(정지 테스트
  이미지 1장 기준) — depth 경로(markerless pose translation)는 `require_depth:=false`
  로 우회했으므로 실카메라+깊이 조건까지 포함한 end-to-end는 다음 하드웨어 세션 과제.

### 다음 작업 우선순위 갱신
- 위 "CUDA 드라이버 정합"(구 5번 항목)은 **코드/빌드 레벨은 완료** — 남은 건 실카메라
  연결 후 스트리밍 FPS 재측정 하나뿐. 아래 목록에서 제거하고 이 항목으로 대체.

---

## 비전마커 YOLO 모델 추가 + 스트리밍 FPS 튜닝 + 문서 정합성 점검 (2026-07-22)

### 이번 세션에서 한 것
- **(미션3 마커 식별) YOLO 모델 추가**: `vision_marker_best.pt`(detect, 8클래스
  `E/K/M/O/R/Y/a/heart`)를 `models/`에 배치, `model_presets.py`에 `"vision_marker"`
  preset 추가(관찰 전용, `pick_classes` 비움), `run_perception_vision_marker.sh` 신규.
- **GStreamer 스트리밍**: `stream_node.py`에 `encoder_threads` 파라미터 +
  `sliced-threads=true` 추가(Orin 6코어 활용). `run_stream.sh`(raw, :5002)·
  `run_stream_debug.sh`(오버레이, :5004) 신규. 실측 FPS ~11~13Hz(목표 30Hz 미달) —
  원인은 gstreamer 설정이 아니라 **컨테이너 CUDA 드라이버 불일치**(호스트 12.6 vs
  torch cu130 빌드) — YOLO가 CPU 폴백돼 6코어를 다 잡아먹음. Dockerfile의 torch
  휠 버전을 Jetson JetPack에 맞는 것으로 교체해야 근본 해결(미착수, 범위 큼).
- **CLAUDE.md 정합성 수정**: DRIVING 언락 안전 결함/`STOWED_LOCKED`·`CARRYING_LOCKED`
  미발행 경고 둘 다 **이미 커밋 `581a83d`(2026-07-16)로 해결돼 있었음**을 코드로
  확인 후 문구 정정(이전 경고는 stale).
- **launch/moveit_config 배선 재확인**: `gripper_a.xacro`를 launch에서 골라 붙이는
  방식은 **2026-07-15에 이미 폐기**되고 Isaac Sim 통합 재export(`robot_arm.urdf`,
  `link_001~057`, `arm_joint_1~5`+`gripper_drive_joint`+mimic)로 대체됨 —
  `display.launch.py`/`robot_arm_moveit_config`가 이미 이 트리로 배선돼 있고
  `demo.launch.py` mock 플래닝까지 실제 기동 검증("You can start planning now!").
  **CLAUDE.md의 robot_arm_description/robot_arm_moveit_config 절이 이 교체 이전
  내용으로 낡아 있음 — 다음 세션에서 전면 재작성 필요** (아직 미착수).
- **Notion 신규 문서 2건**(실측 절차 없던 gap 채움):
  [wrist_camera_link 동적 TF 통합 절차](https://app.notion.com/p/3a42d27b08d38139948bf7f74a426ec8),
  [stow_joint_positions 결정 절차](https://app.notion.com/p/3a42d27b08d381cba25ecee33cb1536c).
- **`power-train-sw` 직접 대조**(코드 레벨): `contract.py`/QoS/`robot_arm_msgs` 전부
  일치 확인(드리프트 0). `파워트레인_계약_충돌점검.md`의 1·2·3번 항목이 커밋
  `581a83d`로 이미 해결됐음을 확인·문서 갱신 — **파워트레인 팀에 통보 필요**
  (저쪽 정본 문서가 2026-07-15 시점 "arm 팀 대기 중"으로 남아있을 가능성).
  상세는 `파워트레인_계약_충돌점검.md` 2026-07-22 절 참고.

### 다음 작업 우선순위 (자율주행 통합 + 대회 대비)
1. **`stow_joint_positions` 실측** — `DRIVE_READY_STATUSES`(`STOWED_LOCKED`/
   `CARRYING_LOCKED`) 신뢰성의 마지막 코드 외 블로커. Notion 절차 문서 있음.
2. **팔 4/5축 서보 실물 배선 + ID 확정** → `arm_fsm_node.py`/
   `moveit_dynamixel_bridge.py`의 `JOINT_CONFIG` 확장 → `ik_mode='moveit'` 정식
   경로 복귀(현재 analytic 3DOF 우회 중, orientation 미제어).
3. **그리퍼 effort threshold 실측** — Notion 문서상 실측 예정일 2026-07-18 지남,
   진행 상태 확인 필요.
4. **`wrist_camera_link` 동적 TF 통합** — Notion 절차 문서 있음, 대회 시나리오에서
   손목 카메라 실사용 계획이 있는지에 우선순위가 갈림(현재는 전방 D435i만 사용 중).
5. ~~CUDA 드라이버 정합~~ **2026-07-22 완료**: Dockerfile torch 휠 교체를 재빌드해
   `torch.cuda.is_available()=True` 실측 확인(위 최상단 섹션 참고). **남은 건 실카메라
   연결 후 스트리밍 파이프라인 전체 FPS 재측정**뿐(추론만 벤치 완료, e2e는 미완료).
6. **CLAUDE.md `robot_arm_description`/`robot_arm_moveit_config` 절 재작성** —
   2026-07-15 URDF 전면 교체 이후 내용으로 갱신(현재 옛 gripper_a.xacro 모듈화
   서술이 낡음).
7. **파워트레인 팀에 통보**: STOW 3건(payload-aware/근접게이트/controller fault)
   완료 + 하트비트 상수(`HEARTBEAT_RATE_HZ`/`_TIMEOUT_S`) `contract.py` 중앙화 제안.
8. **대회 요구사항 재확인**: `ARRIVED_PICKUP`/`ARRIVED_DROP`이 현재 파워트레인 쪽
   오퍼레이터 수동 트리거 구조 — "완전 자율주행"이 대회 채점 기준이면 이 갭이
   파워트레인 쪽 과제로 명시돼야 함.
9. ~~비전마커(구간3) 클래스 확정~~ **2026-07-24 확인 완료, 조치 불필요**:
   대회 규정집(파워트레인 워크스페이스 `docs/`에서 원문 확인) §3 원문은
   "다수의 물체 **5개**(비전마커)를 각각 정확히 식별" — 이는 **한 미션당 트랙에
   놓이는 마커 개체 수**를 규정한 것이지 심볼 종류의 전체 가짓수가 아님. 붙임2에도
   실제 심볼 목록은 없음(팀이 실물 마커 보고 직접 라벨링). 즉 모델의 8클래스는
   "당일 어떤 5개가 나올지 몰라 가능한 심볼을 넓게 학습"한 것으로 의도된 설계이며
   8 vs 5는 애초에 모순이 아니었음 — `model_presets.py`의 `classes`를 좁힐 필요
   없음.

---

## 서보 디버깅 스크립트 정리 (2026-07-15)

HW-7 섹션(216행)의 "다음 세션 확인 포인트 — `check_servo.py` 등을 삭제할지 편입할지"를
해결. `check_servo.py`/`diag_servo.py`/`fix_servo.py`/`fix_servo2.py`/`move_servo.py` 5개는
전부 ID=0 서보 하드코딩 + Operating Mode·Position Limit 복구(HW-2~6 세션 당시 이상 대응)용
1회성 스크립트로, 이후 HW-7·HW-8·STOWING 세션에서 해당 서보들이 정상 동작해 문제가 재현되지
않아 삭제. `capture_pick.py`(범용 디버그 스냅샷)·`hw7_gripper_bottle_test.py`(그리퍼 캘리브값
280°/215° 출처, `moveit_dynamixel_bridge`에 아직 반영 전이라 유지)·`measure_position_error.py`
(범용 perception 정확도 측정)는 계속 사용 중이라 유지.

---

## `feat/contract-v2-arm-fsm` 브랜치 origin/main 재합류 + 실제 STOWING 모션 구현 (2026-07-15)

이전 세션들이 `feat/contract-v2-arm-fsm` 브랜치에서 계약 v2 FSM(conjunction 게이트·
`GRIP_LOST` 래치·`_is_settled()` 등)을 독립적으로 작업하는 동안, **같은 시기에 `main`에
PR #15(그리퍼 URDF 모듈화 + YOLO 재학습 모델)·#16(Jetson GPU compose 분리)·#17(파워트레인
DDS `ipc:host` 복구 + `arm_status` heartbeat + `contract.py`/`qos_profiles.py` 단일 출처
신설)이 각각 병합되어 로컬 `main`이 origin보다 19커밋 뒤처져 있었음. 특히 PR #17이
`arm_fsm_node.py`를 이 브랜치와 무관하게 독립적으로 다시 손대(계약 v2 상태/게이트 로직이
없는 이전 버전 위에 heartbeat 인프라만 추가) 두 버전이 같은 파일을 서로 다른 방향으로
크게 바꿔놓은 상태였음 — 단순 `git merge`로는 자동 해결 불가.

**해결 방식**: 로컬 `main`을 origin과 fast-forward 동기화 → 새 base(origin/main) 위에
브랜치를 다시 만들고, 우리 브랜치의 로직을 파일별로 수동 재적용(기존 `feat/contract-v2-arm-fsm`은
`feat/contract-v2-arm-fsm-old`로 백업 보존).

### `arm_fsm_node.py` 재적용
- PR #17의 heartbeat 아키텍처(`_set_status()`/`_publish_heartbeat()` 타이머 분리,
  `MultiThreadedExecutor` + 콜백그룹, `contract.py`/`qos_profiles.py` 단일 출처)는 그대로 두고,
  그 위에 계약 v2 FSM 로직(MISSION_STOP+ArrivalStatus conjunction 게이트, `GRIP_LOST`
  완전 래치, `STOW_ABORTABLE_STATES`, mission_id 멱등성, stamp freshness, chassis_mode
  워치독, `_is_settled()`)을 재적용.
- **`LOCK_MODES`를 `contract.py` 것으로 통일**(기존엔 이 파일이 로컬로 `DRIVING`을 제외한
  부분집합을 따로 들고 있었음) — `contract.py`(파워트레인 것과 짝인 단일 출처)는 `DRIVING`도
  포함한다. 즉 이제 PERCEIVE~LIFT 중 `DRIVING` 수신 시에도 `_enter_locked()`가 걸린다
  ("MISSION_STOP만 허가, 나머지 전부 잠금"을 문자 그대로 적용 — PR #17이 지적했던 "DRIVING에서
  자동 언락되는 버그"는 애초에 이 파일에 그 분기가 없어 해당 없음, `_try_advance()`의
  conjunction으로만 탈출).
- **`STOWING` 실제 접이 모션 신규 구현**(`_begin_stow_move`) — 이전엔 스켈레톤이라 현재 자세
  그대로 `_is_settled()`만 확인했음(모션이 없어 "접힘 검증"이 아니라 "정지 검증"에 불과).
  이제 `stow_joint_positions` 파라미터가 정의하는 목표 관절각으로
  `/arm_controller/joint_trajectory`에 직접 궤적 발행 → 완료 후 `_is_settled()` 게이트 →
  `STOWED_LOCKED`. ⚠️ **`stow_joint_positions` 기본값은 CAD 미검증 placeholder다.** 계약상
  all-zero home을 접힘 자세로 쓰는 건 금지(PR #17 회신) — 그래서 0이 아닌 임의값을
  넣어뒀지만 실제 안전 각도인지는 실기 검증 전까지 모른다. **실기 검증 없이 이 기본값으로
  실제 서보를 구동하지 말 것.**
- **버그 발견·수정**: `_do_stowing`이 모션 완료(`motion_state='done'`) 후 바로 `'idle'`로
  되돌리는데, 다음 tick에서 이게 다시 `_begin_stow_move()`를 재호출해 `_is_settled()`의
  dwell 타이머가 절대 누적되지 못하고 궤적이 계속(~2.7초 주기) 재발행되는 문제를 스모크테스트
  중 발견. `_grip_sent`와 같은 패턴의 상태-진입당-1회 플래그 `_stow_move_sent`(`_transition()`에서
  리셋)로 수정 — 모션은 상태 진입당 한 번만 발행되고, 이후 tick은 `_is_settled()`만 폴링.

**검증 완료 (컨테이너)**:
```bash
colcon build   # 전체 워크스페이스 6개 패키지 성공
```
- `arm_fsm` 기동 확인, heartbeat 10.0Hz 안정 발행(`ros2 topic hz /arm_status`).
- 표적 스모크테스트(`RELEASE`로 강제 진입 → `STOWING` → `STOWED_LOCKED`, identity TF
  `base_link↔Link4_1_1` 임시 발행): `/arm_controller/joint_trajectory`에 `stow_joint_positions`
  목표로 **정확히 1회** 궤적 발행 확인(버그 수정 전엔 반복 재발행), `_is_settled()` dwell(0.5s)
  통과 후 `STOWED_LOCKED` 전이 확인.

### perception 쪽 재적용
`robot_arm_perception/perception_node.py`도 이 브랜치(캡처/추론 스레드 분리 + `D435_SERIAL`
하드코딩 + `depth_img is None` 가드 수정 + `/perception/raw_image`)와 origin/main(PR #15의
재학습 모델 기본값 교체 + detect-only 폴백 색상마스크 + 디버그 오버레이 yaw 시각화)이 각각
독립적으로 바꿔놓은 상태라 동일하게 수동 재적용:
- `perception_node.py`: origin/main 버전(재학습 모델·색상마스크 폴백·yaw 오버레이) 위에
  캡처/추론 스레드 분리 + `D435_SERIAL` + `depth_img` 가드 + `/perception/raw_image` 재적용.
- `stream_node.py`: `image_topic` 파라미터(기본 `/perception/raw_image`)·포트 5000→5002·
  fps 15→30·x264 `ultrafast`+`threads=3` 재적용(origin/main엔 이 변경이 전혀 없었음 —
  여전히 `/perception/debug_image`·포트 5000·fps 15 상태였음).
- `metadata_sender_node.py`: origin/main에 아예 없어서 파일 그대로 복원 + `setup.py`
  entry_point 재등록.

**검증**: `colcon build`(robot_arm_perception 단독 + 전체) 성공, 3개 노드
(`perception_node`/`stream_node`/`metadata_sender_node`) 모두 import·실행파일 생성 확인.

### 남은 것 (이번 세션 범위 밖)
- 컨트롤러 fault 확인 — 브릿지(`moveit_dynamixel_bridge.py`)에 해당 필드 없음, 미포함.
- 브릿지가 `/joint_states`에 `PRESENT_VELOCITY`(SyncRead 범위엔 이미 포함되어 있음, 파싱만
  안 함)를 안 실음 — `_is_settled()`는 위치 유한차분으로 자체 계산해 우회하므로 당장 blocking은
  아니지만, 정식 velocity 필드 파싱은 여전히 별도 과제.
- `stow_joint_positions` 실측 캘리브(CAD/실기 검증) — 위 경고 참고.
- URDF 조인트 명명 불일치(`Revolute 23/29/42/48/72` vs `joint_1~N`)는 이번 세션 범위 밖으로
  보류(다른 곳에서 별도 진행 중이라고 전달받음).
---

## 그리퍼 preset 기반 FSM/브릿지 모듈화 (2026-07-13, 브랜치 `feat/gripper-fsm-modular`)

`upstream/main`(PR #15 `Gripper_YOLO_FSM`)을 로컬 main에 merge하는 과정에서 발견: `robot_arm_moveit_config`(SRDF/controllers/joint_limits/initial_positions/ros2_control.xacro)는 전부 `gripper_a_joint5`/`gripper_a_joint6`로 이미 정합돼 있는데, 실서보를 구동하는 `dynamixel_control`의 `moveit_dynamixel_bridge.py`/`arm_fsm_node.py` 두 노드는 여전히 옛 이름 `left_finger_joint`/`right_finger_joint`를 `gripper_joints` 기본값으로 쓰고 있었음 — 이 상태로 실행하면 `/gripper_controller`가 실제 URDF 조인트와 이름이 안 맞아 그리퍼가 안 움직이는 상태였음.

- **신규 공유 모듈** `dynamixel_control/dynamixel_control/gripper_presets.py`: `GRIPPER_PRESETS` dict(그리퍼 이름 → 조인트명/서보ID/틱 캘리브/전류 임계값/동작시간)을 두 노드가 공용으로 import. 그리퍼 설정이 두 파일에 각각 하드코딩돼 있던 게 이번 이름 불일치의 근본 원인이라, 앞으로 `gripper_b` 등을 추가할 때도 이 preset에 항목만 추가하면 되도록 구조화(URDF의 `xacro:arg gripper` 모듈화 패턴에 대응).
- 두 노드에 `gripper_type` 파라미터(기본 `gripper_a`) 신설 — preset에서 기본값을 가져오되, 기존처럼 `-p gripper_ids:=...` 등 개별 CLI 오버라이드는 그대로 동작.
- **HW-8 실측 틱 값 반영**: `gripper_open_tick`/`gripper_close_tick`을 placeholder(2400/2048)에서 HW-8에서 실측한 2446(215°)/3186(280°)로 교체 — 아래 HW-8 섹션에 남아있던 TODO 해소.
- `gripper_open_m`/`gripper_close_m`/`grasp_effort_thresh`/`drop_effort_thresh`는 여전히 placeholder — 실측 캘리브 필요.
- **검증**: 컨테이너가 안 떠 있어 `colcon build`/`ros2 run` 실기 검증은 못 함 — Python 문법(`ast.parse`)만 확인. 다음 세션에서 컨테이너 안에서 `colcon build --packages-select dynamixel_control` + `ros2 run dynamixel_control moveit_dynamixel_bridge --ros-args -p gripper_type:=gripper_a` 기동 후 `/joint_states`에 `gripper_a_joint5`/`gripper_a_joint6`로 발행되는지 확인 필요.

---

## YOLO 인식 모델 재교체 — 재학습 segmentation 가중치 적용 (2026-07-13, 브랜치 `Gripper_YOLO_FSM`)

아래 2026-07-08 섹션에서 "미확인"으로 남겨뒀던 `best.pt`의 task/클래스명을 확인하고, 이후
Roboflow에서 **segmentation으로 재학습한** 가중치로 다시 교체함.

- 컨테이너 안에서 확인: `task: segment`, `names: {0: 'box-segmentation'}` — 클래스 1개.
- `models/best.pt`를 재학습 가중치로 교체(커밋 `774edb2`). detect 전용 모델을 대비해 만들어뒀던
  `_color_mask_in_box()` HSV 폴백은 이번 모델(seg)에는 불필요하지만 이후 detect 전용 모델로
  되돌아갈 가능성을 감안해 코드에는 유지.
- 실기 RealSense로 `perception_node`+`stream_node` 기동 후 SRT로 원격 확인 — `box-segmentation`
  검출 및 pick 타겟 지정(초록 오버레이) 정상 확인.
- **주의**: 이 작업 중 `_fill_markerless_pose()`의 PCA yaw quaternion 대입 코드를 제거했는데,
  아래 HW-8 섹션 이후 `20260708_YOLO_URDF_Change` 브랜치에서도 디버그 오버레이의 PCA yaw 표시를
  독립적으로 제거한 상태였음(동일 위치라 머지는 충돌 없이 자동 해소됨). 사용자가 "yaw view는
  임시로 꺼둔 것"이라며 복원을 요청해 다시 살리는 작업 진행 중 — markerless pose의 orientation은
  설계상 PCA 주축각으로 채우는 게 맞음.

---

## YOLO 인식 모델 교체 — Roboflow 커스텀 학습 가중치 `best.pt` 적용 (2026-07-08, 브랜치 `Gripper_YOLO_FSM`)

기존 `perception_node`는 COCO 사전학습 `yolov8n-seg.pt`(사람/병/의자 등 범용 클래스)로 markerless pose를 뽑고 있었음. 대회 타겟 물체로 직접 라벨링·학습한 Roboflow 모델을 붙이는 작업.

- `ddkk0714/main`의 `b605666`(YOLO seg 학습 가중치 `best.pt` 추가 — `ros2_ws/src/robot_arm_perception/models/best.pt`, 6.2MB)을 그리퍼 커밋(`b4aa455`)과 함께 fast-forward로 받아옴. Notion "Roboflow 데이터 관리" 문서(라벨링→export→`best.pt` 로컬 배치 흐름)를 참고해 진행.
- `perception_node.py`의 `model_path` 파라미터 기본값을 `yolov8n-seg.pt` → `src/robot_arm_perception/models/best.pt`로 교체(커밋 `c3bc32e`). `CLAUDE.md`도 함께 갱신.
- **설계상 안전장치**: 코드가 이미 마스크 유무에 관계없이 동작하도록 짜여 있음 — seg 모델이면 markerless pose(translation+PCA yaw) 전체 활성, detection 전용 모델이면 마스크가 없어 bbox 중심 depth로 translation만 폴백하고 orientation은 스킵. 그래서 새 모델이 Instance Segmentation인지 Object Detection인지 몰라도 즉시 깨지진 않음.
- **미확인 — 다음 세션에서 컨테이너 안에서 확인 필요**:
  1. `python3 -c "from ultralytics import YOLO; m=YOLO('src/robot_arm_perception/models/best.pt'); print(m.task, m.names)"`로 이 모델이 `segment`인지 `detect`인지, 실제 클래스명이 뭔지 확인 (torch/ultralytics가 host엔 없어서 이번 세션에선 확인 못 함).
  2. 확인된 클래스명으로 `-p classes:='...' -p pick_classes:='...'`를 맞춰 실행 테스트(`ros2 run robot_arm_perception perception_node --ros-args -p model_path:=src/robot_arm_perception/models/best.pt ...`). 기존 COCO 클래스(`bottle`/`cell phone` 등)는 더 이상 안 맞을 가능성 높음.
  3. 실기 카메라로 대회 타겟 인식률이 기존 COCO 모델 대비 실제로 개선됐는지 실측.
- **커밋 여부**: 모델 교체(`c3bc32e`)는 커밋 완료. `models/best.pt` 자체(`b605666`)는 `ddkk0714/main`에서 받아온 상태로 이미 커밋됨.

---

## 그리퍼 URDF 모듈화 — gripper_a xacro 파싱 버그 수정·검증 (2026-07-08, 브랜치 `Gripper_YOLO_FSM`)

`main`에서 분기한 `Gripper_YOLO_FSM` 브랜치에 `ddkk0714/main`의 `b4aa455`(그리퍼 모듈화 — gripper_a URDF 추가, 5217073 위에 커밋)를 fast-forward로 받아옴. 이 커밋이 추가한 `urdf/grippers/gripper_a.xacro`(Fusion 360 fusion2urdf export 편입, `gripper_a_` prefix, 4절링크 닫힌 루프 단순화) + `meshes/grippers/gripper_a/`(mesh 16개) + `urdf/robot_arm.urdf.xacro`(신규, 몸체+그리퍼 xacro:include, `wrist_to_gripper` fixed joint)를 xacro로 실제 처리해 검증.

- **버그 발견·수정**: `gripper_a.xacro`에 `<robot>` 루트 태그가 없어 `xacro:include`가 `junk after document element`로 즉시 실패. 파일 앞뒤에 `<robot xmlns:xacro="..." name="gripper_a">...</robot>` 래퍼 추가로 해결(내용은 그대로).
- **검증 절차/결과** (컨테이너 내부, `docker exec ros2_humble`):
  1. `colcon build --packages-select robot_arm_description` — 성공
  2. `xacro robot_arm.urdf.xacro -o /tmp/robot_arm_out.urdf` — 에러 없이 처리됨 (link 57개, joint 61개)
  3. `check_urdf /tmp/robot_arm_out.urdf` — `Successfully Parsed XML`, 단일 트리 구조(중복 parent 없음) 확인
  4. 트리 확인: `...→ module_connector_5axis_Component41_1 → wrist_to_gripper(fixed) → gripper_a_base_link → ...`
  5. mesh 참조 57개(`package://robot_arm_description/...`) 전부 실제 파일로 해석됨 — 누락 0개
- **아직 안 한 것**: RViz 시각화 확인(수동), `wrist_to_gripper` origin 오프셋(x=147.544/y=0/z=239.50mm) CAD 재실측, `display.launch.py`/`robot_arm_moveit_config`를 이 xacro 경로로 배선.
- **커밋 여부**: `gripper_a.xacro`의 `<robot>` 래퍼 수정은 아직 미커밋 — 다음 세션(또는 이어서) 커밋 필요.

---

## HW-8 그리퍼 반응 테스트 + Profile 과부하 트립 원인 규명 (2026-07-08, 미커밋)

HW-7 다음 세션. 그리퍼 단독(병 인식→닫기/없음→열기) 반응 테스트를 실기로 진행하며 발견한
과부하 트립 문제의 원인을 규명하고 해결.

- **신규 스크립트** `ros2_ws/hw7_gripper_bottle_test.py` (컨테이너 내부 독립 실행 스크립트,
  `capture_pick.py`/`move_servo.py`와 같은 패키지 미편입 임시 테스트 — ros2_ws root, 미추적).
  `/pick_target`은 transient_local(latched)라 병이 사라져도 마지막 값이 남아 "없음"을 감지 못함
  → 매 프레임 발행되는 `/detected_objects`로 현재 프레임 기준 병 유무 판단. XL430 그리퍼에
  `moveit_dynamixel_bridge` 경유 없이 `dynamixel_sdk`로 직접 write.
- **그리퍼 서보 확정**: id=5, model=1060(XL430-W250), Operating Mode=3(Position Control).
  포지션 실측 조정 끝에 **닫힘 280°(tick 3186) / 열림 215°(tick 2446)**로 확정
  (`tick = round(deg/360*4096)`). `bridge.log`에 남아있던 "gripper id=5 토크 활성화 실패"는
  그 세션에 실장치 없이 컨테이너를 띄웠던 것으로 추정 — id=5 자체는 정상 확인.
- **perception_node 기본 해상도 불일치 발견**: 기본 `width=848,height=480`이 이 D435IF
  유닛 컬러 센서에서 미지원 조합이라 `RealSense init failed: Couldn't resolve requests`로
  실패. 이 카메라는 컬러 스트림이 424x240/640x480/1280x720/1920x1080만 지원(848x480은
  depth/IR 전용) — **640x480@30fps**로 띄워야 함.
- **핵심 발견 — 매 그리퍼 동작마다 토크 자동 해제(과부하 트립)**: 처음엔 원인 불명(Hardware
  Error Status가 읽을 때마다 0이라 안 보임)이었으나, Profile Acceleration/Velocity(주소
  108/112)가 **기본값 0(=최고속 즉시 이동)**이라 매 이동마다 순간 전류가 튀어 과부하 보호가
  걸리는 것으로 확인 — 재현율 100%(열림/닫힘 양방향 공통), 명령 후 0.3초 내 트립. Hardware
  Error Status는 트립 조건 해소 후 자동으로 0 복귀해 관찰 시점엔 안 보였을 뿐 실제로는
  발생하고 있었음.
  - **해결**: Profile Acceleration=25, Profile Velocity=80으로 설정 후 60초 반복 토글
    테스트에서 트립 0건 확인(accel=10/velocity=30도 안전하지만 더 느림, 도달 약 0.6초 vs
    25/80의 약 0.3~0.6초). 스크립트 기본값으로 반영.
  - 예기치 않은 토크 해제에 대한 방어로 2초 주기 하트비트(`_reassert_torque`)와 위치 명령
    직전 재활성화를 `_write_position`에 추가 — 근본 원인(Profile) 해결 후에도 안전망으로 유지.
- **실기 검증 완료**: bottle 인식 → 그리퍼 닫힘(280°) → bottle 사라짐 → 그리퍼 열림(215°),
  60초 연속 테스트에서 여러 차례 안정적으로 토글, 트립 없음. accel=0/velocity=0(최고속)으로
  되돌려 트립 재현도 별도 확인(양방향 100% 재현) 후 다시 안전 설정(25/80)으로 복구.
- **다음 세션 확인 포인트**:
  1. 그리퍼 각도(280°/215°)가 실제 파지 대상(병)에 맞는 stroke인지 재확인 (지금은 열림/닫힘
     반응 로직 검증 목적으로 임의 조정한 값).
  2. ✅ `moveit_dynamixel_bridge.py`의 `gripper_open_tick`(2400)/`gripper_close_tick`(2048)
     placeholder를 이번 실측값(2446/3186)으로 갱신 완료 (2026-07-13, `gripper_presets.py` 참고).
     단, 그 브릿지의 즉시-이동 방식(Profile Accel/Velocity 미설정)에도 동일한 과부하 트립
     위험이 있는지는 아직 미점검 — 브릿지는 아직 이 세션에서 발견한 Profile 이슈를 반영하지 않음.
  3. `hw7_gripper_bottle_test.py`는 현재 ros2_ws root의 미추적 독립 스크립트 — 계속 쓸 거면
     `dynamixel_control` 패키지 정식 유틸로 편입 검토.

---

## HW-2~6 실하드웨어 테스트 완료 (2026-07-04, 커밋 `3bed8bd`)

Phase 3 문서화(아래 섹션들) 이후 실제 젯슨/서보/카메라로 진행한 하드웨어 검증 세션. 변경 6개 파일:

- **`Dockerfile` / `docker-compose.yml`**: 베이스 이미지를 `osrf/ros:humble-desktop-full` → `ros:humble-ros-base` + `ros-humble-desktop`로 분리하고 `linux/arm64` 플랫폼을 명시(젯슨 실기 배포용). `pyrealsense2`(pip) + gstreamer 풀세트(`gstreamer1.0-plugins-{base,good,bad,ugly}`, `-libav`, `libgstreamer*-dev`) 신규 설치 — 아래 `stream_node`용.
- **`moveit_dynamixel_bridge.py`**: `_enable_torque()`가 `bool` 반환하도록 변경, **토크 활성화에 성공한 ID만** `group_sync_read.addParam()`으로 등록. 이전엔 버스에 없는 서보(전원 미연결 등)가 하나만 있어도 SyncRead 대상 전체가 얽혀 있었는데, 실하드웨어에서 일부 관절 서보가 없거나 응답 없는 상태로도 나머지 서보는 정상 구동되도록 방어. `publish_joint_states()`도 `txRxPacket()` 결과값을 더 이상 체크하지 않고 응답 온 ID만 처리(일부 미응답 허용).
- **`camera_tf.launch.py`**: 카메라 2대 체계로 확장.
  - 전방 RGB-D(RealSense D435i, 차체 고정): `cam_x/y/z/roll/pitch/yaw` 기본값을 placeholder(0)에서 **CAD 실측값**으로 교체(`x=0.123, z=0.082, pitch=-0.26`).
  - **손목 RGB(그리퍼 위, 신규)**: `base_link → wrist_camera_link` static TF 추가, CAD 실측값 기준(`x=0.040, z=0.295`). 현재는 **홈 포즈 기준 static placeholder** — 팔이 움직이면 실제 카메라 위치와 어긋남. URDF 관절 통합은 여전히 후속 과제.
- **`perception_node.py`**: `/perception/debug_image`(`sensor_msgs/Image`) 퍼블리셔 신규 — 구독자 있을 때만(`get_subscription_count() > 0`) `_draw_debug()`로 마스크 반투명 오버레이 + bbox + `클래스명/conf/거리` 라벨을 그려 발행(pick 타겟=초록, 나머지=파란색).
- **`stream_node.py`(신규 노드, `robot_arm_perception`)**: `/perception/debug_image` 구독 → `gst-launch-1.0` 서브프로세스(rawvideoparse→x264enc zerolatency→mpegtsmux→**srtsink**)로 H.264/SRT 송신. 파라미터 `port`(기본 5000)/`fps`(15)/`bitrate_kbps`(3000)/`latency_ms`(60). PC 쪽에서 `recv_stream.sh <port> <JetsonIP>`로 수신(파워트레인 레포 스크립트). 프레임 크기 바뀌면 gst 프로세스 재시작, 파이프 끊기면 자동 재시작.
  - 실행: `ros2 run robot_arm_perception stream_node --ros-args -p host_ip:=<젯슨IP>` (entry point `setup.py` 등록 완료)

**검증 상태**: 커밋 메시지상 "HW-2~6 실하드웨어 테스트 완료"이나, 이 문서의 나머지 섹션(그리퍼 tick/전류 임계값 실측, 카메라 마운트 캘리브 등)이 갱신되지 않았으므로 어디까지 실측 완료됐는지는 다음 세션에서 재확인 필요. 회귀 확인 포인트: SyncRead 필터링 변경 후 정상 서보들의 `/joint_states` 발행 주기·값이 기존과 동일한지.

**진행 중(미커밋)**: 저장소 루트 `ros2_ws/`에 `check_servo.py`/`diag_servo.py`/`fix_servo.py`/`fix_servo2.py`/`move_servo.py` 임시 스크립트 존재 — ID 0 서보의 Operating Mode·Position Limit 이상 및 Hardware Error 복구(토크 OFF→리밋 재설정→리부트→토크 ON) 시도 흔적. 다음 세션에서 원인 파악 후 정리(성공했으면 삭제, 재현되면 `dynamixel_control`에 정식 유틸로 편입 검토).
→ **HW-7(2026-07-05, 커밋 `3048f02`)에서 정식 커밋됨** (`fix_servo2.py` 포함). 아래 HW-7 섹션 참고 — ID 0 이상 자체의 근본 원인 확인 여부는 미기록, 다음 세션 재확인 필요.

---

## HW-7 실하드웨어 픽 시퀀스 검증 및 analytic IK 우회 경로 추가 (2026-07-05, 커밋 `3048f02`)

HW-2~6 다음 세션. `arm_fsm`을 실제 서보로 처음 끝까지(인식→IK→하강→파지판정) 돌려본 세션. 핵심 발견은 **결정 '가'(MoveIt 단일 경로)가 현재 하드웨어에서 전제부터 깨져 있었다는 것**.

- **핵심 발견 — MoveIt 6DOF IK 원천 불가**: URDF/SRDF가 아직 팔 5축 중 `joint_1`~`joint_3` **3축만** 반영(CAD 미완성, WIP). 이 상태로 MoveIt `/compute_ik`를 호출하면 **현재 실제 tip pose에 대해서도 `NO_IK_SOLUTION`**이 반환됨을 실측 확인 — 3관절로는 위치+방향(6DOF) 목표를 만족시킬 자유도가 애초에 없음(자유도 3 < 목표 자유도 6).
- **대응 — analytic IK 우회 경로**: `arm_fsm_node.py`에 `ik_mode` 파라미터 신설, 기본값 `'analytic'`.
  - MoveGroup(MoveIt) 대신 FK 서비스(`/compute_fk`) + 수치 자코비안(finite-difference, Levenberg-Marquardt 유사 댐핑 최소자승)으로 **위치만** 맞추는 3DOF IK(`_solve_position_ik`/`_fk_tip`)를 구현, 결과를 `/arm_controller/joint_trajectory`에 직접 publish. 방향(orientation)은 이번엔 포기.
  - **폐기 아님 — 임시 우회**: 결정 '가'의 MoveGroup 경로(§6-A)는 코드에 그대로 남겨둠. URDF가 5축으로 확장되면 `ik_mode:='moveit'`로 전환해 즉시 재사용 가능.
  - `tip_link` 파라미터 기본값을 placeholder에서 실제 SRDF 값(`Link4_1_1`)으로 수정.
- **버그 수정 — FK 서비스 타임아웃**: `/compute_fk` 호출을 `_tick`(타이머 콜백) 안에서 `self`를 `spin_until_future_complete`하면, 이미 실행 중인 콜백을 재진입 spin하게 돼 응답을 못 받고 항상 타임아웃(독립 스크립트로는 2회 반복 만에 수렴하는데 노드 내부에서는 즉시 실패하는 걸로 실측 확인). → 별도 헬퍼 노드(`arm_fsm_fk_client`)로 FK 클라이언트를 분리해 우회.
- **서보 디버깅 스크립트 정식 커밋**: HW-2~6 세션에 미커밋 상태로 남아있던 `check_servo.py`/`diag_servo.py`/`fix_servo.py`/`fix_servo2.py`(ID 0 서보 Operating Mode·Position Limit 이상 및 Hardware Error 복구용) + 실행 스크립트 `run_perception.sh`가 이번 커밋에 반영됨.
- **실기 검증 결과**: bottle 인식 → analytic IK 계산 → 팔 하강 → 그리퍼 닫힘 → effort(전류) 기반 파지 판정까지 **실제 모터로 end-to-end 확인**. 단, 방향까지 맞추는 정밀 파지는 URDF가 5축으로 확장된 뒤(`ik_mode='moveit'` 전환 후)에야 가능.

**검증 상태**: analytic 3DOF 경로는 실기 동작 확인됨(위치만). `ik_mode='moveit'` 경로는 URDF 5축 확장 전까지 검증 보류(코드는 유지, 전환 스위치만 남음). **다음 세션 확인 포인트**: (1) 서보 스크립트가 이전 세션의 ID 0 이상을 실제로 해결했는지, (2) `check_servo.py` 등을 삭제할지 `dynamixel_control` 정식 유틸로 편입할지, (3) URDF 5축 확장 일정.

---

## 현재 완료된 작업

### 신규 패키지 (모두 빌드 완료)

| 패키지 | 위치 | 상태 |
|--------|------|------|
| `robot_arm_msgs` | `src/robot_arm_msgs/` | ✅ 빌드 완료 |
| `robot_arm_perception` | `src/robot_arm_perception/` | ✅ 빌드 완료 |
| `dynamixel_control` | `src/dynamixel_control/` | ✅ `arm_fsm`(FSM+carry_pose) + `moveit_dynamixel_bridge`(effort+그리퍼 확장) |
| `robot_arm_description` | `src/robot_arm_description/` | ✅ `launch/camera_tf.launch.py` 추가(카메라→base_link static TF) |

### robot_arm_msgs — 메시지 5개 정의 완료

```
msg/DetectedObject.msg         int32 class_id / string class_name / float32 confidence
                               geometry_msgs/Pose pose / sensor_msgs/RegionOfInterest bbox
msg/DetectedObjectArray.msg    std_msgs/Header header / DetectedObject[] objects
msg/ArrivalStatus.msg          Header / int32 mission_id / string status
msg/ChassisMode.msg            Header / string mode
msg/ArmStatus.msg              Header / int32 mission_id / string status
```

### robot_arm_perception — Phase 2 완료 (Step 1·2·3, markerless)

**파일**: `src/robot_arm_perception/robot_arm_perception/perception_node.py`

> ⚠️ **2026-06-29 설계 변경**: 대회 규정상 **타겟 객체에 ArUco 마커 부착 금지** 확인
> → 마커 기반 pose 추정 폐기, **markerless(YOLO seg + depth + 2D PCA)** 로 전환.
> ArUco/solvePnP 코드·`camera_calibration.yaml`·Phase 1 더미 스켈레톤
> (`dynamixel_control/perception_node.py`)은 혼동 방지 위해 **완전 삭제**함.

- **Step 1 완료**: YOLO **segmentation** 추론 → `class_id`/`class_name`/`confidence`/`bbox`/`mask`
  - ultralytics YOLO 로드 (TensorRT `.engine` 캐시 지원, `_resolve_model()`)
  - ⚠️ `model_path` 기본값 `yolov8n-seg.pt` — **반드시 seg 모델** (detection 모델이면 mask 없어 orientation 미산출)
  - RealSense D435i 파이프라인 (yolo_depth_3d.py 포팅, `_latest_frames()`)
  - `camera_mode` 파라미터: `realsense`(기본) / `test`(정지 이미지)
  - `/detected_objects` (`DetectedObjectArray`) publish, 30fps

- **Step 2 완료 (markerless pose)**:
  - **Translation**: 마스크 centroid color 픽셀 → depth 픽셀 투영 → depth 패치 median
    → deproject (`DepthCal`/`_deproject_centroid`, yolo_depth_3d.py 포팅, align 생략).
    카메라 내부파라미터는 RealSense 스트림 프로파일에서 직접 취득 (yaml 불필요·더 정확).
  - **Orientation**: 마스크 (u,v) 픽셀에 2D PCA (`_mask_pca_yaw_quat`) → 주축 각도를
    optical Z 축 yaw 로 근사 → quaternion `(0,0,sin θ/2,cos θ/2)`. depth 노이즈 무관.
  - 마스크 없거나 depth 측정불가 시: position 0 / orientation 단위쿼터니언 유지.

**설정 파일**: 없음. markerless 경로는 RealSense 스트림 intrinsics를 직접 사용하므로
별도 calibration yaml 불필요 (기존 `config/camera_calibration.yaml`은 삭제됨).

**검증 상태 (2026-06-29)**:
- ✅ `colcon build --packages-select robot_arm_msgs robot_arm_perception dynamixel_control` 성공
- ✅ 런타임 검증 (test 모드, `bus.jpg`, `yolov8n-seg.pt`): `/detected_objects`에 person 4개
  검출, 객체별로 **서로 다른 yaw quaternion**(2D PCA 정상 동작), test 모드라 position=0.
- ⚠️ **translation(depth median) 실측 검증은 RealSense D435i 하드웨어 필요** — 미수행.
  로직은 실카메라 검증된 `yolo_depth_3d.py` 포팅이라 하드웨어 연결 시 동작 기대.

---

### Phase 2 Step 3 — `/pick_target` 선별 로직 완료 (2026-06-29)

**파일**: `perception_node.py` — `_select_pick_target()` + `/pick_target` 퍼블리셔.

- `/pick_target` (`DetectedObject`) 퍼블리셔, **`transient_local`(latched) QoS** — 도착 타이밍 최신 타깃 유실 방지.
- 선별 조건 (3개 모두 만족하는 객체 중 **confidence 최고 1개**):
  1. `class_name ∈ pick_classes` (쉼표구분 **화이트리스트**, 빈값=후보없음 → 신호등/정지선 등 관찰 전용 자동 제외)
  2. `confidence ≥ pick_min_conf` (기본 0.5)
  3. `require_depth=True`(기본)면 `pose.position.z != 0.0` 필수 / `False`면 conf만 (test 검증용)
  - 후보 없으면 publish 안 함 (이전 latched 값 유지).
- **신규 파라미터**: `pick_classes`(필수), `pick_min_conf`(0.5), `require_depth`(True).

**검증 완료 (test 모드, bus.jpg, `require_depth:=false`, `pick_classes:=person`)**:
person 4 + airplane 검출 중 → `/pick_target`에 **confidence 최고 person(0.87)** 발행.
airplane(화이트리스트 제외)·person 0.46(min_conf 미달) 정상 탈락. 로그 `pick=person(0.87)` 확인.
⚠️ 실주행(`require_depth=True`)은 RealSense depth 필요.

---

### Phase 2 커밋 완료 (2026-06-29)

- 커밋 `22c25a1` `feat(perception): Phase 2 markerless 인식 파이프라인 구현` (브랜치 `Depth_LiDAR_RViz`, **push 안 함**).
  - perception_node markerless 전환 + `/pick_target` 선별 / ArUco·더미노드 제거 / setup.py 정리.
  - 개인 작업 문서 3종(`CLAUDE.md`·`WORK_STATUS.md`·`CLAUDE_Plan.md`) 추적 해제 + `.gitignore` 등록(비공유, 로컬 보존).
  - `ros2_ws/yolov8n-seg.pt`(6MB)는 커밋 제외(미추적).

---

## Phase 3 착수 — 로봇팔 FSM (2026-06-29, 진행 중)

### 설계 문서 작성: `PHASE3_FSM_설계.md` (레포 루트)

- 사용자 기존 FSM 미션 FSM 다이어그램(5구간 미션 FSM) 분석 반영.
- **핵심 발견**: 로봇팔 실작업은 **(미션2 박스 운반)의 `ARM_GRASP_BOX`** 하나. 나머지 4구간은 팔이 IDLE+자세 락만.
- drawio(전체 로봇) → 팔 FSM 노드/파워트레인 노드 분리 매핑, §4 팔 상태표, §5 핸드셰이크, §6 구현 갭 정리.

### 구현 방식 결정 (사용자 확정 2026-06-29)

| 항목 | 결정 |
|------|------|
| A. 모션 경로 | **MoveIt 단일 경로(결정 '가')** — FSM→MoveIt(IK·계획)→`arm_controller`→upstream `moveit_dynamixel_bridge`→서보. *(당초 position_node 직접(A)이었으나 upstream #9에 브릿지 존재 발견 → 가로 전환)* |
| B. 그리퍼 | **Dynamixel** — `/joint_states` effort(전류)로 파지/DROP 판정 |
| C. status enum | **보류** — 잠정값으로 두고 파워트레인 팀 합의 후 확정 |
| D. 구간4 제설 주체 | **미정** (팔로 치울지/밟고 갈지) |
| E. 95mm 박스 파지 | 그리퍼로 **가능** |

### 팔 FSM 스켈레톤 작성: `arm_fsm_node.py` (가 방향)

**파일**: `src/dynamixel_control/dynamixel_control/arm_fsm_node.py` (entry point `arm_fsm`, setup.py 등록 완료). **빌드 성공 + mock 스모크테스트 통과**(2026-06-29, 컨테이너).

- §4 상태표 12개 상태(`IDLE/PERCEIVE/PLAN/DESCEND/GRASP_CHECK/LIFT/CARRY/REGRASP/RELEASE/DONE/ABORT/LOCKED`) Enum + `_do_<state>()` 디스패치.
- 액추에이션(가): 팔=MoveIt `move_action`(MoveGroup, pose goal), 그리퍼=`/gripper_controller/follow_joint_trajectory`, 피드백=`/joint_states.effort`.
- 토픽 I/O: 구독 `/pick_target`(latched)·`/arrival_status`·`/chassis_mode`·`/joint_states`, 발행 `/arm_status`.
- effort 기반 파지/DROP 판정, 자세 락(진행 모션 취소+홀드), 재파지 루프 — 골격 동작.
- 스모크테스트: `IDLE→PERCEIVE→PLAN→DESCEND` 전이 확인, move_group 없으면 `move_action 미준비` 경고 후 대기(정상).

### Phase 3 선결 과제 / TODO (대부분 **브릿지 측**으로 이관)

- [x] **브릿지 effort(전류) 발행** *(2026-06-29 완료)* — `moveit_dynamixel_bridge`가 PRESENT_CURRENT(126,2 signed)~PRESENT_POSITION(132,4)을 연속 10바이트 SyncRead 블록으로 한 번에 읽어 `/joint_states`에 position+effort(**raw signed current**) 동시 발행. FSM이 effort로 파지/DROP 판정.
- [x] **브릿지에 그리퍼 실행 경로 추가** *(2026-06-29 완료)* — 같은 브릿지 노드에 `/gripper_controller/follow_joint_trajectory` 액션 서버 추가(단일 서보 양 핑거 미러링). 그리퍼 ID·미터↔틱 매핑·열림/닫힘 전부 파라미터화(`gripper_ids` 기본 [5], `gripper_open/close_tick` placeholder).
  - [ ] **남은 캘리브**: `gripper_open_tick`/`gripper_close_tick` 실측 (2모터 랙피니언 전환 후 미검증, 상단 §8/CLAUDE.md 참고). `gripper_ids`는 [3, 4]로 확정됨(완료). 전류 임계값(`grasp_effort_thresh`=80·`drop_effort_thresh`=20 raw placeholder)은 2026-07-28 ratio 실측(최상단 섹션 참고)으로 진전됐으나 여전히 미확정 — empty/grasp/drop 3그룹 반복측정 필요.
- [x] **TF** 카메라(`camera_color_optical_frame`)→`base_link` 연결 *(2026-06-29 완료)* — `robot_arm_description/launch/camera_tf.launch.py` 추가. 뎁스 카메라(베이스 고정) static TF 2단: `base_link→camera_link`(장착 오프셋, launch arg `cam_x/y/z·cam_roll/pitch/yaw`, placeholder=0) + `camera_link→camera_color_optical_frame`(REP-103 optical 회전 고정). tf2_echo로 체인·회전 검증 완료.
  - [ ] **남은 캘리브**: 장착 오프셋 실측값을 launch arg로 지정. **RGB 카메라(eye-in-hand)는 URDF 관절 통합 후속 과제.**
- [x] `_carry_pose()` 구현 *(2026-06-29 완료)* — TF(`base_frame`←`tip_link`)로 현재 TCP 자세 조회 → z+`lift_height`(기본 0.10m), orientation 유지. base_link(planning frame) 기준이라 MoveIt 바로 계획. TF 미가용 시 None→LIFT 스킵(graceful). 파라미터 `base_frame`/`lift_height` 추가, `tf2_ros` 의존 추가. 가짜 TF 스모크테스트 통과.
- [ ] status enum 파워트레인 팀 합의(§6-D) → 파일 상단 상수 교체.
- [ ] 구간4 제설 주체 결정(D); upstream 머지 시점 결정(브릿지 파일 필요).

### 검증 (하드웨어 없이 mock)

```bash
cd /root/ros2_ws && colcon build --packages-select robot_arm_msgs dynamixel_control
source install/setup.bash && ros2 run dynamixel_control arm_fsm
# 다른 터미널 — /pick_target은 transient_local이라 durability 맞춰야 전달됨
ros2 topic pub --qos-durability transient_local /pick_target robot_arm_msgs/DetectedObject \
  '{class_name: box, confidence: 0.9, pose: {position: {z: 0.4}, orientation: {w: 1.0}}}'
ros2 topic pub --once /arrival_status robot_arm_msgs/ArrivalStatus '{status: ARRIVED_PICKUP}'
# 기대: IDLE→PERCEIVE→PLAN→DESCEND (move_group 없으면 move_action 미준비 경고 후 대기)
```

---

## 다음 작업 (Phase 3 — FSM 통합)

→ 아래 "그 이후 작업 (Phase 3)" 섹션 참조. 실행 명령 예시:

```bash
docker exec -it ros2_humble bash
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash

# 인식 노드 (실센서) — 픽 대상 클래스는 실제 타겟으로 교체
ros2 run robot_arm_perception perception_node --ros-args \
  -p model_path:=/root/ros2_ws/yolov8n-seg.pt \
  -p pick_classes:=<타겟클래스> -p pick_min_conf:=0.5

# 하드웨어 없이 선별까지 검증할 때 (test 모드)
BUS=/usr/local/lib/python3.10/dist-packages/ultralytics/assets/bus.jpg
ros2 run robot_arm_perception perception_node --ros-args \
  -p camera_mode:=test -p test_image_path:=$BUS \
  -p model_path:=/root/ros2_ws/yolov8n-seg.pt -p conf_threshold:=0.3 \
  -p pick_classes:=person -p require_depth:=false
# 확인: ros2 topic echo /detected_objects  /  ros2 topic echo /pick_target
```

---

## 그 이후 작업 (Phase 3 — FSM 통합)

### Phase 3 체크리스트 (CLAUDE.md §3 Phase 3)

- [~] **로봇팔 FSM**: `/arrival_status` 수신 → `/pick_target` 읽기 → 픽 시퀀스 *(스켈레톤 완료, 2026-06-29)*
  - ✅ 신규 노드 `src/dynamixel_control/dynamixel_control/arm_fsm_node.py` (위치: perception 아님 dynamixel_control)
  - ✅ `/arrival_status`(ArrivalStatus) 구독, `status=='ARRIVED_PICKUP'` 시 FSM 전환
  - 🔧 픽 모션: **MoveIt 단일 경로(결정 가)** → 브릿지 effort 발행 + 그리퍼 실행 경로 + TF 연결 남음

- [ ] **자세 락**: `/chassis_mode` 구독
  - `mode == 'CORNERING'` 또는 `'ROUGH_TERRAIN'` → 현재 관절각 유지 명령
  - `mode == 'DRIVING'` 복귀 시 언락
  - 구현 방식은 파워트레인 팀과 합의 필요 (오픈 이슈 5번)

- [ ] **완료 신호**: 픽 완료 시 `/arm_status`(ArmStatus, status='DONE') publish
  - 파워트레인이 이 신호 받아야 재출발 가능

- [ ] **파워트레인 연동**: `/detected_objects`에서 신호등/정지선/마커 필터링
  - 파워트레인 팀 쪽 작업이나 인터페이스 스펙은 우리가 정의

---

## 중요 설정값 (확정값 / 미확정값)

| 항목 | 값 | 상태 |
|------|-----|------|
| pose 추정 방식 | markerless (YOLO seg + depth median + 2D PCA yaw) | ✅ 확정 (2026-06-29 전환) |
| YOLO 모델 | segmentation 모델 (`yolov8n-seg.pt` 등) | ⚠️ seg 필수, 커스텀 학습 모델로 교체 예정 |
| ArUco 경로 | (삭제됨) | ❌ 대회 규정상 타겟 마커 금지 → 코드·yaml 완전 제거 |
| camera_matrix 출처 | RealSense 스트림 intrinsics 직접 사용 | ✅ 확정 (markerless는 yaml 불필요) |
| optical frame 이름 | `camera_color_optical_frame` | ⚠️ placeholder, 실값 확인 필요 |
| status 문자열 enum | `ARRIVED_PICKUP`, `DONE` 등 | ⚠️ 파워트레인 팀과 합의 필요 |
| ChassisMode 자세 락 구현 | 현재 각도 유지 vs 안전 자세 이동 | ⚠️ 합의 필요 |

---

## 빌드 방법 (컨테이너 내부)

```bash
docker exec -it ros2_humble bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash

# 신규 패키지만 빌드
colcon build --packages-select robot_arm_msgs robot_arm_perception
source install/setup.bash

# 또는 전체 빌드
colcon build
source install/setup.bash
```

## 파일 구조 스냅샷

```
extreme-robot/ros2_ws/src/
├── robot_arm_msgs/              ← 신규 (메시지 정의)
│   └── msg/
│       ├── DetectedObject.msg
│       ├── DetectedObjectArray.msg
│       ├── ArrivalStatus.msg
│       ├── ChassisMode.msg
│       └── ArmStatus.msg
├── robot_arm_perception/        ← 신규 (markerless 인식 노드)
│   └── robot_arm_perception/
│       ├── perception_node.py        ← 핵심 파일 (YOLO seg + depth median + 2D PCA + debug_image)
│       └── stream_node.py            ← 신규 (debug_image → H.264/SRT 스트리밍, 2026-07-04)
├── dynamixel_control/           ← 기존 (더미 perception_node 스켈레톤은 삭제됨)
├── robot_arm_description/       ← 기존 (URDF)
├── robot_arm_moveit_config/     ← 기존 (MoveIt2)
└── pick_test_pkg/               ← 기존
```

## 오픈 이슈 (CLAUDE.md §5 참조)

1. **optical frame 실제 이름** 확인 (`ros2 run tf2_tools view_frames`)
2. **커스텀 seg 모델** — 대회 타겟 클래스로 학습한 YOLO **segmentation** 모델로 교체 (현재 `yolov8n-seg.pt` COCO)
3. **2D PCA yaw 한계** — 객체가 이미지 평면 밖으로 크게 기울면 부정확 → 필요 시 3D PCA(마스크 erode+outlier 제거) 업그레이드
4. **status enum 합의** — 파워트레인 팀과 `ARRIVED_PICKUP`, `DONE` 등 문자열 통일
5. **자세 락 구현 방식** — 파워트레인 팀 합의 후 FSM에 반영
6. **ChassisMode → ArrivalStatus 트리거 순서** 합의
