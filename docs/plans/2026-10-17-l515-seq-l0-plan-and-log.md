# L515 기록 도구(L0) 구현 계획·검증 로그 — 2026-10-17

설계: `docs/specs/2026-10-17-l515-seq-recorder-and-mapping-design.md` §1.

## 구현

| 파일 | 내용 |
|---|---|
| `lidar_dev/record_seq.py` | 노트북 직결 L515 기록(depth 640×480 z16 + IR y8 + 컬러 1280×720 bgr8 + accel/gyro), pipeline 콜백 하나, 제한 큐(60) + 저장 스레드 4개, Gateway 보호(잠금 파일·`gateway_main` 프로세스·젯슨이면 거부, `--maintenance`는 잠금을 기록 끝까지 유지) |
| `lidar_dev/seq_format.py` | l515-seq v1 형식: `SeqWriter`(임시 이름 → rename, save_idx 연속 발급, 쓰기 실패 즉시 중지, frames.csv는 저장 성공분만), `extrinsics_dict`(SDK 열 우선 → 행 우선), `check_sequence` |
| `lidar_dev/check_seq.py` | 점검 CLI(구조 오류면 종료 코드 1) |
| `lidar_dev/tests/test_lidar_seq_format.py` | 장치 없는 시험 12개(음성 대조 7개 포함). numpy·cv2 없으면 skip, terrain 호환 시험은 open3d 없으면 skip |

기존 `lidar_dev/` 파일은 고치지 않았다. `terrain_node.py`(ROS 노드 호환)는 별도 커밋·젯슨 격리 스모크로 따로 받는다.

## 검증 로그

| 시각 | 무엇 | 결과 |
|---|---|---|
| 10-17 22:0x | `python -m pytest lidar_dev/tests -q`(파이썬 3.11, numpy 2.4.6, opencv 5.0, open3d 0.20) | 11 통과·1 실패 → 실패한 시험이 수집 로그 버퍼 결함을 잡음(도중 종료 시 로그 유실) → 줄 단위 버퍼로 수정 → **12 통과** |
| 10-17 22:00 | 실기 5 s(책상, 벽 쪽) → `check_seq` | 통과. 80프레임, 버림 0, depth–IR 0 ms·depth–컬러 9.8 ms, IMU 각 약 200 Hz |
| 10-17 22:09 | 실기 5 s(바닥이 보이게) → `check_seq` → 인수받은 `terrain.py`·`ground.py`(NO3D=1) | 통과. `terrain.py` OK 79/79, 카메라 높이 0.666 m ± 0.8 mm(반복성), 기울기 19.7°. `ground.py` 66.8 cm·19.7°, 1 m 앞 수직면 검출 |
| 10-17 22:1x | IMU 가속도 평균 대 바닥 법선(depth→accel 공장 외부 파라미터) | **1.74° 차이**, |a| 9.69 m/s²(다른 자세에서 9.58) → IMU 정지 보정 전에는 바닥 평면을 중력 기준으로 쓴다 |

## 남은 것

- 줄자로 잰 높이와 `terrain.py` 높이 비교(정확도)
- IMU 6자세 정지 보정 후 바닥 법선과 재비교
- 긴 기록(수 분)의 저장 속도, 1024×768 기록, `--maintenance` 경로
