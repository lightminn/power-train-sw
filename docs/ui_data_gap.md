# Operator Console UI 데이터 공백

2026-07-29 현재 `operator_console` 파서, 실제 UDP 발행 코드, ROS 메시지
미러와 테스트 계약을 대조한 결과다. `구현 가능 여부`가 `불가` 또는 `부분`인
항목은 화면 완성도를 위해 0이나 임의 값을 만들지 않는다.

| UI 항목 | 필요한 데이터 | 현재 소스 | 상태 | 부족한 필드 | 확인 담당 |
|---|---|---|---|---|---|
| 배터리 전압 | V | Power UDP :5004 `voltage_v` | 가능 | 없음 | 김선 확인 |
| 배터리 전류 | 방전/충전 부호와 A 스케일 | :5004 `current_a`, `pdist_charge_current_a` | 부분(기본 그래프 제외) | 운용 기준 부호·스케일 검증 결과 | 김선 확인 |
| SOC | % | :5004 `pdist_soc_percent` | 가능 | 없음 | 김선 확인 |
| 전원 보호 상태 | 배터리·보호 플래그 의미 | :5004 `pdist_battery_flags`, `pdist_protection_flags` | 가능 | 사용자용 원인 문구 매핑 | 김선 확인 |
| 전원 장치 연결 | RS485 상태·age | :5004 `rs485_state`, 로컬 수신 age | 가능 | 없음 | 김선 확인 |
| 각 전원 Rail | 48/24/18/12/5V별 상태 | 데이터 소스 없음 | 불가 | rail 이름·전압·enabled/health | 담당자 협의 필요 |
| Command Velocity | 선속도 지령 m/s | 데이터 소스 없음 | 불가 | command linear velocity | 김선 확인 |
| Actual Velocity | 실제 차체 선속도 m/s | 데이터 소스 없음 | 불가 | measured linear velocity | 김선 확인 |
| Angular Velocity | 지령/실측 rad/s | 데이터 소스 없음 | 불가 | command/actual angular velocity | 김선 확인 |
| 바퀴 속도 | 바퀴별 turn/s | Chassis UDP :5005 `wheel_statuses[].drive_turns_per_s` | 가능 | m/s 환산에 필요한 정본 반지름은 payload에 없음 | 김선 확인 |
| 조향각 | 바퀴별 deg | :5005 `wheel_statuses[].steer_deg` | 가능 | FL/FR/RL/RR 정본 name 목록 | 김선 확인 |
| CAN 상태 | 추상화된 health | :5005 `can_state` | 가능 | 모터별 응답 지연 상태 | 김선 확인 |
| ODrive·Motor 상태 | stale, axis error, steer fault | :5005 `wheel_statuses[]` | 부분 | 모터별 연결 상태 enum | 김선 확인 |
| Safety 상태 | status, E-stop, distance, failures | :5005 `safety_*`, `component_mask` | 가능 | MOTION_HOLD 전용 bool/enum | 김선 확인 |
| 관절각 | 관절명, rad | Arm UDP :5007 `joints.names`, `joints.position_rad` | 가능 | 없음 | 로봇팔팀 확인 |
| Dynamixel 전류 | raw current와 단위 스케일 | :5007 `dynamixel[].current` | 부분(기본 그래프 제외) | 모델별 A/mA 변환 스케일 | 로봇팔팀 확인 |
| Dynamixel 온도 | ℃ | :5007 `dynamixel[].temperature_c` | 가능 | 현 55/65℃ 임계값의 모델·설정 근거 확정 | 로봇팔팀 확인 |
| Arm Mode | master/slave, teleop, auto, idle | 데이터 소스 없음 | 불가 | arm mode enum | 광민 확인 |
| End-Effector ID | 실시간 도구 ID | 데이터 소스 없음 | 불가 | tool id, display name | 로봇팔팀 확인 |
| Tool Attached | 체결 상태 | 데이터 소스 없음 | 불가 | attached/locked 상태 | 로봇팔팀 확인 |
| 작업 완료 | arm task completion | 직접 필드 없음 | 불가 | task state 또는 done bool | 광민 확인 |
| Object Class | class name | YOLO UDP :5003 `detections[].class_name` | 가능 | 한글 표시명 사전 | 광민 확인 |
| Confidence | 0..1 | :5003 `detections[].confidence` | 가능 | 없음 | 광민 확인 |
| Bounding Box | xywh | :5003 `detections[].bbox_xywh` | 가능 | 영상 capture sequence 동기 HIL | 광민 확인 |
| Target Distance | 카메라 전방 m | :5003 `detections[].position_m[2]` | 가능 | 좌표계·유효 범위 보증 | 광민 확인 |
| Distance Source | 거리 생성 센서·좌표계 | 데이터 소스 없음 | 불가 | `target_distance_source` | 광민 확인 |
| Distance Age | 거리 측정 자체의 age | 로컬 Metadata 수신 age만 존재 | 부분 | `target_distance_age_ms` | 광민 확인 |
| LiDAR Target Distance | LiDAR 기반 표준 대상 거리 | 데이터 소스 없음 | 불가 | 거리·source·age 및 target association | 담당자 협의 필요 |
| Metadata 카메라 귀속 | source camera ID | v1 :5003 계약이 D435i 전용으로 고정 | 부분 | 다중 카메라 확장 시 `source_camera_id` 계약 필요 | 광민 확인 |
| RGB/Depth 시간 동기 | color/depth/metadata timestamp | 팔 노드는 동일 frameset color/depth와 동일 ROS stamp를 사용, :5003 `capture_stamp_ns` 전달 | 부분 | 개별 color/depth device timestamp와 허용 차이 계측 | 로봇팔팀 확인 |
| YOLO 전처리 좌표 | 원본 크기·입력 크기·letterbox padding | :5003 frame 크기와 bbox만 존재 | 부분 | input size, scale, pad 또는 원본 좌표 보증 | 로봇팔팀 확인 |
| RGB-D 거리 품질 | mask 투영 depth 통계와 유효 픽셀 수 | 팔 GitHub 소스에서 mask erosion·다점 투영·median/MAD 구현, :5003은 결과만 전달 | 부분 | valid pixel count·거리 상태를 payload에 싣지 않음, 실기 오차 계측 | 로봇팔팀 확인 |
| 현재 YOLO 모델 | 파일·class names·threshold·NMS | `models/best.pt` 6.5MB, class 0 `box-segmentation`, 기본 conf 0.55 | 부분 | 학습 검증셋 precision/recall, 현장 오검출 재학습 판단 | 로봇팔팀 확인 |
| 3D Position | xyz m | :5003 `detections[].position_m` | 가능(개발자 정보) | 좌표 frame ID | 광민 확인 |
| Yaw | rad | :5003 `detections[].yaw_rad` | 가능 | 없음 | 광민 확인 |
| Pick Target | bool | :5003 `detections[].is_pick_target` | 가능 | latch 해제 시점 보증 | 광민 확인 |
| Video FPS | 전방/작업 실제 표시 FPS | `VideoPanel` sink buffer 계수, :5005 L515 rates | 가능 | 송신/표시 FPS 의미 구분 | 김선 확인 |
| Last Frame Age | 로컬 프레임 도착 age | `VideoPanel._last_frame_monotonic` | 가능 | 없음 | 김선 확인 |
| Metadata Age | 로컬 UDP 도착 age | :5003 `received_monotonic_s` | 가능 | 없음 | 김선 확인 |
| Connection State | SRT/UDP freshness | VideoPanel·Latest Receiver 로컬 상태 | 가능 | 없음 | 김선 확인 |
| Packet Drop | L515 SRT drop Hz | :5005 `l515_drop_hz` | 부분 | 작업 카메라 drop | 광민 확인 |
| RTT | 왕복 지연 ms | 데이터 소스 없음 | 불가 | receiver feedback RTT | 담당자 협의 필요 |

Raw CAN frame, ODrive 전체 parameter, PID gain, encoder raw count,
PDIST raw hex, ROS topic 전체 목록, TF tree와 point cloud는 기본 화면에
노출하지 않는다. 현재 계약에 존재하는 기술 필드는 개발자 정보 보기가 켜진
경우에만 표시한다.
