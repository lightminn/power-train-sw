# 현장 통합 브랜치 안전 결함 수정 결과 (2026-11-05)

`deploy/jetson-field-20261028`을 main에 병합하기 전 검토에서 확인한 다섯 결함(F1~F5)을 고쳤다.
설계와 검토 기록은 `docs/specs/2026-11-05-field-safety-fixes.md`다. 브랜치 `fix/field-safety-20261105`
(`docs/final-audit-20261105` 8b023ba 위). 젯슨이 오프라인이어서 실기 배포·실주행 확인은 하지 않았다.

## 바뀐 동작

| 결함 | 바뀐 동작 |
|---|---|
| F1 패드 E-stop 유실 | 누를 때마다 E-stop 이벤트 ID를 만들고, 서버의 연결별 `G` 줄 확인(`estop_ack_id`)을 받을 때까지 보존한다. 송신 정체·재연결을 넘어 다음 레코드에 실린다. 옛 단독 송신기는 재연결 샘플에 E-stop을 유지한다. |
| F2 정체 뒤 옛 명령 재출발 | 비고정 모드에서 입력이 만료되면 서버가 입력 세대를 올리고, 노트북이 그 세대를 받은 뒤 만든 레코드(서버 시각으로 `input_timeout_s` 안에 발급된 세대)만 운전에 쓴다. 링크가 회복되면 운용자 재조작 없이 이어진다. 고정 모드에서는 수신 간격으로 감지한 만료도 MOTION_HOLD다. HOLD 해제 뒤에도 해제 이후의 입력만 쓴다. |
| F3 영상 워치독 | 나이 검사를 먼저 해 조용히 멎은 영상도 5 s 뒤 재연결한다. 첫 프레임이 5 s 안에 없으면(프리롤 정체 포함) 재연결한다. 예약 뒤 회복되면 재시작을 취소·생략하되 접속 대상 변경은 항상 재시작한다. |
| F4 compose 설정 | `TELEOP_INPUT_TIMEOUT_S`·`TELEOP_LATCH_STALE_INPUT`·`TELEOP_PARTIAL_CLOSE_VIOLATION`을 호스트 환경/.env로 바꿀 수 있다(기본 0.5/false/false). |
| F5 비전 스모크 | 탭 숨김 idle이 끝난 뒤 측정·결함 주입, 주입한 결함 유지 확인, Top/Oblique 중 하나만 mapped 기대. |

프로토콜: 상태 회신에 `G <input_epoch> <estop_ack_id>` 줄이 추가됐고 레코드 schema 3(`input_epoch`, `estop_id`)이
생겼다. 노트북은 `G`를 받은 연결에서만 schema 3을 보내므로 젯슨·노트북 배포 순서와 무관하게 계약 위반이 없다.
옛 노트북(schema 2)은 비고정 모드에서 정체·HOLD 해제 뒤 재연결해야 다시 운전된다(안전 쪽 저하).

남는 한계: 끊김 없이 전체가 일정하게 늦은 스트림(예: 5 GHz 400 ms)은 수신 시각 신선도로 잡지 못한다.
현장 대책은 2.4 GHz 로봇 AP다. 확인(`G`)만 유실된 E-stop은 다음 연결에서 한 번 더 걸릴 수 있다(안전 방향).

## 검증

| 항목 | 결과 |
|---|---|
| 노트북 `motor_control/laptop/tests` | 77 passed (로컬) |
| 게이트웨이·디코더·노드 순수 시험 | 61 + 5 passed (로컬) |
| Humble 컨테이너(`terrain-ros-test:local` + `ros-humble-control-msgs`) colcon 4패키지 빌드 뒤 ROS 시험 | 76 passed |
| 설치 엔트리포인트 스모크 `scripts/teleop_stall_smoke.py`(도메인 77, `ros2 run powertrain_ros teleop_command`, 비고정·0.5 s) | PASS — 운전 1.05 m/s → 정체 0(고정 없음) → 늦은 schema 2·옛 세대 전진 재출발 없음(단계 중 모든 상태 줄 0) → 새 세대 복귀 1.05 m/s → E-stop ID 확인·MOTION_HOLD |
| 음성 대조: 같은 스모크(`--legacy`)를 수정 전 노드(cd776a2)에 | FAIL — 늦은 schema 2 전진으로 1.05 m/s 재출발 |
| 콘솔 시험(로컬 GTK): 영상·앱 122, 통합앱·지형·상호작용·ops 139 | 통과 |
| 비전 스모크 시험 `-k vision_runtime` | 음성 대조 3회 반복 정상 판정, 양성 통과 |
| `python -m operator_console.runtime_smoke` | PASS |
| ws1 전체 스위트(Slurm 15981) | 병합본 대비 새 실패 없음. 새 영상 시험 6건은 ws1에 `gi._gi_cairo`가 없어 실패(로컬 통과) |

남은 기존 실패(이번 범위 밖, main에도 있음): `scripts/tests`의 colcon 3패키지 기대 2건, `l515_dashboard`
entrypoint 시험 등 — ws1 환경 탓 실패와 함께 별건으로 둔다.

## 교차검증

설계는 Codex 적대 검토 2개 스레드(서로 결론 비공유) → 개정 3회 → "새 P1/P2 없음". 기각한 안(클라이언트 시각
offset, 만료 시 연결 끊기, 세대만)과 반례는 설계 문서 자체 검토 절에 있다. 구현은 Codex 리뷰 2회
(P2 3·P3 1 반영) 뒤 "새 P1/P2 없음".

## 남은 일

- 젯슨 배포(`JETSON_REPO=/home/zetin/power-train-sw-integrated bash scripts/deploy_branch_to_jetson.sh --prepare`)
  뒤 젯슨에서 같은 스모크를 설치 엔트리포인트로 다시 돌리고, 실차에서 Wi-Fi 끊김 → 복귀·패드 E-stop을 1회 확인한다.
- 노트북 콘솔도 같은 커밋으로 갱신해야 schema 3·E-stop 확인을 쓴다.
