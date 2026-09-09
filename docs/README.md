# 문서 안내

현재 구현·검증 상태는 [프로젝트 지침](../AGENTS.md)의 §2가 정본이다.
[2026-09-08 정합성 정리 기록](reports/2026-09-08-workspace-consistency.md)에서
코드·문서·Notion의 수정 항목과 실기 확인이 남은 범위를 확인할 수 있다.

| 문서 | 용도 |
|---|---|
| [통합 운용 안내](integrated-operation.md) | 젯슨 한 번 기동 + 노트북 콘솔 자동 연결의 최초 준비·매일 사용 |
| [통합 운용 검증 기록](reports/2026-09-08-integrated-operation.md) | 호스트·격리 ROS 검증과 남은 실차 인수 |
| [Jetson 배포·루프 검증](reports/2026-09-09-integrated-jetson-validation.md) | 실제 배포·콘솔 수신·가상 모터 ROS 루프와 CAN/센서 미통과 항목 |
| [국방로봇 자율주행 전체 계획](plans/2026-07-12-defense-robot-autonomy-software-plan.md) | 범위·의존 순서·완료 기준 |
| [WP5.2 팔 협업 안전 계획](plans/2026-07-13-wp5.2-arm-collaboration-safety-plan.md) | 팔 계약·명령권·안전 게이트 |
| [관측성·데이터 품질·원격 보조 계획](plans/2026-07-13-observability-data-quality-remote-assist-plan.md) | 관측·영상·품질·연결 작업 |
| [Notion 전체 계획](https://www.notion.so/39c2d27b08d381728c1ade21cc72216b) | 위 세 계획의 팀 설명본 |
| [현재 DualSense 운용 매뉴얼](https://www.notion.so/3b02d27b08d381d99641e3565fe40ca2) | 실제 운용 절차·축별 NVM 검증 이력 |

날짜가 붙은 `specs/`, `plans/`, `reports/`의 과거 상태 선언·일정·실험 수치는
그 날짜의 기록이다. 구현 계획의 미체크 항목을 현재 코드 부재의 증거로 사용하거나,
과거 벤치 시험을 현재 배포·실차 주행 승인으로 해석하지 않는다.

`parameter_calc/`의 50 kg·f_opt 0.2004는 종료된 준정적 최적화 결과이며 제작 v2 CAD와
실차 동역학 성능은 별도다. `powertrain_sim/`의 MuJoCo는 폐기된 읽기 전용 트랙이다.

로컬 `docs/patent/`의 백서·PDF·Typst·도면·staging ZIP과 예전 출품 DOCX는 당시 자료다.
특허 Markdown에는 별도 아카이브 경고를 추가했으나 PDF·도면을 새로 제작한 것은 아니다.
구형 8모터·MuJoCo·형상 수치를 담은 파일을 현재 제출본으로 사용하지 않는다.
특허 폴더는 이 checkout에서 Git 제외 대상이므로 Markdown 경고도 자동 추적되지 않는다.
