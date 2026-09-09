# CAN 통신 결함 수정·Jetson 동기화 계획

사용자 승인: 2026-09-09 전체 실제 결함 수정·배포·GitHub 동기화 요청. 정본 요구사항은 `docs/reports/2026-09-09-can-software-interference-audit.md`의 확인된 결함과 수정 우선순위다.

- [x] Jetson 접근·운용/팀원 체크아웃 분리·소스 스냅샷·수신만으로 10모터 확인.
- [x] CAN 구동 코어의 송신 실패·실제 arm 확인·freshness·폴링·ROS 입력 정체 수정.
- [x] CAN 준비 파서·단일 리셋·워치독·기동 구성 충돌 수정.
- [x] GUI 대상 전환/RX/ACK/프로토콜 및 USB/NVM 소유권·레거시 진입점 수정.
- [x] 세션 accept 복구·실제 wheel 정지 proof·Jetson 전용 코드 대조.
- [x] 호스트 및 Jetson 설치 엔트리포인트, 가상 CAN·설치 ROS 전체 루프, 콘솔 실행 검증.
- [x] 실제 CAN 읽기/무속도 운용 상태 재검증 및 변경 검토. 결과: 13·14 재소실로 실물 안정성 미통과.
- [x] 로컬/GitHub/Jetson 정본 동기화. 수정 코드 b9292af, 추적 파일 980개 바이트 대조 차이 0.
- [x] 사용자 요청에 따라 실행 순서와 신규 유휴 조회의 SW 유발 가능성을 복기하고 세 버전 송신 패턴을 대조. 원인 확정은 미완료.

원격 자율주행·분리 센서의 기능 인수와 실물 비영점 주행은 제외한다. 확인되지 않은 ODrive 펌웨어 후보를 임의 패치/플래시하지 않는다. 팀원 브랜치·미커밋 코드는 별도 보존한다. 이번 통합 운용 정본을 같은 commit으로 맞추며 팀원 작업 복제본까지 강제로 main으로 덮지 않는다.

파일별 독립 병렬 작업: core는 CANdriver/CornerModule/ChassisManager/chassis_node, startup은 scripts/Compose/watchdog/runtime_lock, GUI/USB는 motor_gui/USB도구/ak_control를 소유한다. 공유 RealCanSession API는 유지한다. 상호의존 arm·wheel proof 계약은 root가 통합 리뷰한다. root는 powertrain_runtime.session, ops_broker, snapshot/배포/문서/Git를 담당한다. 의미 있는 실패 테스트→수정→통과 순서를 적용한다.

최종 실물 게이트: 최초 10/10 후 ODrive 13·14 재소실. 제어·워치독 정지와 CAN 수동 reset 후에도 8/10이며 실물 CAN 안정성은 미통과다. 원인은 미확정이며 USB 진단이 필요하다. 소프트웨어·가상 CAN 통과와 분리한다.
