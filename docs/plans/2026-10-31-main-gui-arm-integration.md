# 메인 GUI 로봇팔 제어 통합

대상은 PR #20 `75ef49c`, 기능 정본은 PR #14 `979611d`다.
사용자는 PR #14의 Jetson 실물 체결 시험도 성공했다고 확인했다.

1. PR #20 기반의 별도 브랜치에 PR #14를 병합한다. 기존 사용자 worktree는 보존한다.
2. 팔 탭의 배치 충돌만 조정한다. 비전·지형 지도·3D 표시와 팔 키보드·패드 조작을 함께 유지한다.
3. 콘솔 스모크, 팔/비전 회귀, 설치 ROS와 인증 TCP/GTK 가상 운용으로 확인한다.

기능 계약은 `docs/specs/2026-10-17-local-arm-console.md`, 결과는
`docs/reports/2026-10-31-main-gui-arm-integration.md`를 따른다.
