# 2026-10-21 MKS CAN patch4 적용 증거

보고서: [../2026-10-21-mks-can-patch4.md](../2026-10-21-mks-can-patch4.md)

## board-336A33523235-patch4/

| 파일 | 내용 |
|---|---|
| `336A33523235-patch3-original-1m.bin.gz` | 적용 전 flash 1 MiB(patch3 application + NVM). 풀면 SHA256 `f1e985fc…864e` |
| `336A33523235-patch3-running-configuration.json` | 적용 전 RAM 설정 307개. 적용 전후 비교 기준 |
| `backup-attempt1-network-none-result.json` | network none 컨테이너의 첫 백업 시도. DFU 미검출로 중단, 쓰기 없음 |
| `backup-result.json` | 두 번 읽기 일치·SHA 기록(앱 복귀 뒤 같은 프로세스가 멈춰 그 뒤 항목 없음) |
| `program-result.json` | 적용 전 점검 결과와 DFU 미검출 종료, 쓰기 없음 |
| `program-dfu-result.json` | 새 프로세스에서 기록·읽기 대조 통과(`program_verified`) |
| `verify-result.json` | 새 프로세스 인수 결과(`accepted: true`) |
| `flash_patch4.py` | 실행 스크립트 최종본. `program` 단계는 `program_from_dfu` 함수를 더하기 전 판으로 실행했고 그 밖의 차이는 없다 |

## build-patch4/

`ODriveFirmware.bin.gz`(풀면 SHA256 `15462f7f…b3fc`), `build.log`, 빌드·소스 매니페스트.
2026-10-15 `firmware/mks_odrive/` 빌드 산출물이다.

결과 JSON의 시각은 UTC다(이번에는 Jetson 시계가 맞았다).
