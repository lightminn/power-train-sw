# 2026-10-29 MKS CAN patch4 node15/16 적용 증거

보고서: [../2026-10-29-mks-node15-16-patch4.md](../2026-10-29-mks-node15-16-patch4.md)

| 파일 | 내용 |
|---|---|
| `337733643235-patch3-original-1m.bin.gz` | 적용 전 flash 1 MiB(patch3 application + NVM). 풀면 SHA256 `c7654dd8…fdd8` |
| `337733643235-patch3-running-configuration.json` | 적용 전 RAM 설정 307개. 적용 전후 비교 기준 |
| `backup-result.json` | `backup-from-dfu` 단계 기록(두 번 읽기 일치·SHA). 앞선 `backup` 단계의 미검출 종료 기록은 이 파일에 덮어써져 남지 않았다 |
| `program-result.json` | 적용 전 점검(설정 차이 0, 양축 IDLE·오류 0)과 DFU 미검출 종료, 쓰기 없음 |
| `program-dfu-result.json` | 새 프로세스에서 기록·읽기 대조 통과(`program_verified`) |
| `verify-result.json` | 새 프로세스 인수 결과(`accepted: true`) |
| `flash_patch4.py` | 실행 스크립트. `2026-10-21-mks-can-patch4-evidence/board-336A33523235-patch4/flash_patch4.py`에서 serial·node 상수만 바꿨다 |

`SHA256SUMS`로 대조한다. 결과 JSON의 시각은 Jetson 시계 기준이며 이날 실제 시각과 맞지 않았다.
