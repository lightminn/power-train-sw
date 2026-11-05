# 2026-10-29 MKS node11/12 보드 flash 복구 증거

보고서: [../2026-10-29-mks-board1-flash-recovery.md](../2026-10-29-mks-board1-flash-recovery.md)

| 파일 | 내용 |
|---|---|
| `335233643235-original-read{1,2,3}-1m.bin.gz` | 기록 전 ROM DFU에서 읽은 flash 1 MiB 세 벌. 섹터 1 흔들림 때문에 서로 다르다. NVM 256 KiB는 셋 다 같다 |
| `335233643235-unstable-30reads.json` | application 30회 읽기에서 흔들린 14개 주소 |
| `335233643235-check-result.json` | 기록 전 읽기 전용 사전조건 확인 |
| `335233643235-program-result.json` | 기록·30회 읽기 대조·NVM 대조 결과 (`program_verified`) |
| `335233643235-verify-result.json` | 부팅 뒤 USB 읽기 전용 인수 (`accepted`) |
| `flash_board1_patch4.py` | 기록 스크립트 (`check` / `program`) |
| `dfu_backup_readonly.py`, `dfu_diff_readonly.py`, `dfu_flaky.py`, `dfu_flaky_full.py`, `dfu_postcheck.py` | ROM DFU 읽기 전용 진단 스크립트 |
| `verify_board1.py` | 부팅 뒤 인수 스크립트 |

`SHA256SUMS`로 대조한다. 결과 JSON의 시각은 UTC다(작업 중 Jetson 시계를 노트북 시각으로 맞췄다).
