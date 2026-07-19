# 터레인 백엔드 젯슨 자격화 — NumPy 확정 (2026-07-19)

핸드오프 §6-1 ③ "JAX Jetson 전체부하 자격화·backend 선택" 종결 기록.

## 결론

**젯슨 배포 백엔드 = NumPy 확정.** JAX는 x86 dev 동등성 테스트 용도로만 유지
(`powertrain_autonomy/terrain/jax_backend.py` 불변, 젯슨 이미지에 JAX 미설치 유지).

근거 3가지:
1. **성능 필요 없음** — 풀로드 실측(아래)에서 커널 p95 ≤ 7 ms. 10 Hz terrain
   예산(100 ms)의 7% 이하.
2. **CUDA JAX 불가** — Jetson(aarch64+CUDA)용 jaxlib은 pip 미배포. CPU JAX만
   가능한데 1의 결과로 동기가 없음.
3. **오프라인 제약** — 로봇 운용망(GL-SFT1200 standalone)에서 pip 자체가 불가
   (`pip install jax` → "no matching distribution"). 현장에서 재현 불가한 의존성을
   배포 경로에 넣지 않는다.

## 측정 (2026-07-19 저녁, 젯슨 Orin Nano 25W 모드)

- 환경: **풀로드 라이브 스택 가동 중** — L515 게이트웨이(:5000 SRT)+팔
  perception(D435i YOLO)+stream(:5002)+chassis/us100 50 Hz+텔레메트리+콘솔 수신.
  tegrastats: 6코어 전부 96~100% @ 1344 MHz, RAM 2.65/7.6 GB, GR3D(GPU) 0~6%.
- 방법: 커널 레벨 `build_terrain_grid_numpy` — 동등성 테스트
  (`test_jax_equivalence._case`)와 동일 입력 3케이스, warmup 5 + n=100,
  autonomy 컨테이너(`powertrain-sw:autonomy`, /workspace ro 마운트). 일회용
  스크립트(커밋 안 함), 이 표가 기록 정본.

| case | mean | p50 | p95 | max |
|---|---|---|---|---|
| flat | 2.80 ms | 2.59 | 3.28 | 7.18 |
| noise | 3.64 ms | 3.52 | 3.88 | 7.83 |
| hole | 3.05 ms | 2.60 | 6.98 | 8.53 |

주: 기존 "x86 29.8 ms/프레임" 수치는 상위 파이프라인 포함 측정이라 직접 비교
대상이 아님. 여기 수치는 커널 단독이며, 백엔드 선택 판단에는 커널 비교가 정본
(JAX 대체 대상이 커널이므로).

## 부수 발견 (별도 조치 필요)

1. **CPU 포화**: 라이브 스택만으로 6코어 96~100%. GPU는 0~6%로 놀고 있음 —
   **팔팀 perception(ultralytics)이 CPU 추론 중인 정황**. 팔 컨테이너는 GPU
   override로 떠 있으나 모델/토치 경로가 CUDA를 안 쓰는 듯. 팔팀 협조 세션
   안건으로 전달(우리 레포 무관, 조치는 팔팀 몫). CPU 포화는 50 Hz 제어 루프
   지터 리스크이기도 함 — perception GPU 이전 시 크게 해소될 것.
2. 전력 모드 25W, CPU 상한 1344 MHz 관측 — 대회 전 nvpmodel/전력 예산 확인
   항목으로 이월.
