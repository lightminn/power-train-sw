# JETIN Isaac Sim 통합 전달본

2026-10-10 실제 Isaac Sim 검증을 마친 동일한 전달본을 2026-10-11 보관했습니다. 기존 실차 SW와 별도인 시뮬레이션 자료입니다.

## 받기 및 실행

Git LFS 설치 후 저장소를 복제하고 `git lfs pull`을 실행하세요. ZIP은 이 저장소의 GitHub Releases에서 받으면 됩니다.

- 본체 시작: `jetin_rover_description/JETIN_Rover_Start_20261009.usda`
- 비전 집기: `jetin_rover_description/scenes/JETIN_VisionPick_20261010.usda`
- 파형 지형: `jetin_rover_description/scenes/JETIN_WaveTerrain_20261010.usda`
- 전면 장애물: `jetin_rover_description/scenes/JETIN_FrontObstacles_20261010.usda`
- URDF: `jetin_rover_description/urdf/jetin_rover.urdf`
- 실행 도구·설치 조건: [통합 사용법](jetin_rover_description/JETIN_통합패키지_사용법_20261010.md)
- 실제 촬영과 보고서: [validation/integrated_20261010](jetin_rover_description/validation/integrated_20261010)

파일명 20261009는 본체의 이름이며 최종 검증 리비전은 integrated_vision_ready_20261010입니다. 세 장면 모두 같은 본체를 참조합니다. 폴더 구조를 유지하세요. USD가 폐루프와 PhysX·센서 실행 설정을 보존하며 URDF만으로 이 기능을 모두 복원하지는 못합니다.

## 검증 범위

실제 92초 비전 집기 영상, 12cm 파형 주행, 4개 AS5048B 엔코더, L515 IMU, 카메라·깊이·점군 및 ROS2 실행 검증 기록을 포함합니다. 이번 업로드 중에는 물리 시험을 새로 촬영하지 않았습니다.

비전 집기는 D435i RGB/깊이의 주황색 HSV 검출과 OV5640 영상 보정, 5축 역기구학으로 진행했습니다. YOLO나 임의 물체를 인식하는 학습 모델이 아닙니다. 물체 고정·강제 부착 없이 양쪽 손가락 접촉과 상승을 검증했습니다.

공개 데이터시트와 CAD 기반 모델이며 실물과 완전히 동일하다고 보장하지 않습니다. 렌즈 보정, 기어박스 효율·백래시, 타이어·패드 마찰·탄성, 질량 및 열·통신 특성은 실측 보정이 남아 있습니다. 상세 사항은 [현실성 검토](jetin_rover_description/JETIN_현실성_검토_20261010.md)를 확인하세요. CAD 원본 Fusion 클라우드 문서는 이 ZIP에 포함되지 않습니다.
