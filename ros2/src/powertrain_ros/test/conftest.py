"""테스트 DDS 도메인 격리.

로봇(젯슨)에서 스위트를 돌리면 기본 도메인 0의 라이브 토픽(실 L515
camera_info/depth 등)이 테스트 구독으로 새어 들어오고, 반대로 테스트의
가짜 /arm_status·/odom이 라이브 그래프를 오염시킨다(2026-07-17 실증 —
실카메라 640×480 camera_info가 테스트 격자를 선점해 합성 프레임이 전부
무시됐다). rclpy.init 전에 도메인을 강제로 분리한다. 의도적으로 라이브
그래프를 대상으로 할 때만 POWERTRAIN_TEST_DOMAIN으로 재정의하라.
"""
import os

os.environ["ROS_DOMAIN_ID"] = os.environ.get("POWERTRAIN_TEST_DOMAIN", "77")
