"""chassis — 4WS(4륜 조향) 차체 레이어.

kinematics : 차체 명령(v, ω) → 각 바퀴 (조향각, 구동속도) 순수 계산 (하드웨어·ROS 무관).
(추후) chassis_manager : 코너모듈 4개를 묶어 kinematics 결과를 실제 모터로 분배 (WP3).
"""
