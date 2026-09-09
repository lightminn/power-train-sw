#!/usr/bin/env bash
set -e
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=.superpowers/can-remediation/gui-deps:.:motor_control:ros2/src/powertrain_ros
/home/light/anaconda3/bin/python -m pytest -p no:cacheprovider -q \
 motor_control motor_gui powertrain_runtime scripts/tests powertrain_observability remote_video \
 --ignore=motor_control/chassis/tests/test_approach.py \
 --ignore=motor_control/chassis/tests/test_follow.py \
 --ignore=motor_control/chassis/tests/test_mission.py \
 --ignore=motor_control/chassis/tests/test_mission_trigger.py \
 --ignore=motor_control/chassis/tests/test_section_enforcement.py \
 --ignore=motor_control/chassis/tests/test_section_profiles.py
