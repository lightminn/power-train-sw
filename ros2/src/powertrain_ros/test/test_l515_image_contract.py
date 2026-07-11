from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
DOCKERFILE = ROOT / "docker" / "Dockerfile.ros"


def dockerfile_text() -> str:
    return DOCKERFILE.read_text(encoding="utf-8")


def test_ros_image_pins_l515_supported_versions():
    text = dockerfile_text()
    assert "ARG LIBREALSENSE_TAG=v2.50.0" in text
    assert "FORCE_RSUSB_BACKEND=ON" in text
    assert "BUILD_PYTHON_BINDINGS=ON" in text
    assert "PYTHON_EXECUTABLE=/usr/bin/python3" in text


def test_ros_image_does_not_install_realsense_wrapper_or_binary_sdk():
    text = dockerfile_text()
    assert "realsense-ros" not in text
    assert "ros-humble-librealsense2" not in text


def test_ros_image_does_not_install_latest_pyrealsense_wheel():
    text = dockerfile_text()
    assert "pip3 install pyrealsense2" not in text
