"""Canonical dual-video and D435i metadata channel contract."""

# This module is the source of truth for remote-video channel contracts.
# l515_dashboard keeps matching local verdict constants during the pure-core
# cycle; whether it imports these constants is deferred to the wiring cycle.

L515_RGB_SRT_PORT = 5000
L515_RGB_WIDTH = 1280
L515_RGB_HEIGHT = 720
L515_RGB_FPS = 30

D435I_RGB_SRT_PORT = 5002
D435I_RGB_WIDTH = 848
D435I_RGB_HEIGHT = 480
D435I_RGB_FPS = 30

D435I_METADATA_UDP_PORT = 5003
METADATA_SCHEMA_VERSION = 1
MAX_METADATA_BYTES = 16 * 1024

L515_UNAVAILABLE_VERDICT = "REMOTE_DRIVE_VIDEO_UNAVAILABLE"
D435I_UNAVAILABLE_VERDICT = "REMOTE_ARM_VIDEO_UNAVAILABLE"
