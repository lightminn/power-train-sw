"""``python -m operator_console.arm_ui`` — the fixture preview entry point.

Kept separate from the console's own ``__main__`` so the arm tabs can be run
and reviewed without starting GStreamer, the video pipelines, or any receiver.
"""
from __future__ import annotations

import sys

from .preview import main

if __name__ == "__main__":
    sys.exit(main())
