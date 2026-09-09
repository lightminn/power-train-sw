from pathlib import Path
import shutil
import subprocess

import pytest


NODE = shutil.which("node")
SCRIPT = Path(__file__).with_name("frontend_timeout_ack_test.mjs")


@pytest.mark.skipif(
    NODE is None,
    reason="Node.js is required for the app.js VM test",
)
def test_frontend_timeout_ack_contract():
    result = subprocess.run(
        [NODE, str(SCRIPT)],
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stdout.strip() == (
        "frontend timeout ACK contract: 9 cases passed"
    )
