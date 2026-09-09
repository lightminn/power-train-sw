from pathlib import Path
import subprocess


def test_can_temperature_unavailable_note_is_rendered():
    completed = subprocess.run(["node", str(Path(__file__).with_name("frontend_can_notes_test.mjs"))],
                               text=True, capture_output=True)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "2 cases passed" in completed.stdout
