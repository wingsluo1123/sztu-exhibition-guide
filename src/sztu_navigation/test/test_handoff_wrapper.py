"""Keep operator markers visible and preserve failure/reuse semantics under DDS noise."""

from pathlib import Path
import subprocess

import pytest


@pytest.mark.parametrize('exit_code', [0, 2])
def test_wrapper_filters_noisy_child_and_preserves_failure(tmp_path, exit_code):
    root = Path(__file__).resolve().parents[3]
    (tmp_path/'tools').mkdir()
    wrapper = tmp_path/'tools/run_restrained_handoff.sh'
    wrapper.write_bytes((root/'tools/run_restrained_handoff.sh').read_bytes())
    # Stub has no ROS imports, services or publishers; wrapper preview does not
    # source the hardware environment. This tests the actual pipeline code.
    child = tmp_path/'tools/restrained_drive_trial.py'
    child.write_text('import sys\n'
        'for _ in range(5000): print("DDS warning noisy participant")\n'
        'print("PULSE_COUNTDOWN: fixture only")\n'
        'print("PULSE_STARTED: fixture only")\n'
        'print("PULSE_ENDED: fixture only")\n'
        'print(\'{"phase": "fixture"}\')\n'
        f'sys.exit({exit_code})\n')
    prefix = str(tmp_path/'report')
    result = subprocess.run(['bash', str(wrapper), '--preview', prefix],
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == exit_code
    assert 'DDS warning' not in result.stdout
    assert 'PULSE_STARTED' in result.stdout
    assert ('RECORDING_COMPLETE' in result.stdout) == (exit_code == 0)
    assert (tmp_path/'report.log').read_text().count('DDS warning') == 5000
    repeat = subprocess.run(['bash', str(wrapper), '--preview', prefix],
                            capture_output=True, text=True, timeout=5)
    assert repeat.returncode == 2
    assert 'PULSE_STARTED' not in repeat.stdout
