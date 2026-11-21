import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('validation_planar_math', Path(__file__).with_name('planar_math.py'))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_rotated_correction_reconstructs_same_reference_pose():
    local = [1.2, -.3, 1.1]
    mapped = [-2., 4., -.9]
    result = module.correction(mapped, local)
    assert np.allclose(module.matrix(result) @ module.matrix(local), module.matrix(mapped))


def test_map_jump_does_not_change_local_velocity_and_wrap_is_short():
    # Local movement in +Y while facing +Y is forward, not lateral.
    assert np.allclose(module.velocity([0, 0, np.pi/2], [0, .01, np.pi/2], .1), [.1, 0, 0])
    result = module.velocity([0, 0, np.pi-.01], [0, 0, -np.pi+.01], .1)
    assert result[2] == pytest.approx(.2)


@pytest.mark.parametrize('dt', [0., -.1, 1.])
def test_bad_intervals_are_rejected(dt):
    with pytest.raises(ValueError):
        module.velocity([0, 0, 0], [0, 0, 0], dt)
