import numpy as np

from sztu_slam.gravity_estimator import StaticGravityEstimator


def feed(estimator, gyro=(0.0, 0.0, 0.0), acceleration=(0.0, 0.0, 9.8)):
    result = None
    for index in range(401):
        result = estimator.add(1.0 + index / 200.0, acceleration, gyro)
    return result


def test_static_window_requires_quiet_gyro():
    estimator = StaticGravityEstimator()
    assert feed(estimator, gyro=(0.8, 0.0, 0.0)) is None
    assert estimator.count == 0


def test_static_window_requires_low_acceleration_variance():
    estimator = StaticGravityEstimator()
    result = None
    for index in range(401):
        value = 9.8 if index % 2 else 8.0
        result = estimator.add(1.0 + index / 200.0, (0.0, 0.0, value), (0.0, 0.0, 0.0))
    assert result is None


def test_static_window_returns_mean_after_continuous_quiet_samples():
    estimator = StaticGravityEstimator()
    result = feed(estimator)
    assert np.allclose(result, [0.0, 0.0, 9.8])


def test_gaps_duplicates_and_nonfinite_samples_restart_window():
    estimator = StaticGravityEstimator()
    for index in range(300):
        assert estimator.add(1 + index / 200, (9.8, 0, 0), (0, 0, 0)) is None
    assert estimator.add(5, (9.8, 0, 0), (0, 0, 0)) is None
    assert estimator.count == 1
    assert estimator.add(5, (9.8, 0, 0), (0, 0, 0)) is None
    assert estimator.count == 1
    assert estimator.add(5.005, (np.nan, 0, 0), (0, 0, 0)) is None
    assert estimator.count == 0
