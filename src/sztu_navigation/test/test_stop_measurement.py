import pytest

from sztu_navigation.stop_measurement import StopMeasurement


def ready_measurement():
    measurement = StopMeasurement()
    for index in range(22):
        measurement.pose(index * 0.1, (0.0, 0.0), 0.0)
    assert measurement.phase == 'ready'
    return measurement


def test_zero_command_and_settled_odometry_produce_bounded_evidence():
    measurement = ready_measurement()
    measurement.command(2.11, 0.1, 0.0, 0.0)
    measurement.pose(2.2, (0.04, 0), 0)
    measurement.command(2.21, 0, 0, 0)
    for index in range(14):
        measurement.pose(2.3 + index * 0.1, (0.055, 0), 0)
    assert measurement.phase == 'finished'
    assert measurement.observed_after_zero == pytest.approx(0.015)
    assert measurement.report()['hardware_stop_verified'] is False


@pytest.mark.parametrize('values', [(0.11, 0, 0), (-0.1, 0, 0), (0.1, .01, 0),
                                   (0.1, 0, .01), (float('nan'), 0, 0)])
def test_out_of_envelope_command_latches_failure(values):
    measurement = ready_measurement()
    measurement.command(2.11, *values)
    assert measurement.phase == 'aborted'
    measurement.command(2.12, 0.1, 0, 0)
    assert measurement.phase == 'aborted'


@pytest.mark.parametrize('position,yaw,reason', [((.2, 0), 0, 'distance_limit'),
                                               ((.01, .06), 0, 'not_straight_forward'),
                                               ((-.04, 0), 0, 'not_straight_forward'),
                                               ((.01, 0), .11, 'heading_changed')])
def test_distance_and_direction_limits(position, yaw, reason):
    measurement = ready_measurement()
    measurement.command(2.11, .1, 0, 0)
    measurement.pose(2.2, position, yaw)
    assert measurement.reason == reason


def test_missing_stream_is_not_mistaken_for_a_stop_command():
    measurement = ready_measurement()
    measurement.command(2.11, .1, 0, 0)
    measurement.tick(2.4, True)
    assert measurement.reason == 'command_stream_lost_without_zero'
    assert measurement.stop_pose is None


def test_sensor_loss_and_duration_limit():
    measurement = ready_measurement()
    measurement.command(2.11, .1, 0, 0)
    measurement.tick(2.12, False)
    assert measurement.reason == 'sensor_data_unavailable'
    measurement = ready_measurement()
    measurement.command(2.11, .1, 0, 0)
    measurement.command(4.10, .1, 0, 0)
    measurement.tick(4.12, True)
    assert measurement.reason == 'duration_limit'


def test_start_requires_recent_odometry():
    measurement = ready_measurement()
    measurement.command(3.0, .1, 0, 0)
    assert measurement.reason == 'odometry_stale'


def test_zero_during_standstill_is_not_a_braking_test():
    measurement = ready_measurement()
    measurement.command(2.11, 0, 0, 0)
    assert measurement.phase == 'ready'
    assert measurement.observed_after_zero is None


def test_command_without_actual_translation_cannot_pass():
    measurement = ready_measurement()
    measurement.command(2.11, .1, 0, 0)
    measurement.command(2.12, 0, 0, 0)
    for index in range(14):
        measurement.pose(2.2 + index * .1, (0, 0), 0)
    assert measurement.reason == 'no_observed_translation'


def test_resume_after_zero_aborts_and_never_automatically_restarts():
    measurement = ready_measurement()
    measurement.command(2.11, .1, 0, 0)
    measurement.command(2.12, 0, 0, 0)
    measurement.command(2.13, .1, 0, 0)
    assert measurement.reason == 'unexpected_motion_command'
