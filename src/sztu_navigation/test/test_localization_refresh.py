from types import SimpleNamespace
from unittest.mock import Mock

from rclpy.task import Future

from sztu_navigation.safety_gateway import SafetyGateway


def make_gateway(monkeypatch):
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr('sztu_navigation.safety_gateway.time.monotonic', lambda: clock.now)
    client = Mock()
    client.service_is_ready.return_value = True
    client.call_async.side_effect = lambda request: Future()
    gateway = SimpleNamespace(amcl_update=client, update_pending=None, update_started=0.0)
    return gateway, clock


def test_lost_reply_times_out_and_retries_without_accumulating_requests(monkeypatch):
    gateway, clock = make_gateway(monkeypatch)
    SafetyGateway.refresh_localization(gateway)
    first = gateway.update_pending
    clock.now = 100.5
    SafetyGateway.refresh_localization(gateway)
    assert gateway.amcl_update.call_async.call_count == 1
    assert not first.cancelled()
    clock.now = 101.0
    SafetyGateway.refresh_localization(gateway)
    gateway.amcl_update.remove_pending_request.assert_called_once_with(first)
    assert first.cancelled()
    assert gateway.update_pending is not first
    assert gateway.amcl_update.call_async.call_count == 2
    second = gateway.update_pending
    clock.now = 102.0
    SafetyGateway.refresh_localization(gateway)
    assert second.cancelled()
    assert gateway.amcl_update.remove_pending_request.call_count == 2
    assert gateway.amcl_update.call_async.call_count == 3


def test_service_restart_clears_timed_out_request_and_recovers(monkeypatch):
    gateway, clock = make_gateway(monkeypatch)
    SafetyGateway.refresh_localization(gateway)
    first = gateway.update_pending
    gateway.amcl_update.service_is_ready.return_value = False
    clock.now = 102.0
    SafetyGateway.refresh_localization(gateway)
    assert first.cancelled()
    assert gateway.update_pending is None
    assert gateway.amcl_update.call_async.call_count == 1
    gateway.amcl_update.service_is_ready.return_value = True
    SafetyGateway.refresh_localization(gateway)
    assert gateway.amcl_update.call_async.call_count == 2


def test_successful_reply_allows_next_periodic_refresh(monkeypatch):
    gateway, clock = make_gateway(monkeypatch)
    SafetyGateway.refresh_localization(gateway)
    first = gateway.update_pending
    first.set_result(None)
    clock.now = 100.5
    SafetyGateway.refresh_localization(gateway)
    assert not first.cancelled()
    gateway.amcl_update.remove_pending_request.assert_not_called()
    assert gateway.amcl_update.call_async.call_count == 2
