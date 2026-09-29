import json

import pytest

from sztu_interaction.wifi_provisioning_server import (
    MAX_REQUEST_BYTES,
    WifiRequestError,
    parse_wifi_request,
)


def test_parse_wifi_request_accepts_utf8_credentials():
    request = json.dumps(
        {'ssid': '机器人 Wi-Fi', 'password': 'correct horse'},
        ensure_ascii=False,
    ).encode('utf-8')

    assert parse_wifi_request(request) == ('机器人 Wi-Fi', 'correct horse')


@pytest.mark.parametrize('payload', [b'', b'{', b'[]', b'{"ssid":1,"password":"x"}'])
def test_parse_wifi_request_rejects_invalid_payload(payload):
    with pytest.raises(WifiRequestError):
        parse_wifi_request(payload)


def test_parse_wifi_request_rejects_oversized_payload():
    payload = b' ' * (MAX_REQUEST_BYTES + 1)

    with pytest.raises(WifiRequestError, match='request_too_large'):
        parse_wifi_request(payload)


def test_parse_wifi_request_rejects_invalid_ssid_length():
    request = json.dumps({'ssid': 'x' * 33, 'password': 'secret'}).encode()

    with pytest.raises(WifiRequestError):
        parse_wifi_request(request)
