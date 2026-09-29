#!/usr/bin/env python3
"""BLE GATT service for configuring the host Wi-Fi connection."""

import ipaddress
import json
import logging
import os
import shlex
import shutil
import subprocess
import threading
import time


SERVICE_UUID = '7d2ea28a-f7bd-485a-bd9d-92ad6ecfe93e'
REQUEST_UUID = '7d2ea28b-f7bd-485a-bd9d-92ad6ecfe93e'
STATUS_UUID = '7d2ea28c-f7bd-485a-bd9d-92ad6ecfe93e'
MAX_REQUEST_BYTES = 512
AIMA_TIMEOUT_SEC = 60
RESULT_MARKER = 'SZTU_WIFI_RESULT='

LOGGER = logging.getLogger(__name__)

AIMA_WORKER = r'''
import json
import sys
from aima.core.nm_api import connect_wifi

request = json.load(sys.stdin)
try:
    ip = connect_wifi(None, request['ssid'], request['password'])
    if not ip:
        raise RuntimeError('connection failed')
    result = {'ip': ip}
except Exception:
    result = {'error': 'connection_failed'}
    print('SZTU_WIFI_RESULT=' + json.dumps(result))
    raise SystemExit(1)
print('SZTU_WIFI_RESULT=' + json.dumps(result))
'''


class WifiRequestError(ValueError):
    """Raised when a BLE request does not contain valid Wi-Fi credentials."""


class WifiConnectionError(RuntimeError):
    """Raised when the vendor Wi-Fi manager cannot connect to the network."""


def parse_wifi_request(value):
    """Decode and validate a JSON request received by the BLE characteristic."""
    try:
        payload = bytes(value)
        if len(payload) > MAX_REQUEST_BYTES:
            raise WifiRequestError('request_too_large')
        request = json.loads(payload.decode('utf-8'))
    except WifiRequestError:
        raise
    except (TypeError, ValueError, UnicodeDecodeError) as error:
        raise WifiRequestError('invalid_request') from error

    if not isinstance(request, dict):
        raise WifiRequestError('invalid_request')

    ssid = request.get('ssid')
    password = request.get('password')
    if not isinstance(ssid, str) or not isinstance(password, str):
        raise WifiRequestError('invalid_request')
    if not ssid or len(ssid.encode('utf-8')) > 32:
        raise WifiRequestError('invalid_request')
    if len(password.encode('utf-8')) > 128:
        raise WifiRequestError('invalid_request')
    return ssid, password


def _aima_python_executable():
    """Resolve the Python interpreter from the installed aima command."""
    aima_path = shutil.which('aima')
    if not aima_path:
        raise WifiConnectionError('aima_unavailable')

    try:
        with open(aima_path, encoding='utf-8') as script:
            shebang = shlex.split(script.readline()[2:].strip())
    except (OSError, IndexError, ValueError) as error:
        raise WifiConnectionError('aima_unavailable') from error

    if not shebang:
        raise WifiConnectionError('aima_unavailable')
    if os.path.basename(shebang[0]) == 'env':
        if len(shebang) < 2 or shebang[1].startswith('-'):
            raise WifiConnectionError('aima_unavailable')
        executable = shutil.which(shebang[1])
    else:
        executable = shebang[0]
    if not executable or not os.path.isfile(executable):
        raise WifiConnectionError('aima_unavailable')
    return executable


class AimaWifiConnector:
    """Call the installed aima Wi-Fi API without putting credentials in argv."""

    def __init__(self, timeout=AIMA_TIMEOUT_SEC):
        self.timeout = timeout

    def connect(self, ssid, password):
        executable = _aima_python_executable()
        try:
            result = subprocess.run(
                [executable, '-c', AIMA_WORKER],
                input=json.dumps({'ssid': ssid, 'password': password}),
                capture_output=True,
                check=False,
                text=True,
                timeout=self.timeout,
            )
        except subprocess.TimeoutExpired as error:
            raise WifiConnectionError('connection_timeout') from error
        except OSError as error:
            raise WifiConnectionError('aima_unavailable') from error

        response = None
        for line in result.stdout.splitlines():
            if line.startswith(RESULT_MARKER):
                try:
                    response = json.loads(line[len(RESULT_MARKER):])
                except json.JSONDecodeError:
                    response = None

        if result.returncode != 0 or not isinstance(response, dict):
            error = response.get('error') if response else None
            raise WifiConnectionError(
                error if error in ('connection_failed',) else 'connection_failed'
            )

        try:
            return str(ipaddress.ip_address(response['ip']))
        except (KeyError, ValueError, TypeError) as error:
            raise WifiConnectionError('connection_failed') from error


class WifiProvisioningServer:
    """Expose Wi-Fi provisioning over an encrypted BLE GATT characteristic."""

    def __init__(self, connector=None):
        self.connector = connector or AimaWifiConnector()
        self._request_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._status = {'status': 'ready'}
        self._status_characteristic = None

    def run(self):
        import dbus
        from bluezero import adapter, peripheral

        while True:
            try:
                addresses = adapter.list_adapters()
            except Exception:
                LOGGER.warning('BLE adapter unavailable; retrying in 5 seconds')
                time.sleep(5)
                continue

            if not addresses:
                LOGGER.warning('No BLE adapter found; retrying in 5 seconds')
                time.sleep(5)
                continue

            ble_name = os.environ.get('SZTU_BLE_NAME', 'SZTU-WiFi')
            device = peripheral.Peripheral(addresses[0], local_name=ble_name)
            device.dongle.pairable = True
            device.dongle.pairabletimeout = dbus.UInt32(0)
            device.add_service(srv_id=1, uuid=SERVICE_UUID, primary=True)
            device.add_characteristic(
                srv_id=1,
                chr_id=1,
                uuid=REQUEST_UUID,
                value=[],
                notifying=False,
                flags=['write'],
                write_callback=self._on_write,
            )
            device.add_characteristic(
                srv_id=1,
                chr_id=2,
                uuid=STATUS_UUID,
                value=list(self._status_bytes()),
                notifying=False,
                flags=['read', 'notify'],
                read_callback=self._read_status,
            )
            self._status_characteristic = device.characteristics[-1]
            LOGGER.info('Advertising BLE Wi-Fi provisioning service as %s', ble_name)
            device.publish()
            return

    def _status_bytes(self):
        with self._status_lock:
            return json.dumps(self._status, separators=(',', ':')).encode('utf-8')

    def _read_status(self):
        return list(self._status_bytes())

    def _publish_status(self, status):
        with self._status_lock:
            self._status = status
            characteristic = self._status_characteristic
            payload = list(json.dumps(
                status, separators=(',', ':')
            ).encode('utf-8'))

        if characteristic is not None:
            try:
                from gi.repository import GLib

                GLib.idle_add(self._set_characteristic_value, characteristic, payload)
            except ImportError:
                characteristic.set_value(payload)

    @staticmethod
    def _set_characteristic_value(characteristic, payload):
        characteristic.set_value(payload)
        return False

    def _on_write(self, value, _options):
        try:
            ssid, password = parse_wifi_request(value)
        except WifiRequestError as error:
            self._publish_status({'status': 'error', 'error': str(error)})
            return

        if not self._request_lock.acquire(blocking=False):
            self._publish_status({'status': 'error', 'error': 'busy'})
            return

        self._publish_status({'status': 'connecting'})
        worker = threading.Thread(
            target=self._connect, args=(ssid, password), daemon=True
        )
        worker.start()

    def _connect(self, ssid, password):
        try:
            ip = self.connector.connect(ssid, password)
        except WifiConnectionError as error:
            self._publish_status({'status': 'error', 'error': str(error)})
        except Exception:
            LOGGER.exception('Wi-Fi provisioning failed')
            self._publish_status({'status': 'error', 'error': 'connection_failed'})
        else:
            self._publish_status({'status': 'connected', 'ip': ip})
        finally:
            self._request_lock.release()


def main():
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    try:
        WifiProvisioningServer().run()
    except KeyboardInterrupt:
        LOGGER.info('BLE Wi-Fi provisioning service stopped')


if __name__ == '__main__':
    main()