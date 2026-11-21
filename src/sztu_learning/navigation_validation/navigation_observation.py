"""One timestamped control observation; legacy display topics stay separate."""
import json
import math
import time

import numpy as np

from planar_math import matrix, pose

FRAMES = ['learning_map', 'learning_nav_odom', 'learning_nav_base']


def observation_packet(status, local_base=None, map_from_odom=None):
    packet = {'schema': 1, 'frames': FRAMES, 'status': status}
    if status['valid']:
        packet.update(odom_pose=list(map(float, local_base)),
                      map_from_odom=list(map(float, map_from_odom)))
    return packet


def decode_observation(data):
    packet = json.loads(data)
    if (packet.get('schema') != 1 or packet.get('frames') != FRAMES
            or not isinstance(packet.get('status'), dict)
            or type(packet['status'].get('valid')) is not bool):
        raise ValueError('invalid observation contract')
    status = packet['status']
    if not status['valid']:
        return status, None, None
    stamp = status.get('stamp')
    if (type(stamp) not in (float, int) or not math.isfinite(stamp)
            or status.get('source_valid') is not True
            or not isinstance(status.get('map_manifest_sha256'), str)):
        raise ValueError('invalid observation timestamp/map')
    for key in ('odom_pose', 'map_from_odom'):
        values = packet.get(key)
        if (not isinstance(values, list) or len(values) != 3
                or not all(type(v) in (float, int) and math.isfinite(v) for v in values)):
            raise ValueError('invalid planar observation')
    return status, tuple(packet['odom_pose']), matrix(packet['map_from_odom'])


class ObservationConsumer:
    """Adopt odometry, map transform and validity together in one callback.

    Requires the TrialObserver fields, or equivalent fields in a diagnostic node.
    Invalid/malformed input immediately invalidates readiness. Old valid packets
    never refresh reception times; the existing source/receipt age gates apply.
    """

    def receive_observation(self, message):
        try:
            status, odom, map_matrix = decode_observation(message.data)
        except (ValueError, TypeError, KeyError, AttributeError):
            self.status = None
            self.invalid_odometry = True
            return
        if not status['valid']:
            self.status = status
            self.status_received = time.monotonic()
            return
        stamp = status['stamp']
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        received = time.monotonic()
        self.odom = odom
        self.map_matrix = map_matrix
        self.map_pose = pose(map_matrix @ matrix(odom))
        self.status = status
        self.last_stamp = self.map_stamp = stamp
        self.odom_received = self.map_received = self.status_received = received
