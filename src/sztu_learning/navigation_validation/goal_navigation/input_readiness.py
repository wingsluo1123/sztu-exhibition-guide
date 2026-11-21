"""Commissioning freshness budget; source validity and MC guard stay separate."""
import math

MAP_HASH='f2b1f398b010e58de0b276646d59d2905c48dac805f90226d67e69429fcf51c0'
MAX_SOURCE_AGE=.35


def inputs_ready(status,last_stamp,map_stamp,odom_received,status_received,map_received,
                 epoch,now,invalid_odometry=False):
    if invalid_odometry or not isinstance(status,dict):return False
    stamp=status.get('stamp')
    times=[stamp,last_stamp,map_stamp,odom_received,status_received,map_received,epoch,now]
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in times):return False
    return bool(status.get('valid') and status.get('map_manifest_sha256')==MAP_HASH
                and all(-.05<=epoch-v<=MAX_SOURCE_AGE for v in [stamp,last_stamp,map_stamp])
                and all(0<=now-v<=.25 for v in [odom_received,status_received,map_received])
                and abs(map_stamp-last_stamp)<.15)
