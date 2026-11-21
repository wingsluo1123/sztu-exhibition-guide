"""Announce valid navigation status only for the pose actually published."""
import math


def navigation_status(source, published_stamp, now, map_hash):
    valid=source.get('valid',False)
    if valid:
        stamp=source.get('stamp')
        if not all(isinstance(v,(int,float)) and math.isfinite(v)
                   for v in (stamp,published_stamp,now)):
            return None
        # The source calls publish_status before publish_pose. Do not announce
        # that new valid sample using the previous navigation output timestamp.
        # The caller invokes us again immediately after publishing pose/TF.
        if abs(stamp-published_stamp)>1e-6:
            return None
    ready=bool(valid and published_stamp is not None and 0<=now-published_stamp<=.5)
    return {'valid':ready,'source_valid':bool(valid),'stamp':published_stamp,
            'map_manifest_sha256':map_hash,'reason':source.get('reason'),
            'mode':'diagnostic_no_actuator'}
