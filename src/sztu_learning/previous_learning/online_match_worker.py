"""Separate CPU process keeps startup searches from blocking sensor callbacks."""
import time


MATCHER = None


def initialize(map_directory, surface_reference):
    global MATCHER
    from spatial_localizer import SpatialLocalizer
    MATCHER = SpatialLocalizer.from_reference(map_directory, surface_reference)
    MATCHER.refinement_workers = 3


def match(scan, seed, tracking):
    started = time.monotonic()
    result = MATCHER.match_tracking(scan, seed) if tracking else MATCHER.match(scan, seed)
    result['matching_wall_s'] = time.monotonic() - started
    return result
