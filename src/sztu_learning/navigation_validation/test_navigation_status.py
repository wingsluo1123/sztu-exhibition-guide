"""Regression for observed RViz run08; all freshness limits stay unchanged."""
from navigation_status import navigation_status
from goal_navigation.input_readiness import inputs_ready,MAP_HASH


def test_new_source_status_waits_for_matching_navigation_output():
    # Run08 stop: pose/TF age ~.270s, old status stamp age ~.371s.
    # The previous implementation announces a fresh status with the OLD stamp.
    epoch=100.;old_stamp=99.6288;new_stamp=99.7295
    source={'valid':True,'stamp':new_stamp,'reason':'map_match_accepted'}
    old_status={'valid':True,'stamp':old_stamp,'map_manifest_sha256':MAP_HASH}
    assert not inputs_ready(old_status,new_stamp,new_stamp,10.,10.,10.,epoch,10.)
    assert navigation_status(source,old_stamp,epoch,MAP_HASH) is None
    aligned=navigation_status(source,new_stamp,epoch,MAP_HASH)
    assert inputs_ready(aligned,new_stamp,new_stamp,10.,10.,10.,epoch,10.)


def test_invalid_source_announced_immediately_even_with_old_output():
    source={'valid':False,'stamp':None,'reason':'input_timeout_requires_reinitialization'}
    status=navigation_status(source,99.,100.,MAP_HASH)
    assert status['valid'] is False and status['source_valid'] is False
    assert status['reason']==source['reason']
    assert not inputs_ready(status,99.9,99.9,10.,10.,10.,100.,10.)


def test_aligned_but_stale_or_mismatched_map_data_still_stops():
    source={'valid':True,'stamp':99.6,'reason':'map_match_accepted'}
    aligned=navigation_status(source,99.6,100.,MAP_HASH)
    assert not inputs_ready(aligned,99.6,99.6,10.,10.,10.,100.,10.)
    source['stamp']=99.9
    wrong_map=navigation_status(source,99.9,100.,'other-map')
    assert not inputs_ready(wrong_map,99.9,99.9,10.,10.,10.,100.,10.)


def test_first_valid_source_does_not_invent_navigation_output():
    assert navigation_status({'valid':True,'stamp':99.9},None,100.,MAP_HASH) is None
    lost=navigation_status({'valid':False,'reason':'initializing'},None,100.,MAP_HASH)
    assert lost['stamp'] is None and not lost['valid']
