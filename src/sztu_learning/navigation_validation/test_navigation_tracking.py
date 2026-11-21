from navigation_tracking import NavigationTracker


def test_acquisition_waits_but_never_becomes_valid_without_match():
    t=NavigationTracker(None,[0,0,0])
    for i in range(151):
        now=i*.1
        assert t.update_odometry(now,[0,0,0],now)
        assert not t.status(now)['valid']
    assert not t.update_odometry(15.1,[0,0,0],15.1)
    assert t.state=='lost'


def test_acquisition_age_and_tracking_deadline_unchanged():
    t=NavigationTracker(None,[0,0,0]);t.update_odometry(0,[0,0,0],0)
    assert not t.status(.41)['valid'] and t.state=='lost'
    t=NavigationTracker(None,[0,0,0]);t.update_odometry(0,[0,0,0],0)
    job={'stamp':0,'odom':[0,0,0]}
    assert t.apply_match(job,{'accepted':True,'pose_xy_yaw':[0,0,0]},.1)
    for i in range(1,21):t.update_odometry(i*.1,[0,0,0],i*.1)
    assert not t.update_odometry(2.1,[0,0,0],2.1)
    assert t.reason=='map_match_timeout_requires_reinitialization'
