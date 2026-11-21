import pytest
from input_readiness import inputs_ready,MAP_HASH


def sample():
    return dict(status={'valid':True,'stamp':99.70,'map_manifest_sha256':MAP_HASH},
                last_stamp=99.70,map_stamp=99.70,odom_received=9.9,
                status_received=9.95,map_received=9.9,epoch=100.,now=10.)


def test_frozen_commissioning_age_budget():
    assert inputs_ready(**sample())
    for name in ['last_stamp','map_stamp']:
        v=sample();v[name]=99.64
        assert not inputs_ready(**v)
    v=sample();v['status']['stamp']=99.64
    assert not inputs_ready(**v)


@pytest.mark.parametrize('change',[
    {'invalid_odometry':True},{'odom_received':9.7},{'map_received':9.7},
    {'status_received':9.7},{'map_stamp':100.1},{'last_stamp':float('nan')},
    {'status':{'valid':False,'stamp':99.7,'map_manifest_sha256':MAP_HASH}},
    {'status':{'valid':True,'stamp':99.7,'map_manifest_sha256':'wrong map'}},
])
def test_gate_rejects_independent_faults(change):
    v=sample();v.update(change)
    assert not inputs_ready(**v)
