import math
from pathlib import Path
import numpy as np
import pytest
from grid_route import GridRoute
from motion_policy import MotionPolicy


def test_unknown_and_wall_block_and_no_corner_cutting():
    grid=np.zeros((25,25),dtype=np.int8);grid[:,12]=-1
    route=GridRoute(grid,.1,[0,0,0],.12)
    with pytest.raises(ValueError,match='no known free'):route.plan([.5,1],[2,1])
    grid[10:15,12]=0
    route=GridRoute(grid,.1,[0,0,0],.12)
    points=route.plan([.5,.5],[2,.5])
    assert len(points)>2
    assert all(route.segment_clear(a,b) for a,b in zip(points,points[1:]))
    assert not route.segment_clear([.5,.5],[2,.5])


def test_rotated_origin_and_outside_rejection():
    r=GridRoute(np.zeros((20,20)),.1,[2,3,math.pi/2],.1)
    assert r.cell(r.world((8,9)))==(8,9)
    with pytest.raises(ValueError,match='outside'):r.cell([20,30])


def test_point_inside_wall_and_invalid_goal_rejected():
    grid=np.zeros((15,15));grid[7,7]=100
    r=GridRoute(grid,.1,[0,0,0],.1)
    with pytest.raises(ValueError):r.plan([.4,.4],[.75,.75])
    with pytest.raises(ValueError):r.plan([.4,.4],[float('nan'),.4])


def test_diagonal_touching_block_is_not_visible():
    grid=np.zeros((20,20));grid[8,9]=100
    r=GridRoute(grid,.1,[0,0,0],.01)
    assert not r.segment_clear(r.world((8,8)),r.world((9,9)))


def simulate(policy,pose=(0.,0.,0.),max_steps=4000):
    pose=list(pose);v=w=0.;history=[];dt=.05
    # Independent toy plant: observed actual speeds differ from command;
    # zero command leaves a 0.65s first-order gait tail. Not hardware evidence.
    for step in range(max_steps):
        cmd=policy.step(step*dt,pose,True)
        history.append((cmd,pose.copy()))
        target_v=cmd.forward*2.;target_w=cmd.angular*(1.14 if cmd.angular>0 else .79)
        v+=(target_v-v)*min(1,dt/.65);w+=(target_w-w)*min(1,dt/.12)
        pose[0]+=v*math.cos(pose[2])*dt;pose[1]+=v*math.sin(pose[2])*dt;pose[2]+=w*dt
        assert not(cmd.forward and cmd.angular)
        if policy.reason:return policy,pose,history
    raise AssertionError('policy did not terminate')


@pytest.mark.parametrize('angle',[math.pi/2,-math.pi/2,math.pi-.1])
def test_both_turn_directions_with_lag(angle):
    p,pose,h=simulate(MotionPolicy([[0,0]],goal_yaw=angle,turn_only=True,duration=15,max_distance=2))
    assert p.reason=='arrived'
    assert abs(math.remainder(pose[2]-angle,2*math.pi))<=.20
    assert any(c.phase=='settle' for c,_ in h)


def test_general_goal_behind_robot_and_final_heading():
    p,pose,h=simulate(MotionPolicy([[0,0],[-.8,.5]],goal_yaw=-.5,duration=80,max_distance=5))
    assert p.reason=='arrived'
    assert math.dist(pose[:2],[-.8,.5])<=.15
    assert abs(math.remainder(pose[2]+.5,2*math.pi))<=.2
    assert {c.phase for c,_ in h}>={'turn','drive','settle'}


def test_corner_route_and_no_false_arrival_at_brake_trigger():
    p,pose,h=simulate(MotionPolicy([[0,0],[.7,0],[.7,.7]],duration=80,max_distance=5))
    assert p.reason=='arrived'
    assert math.dist(pose[:2],[.7,.7])<=.15
    assert any(c.phase=='settle' and math.dist(x[:2],[.7,.7])>.15 for c,x in h)


def test_fault_latches_no_resume_and_timeout():
    p=MotionPolicy([[0,0],[1,0]])
    assert p.step(0,[0,0,0],True).forward>0
    assert p.step(.1,[0,0,0],False).forward==0
    assert p.step(.2,[0,0,0],True).forward==0
    assert p.reason=='input_invalid'
    p=MotionPolicy([[0,0],[1,0]],duration=1)
    for i in range(11):p.step(i*.1,[0,0,0],True)
    assert p.reason=='duration_limit'


def test_real_map_lineage_and_reachable_route():
    root=Path(__file__).resolve().parents[2]/'previous_learning/maps/room_7x3_20261120_map01'
    r=GridRoute.load(root,.4)
    assert r.manifest_hash=='f2b1f398b010e58de0b276646d59d2905c48dac805f90226d67e69429fcf51c0'
    # Choose analytical endpoints from allowed cells; no invented on-site goal.
    cells=np.argwhere(r.allowed);a=r.world(cells[len(cells)//2]);b=r.world(cells[len(cells)//2+10])
    path=r.plan(a,b)
    assert all(r.segment_clear(a,b) for a,b in zip(path,path[1:]))
