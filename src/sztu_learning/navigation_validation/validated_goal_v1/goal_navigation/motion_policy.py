"""Stop-turn-go policy for supervised static-map commissioning.

No forward/turn blending. Goals are checked after a zero-command settling
interval; braking trigger is never reported as arrival. No automatic recovery.
"""
from dataclasses import dataclass
import math


def wrap(angle):return math.remainder(angle,2*math.pi)


@dataclass
class Command:
    forward: float=0.
    angular: float=0.
    phase: str='idle'


class MotionPolicy:
    def __init__(self, route, goal_yaw=None, turn_only=False, duration=30.,
                 linear=.15, angular=.70, tolerance=.15, yaw_tolerance=.20,
                 coast=.21, turn_lead=.04, settle=2., max_distance=2.):
        if (not route or any(len(p)!=2 for p in route)
                or not all(math.isfinite(v) for p in route for v in p)
                or (goal_yaw is not None and not math.isfinite(goal_yaw))):
            raise ValueError('invalid route/heading')
        if not (0<duration<=120 and 0<linear<=.15 and 0<angular<=.70
                and .1<=tolerance<=.3 and .05<=yaw_tolerance<=.3
                and 0<=coast<=.3 and 0<=turn_lead<=.15 and 1<=settle<=5 and 0<max_distance<=10):
            raise ValueError('unsupported commissioning limits')
        self.route=[list(p) for p in route];self.goal_yaw=goal_yaw
        self.turn_only=turn_only;self.duration=duration;self.linear=linear;self.angular=angular
        self.tolerance=tolerance;self.yaw_tolerance=yaw_tolerance;self.coast=coast
        self.turn_lead=turn_lead;self.settle=settle;self.max_distance=max_distance
        self.index=0 if len(route)==1 else 1
        self.phase='decide';self.reason=None;self.started=None;self.settle_since=None
        self.travel=0.;self.last=None;self.reference=None;self.attempts=0
        self.turn_direction=None

    def stop(self,reason):
        if self.reason is None:self.reason=reason
        self.phase='arrived' if self.reason=='arrived' else 'stopped'
        return Command(phase=self.phase)

    def step(self,now,pose,ready):
        if self.reason:return Command(phase=self.phase)
        if not ready or len(pose)!=3 or not all(math.isfinite(v) for v in [now,*pose]):return self.stop('input_invalid')
        if self.started is None:self.started=now
        if now<self.started or now-self.started>=self.duration:return self.stop('duration_limit')
        if self.last is not None:
            dt=now-self.last[0]
            if dt<=0 or dt>.3:return self.stop('control_gap')
            displacement=math.dist(pose[:2],self.last[1][:2])
            if displacement>.15:return self.stop('odometry_jump')
            self.travel+=displacement
            if self.travel>self.max_distance:return self.stop('travel_limit')
        self.last=(now,tuple(pose))
        target=self.route[self.index];distance=math.dist(pose[:2],target)
        if self.phase=='settle':
            if now-self.settle_since<self.settle:return Command(phase='settle')
            self.phase='decide'
        if self.phase=='drive':
            dx,dy=pose[0]-self.reference[0],pose[1]-self.reference[1]
            heading=self.reference[2]
            forward=math.cos(heading)*dx+math.sin(heading)*dy
            lateral=-math.sin(heading)*dx+math.cos(heading)*dy
            if abs(lateral)>.20:return self.stop('route_deviation')
            if forward>=self.drive_length or distance<=self.coast+.05 or abs(wrap(pose[2]-heading))>.25:
                return self._settle(now)
            return Command(self.linear,0.,'drive')
        if self.phase=='turn':
            error=wrap(self.turn_target-pose[2])
            if abs(error)<=self.yaw_tolerance+self.turn_lead or error*self.turn_direction<0:
                return self._settle(now)
            return Command(0.,self.turn_direction*self.angular,'turn')
        if self.turn_only or distance<=self.tolerance:
            if not self.turn_only and self.index<len(self.route)-1:
                self.index+=1;return Command(phase='next_segment')
            if self.goal_yaw is None or abs(wrap(self.goal_yaw-pose[2]))<=self.yaw_tolerance:
                return self.stop('arrived')
            return self._turn(now,pose,self.goal_yaw)
        heading=math.atan2(target[1]-pose[1],target[0]-pose[0])
        if abs(wrap(heading-pose[2]))>self.yaw_tolerance:
            return self._turn(now,pose,heading)
        self.attempts+=1
        if self.attempts>20:return self.stop('maneuver_limit')
        self.reference=tuple(pose)
        # Bound each straight maneuver. Leave room for measured gait stopping.
        self.drive_length=min(.55,max(.04,distance-self.coast-.04))
        self.phase='drive'
        return Command(self.linear,0.,'drive')

    def _turn(self,now,pose,target):
        self.attempts+=1
        if self.attempts>20:return self.stop('maneuver_limit')
        self.turn_target=target;self.turn_direction=1 if wrap(target-pose[2])>0 else -1
        self.phase='turn'
        return Command(0.,self.turn_direction*self.angular,'turn')

    def _settle(self,now):
        self.phase='settle';self.settle_since=now
        return Command(phase='settle')
