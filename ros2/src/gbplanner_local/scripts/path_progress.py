#!/usr/bin/env python3
"""Track progress along an ordered polyline using measured position only.

This is a ROS 2 safety adaptation, not extracted GBPlanner logic. Keep a short
already-travelled guard segment so projection tolerance cannot omit a corner.
No elapsed-time estimate or globally nearest later branch advances the cursor.
"""
import bisect
import math


class PathProgress:
    def __init__(self,points,tolerance=.15,guard=.25,max_step=.75):
        if not points or any(len(p)!=3 or not all(math.isfinite(x) for x in p) for p in points):
            raise ValueError('invalid_progress_path')
        if not 0<tolerance<=guard or max_step<=0 or not all(math.isfinite(x) for x in (tolerance,guard,max_step)):
            raise ValueError('invalid_progress_parameters')
        self.points=[tuple(points[0])]
        for point in points[1:]:
            if math.dist(point,self.points[-1])>1e-9:self.points.append(tuple(point))
        self.arc=[0.]
        for a,b in zip(self.points,self.points[1:]):self.arc.append(self.arc[-1]+math.dist(a,b))
        self.tolerance=tolerance;self.guard=guard;self.max_step=max_step
        self.progress=0.;self.previous=self.points[0]

    def point_at(self,distance):
        if len(self.points)==1 or distance>=self.arc[-1]:return self.points[-1]
        index=max(0,bisect.bisect_right(self.arc,distance)-1)
        fraction=(distance-self.arc[index])/(self.arc[index+1]-self.arc[index])
        return tuple(a+(b-a)*fraction for a,b in zip(self.points[index],self.points[index+1]))

    def update(self,current):
        current=tuple(current)
        if len(current)!=3 or not all(math.isfinite(v) for v in current):raise ValueError('invalid_progress_position')
        displacement=math.dist(self.previous,current)
        if displacement>self.max_step:raise ValueError('odometry_progress_jump')
        lower=max(0.,self.progress-self.guard)
        upper=min(self.arc[-1],self.progress+displacement+self.tolerance)
        candidates=[]
        if len(self.points)==1:candidates=[(math.dist(current,self.points[0]),0.)]
        start=max(0,bisect.bisect_right(self.arc,lower)-1)
        for i in range(start,len(self.points)-1):
            if self.arc[i]>upper:break
            a,b=self.points[i:i+2];length=self.arc[i+1]-self.arc[i]
            raw=sum((c-x)*(y-x) for c,x,y in zip(current,a,b))/length
            along=max(lower,min(upper,self.arc[i]+max(0.,min(length,raw))))
            point=self.point_at(along);candidates.append((math.dist(current,point),along))
        if not candidates:raise ValueError('path_progress_missing')
        error,along=min(candidates,key=lambda value:(round(value[0],10),value[1]))
        if error>self.tolerance:raise ValueError('odometry_off_commanded_path')
        if along+self.tolerance<self.progress:raise ValueError('odometry_path_backtracking')
        self.progress=max(self.progress,along);self.previous=current
        return self.progress

    def remaining(self,current):
        # Connect toward the next original forward vertex, as the commanded
        # route does. A lateral chord to the progress projection can introduce
        # a cross-slope the robot never drives; a chord to the guard can sweep
        # an obstacle outside the forward connector and original polyline.
        # Retrace the original vertices to that guard, then retain the full
        # forward suffix. This is validation geometry, never a motion command.
        cutoff=max(0.,self.progress-self.guard)
        first=bisect.bisect_right(self.arc,cutoff)
        if len(self.points)==1:return [tuple(current),self.points[0]]
        end=min(bisect.bisect_right(self.arc,self.progress),len(self.points)-1)
        return ([tuple(current),self.points[end]]+
                list(reversed(self.points[first:end]))+
                [self.point_at(cutoff)]+self.points[first:])
