"""K-only PID sampling adapter. No hardware or path-generation operations."""
import math
from visual_pid import PID, path_error

def continuous_paths(points):
    # Same image-space continuity limits as road_segments_v32.split_nodes (x2).
    chains=[]
    for point in points:
        if len(point)!=2 or any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in point):return []
        x,y=point
        if not (0<=x<1280 and 0<=y<720):return []
        if chains and y<=chains[-1][-1][1]:return []
        if not chains or y-chains[-1][-1][1]>24 or abs(x-chains[-1][-1][0])>100:chains.append([])
        chains[-1].append([x,y])
    return [c for c in chains if len(c)>=2]

class KAdaptivePID:
    def __init__(self,parameters):
        if parameters.get('direction_enabled'):raise ValueError('K動態取樣目前需關閉近遠列方向混合')
        self.pid=PID(parameters);self.requested=parameters['row_ratio'];self.row=self.requested
        from outward_path import Outward
        self.outward=Outward(parameters.get('curve_outward_px_640',0.))
        self.previous=[];self.t=None

    def step_path(self,t,points,envelope=None):
        choices=[]
        for chain in continuous_paths(points):
            lo=max(self.requested,chain[0][1]/719+.001);hi=min(.95,chain[-1][1]/719-.001)
            if lo<=hi:choices.append((lo,hi,chain))
        old=self.row;rebase=False;clamped=False;path=[];bounds=None;outward=None
        if choices:
            lo,hi,path=min(choices,key=lambda c:(c[0],-c[1]));bounds=[lo,hi]
            dt=max(0,t-self.t) if self.t is not None else 0
            smooth=old+max(-.2*dt,min(.2*dt,lo-old))
            self.row=max(lo,min(hi,smooth));clamped=abs(self.row-old)>.2*dt+1e-6
            if self.outward.amount:
                outward=self.outward.step(points,envelope or [],self.row*719/2,t)
                shifted={y:x for x,y in outward['path']}
                path=[[shifted[y],y] for x,y in path]
            error=path_error(path,self.row,visual_center_offset_px_640=self.pid.p.get('visual_center_offset_px_640',0.))
            if abs(self.row-old)>1e-8 and self.pid.e is not None:
                previous_error=path_error(self.previous,self.row,visual_center_offset_px_640=self.pid.p.get('visual_center_offset_px_640',0.))
                self.pid.e=error if previous_error is None else previous_error
                if previous_error is None:self.pid.d=0.
                rebase=True
        else:error=None
        result=self.pid.step(t,error)
        result.update(visual_center_version=1,visual_center_offset_px_640=self.pid.p.get('visual_center_offset_px_640',0.),sample_row_ratio=self.row,requested_row_ratio=self.requested,
                      sample_reason='NO_SUPPORTED_ROW' if bounds is None else 'ADAPTED_NEAR' if self.row>self.requested+.001 else 'REQUESTED',
                      derivative_rebased=rebase,sample_clamped=clamped,sample_bounds=bounds,
                      sampling_version='k-adaptive-r1')
        if outward is not None:result['outward']=outward
        self.previous=path;self.t=t
        return result
