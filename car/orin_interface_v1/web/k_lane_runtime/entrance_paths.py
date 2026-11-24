"""Offline candidates only. Shared entrance association and adaptive PID sampling.

Heuristic scores are diagnostic, not calibrated probabilities. No hardware imports.
"""
import sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'web'))
from visual_pid import PID, path_error
from calibrated_path import offset
from ground_calibration import project
from temporal_lane import estimate_motion, transform

ALGORITHMS = ('entrance-image-v1', 'entrance-temporal-v1', 'entrance-v7-v1')

def traces(mask):
    """Trace all near-field white runs upward, retaining boundary identity in bends."""
    mask=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
    tracks=[]
    for y in range(354,111,-4):
        line=mask[max(0,y-1):y+2].mean(0)>.3
        ds=np.diff(np.r_[False,line,False].astype('int8'))
        runs=[(float(a),float(b-1)) for a,b in zip(np.flatnonzero(ds==1),np.flatnonzero(ds==-1)) if 2<=b-a<=180]
        used=set()
        links=[]
        for j,tr in enumerate(tracks):
            last=tr[-1];gap=last[0]-y
            if gap>16:continue
            slope=0 if len(tr)<2 else np.clip(((last[1]+last[2])-(tr[-2][1]+tr[-2][2]))/2/(last[0]-tr[-2][0]),-4,4)
            pred=(last[1]+last[2])/2-slope*gap
            for k,(a,b) in enumerate(runs):
                distance=abs((a+b)/2-pred)
                if distance<=12+gap*2:links.append((distance,j,k))
        linked=set()
        for _,j,k in sorted(links):
            if j in linked or k in used:continue
            tracks[j].append((y,*runs[k]));linked.add(j);used.add(k)
        # Side-entry boundaries can begin above the bottom edge.
        for k,(a,b) in enumerate(runs):
            if k not in used and y>=210:tracks.append([(y,a,b)])
    return [np.array(tr[::-1],float) for tr in tracks if len(tr)>=6]

def clean_path(points):
    a=np.asarray(points,float).reshape(-1,2)
    a=a[np.isfinite(a).all(1)&(a[:,0]>=0)&(a[:,0]<1280)&(a[:,1]>=0)&(a[:,1]<720)]
    a=a[np.argsort(a[:,1])]
    if len(a):a=a[np.r_[True,np.diff(a[:,1])>1]]
    return a

def sample_range(path, requested):
    if len(path)<2:return None
    lo=max(requested,float(path[0,1]/719)+.001);hi=min(.95,float(path[-1,1]/719)-.001)
    return (lo,hi) if hi>=lo else None

class EntrancePath:
    def __init__(self, algorithm, calibration=None, row=.6):
        self.algorithm=algorithm;self.calibration=calibration;self.row=row
        self.prev=None;self.prev_t=None;self.gray=None;self.width=None;self.width_t=None;self.mode=None
        self.H=np.asarray(calibration['matrix']) if calibration else None

    def step(self,image,mask,t):
        gray=cv2.cvtColor(cv2.resize(image,(640,360)),cv2.COLOR_BGR2GRAY)
        fresh=self.prev_t is not None and 0<t-self.prev_t<=.4
        prior=None;motion={'valid':False,'reason':'NOT_TEMPORAL'}
        if self.algorithm=='entrance-temporal-v1' and fresh and self.prev is not None:
            H,motion=estimate_motion(self.gray,gray)
            if H is not None:prior=clean_path(transform(self.prev/2,H)*2)
        if not fresh:self.width=None;self.mode=None
        curves=traces(mask);candidates=[];pairs=[]
        for i,l in enumerate(curves):
            for j,r in enumerate(curves):
                if i==j:continue
                ys=np.intersect1d(l[:,0],r[:,0])
                if len(ys)<6:continue
                left=np.interp(ys,l[:,0],l[:,2]);right=np.interp(ys,r[:,0],r[:,1])
                widths=right-left
                if np.any(widths<=8):continue
                # Only the near entrance must contain the camera-forward anchor.
                if not left[-1]<319.5<right[-1]:continue
                pair=(ys,left,right)
                pairs.append(pair)
        def add(mode,path,evidence,entry,extra=None):
            path=clean_path(path);span=sample_range(path,self.row)
            if len(path)<6:return
            continuity=0.
            if prior is not None and len(prior)>1:
                lo=max(path[0,1],prior[0,1]);hi=min(path[-1,1],prior[-1,1])
                if hi>lo:
                    yy=np.linspace(lo,hi,12)
                    continuity=float(np.mean(np.abs(np.interp(yy,path[:,1],path[:,0])-np.interp(yy,prior[:,1],prior[:,0])))/1280)
            # Comparable, inspectable components; no averaging unrelated routes.
            score=entry+evidence+(path[-1,1]-path[0,1])/720-2*continuity-(0 if span else 1)
            if mode==self.mode:score+=.08
            candidates.append(dict(mode=mode,path=path,score=float(score),sample_range=span,
                evidence=evidence,entrance=entry,continuity_cost=continuity,**(extra or {})))
        for ys,l,r in pairs:
            path=np.column_stack([(l+r),ys*2])
            extra={}
            if self.H is not None:
                # Equal-depth midpoint in ground coordinates, within observed support.
                sides=[project(np.column_stack([x,ys]),self.H) for x in (l,r)]
                sides=[s[np.isfinite(s).all(1)&(s[:,1]>0)] for s in sides]
                sides=[s[np.argsort(s[:,1])] for s in sides]
                if any(len(s)<6 for s in sides):continue
                lo=max(s[0,1] for s in sides);hi=min(s[-1,1] for s in sides)
                if hi<=lo:continue
                z=np.linspace(lo,hi,60)
                mid=np.column_stack([(np.interp(z,sides[0][:,1],sides[0][:,0])+np.interp(z,sides[1][:,1],sides[1][:,0]))/2,z])
                path=project(mid,np.linalg.inv(self.H))*2
                extra={'metric_support_cm':[float(lo),float(hi)],'outside_calibration_range':bool(lo<50 or hi>120)}
            add('double',path,.3,ys[-1]/360,dict(extra,width_profile=np.column_stack([ys,r-l]).tolist()))
        # All singles are evaluated in parallel with pairs, not after failure.
        for tr in curves:
            for side,col,sign in [('left',2,1),('right',1,-1)]:
                if (side=='left' and tr[-1,col]>=319.5) or (side=='right' and tr[-1,col]<=319.5):continue
                xy=np.column_stack([tr[:,col],tr[:,0]])
                extra={}
                if self.H is not None:
                    metric=project(xy,self.H);metric=metric[np.isfinite(metric).all(1)&(metric[:,1]>0)]
                    metric=metric[np.argsort(metric[:,1])]
                    if len(metric)<6:continue
                    try:middle=offset(metric,self.calibration['lane_width_cm']/2,side)
                    except ValueError:continue
                    path=project(middle,np.linalg.inv(self.H))*2
                    extra={'metric_support_cm':[float(middle[0,1]),float(middle[-1,1])],
                           'outside_calibration_range':bool(middle[0,1]<50 or middle[-1,1]>120)}
                elif self.width is not None and t-self.width_t<=.65:
                    use=(tr[:,0]>=self.width[0,0])&(tr[:,0]<=self.width[-1,0]);xy=xy[use]
                    if len(xy)<6:continue
                    path=np.column_stack([xy[:,0]+sign*np.interp(xy[:,1],self.width[:,0],self.width[:,1])/2,xy[:,1]])*2
                    extra={'width_age_s':t-self.width_t}
                else:continue
                add(side,path,0.,tr[-1,0]/360,extra)
        selected=max(candidates,key=lambda c:c['score']) if candidates else None
        path=selected['path'] if selected else np.empty((0,2));history_points=0
        # Only aligned recent history can fill a short missing near segment in B.
        if prior is not None and len(path)>1 and len(prior)>1:
            boundary=path[-1];close=abs(np.interp(boundary[1],prior[:,1],prior[:,0])-boundary[0])<60
            if close:
                fill=prior[(prior[:,1]>boundary[1])&(prior[:,1]<=boundary[1]+64)]
                history_points=len(fill);path=clean_path(np.vstack([path,fill]))
        if selected and selected['mode']=='double':
            self.width=np.asarray(selected['width_profile']);self.width_t=t
        before=self.mode;self.mode=selected['mode'] if selected else None
        # Do not recursively carry imputed history forward.
        self.prev=selected['path'].copy() if selected else None;self.prev_t=t;self.gray=gray
        diagnostics=[{k:v for k,v in c.items() if k not in ('path','width_profile')} for c in candidates]
        return dict(path=path.tolist(),valid=len(path)>=6,quality=float(selected['score']) if selected else 0,
            mode=self.mode or 'no_evidence',reason='ENTRANCE_CANDIDATE' if selected else 'NO_ENTRANCE_SUPPORTED_PATH',
            candidates=diagnostics,transition=f'{before} → {self.mode}',history_points=history_points,motion=motion,
            boundaries=[tr.tolist() for tr in curves],dynamic_sampling=True,
            metric_exploration=self.H is not None,hardware_output=False)

class AdaptivePID:
    """Frozen v37 gains; only sampling location adapts. Compensate coordinate changes."""
    def __init__(self,parameters):
        self.pid=PID(parameters);self.requested=parameters['row_ratio'];self.row=self.requested
        self.previous=[];self.t=None
    def step(self,t,points):
        path=clean_path(points);bounds=sample_range(path,self.requested)
        old=self.row;rebase=False
        if bounds:
            dt=0 if self.t is None else max(0,t-self.t)
            target=bounds[0]
            smooth=self.row+np.clip(target-self.row,-.2*dt,.2*dt)
            # Clamp to actual support rather than extrapolating during transition.
            self.row=float(np.clip(smooth,*bounds))
            error=path_error(path.tolist(),self.row,visual_center_offset_px_640=self.pid.p.get('visual_center_offset_px_640',0.))
            if abs(old-self.row)>1e-8 and self.pid.e is not None:
                prev=path_error(self.previous,self.row,visual_center_offset_px_640=self.pid.p.get('visual_center_offset_px_640',0.))
                self.pid.e=error if prev is None else prev
                if prev is None:self.pid.d=0.
                rebase=True
        else:error=None
        result=self.pid.step(t,error)
        result.update(visual_center_version=1,visual_center_offset_px_640=self.pid.p.get('visual_center_offset_px_640',0.),sample_row_ratio=self.row,requested_row_ratio=self.requested,
            sample_reason='NO_SUPPORTED_ROW' if bounds is None else 'ADAPTED_NEAR' if self.row>self.requested+.001 else 'REQUESTED',
            derivative_rebased=rebase,sample_clamped=bool(bounds and abs(self.row-old)>.2*max(0,t-(self.t or t))+.0001))
        self.previous=path.tolist();self.t=t
        return result
