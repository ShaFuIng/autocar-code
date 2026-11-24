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

ALGORITHMS = ('entrance-image-v2', 'entrance-temporal-v2', 'entrance-v7-v2')

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
    a=a[np.isfinite(a).all(1)&(a[:,0]>=0)&(a[:,0]<=1279)&(a[:,1]>=0)&(a[:,1]<=719)]
    a=a[np.argsort(a[:,1])]
    if len(a):a=a[np.r_[True,np.diff(a[:,1])>1]]
    return a

def sample_range(path, requested):
    if len(path)<2:return None
    lo=max(requested,float(path[0,1]/719)+.001);hi=min(.95,float(path[-1,1]/719)-.001)
    return (lo,hi) if hi>=lo else None


def match_edge(current, previous):
    best=None
    for old in previous:
        old=np.asarray(old);old=old[np.argsort(old[:,1])]
        lo=max(current[0,1],old[0,1]);hi=min(current[-1,1],old[-1,1])
        if hi-lo<20:continue
        ys=np.linspace(lo,hi,20)
        delta=np.interp(ys,current[:,1],current[:,0])-np.interp(ys,old[:,1],old[:,0])
        value=(float(np.median(np.abs(delta))),float(np.median(delta)),float(lo),float(hi))
        if best is None or value[0]<best[0]:best=value
    return best

def local_offset(metric,distance,side):
    """Smooth local tangent; split genuine foldbacks without discarding good runs."""
    metric=np.asarray(metric,float);metric=metric[np.argsort(metric[:,1])]
    metric=metric[np.r_[True,np.diff(metric[:,1])>1e-5]]
    if len(metric)<6:return [],0
    z=np.linspace(metric[0,1],metric[-1,1],max(30,len(metric)))
    x=np.interp(z,metric[:,1],metric[:,0])
    # Fit locally, preserving ends; avoids derivative spikes from mask stair-steps.
    smooth=[];slope=[]
    for i,zi in enumerate(z):
        ix=slice(max(0,i-4),min(len(z),i+5));deg=min(2,len(z[ix])-1)
        c=np.polyfit(z[ix]-zi,x[ix],deg)
        smooth.append(float(np.polyval(c,0)));slope.append(float(np.polyval(np.polyder(c),0)))
    tangent=np.column_stack([slope,np.ones(len(z))]);normal=np.column_stack([tangent[:,1],-tangent[:,0]])
    normal/=np.linalg.norm(normal,axis=1)[:,None]
    middle=np.column_stack([smooth,z])+normal*distance*(1 if side=='left' else -1)
    cuts=np.flatnonzero(np.diff(middle[:,1])<=0)+1
    sections=[a for a in np.split(middle,cuts) if len(a)>=6]
    return sections,len(cuts)

def section_score(section,H,row):
    pixels=clean_path(project(section,np.linalg.inv(H))*2)
    return (sample_range(pixels,row) is not None,len(pixels),float(np.ptp(section[:,1])))

class EntrancePath:
    def __init__(self, algorithm, calibration=None, row=.6):
        self.algorithm=algorithm;self.calibration=calibration;self.row=row
        self.prev=None;self.prev_t=None;self.gray=None;self.width=None;self.width_t=None;self.mode=None
        self.selected_edges=[];self.rejections=[]
        self.H=np.asarray(calibration['matrix']) if calibration else None

    def step(self,image,mask,t):
        gray=cv2.cvtColor(cv2.resize(image,(640,360)),cv2.COLOR_BGR2GRAY)
        fresh=self.prev_t is not None and 0<t-self.prev_t<=.4
        prior=None;motion={'valid':False,'reason':'NOT_TEMPORAL'}
        H=None
        if self.algorithm=='entrance-temporal-v2' and fresh and self.prev is not None:
            H,motion=estimate_motion(self.gray,gray)
            if H is not None:prior=clean_path(transform(self.prev/2,H)*2)
        if self.prev_t is not None and t<=self.prev_t:self.width=None;self.mode=None;self.selected_edges=[]
        self.rejections=[]
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
            add('double',path,.3,ys[-1]/360,dict(extra,width_profile=np.column_stack([ys,r-l]).tolist(), observed_edges=[np.column_stack([l,ys]).tolist(),np.column_stack([r,ys]).tolist()]))
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
                    sections,folds=local_offset(metric,self.calibration['lane_width_cm']/2,side)
                    if not sections:
                        self.rejections.append(dict(mode=side,reason='NO_LOCAL_OFFSET_SECTION'));continue
                    middle=max(sections,key=lambda a:section_score(a,self.H,self.row))
                    path=project(middle,np.linalg.inv(self.H))*2
                    extra={'fold_splits':folds,'retained_sections':len(sections),'metric_support_cm':[float(middle[0,1]),float(middle[-1,1])],
                           'outside_calibration_range':bool(middle[0,1]<50 or middle[-1,1]>120)}
                elif self.width is not None:
                    match=match_edge(xy,self.selected_edges)
                    if match is None or match[0]>60:
                        self.rejections.append(dict(mode=side,reason='BOUNDARY_IDENTITY_CHANGED',residual_px=None if match is None else match[0]));continue
                    use=(tr[:,0]>=self.width[0,0])&(tr[:,0]<=self.width[-1,0]);xy=xy[use]
                    if len(xy)<6:continue
                    path=np.column_stack([xy[:,0]+sign*np.interp(xy[:,1],self.width[:,0],self.width[:,1])/2,xy[:,1]])*2
                    extra={'width_age_s':t-self.width_t,'boundary_residual_px':match[0],'width_source':'last_double_continuously_supported'}
                else:continue
                extra['observed_edges']=[xy.tolist()]
                add(side,path,0.,tr[-1,0]/360,extra)
        # B creates an independent history candidate, anchored by current visible
        # boundary evidence. It can win when geometric singles are unavailable.
        if prior is not None and H is not None and len(prior)>=6 and not any(c['mode']=='double' and c['sample_range'] for c in candidates):
            predicted_edges=[transform(e,H) for e in self.selected_edges]
            for tr in curves:
                for side,col in [('left',2),('right',1)]:
                    if (side=='left' and tr[-1,col]>=319.5) or (side=='right' and tr[-1,col]<=319.5):continue
                    xy=np.column_stack([tr[:,col],tr[:,0]])
                    match=match_edge(xy,predicted_edges)
                    if match is None or match[0]>35:continue
                    residual,delta,lo,hi=match
                    use=prior[(prior[:,1]>=lo*2)&(prior[:,1]<=hi*2)]
                    if len(use)<6:continue
                    corrected=use.copy();corrected[:,0]+=2*delta
                    add('history_'+side,corrected,.4-residual/70,tr[-1,0]/360,
                        dict(observed_edges=[xy.tolist()],history_path=corrected.tolist(),
                             history_residual_px=residual,history_source='aligned_previous_path_current_boundary'))
        # Prefer an actionable candidate; score chooses among supported alternatives.
        selected=max(candidates,key=lambda c:(c['sample_range'] is not None,c['score'])) if candidates else None
        path=selected['path'] if selected else np.empty((0,2))
        history_points=len(selected.get('history_path',[])) if selected else 0
        if selected and selected['mode']=='double':
            self.width=np.asarray(selected['width_profile']);self.width_t=t
        before=self.mode;self.mode=selected['mode'] if selected else None
        # History may propagate only after matching a current observed boundary.
        self.prev=selected['path'].copy() if selected else None;self.prev_t=t;self.gray=gray
        self.selected_edges=[np.asarray(e) for e in selected.get('observed_edges',[])] if selected else []
        diagnostics=[{k:v for k,v in c.items() if k not in ('path','width_profile','observed_edges','history_path')} for c in candidates]
        return dict(path=path.tolist(),valid=len(path)>=6,quality=float(selected['score']) if selected else 0,
            mode=self.mode or 'no_evidence',reason='ENTRANCE_CANDIDATE' if selected else 'NO_ENTRANCE_SUPPORTED_PATH',
            candidates=diagnostics,transition=f'{before} → {self.mode}',history_points=history_points,motion=motion,
            boundaries=[tr.tolist() for tr in curves],dynamic_sampling=True,
            observed_edges=selected.get('observed_edges',[]) if selected else [],
            history_path=selected.get('history_path',[]) if selected else [],
            rejections=self.rejections, path_semantics='estimated_road_center_not_vehicle_trajectory',
            metric_exploration=self.H is not None,hardware_output=False)

