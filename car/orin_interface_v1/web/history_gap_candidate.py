"""Offline only: anchored, age-limited path repair. Never refresh from repaired output."""
import copy
import cv2
import numpy as np
from temporal_lane import estimate_motion, transform

class HistoryGap:
    def __init__(self, max_age=.3):
        self.anchor=None
        self.max_age=max_age  # Experimental bound on evidence, not PID freshness.
        self.last_t=None

    def step(self,image,mask,t,result,motion_provider=None,repair_enabled=True):
        out=copy.deepcopy(result);path=np.asarray(result['path'],float).reshape(-1,2)/2
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        diag=dict(used=False,reason='CURRENT_ONLY',observation_age_s=None,points=0)
        history=[]
        if self.last_t is not None and t<=self.last_t:self.anchor=None
        self.last_t=t
        anchor=self.anchor
        if anchor is not None:
            age=t-anchor['t'];diag['observation_age_s']=age
            if age>self.max_age:diag['reason']='HISTORY_EXPIRED'
            elif result['mode']!=anchor['mode']:diag['reason']='MODE_CHANGED'
            elif result.get('geometry_guard'):diag['reason']='GEOMETRY_CUT_NOT_RESTORED'
            elif len(path)>=2 and not (path[:,1].min()-anchor['path'][:,1].min()>=8 or anchor['path'][:,1].max()-path[:,1].max()>=8):diag['reason']='NO_RANGE_LOSS'
            elif not repair_enabled:diag['reason']='CURRENT_CONTROL_PATH_SUFFICIENT'
            else:
                H,md=(motion_provider(anchor,gray,t) if motion_provider is not None else estimate_motion(anchor['gray'],gray));diag['motion']=md
                if H is None:diag['reason']='ALIGNMENT_FAILED'
                else:
                    prior=transform(anchor['path'],H)
                    prior=prior[np.isfinite(prior).all(axis=1)]
                    prior=prior[(prior[:,0]>=0)&(prior[:,0]<640)&(prior[:,1]>=180)&(prior[:,1]<350)]
                    prior=prior[prior[:,1].argsort()]
                    # Warp actual boundary anchors, not an assumed constant lane width.
                    edges=transform(anchor['edges'],H)
                    support=[]
                    for x,y in edges:
                        ix,iy=int(round(x)),int(round(y))
                        support.append(0<=ix<640 and 0<=iy<360 and bool(mask[max(0,iy-3):min(360,iy+4),max(0,ix-5):min(640,ix+6)].any()))
                    diag['edge_support']=sum(support);diag['edge_count']=len(support)
                    if len(prior)<4 or len(edges)<8 or sum(support)<max(6,len(edges)*.65):diag['reason']='CURRENT_BOUNDARIES_NOT_CONFIRMED'
                    elif len(path)>=2:
                        current=path[path[:,1].argsort()];lo=max(current[0,1],prior[0,1]);hi=min(current[-1,1],prior[-1,1])
                        ys=np.arange(lo,hi,4)
                        error=np.max(abs(np.interp(ys,current[:,1],current[:,0])-np.interp(ys,prior[:,1],prior[:,0]))) if len(ys)>=4 else float('inf')
                        diag['overlap_error_px']=float(error) if np.isfinite(error) else None
                        if error>8:diag['reason']='CURRENT_HISTORY_CONFLICT'
                        else:
                            # Only extend up to 24px from an agreed endpoint; never bridge internal gaps.
                            history=prior[((prior[:,1]<current[0,1])&(prior[:,1]>=current[0,1]-24))|((prior[:,1]>current[-1,1])&(prior[:,1]<=current[-1,1]+24))].tolist()
                            diag['reason']='PARTIAL_EXTENSION' if history else 'NO_MISSING_ENDPOINT'
                    else:
                        # Require both sides of the historical corridor in this image.
                        if sum(support[::2])>=3 and sum(support[1::2])>=3:
                            history=prior.tolist();diag['reason']='ALIGNED_HISTORY'
                        else:diag['reason']='BOTH_BOUNDARIES_REQUIRED'
        if history:
            merged=np.vstack([path,np.asarray(history)]);merged=merged[merged[:,1].argsort()]
            # Reject sharp seam rather than forcing a connection.
            delta=np.diff(merged,axis=0);slopes=delta[:,0]/np.maximum(delta[:,1],.01)
            if np.any(delta[:,1]>12) or (len(slopes)>1 and np.max(abs(np.diff(slopes)))>1):
                history=[];diag['reason']='SEAM_REJECTED'
            else:
                out['path']=(merged*2).tolist();out['valid']=True;diag.update(used=True,points=len(history))
        out['gap_history_path']=(np.asarray(history)*2).tolist() if history else []
        out['gap_repair']=diag
        # Only store current, mask-supported geometry, never repaired path or empty output.
        # Per-row flanking boundary measurements reject broad/ambiguous corridors.
        if len(path)>=4 and not result.get('geometry_guard'):
            ordered=path[path[:,1].argsort()];edges=[]
            for y in range(200,331,10):
                if not ordered[0,1]<=y<=ordered[-1,1]:continue
                x=float(np.interp(y,ordered[:,1],ordered[:,0]));xs=np.flatnonzero(mask[y]);left=xs[xs<x-8];right=xs[xs>x+8]
                if len(left) and len(right):
                    l,r=float(left[-1]),float(right[0])
                    if 30<r-l<500 and abs((l+r)/2-x)<12:edges.extend([[l,y],[r,y]])
            if len(edges)>=8:
                self.anchor=dict(t=t,gray=gray.copy(),path=path.copy(),edges=edges,mode=result['mode'])
        return out
