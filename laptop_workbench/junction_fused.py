"""Offline G: bounded temporal fusion and current-mask-supported segments."""
import copy
import cv2
import numpy as np
from junction_corridor import JunctionCorridor, PARAMS as F_PARAMS
from temporal_lane import estimate_motion,transform
from junction_straight import white_runs
PARAMS=dict(F_PARAMS,fusion_weight=.3,correction_limit_px=8,unaligned_hold_s=.45,min_retained_span_px=40)

def retain_segment(mask,points):
    chains=[];chain=[]
    for x,y in points:
        iy=int(round(y));ix=int(round(x))
        ok=0<=iy<360 and 18<=ix<622
        if ok:
            runs=white_runs(mask,iy);left=[b for a,b in runs if b<x];right=[a for a,b in runs if a>x]
            ok=bool(left and right and x-max(left)>18 and min(right)-x>18 and not mask[iy,ix-18:ix+19].any())
        if ok:chain.append([float(x),float(y)])
        elif chain:chains.append(chain);chain=[]
    if chain:chains.append(chain)
    best=max(chains,key=lambda a:a[-1][1]-a[0][1],default=[])
    return best if best and best[-1][1]-best[0][1]>=PARAMS['min_retained_span_px'] else []

class JunctionFused(JunctionCorridor):
    def __init__(self):
        super().__init__();self.fused=None;self.fused_at=None;self.observed_at=None
    def step(self,image,mask,t,baseline):
        oldgray=self.gray;oldpath=self.fused;oldt=self.fused_at;observed=self.observed_at
        r=super().step(image,mask,t,baseline);j=r['junction'];original=r['valid'];base_reason=j['reason'];input_result=copy.deepcopy(r)
        if oldt is not None and (t<=oldt or t-oldt>1):oldpath=None;observed=None
        H,flow=estimate_motion(oldgray,self.gray)
        previous=transform(oldpath,H) if H is not None and oldpath is not None else np.array(oldpath) if oldpath is not None and observed is not None and t-observed<=PARAMS['unaligned_hold_s'] else None
        if original:observed=t
        if j['active']:
            p=np.array(r['path'],dtype=float)/2 if original else np.array([[float(np.polyval(self.entry,y)),y] for y in range(190,335,4)]) if self.entry is not None and (H is not None or previous is not None) else np.empty((0,2))
            if len(p):
                p=p[np.argsort(p[:,1])];ys=np.arange(max(190,p[0,1]),min(334,p[-1,1])+1,4);xs=np.interp(ys,p[:,1],p[:,0]);candidate=np.column_stack([xs,ys])
                if previous is not None and len(previous)>1:
                    previous=previous[np.argsort(previous[:,1])];inside=(ys>=previous[0,1])&(ys<=previous[-1,1]);px=np.interp(ys,previous[:,1],previous[:,0]);candidate[inside,0]=px[inside]+np.clip((xs[inside]-px[inside])*PARAMS['fusion_weight'],-PARAMS['correction_limit_px'],PARAMS['correction_limit_px'])
                m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
                kept=retain_segment(m,candidate);partial=len(kept)<len(candidate)
                reason='FUSED_PARTIAL_PATH' if partial else 'FUSED_CONTINUOUS_PATH'
                if not kept:reason='FUSION_NO_SUPPORTED_SEGMENT'
                r.update(path=[[x*2,y*2] for x,y in kept],valid=bool(kept),quality=.35 if kept else 0,mode='junction_fused',reason=reason,road_segments=[dict(source='junction_inferred',points=[[x*2,y*2] for x,y in kept],confidence=.35,age_s=0)] if kept else [],coverage=dict(visible_rows=0,double_rows=0,components=int(bool(kept)),visible_span_px=0,control_span_px=(kept[-1][1]-kept[0][1])*2 if kept else 0))
                j.update(reason=reason,inferred_path=kept)
                if not kept and original:
                    r=input_result;j=r['junction'];j['fusion_fallback']='F_PATH_RETAINED_NO_SUPPORTED_FUSION'
            j['fusion']=dict(input_reason=base_reason,input_valid=original,aligned=H is not None,observation_age_s=t-observed if observed is not None else None,retained_span_px=(r['path'][-1][1]-r['path'][0][1])/2 if r['path'] else 0)
        j['parameters']=PARAMS
        self.fused=(np.array(r['path'])/2).tolist() if r['valid'] else None;self.fused_at=t;self.observed_at=observed
        return r
