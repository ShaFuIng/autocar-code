"""Offline candidate: validate distant pairs with current-image tape evidence.

Never alters the network mask, never reuses unaligned history coordinates.
Only observed mask edges and independently supported image edges become paths.
"""
import inspect
import cv2
import numpy as np
from entrance_paths_v2 import traces
from road_segments_v32 import RoadPath as Baseline, resample_memory

# Permit diagnostic far traces to start above 210; the original extractor stays intact.
_source=inspect.getsource(traces).replace('def traces(', 'def far_traces(').replace('y>=210', 'y>=150')
exec(_source)

def extend_tape(image, edge, side):
    """Follow a bright tape's inner edge; do not label geometric extrapolation observed."""
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY).astype(float)
    tail=edge[-min(12,len(edge)):];fit=np.polyfit(tail[:,1],tail[:,0],1)
    residual=float(np.max(np.abs(np.polyval(fit,tail[:,1])-tail[:,0])))
    if residual>6:return edge,[],dict(reason='NONLINEAR_TAIL',residual=residual)
    added=[];sign=-1 if side=='left' else 1
    for y in range(int(edge[-1,1])+4,335,4):
        pred=float(np.polyval(fit,y))
        if not 18<=pred<622:break
        options=[]
        for x in range(max(18,int(pred)-10),min(622,int(pred)+11)):
            stripe=gray[y-1:y+2,x+sign*np.arange(3,12)]
            road=gray[y-1:y+2,x-sign*np.arange(3,12)]
            contrast=float(np.median(stripe)-np.median(road))
            # Require contrast on all three rows, not one specular pixel.
            rowcontrast=np.median(stripe,axis=1)-np.median(road,axis=1)
            if contrast>=25 and min(rowcontrast)>=18:options.append((contrast-.8*abs(x-pred),x))
        if not options:break
        x=max(options)[1];added.append([float(x),float(y)])
    return np.vstack([edge,added]) if added else edge,added,dict(residual=residual)

class RoadPath(Baseline):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs);self.far_previous=None;self.far_votes=0;self.far_time=None
    def step(self,image,mask,t):
        result=super().step(image,mask,t)
        result['far_recovery']={'accepted':False,'reason':'NORMAL_ENTRANCE'}
        # Existing valid double entrance remains byte-for-byte algorithmically unchanged.
        if result['coverage']['double_rows']>=6 and any(len(e)>5 and e[-1][1]>=270 for e in result['observed_edges']):
            self.far_votes=0;self.far_previous=None;return result
        curves=far_traces(mask);options=[];checks=[]
        for i,l in enumerate(curves):
            for j,r in enumerate(curves):
                if i==j:continue
                ys=np.intersect1d(l[:,0],r[:,0])
                if len(ys)<8 or np.ptp(ys)<28:continue
                lx=np.interp(ys,l[:,0],l[:,2]);rx=np.interp(ys,r[:,0],r[:,1])
                if np.any(rx-lx<16) or not lx[-1]<319.5<rx[-1] or ys[-1]>=270:continue
                left=np.column_stack([lx,ys]);right=np.column_stack([rx,ys])
                le,la,ld=extend_tape(image,left,'left');re,ra,rd=extend_tape(image,right,'right')
                # Both sides must independently show actual current-image support.
                if len(la)<4 or len(ra)<4 or min(le[-1,1],re[-1,1])<self.row*719/2+20:continue
                if self.width is None:continue
                wc=np.polyfit(self.width[:,0],self.width[:,1],1);expected=np.polyval(wc,ys)
                width_error=float(np.median(np.abs((rx-lx)-expected)/np.maximum(expected,16)))
                if width_error>.25:continue
                prior=resample_memory(self.memory,t) if result.get('memory_state',{}).get('aligned') else {}
                deltas=[abs((np.interp(y,ys,lx)+np.interp(y,ys,rx))/2-h['x']) for y,h in prior.items() if ys[0]<=y<=ys[-1] and h['confidence']>.15]
                if deltas and np.median(deltas)>40:continue
                center=np.polyfit(ys,(lx+rx)/2,1);signature=np.r_[np.polyval(center,[190,230]),np.polyval(np.polyfit(ys,rx-lx,1),230)]
                options.append((len(ys),le,re,la,ra,signature,width_error))
        if len(options)!=1:
            self.far_votes=0;self.far_previous=None;result['far_recovery']={'accepted':False,'reason':'NO_UNIQUE_PIXEL_SUPPORTED_PAIR','candidates':len(options)};return result
        _,left,right,la,ra,sig,we=options[0]
        consistent=self.far_previous is not None and self.far_time is not None and 0<t-self.far_time<.4 and np.max(np.abs(sig-self.far_previous))<24
        self.far_votes=self.far_votes+1 if consistent else 1;self.far_previous=sig;self.far_time=t
        detail=dict(accepted=False,reason='CONFIRMING_CURRENT_PAIR',votes=self.far_votes,width_relative_error=we,pixel_edges=[la,ra])
        result['far_recovery']=detail
        if self.far_votes<3:return result
        # Use only the jointly observed continuous range; no extrapolation across gaps.
        lo=max(left[0,1],right[0,1]);hi=min(left[-1,1],right[-1,1])
        ys=np.arange(lo,hi+1,4);xs=(np.interp(ys,left[:,1],left[:,0])+np.interp(ys,right[:,1],right[:,0]))/2
        path=np.column_stack([xs*2,ys*2]).tolist()
        if len(path)<6 or hi*2<self.row*719+2:return result
        detail.update(accepted=True,reason='CURRENT_PIXEL_PAIR_CONFIRMED')
        result.update(path=path,valid=True,quality=.45,observed_edges=[left.tolist(),right.tolist()],
            road_segments=[dict(source='current_pixel_pair',points=path,confidence=.45,age_s=0)],
            history_segments=[],path_samples=[],reason='FAR_PAIR_CURRENT_IMAGE_VERIFIED')
        result['coverage']=dict(visible_rows=len(path),double_rows=len(path),components=1,control_span_px=(hi-lo)*2,visible_span_px=(hi-lo)*2)
        # Do not write inferred or pixel-assisted edges into baseline mask history/width.
        return result
