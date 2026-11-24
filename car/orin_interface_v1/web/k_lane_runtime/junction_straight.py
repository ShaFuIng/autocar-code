"""Explicit straight intent, image-space junction hypothesis; offline only.

Candidate scores are heuristics, not calibrated probabilities. Dashed bridge is
inferred road center, not measured free space or a vehicle trajectory.
"""
import cv2
import numpy as np
from road_segments_v31 import RoadPath

def white_runs(mask,y):
    line=mask[max(0,y-1):y+2].mean(0)>.3
    d=np.diff(np.r_[False,line,False].astype('int8'))
    return [(float(a),float(b-1)) for a,b in zip(np.flatnonzero(d==1),np.flatnonzero(d==-1)) if 2<=b-a<=180]

def exit_candidates(mask,entry,width):
    """Pair far-field inner edges into continuous corridor hypotheses."""
    chains=[]
    for y in range(150,239,4):
        runs=white_runs(mask,y);options=[]
        expected=max(12,float(np.polyval(width,y)))
        for i,l in enumerate(runs):
            for r in runs[i+1:]:
                w=r[0]-l[1];x=(l[1]+r[0])/2
                if 8<w<expected*2.2:
                    options.append((x,w))
        used=set()
        for x,w in options:
            matches=[(abs(c[-1][0]-x),i) for i,c in enumerate(chains) if i not in used and 0<y-c[-1][1]<=8 and abs(c[-1][0]-x)<25]
            if matches:
                _,i=min(matches);chains[i].append((x,y,w));used.add(i)
            else:chains.append([(x,y,w)]);used.add(len(chains)-1)
    result=[]
    for chain in chains:
        if len(chain)<3:continue
        a=np.array(chain);head=a[:min(5,len(a))];fit=np.polyfit(head[:,1],head[:,0],1)
        deviation=float(np.median(abs(head[:,0]-np.polyval(entry,head[:,1]))))
        width_error=float(np.median(abs(head[:,2]-np.polyval(width,head[:,1]))/np.maximum(12,np.polyval(width,head[:,1]))))
        score=deviation+30*width_error+40*abs(fit[0]-entry[0])
        result.append(dict(points=a[:,:2].tolist(),score=score,deviation_px=deviation,width_error=width_error,slope=float(fit[0])))
    return sorted(result,key=lambda c:c['score'])[:6]

class JunctionStraight:
    def __init__(self,algorithm,calibration=None,row=.6):
        self.base=RoadPath('road-memory-v3.1',None,row);self.entry=None;self.width=None;self.active=False;self.stable=0
    def step(self,image,mask,t):
        r=self.base.step(image,mask,t)
        m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
        samples=r.get('path_samples',[])
        near=[n for n in samples if n['source']=='double' and 250<=n['y']<=334]
        current_width=self.base.width
        candidates=[];bridge=[];entry_points=[];selected=None;reason='FOLLOW_BASELINE'
        if self.entry is not None:
            entry_points=[[float(np.polyval(self.entry,y)),float(y)] for y in [354,300,250,200,160]]
            # Missing near dual-line support or a widened crossing opens selection.
            widened=False
            if current_width is not None:
                ys=current_width[:,0];valid=(ys>=230)&(ys<=310)
                if np.count_nonzero(valid)>=4:
                    ratio=current_width[valid,1]/np.maximum(12,np.polyval(self.width,ys[valid]))
                    widened=float(np.median(ratio))>1.35
            far_deviation=[abs(n['x']-np.polyval(self.entry,n['y'])) for n in samples if n['y']<230]
            branching=sum(len(white_runs(m,y))>=3 for y in range(170,251,4))>=3
            junction=widened or len(near)<10 or (branching and far_deviation and max(far_deviation)>25)
            self.stable=self.stable+1 if not junction else 0
            if junction:self.active=True
            if self.active and self.stable>=10:self.active=False
            if self.active:
                candidates=exit_candidates(m,self.entry,self.width)
                # Reject a sideways exit rather than claim it is straight.
                selected=0 if candidates and candidates[0]['deviation_px']<100 else None
                if selected is not None:
                    c=candidates[selected];a=np.array(c['points']);far=a[a[:,1]<=min(210,a[-1,1])]
                    target=a[min(2,len(a)-1)]
                    y0=334.;x0=float(np.polyval(self.entry,y0));y1=float(target[1]);x1=float(target[0])
                    for y in np.arange(y1,y0+1,4):
                        f=(y-y1)/max(1,y0-y1);smooth=f*f*(3-2*f)
                        bridge.append([x1+(x0-x1)*smooth,float(y)])
                    reason='STRAIGHT_EXIT_BRIDGE'
                else:
                    # Limited near-field continuation, explicitly inferred.
                    bridge=[[float(np.polyval(self.entry,y)),float(y)] for y in range(250,335,4)]
                    reason='ENTRY_DIRECTION_ONLY_NO_EXIT'
                bridge=[p for p in bridge if 0<=p[0]<=639]
                if len(bridge)>=6:
                    r.update(path=[[x*2,y*2] for x,y in bridge],valid=True,quality=.35,
                        mode='junction_inferred',reason=reason,history_points=0,history_segments=[],history_path=[],
                        road_segments=[dict(source='junction_inferred',points=[[x*2,y*2] for x,y in bridge],confidence=.35,age_s=0)],
                        coverage=dict(visible_rows=len(bridge),double_rows=0,components=1,visible_span_px=(bridge[-1][1]-bridge[0][1])*2,control_span_px=(bridge[-1][1]-bridge[0][1])*2))
        if not self.active and len(near)>=10 and current_width is not None:
            fit=np.polyfit([n['y'] for n in near],[n['x'] for n in near],1)
            self.entry=fit if self.entry is None else .8*self.entry+.2*fit
            fitw=np.polyfit(current_width[:,0],current_width[:,1],1)
            self.width=fitw if self.width is None else .8*self.width+.2*fitw
        r['junction']=dict(intent='straight',active=self.active,reason=reason,entry_direction=entry_points,candidates=candidates,selected=selected,inferred_path=bridge,
            limitation='Image-space hypothesis; no obstacle detector, metric calibration or vehicle trajectory. Entry direction held when current evidence is incomplete.')
        return r
