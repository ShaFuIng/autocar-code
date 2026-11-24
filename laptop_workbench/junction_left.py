"""H offline: preserve left boundary identity and pre-junction image width."""
import cv2
import numpy as np
from junction_fused import JunctionFused,PARAMS as G_PARAMS
from temporal_lane import estimate_motion,transform
from junction_straight import white_runs
PARAMS=dict(G_PARAMS,left_gate_px=45,left_min_rows=12,left_min_span_px=44,width_history_frames=12)
class JunctionLeft(JunctionFused):
    def __init__(self):
        super().__init__();self.left_track=None;self.left_seen=None;self.width_history=[];self.lane_width=None
    def step(self,image,mask,t,baseline):
        oldgray=self.gray;oldleft=self.left_track;oldseen=self.left_seen;history=self.width_history;lane_width=self.lane_width
        reset=self.last_t is not None and (t<=self.last_t or t-self.last_t>1)
        if reset:oldleft=None;oldseen=None;history=[];lane_width=None
        r=super().step(image,mask,t,baseline);j=r['junction'];self.width_history=history;self.lane_width=lane_width
        m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0;H,flow=estimate_motion(oldgray,self.gray)
        prediction=transform(oldleft,H) if oldleft is not None and H is not None else np.array(oldleft) if oldleft is not None and oldseen is not None and t-oldseen<=.45 else None
        if prediction is not None:prediction=prediction[np.argsort(prediction[:,1])]
        doubles=sorted([s for s in baseline.get('path_samples',[]) if s['source']=='double'],key=lambda n:n['y'])
        seed=j['state']=='FOLLOW' and j['evidence'].get('near_supported',False) and not j['evidence'].get('widened',False)
        found=[];widths=[]
        for y in range(170,335,4):
            runs=white_runs(m,y)
            if seed and len(doubles)>=12 and doubles[0]['y']<=y<=doubles[-1]['y']:
                center=float(np.interp(y,[n['y'] for n in doubles],[n['x'] for n in doubles]));pairs=[(a[1],b[0]) for a,b in zip(runs,runs[1:]) if a[1]<center<b[0]]
                if not pairs:continue
                l,rr=min(pairs,key=lambda p:abs((p[0]+p[1])/2-center));found.append([float(l),float(y)]);widths.append([y,rr-l])
            elif prediction is not None:
                expected=float(np.interp(y,prediction[:,1],prediction[:,0]))
                if y<prediction[0,1]:expected=float(np.polyval(np.polyfit(prediction[:min(8,len(prediction)),1],prediction[:min(8,len(prediction)),0],1),y))
                if y>prediction[-1,1]:expected=float(np.polyval(np.polyfit(prediction[-min(8,len(prediction)):,1],prediction[-min(8,len(prediction)):,0],1),y))
                options=[b for a,b in runs if abs(b-expected)<=PARAMS['left_gate_px']]
                if options:found.append([float(min(options,key=lambda x:abs(x-expected))),float(y)])
        chains=[];chain=[]
        for pt in found:
            if chain and (pt[1]-chain[-1][1]>12 or abs(pt[0]-chain[-1][0])/(pt[1]-chain[-1][1])>=5):chains.append(chain);chain=[]
            chain.append(pt)
        if chain:chains.append(chain)
        found=max(chains,key=len,default=[])
        supported=len(found)>=PARAMS['left_min_rows'] and found[-1][1]-found[0][1]>=PARAMS['left_min_span_px']
        if supported:
            a=np.array(found);slope=np.diff(a[:,0])/np.diff(a[:,1]);supported=bool(np.max(abs(slope))<5)
        if seed and supported and len(widths)>=12:
            v=np.array(widths);fit=np.polyfit(v[:,0],v[:,1],1)
            if fit[0]>.15 and np.max(abs(v[:,1]-np.polyval(fit,v[:,0])))<20:
                self.width_history.append(fit.tolist());self.width_history=self.width_history[-PARAMS['width_history_frames']:];self.lane_width=np.median(self.width_history,axis=0)
        if supported:self.left_track=found;self.left_seen=t
        else:self.left_track=prediction.tolist() if prediction is not None else None;self.left_seen=oldseen
        inferred=[]
        if supported and self.lane_width is not None:
            inferred=[[x+float(np.polyval(self.lane_width,y))/2,y] for x,y in found if np.polyval(self.lane_width,y)>12]
        j['left_boundary']=found if supported else [];j['left_center']=inferred
        j['left_diagnostics']=dict(supported=bool(supported),motion=flow,width_seed_frames=len(self.width_history),historical_width=self.lane_width.tolist() if self.lane_width is not None else None,width_frozen=not seed,reference_only=not j['active'])
        if j['active'] and len(inferred)>=12:
            a=np.array(inferred);self.entry=np.polyfit(a[:,1],a[:,0],1);self.width=self.lane_width.copy()
            j['uncorrected_entry_direction']=j['entry_direction'];j['entry_direction']=inferred;j['width_model']=self.lane_width.tolist()
            # Piecewise observed left boundary; no hard straight-line fit to output.
            clear=[p for p in inferred if 18<=p[0]<622 and not m[int(p[1]),int(p[0])-10:int(p[0])+11].any()]
            chains=[];chain=[]
            for p in clear:
                if chain and p[1]-chain[-1][1]>4:chains.append(chain);chain=[]
                chain.append(p)
            if chain:chains.append(chain)
            kept=max(chains,key=len,default=[])
            if len(kept)>=12:
                pts=[[x*2,y*2] for x,y in kept];r.update(path=pts,valid=True,quality=.4,mode='junction_left_width',reason='LEFT_BOUNDARY_WIDTH_CENTER',road_segments=[dict(source='junction_inferred',points=pts,confidence=.4,age_s=0)],coverage=dict(visible_rows=len(found),double_rows=0,components=1,visible_span_px=(found[-1][1]-found[0][1])*2,control_span_px=(kept[-1][1]-kept[0][1])*2))
                j.update(reason='LEFT_BOUNDARY_WIDTH_CENTER',inferred_path=kept);self.fused=kept
        j['parameters']=PARAMS
        return r
