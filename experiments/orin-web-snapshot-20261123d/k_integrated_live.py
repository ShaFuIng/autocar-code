"""Complete K r2 image/path adapter; never opens hardware or starts PID."""
import sys,time,copy
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent/'k_lane_runtime'))
from warm_color_candidate import WarmColor
from history_gap_candidate import HistoryGap
from k_integrated_parallel import IntegratedParallelKLane

class IntegratedK:
    algorithm='junction-k-integrated-r2'
    revision='k-integrated-r2'
    def __init__(self,row=.6):
        self.row=row;self.tracker=IntegratedParallelKLane(row)
        self.color=WarmColor();self.history=HistoryGap();self.t=None;self.prepared=None
    def set_row(self,row):
        self.row=row;self.tracker.set_row(row);self.color=WarmColor();self.history=HistoryGap();self.t=None
    def preprocess(self,image,captured):
        start=time.perf_counter()
        if self.t is not None and (captured<=self.t or captured-self.t>1):
            self.color=WarmColor();self.history=HistoryGap()
        self.t=captured
        raw=cv2.resize(image,(640,360));corrected,d=self.color.step(raw,captured)
        d['elapsed_ms']=(time.perf_counter()-start)*1000
        self.prepared=(captured,raw,corrected,d)
        return corrected
    def step(self,image,mask,captured,on_a=None):
        if self.prepared is None or self.prepared[0]!=captured:raise ValueError('Same-frame preprocessing required')
        _,raw,corrected,color=self.prepared
        if image is not corrected:raise ValueError('A and K must receive the same corrected image')
        mask=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
        emitted=[]
        def finish(r):
            start=time.perf_counter();r=self.history.step(raw,mask,captured,r)
            r['gap_repair']['elapsed_ms']=(time.perf_counter()-start)*1000
            # Keep sampled/drawn evidence consistent with the control path.
            pts=r.get('gap_history_path',[])
            if pts:
                age=r['gap_repair']['observation_age_s']
                r.setdefault('history_segments',[]).append(dict(source='aligned_gap_history',points=pts,confidence=.25,age_s=age))
                r.setdefault('path_samples',[]).extend(dict(x=x/2,y=y/2,source='aligned_gap_history',confidence=.25,age_s=age,observed_at=captured-age) for x,y in pts)
                r['path_samples'].sort(key=lambda p:p['y'])
            return r
        def early(a):
            a=finish(a);emitted.append(a)
            if on_a:on_a(self.format(a,captured,color))
        r=self.tracker.step(corrected,mask,captured,on_a=early)
        if emitted:
            a=emitted[0]
            for key in ['path','valid','gap_history_path','gap_repair','history_segments','path_samples']:
                if key in a:r[key]=a[key]
        else:r=finish(r)
        self.prepared=None
        return self.format(r,captured,color)
    def format(self,r,captured,color):
        fields=('mode','reason','k','road_segments','history_segments','path_samples','path_semantics','observed_edges','coverage','motion','rejections','parallel','geometry_guard','gap_history_path','gap_repair')
        d={k:r[k] for k in fields if k in r}
        d.update(path_revision=self.revision,integrated_k_version=2,color_correction=copy.deepcopy(color),capture_s=captured,opencv=cv2.__version__,control_connected=False)
        return dict(path=r['path'],valid=bool(r['valid']),quality=float(r.get('quality',.25)),path_diagnostic=d)
    def close(self):self.tracker.close()
