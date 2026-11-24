"""I offline: independent full-span straight reference, local observations correct it."""
import cv2
import numpy as np
from junction_left import JunctionLeft,PARAMS as H_PARAMS
from temporal_lane import estimate_motion,transform
PARAMS=dict(H_PARAMS,reference_y_range=[180,334],reference_observation_weight=.25,opening_weight=.02,opening_residual_px=12,reference_hold_s=.45)

def observation_fit(points):
    a=np.array(points,dtype=float)
    if len(a)<8:return None,[],[]
    far=a[a[:,1]<=230]
    if len(far)<8 or np.ptp(far[:,1])<28:return None,[],[]
    fit=np.polyfit(far[:,1],far[:,0],1)
    for _ in range(2):
        residual=abs(far[:,0]-np.polyval(fit,far[:,1]));good=residual<=max(3,float(np.median(residual))*2.5)
        if sum(good)>=6:fit=np.polyfit(far[good,1],far[good,0],1)
    err=abs(a[:,0]-np.polyval(fit,a[:,1]));weights=np.where(err<=PARAMS['opening_residual_px'],1.,PARAMS['opening_weight'])
    fit=np.polyfit(a[:,1],a[:,0],1,w=weights)
    return fit,a[weights==1].tolist(),a[weights<1].tolist()

class JunctionReference(JunctionLeft):
    def __init__(self):
        super().__init__();self.main_reference=None;self.main_seen=None
    def step(self,image,mask,t,baseline):
        previous=self.main_reference;seen=self.main_seen;oldgray=self.gray
        if self.last_t is not None and (t<=self.last_t or t-self.last_t>1):previous=None;seen=None
        r=super().step(image,mask,t,baseline);j=r['junction'];H,flow=estimate_motion(oldgray,self.gray)
        ys=np.array([180.,230.,280.,334.]);prediction=None
        if previous is not None:
            if H is not None:
                p=transform(np.column_stack([np.polyval(previous,ys),ys]),H);prediction=np.polyfit(p[:,1],p[:,0],1)
            elif seen is not None and t-seen<=PARAMS['reference_hold_s']:prediction=previous.copy()
        observed,accepted,rejected=observation_fit(j['left_center'])
        source='UNAVAILABLE';fit=None
        if not j['active']:
            if self.entry is not None:fit=self.entry.copy();seen=t;source='NORMAL_ROAD_SEED'
        elif prediction is not None:
            fit=prediction.copy();source='MOTION_PREDICTION' if H is not None else 'SHORT_HOLD'
            if observed is not None:
                # Observation modifies a persistent line; never replaces it with a short polyline.
                delta=np.clip((np.polyval(observed,ys)-np.polyval(prediction,ys))*.25,-6,6)
                fit=np.polyfit(ys,np.polyval(prediction,ys)+delta,1);seen=t;source='HISTORY_OBSERVATION_FUSION'
        elif observed is not None:
            fit=observed.copy();seen=t;source='OBSERVATION_REINITIALIZED'
        j['reference_diagnostics']=dict(source=source,motion=flow,accepted_points=accepted,downweighted_points=rejected,observation_age_s=t-seen if seen is not None else None)
        if fit is not None:
            self.main_reference=fit;self.main_seen=seen
            if j['active']:
                points=[[float(np.polyval(fit,y)),float(y)] for y in range(180,335,2)]
                j['local_entry_before_reference']=j['entry_direction'];j['entry_direction']=points;self.entry=fit.copy()
                # Full reference is an inferred path, never current boundary observation.
                if all(0<=x<640 for x,y in points):
                    pts=[[x*2,y*2] for x,y in points];r.update(path=pts,valid=True,quality=.35,mode='junction_straight_reference',reason='HISTORICAL_STRAIGHT_REFERENCE',road_segments=[dict(source='junction_inferred',points=pts,confidence=.35,age_s=t-seen if seen is not None else 0)],coverage=dict(visible_rows=len(accepted),double_rows=0,components=1,visible_span_px=0,control_span_px=308))
                    j.update(reason='HISTORICAL_STRAIGHT_REFERENCE',inferred_path=points);self.fused=points
        j['parameters']=PARAMS
        return r
