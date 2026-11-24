"""J: segmented boundary geometry, with frame-bound reference compensation. Offline only."""
import cv2
import numpy as np
from junction_left import JunctionLeft, PARAMS as H_PARAMS
from temporal_lane import estimate_motion, transform

PARAMS = dict(H_PARAMS, segment_min_dy=18, segment_far_y=195,
              geometry_weight=.65, geometry_step_limit_px=6, reference_hold_s=.6, motion_endpoint_limit_px=18)
YS = np.array([180., 230., 280., 334.])

def line_points(fit):
    return [[float(np.polyval(fit, y)), float(y)] for y in range(180, 335, 2)]

def geometry(mask, width, expected_center=None):
    m = cv2.resize(mask.astype('uint8') * 255, (640, 360), interpolation=cv2.INTER_NEAREST)
    edges = cv2.Canny(m, 30, 80); edges[:160] = 0; edges[300:] = 0
    lines = cv2.HoughLinesP(edges, 1, np.pi/360, 12, minLineLength=22, maxLineGap=5)
    candidates = []
    for x, y, xx, yy in ([] if lines is None else np.asarray(lines).reshape(-1, 4)):
        if abs(yy-y) < PARAMS['segment_min_dy']: continue
        fit = np.array([(xx-x)/(yy-y), x-(xx-x)/(yy-y)*y])
        far = float(np.polyval(fit, 180)); side = 'left' if fit[0] < 0 else 'right'
        eligible = bool(min(y, yy) <= PARAMS['segment_far_y'] and .25 < abs(fit[0]) < 3
                        and ((120 < far < 330) if side == 'left' else (310 < far < 500)))
        candidates.append(dict(side=side, fit=fit.tolist(), points=[[int(x),int(y)],[int(xx),int(yy)]],
                               eligible=eligible, span=int(abs(yy-y))))
    selected = []; center = None; source = 'NO_GEOMETRY'
    left = [c for c in candidates if c['eligible'] and c['side']=='left']
    right = [c for c in candidates if c['eligible'] and c['side']=='right']
    pairs=[]
    if width is not None:
        expected=np.polyval(width, YS)
        for l in left:
            for r in right:
                w=np.polyval(np.array(r['fit'])-l['fit'], YS)
                error=float(np.mean(abs(w-expected)/np.maximum(expected,20)))
                if np.all(w>20) and error<.3:
                    pairs.append((error-.001*(l['span']+r['span']),l,r))
    if pairs:
        _,l,r=min(pairs,key=lambda x:x[0]);selected=[l,r]
        center=(np.array(l['fit'])+r['fit'])/2;source='PAIRED_BOUNDARIES'
    elif width is not None:
        # Near branch segments are not allowed to anchor a full extrapolation.
        for side, pool in [('left',left),('right',right)]:
            if not pool: continue
            longest=max(c['span'] for c in pool)
            pool=[c for c in pool if c['span']>=max(18,longest*.6)]
            # Inner paint edge, among far straight segments with adequate support.
            def rank(c):
                f=np.array(c['fit'])+np.array(width)*(.5 if side=='left' else -.5)
                if expected_center is not None:
                    return -float(np.mean(abs(np.polyval(f,YS)-np.polyval(expected_center,YS))))+.05*c['span']
                return np.polyval(c['fit'],210)*(1 if side=='left' else -1)
            c=max(pool,key=rank)
            candidate=np.array(c['fit'])+np.array(width)*(.5 if side=='left' else -.5)
            if np.all((np.polyval(candidate,YS)>180)&(np.polyval(candidate,YS)<460)):
                selected=[c];center=candidate;source=side.upper()+'_HISTORICAL_WIDTH';break
    return center, dict(source=source, segments=candidates, selected=selected,
                        extensions=[line_points(c['fit']) for c in selected],
                        center=line_points(center) if center is not None else [],
                        evidence='current mask edges; extension and center are inferred')

class JunctionGeometry(JunctionLeft):
    def __init__(self):
        super().__init__(); self.main=None; self.anchor_gray=None; self.anchor_t=None

    def step(self,image,mask,t,baseline):
        if self.last_t is not None and (t<=self.last_t or t-self.last_t>1):
            self.main=None;self.anchor_gray=None;self.anchor_t=None
        r=super().step(image,mask,t,baseline);j=r['junction']
        H,motion=estimate_motion(self.anchor_gray,self.gray)
        age=t-self.anchor_t if self.anchor_t is not None else None
        prediction=None; applied=False
        if self.main is not None and H is not None:
            p=transform(np.column_stack([np.polyval(self.main,YS),YS]),H)
            prediction=np.polyfit(p[:,1],p[:,0],1)
            delta=np.polyval(prediction,YS)-np.polyval(self.main,YS)
            if np.max(abs(delta))>PARAMS['motion_endpoint_limit_px']:
                prediction=None;motion=dict(motion,valid=False,reason='REFERENCE_DISPLACEMENT_REJECTED')
            else: applied=True
        # This retained line remains tied to anchor_gray; failed transforms never advance it.
        if prediction is None and self.main is not None and age<=PARAMS['reference_hold_s']:
            prediction=self.main.copy()
        observed,g=geometry(mask,self.lane_width,prediction if prediction is not None else self.main)
        fit=None;source='UNAVAILABLE';advance=False
        if not j['active'] and self.entry is not None:
            fit=self.entry.copy();source='NORMAL_ROAD_SEED';advance=True
        elif observed is not None:
            if prediction is not None:
                # Bound both endpoints, so refitting cannot exceed the endpoint correction cap.
                yy=np.array([180.,334.]);xx=np.polyval(prediction,yy)
                correction=np.clip(PARAMS['geometry_weight']*(np.polyval(observed,yy)-xx),
                                   -PARAMS['geometry_step_limit_px'],PARAMS['geometry_step_limit_px'])
                fit=np.polyfit(yy,xx+correction,1)
                source='GEOMETRY_FUSION'
            else:fit=observed;source='GEOMETRY_REACQUIRED'
            advance=True
        elif prediction is not None:
            fit=prediction;source='ALIGNED_REFERENCE' if applied else 'UNALIGNED_HOLD';advance=applied
        g.update(reference_source=source,motion=motion,anchor_age_s=age,anchor_advanced=advance,
                 motion_applied=applied,anchor_capture_s=self.anchor_t)
        if fit is not None:
            if advance:self.main=fit.copy();self.anchor_gray=self.gray.copy();self.anchor_t=t
            if j['active']:
                points=line_points(fit);j['entry_direction']=points;self.entry=fit.copy()
                pts=[[x*2,y*2] for x,y in points]
                r.update(path=pts,valid=True,quality=.4,mode='junction_geometry',reason='SEGMENT_GEOMETRY_REFERENCE',
                         road_segments=[dict(source='junction_inferred',points=pts,confidence=.4,age_s=0)])
                j.update(reason='SEGMENT_GEOMETRY_REFERENCE',inferred_path=points);self.fused=points
        elif j['active']:
            j['entry_direction']=[];j['inferred_path']=[];j['reason']='GEOMETRY_REFERENCE_UNAVAILABLE'
            r.update(path=[],road_segments=[],valid=False,quality=0,reason=j['reason'])
        j['geometry']=g;j['parameters']=PARAMS
        return r
