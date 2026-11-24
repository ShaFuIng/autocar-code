"""Offline mask tracking experiment. No transport, serial, or actuator imports."""
import math
import numpy as np

GEOMETRY = dict(height_m=.1475, pitch_deg=4.746748, hfov_deg=66., vfov_deg=40.,
                camera_ahead_rear_m=.33, line_center_width_m=.505,
                status="ESTIMATED_NOT_CALIBRATED")

def ground(u, v, w, h, g=GEOMETRY):
    a=math.radians(g['pitch_deg'])
    rx=(u-(w-1)/2)/(w/2/math.tan(math.radians(g['hfov_deg']/2)))
    ry=(v-(h-1)/2)/(h/2/math.tan(math.radians(g['vfov_deg']/2)))
    down=math.sin(a)+ry*math.cos(a)
    if down<=.015:return None
    scale=g['height_m']/down
    z=scale*(math.cos(a)-ry*math.sin(a))
    if not .05<z<3:return None
    return [float(rx*scale),float(z+g['camera_ahead_rear_m'])]

def pixel(x,z,w,h,g=GEOMETRY):
    z-=g['camera_ahead_rear_m'];a=math.radians(g['pitch_deg']);height=g['height_m']
    depth=z*math.cos(a)+height*math.sin(a)
    if depth<=0:return None
    return [float((w-1)/2+w/2/math.tan(math.radians(g['hfov_deg']/2))*x/depth),
            float((h-1)/2+h/2/math.tan(math.radians(g['vfov_deg']/2))*(height*math.cos(a)-z*math.sin(a))/depth)]

class BoundaryTracker:
    """Track row identities, with bounded single-side inference and no blind hold."""
    def __init__(self):
        self.previous={};self.last_t=None;self.last_pair_t=None

    def step(self,mask,t):
        if self.last_t is not None and (t<=self.last_t or t-self.last_t>.5):
            self.previous={};self.last_pair_t=None
        self.last_t=t
        h,w=mask.shape;observed=[];new={};lower=None
        for index,f in enumerate(np.linspace(.94,.40,64)):
            y=int((h-1)*f)
            line=mask[max(0,y-2):y+3].mean(0)>.3
            changes=np.diff(np.r_[False,line,False].astype(np.int8))
            spans=[(int(a),int(b)) for a,b in zip(np.flatnonzero(changes==1),np.flatnonzero(changes==-1)) if 2<=b-a<w*.45]
            xs=[(a+b-1)/2 for a,b in spans]
            prior=self.previous.get(index)
            guess=prior or lower
            candidates=[]
            for a,b in zip(xs,xs[1:]):
                ga,gb=ground(a,y,w,h),ground(b,y,w,h)
                if ga is None or gb is None:continue
                width=gb[0]-ga[0]
                if not .28<width<.85:continue
                # Initialization is conservative. Once acquired, image center is not a gate.
                if guess is None and not a<w/2<b:continue
                center=(a+b)/2
                reference=sum(guess)/2 if guess else w/2
                if abs(center-reference)>w*.22:continue
                score=abs(center-reference)/w+abs(width-.505)*.35
                candidates.append((score,a,b))
            candidates.sort()
            mode=None;pair=None
            if candidates and (len(candidates)==1 or candidates[1][0]-candidates[0][0]>.025):
                _,a,b=candidates[0];pair=(a,b);mode='double'
            elif guess and xs and (sum(p['mode']=='double' for p in observed)>=5 or
                    (self.last_pair_t is not None and t-self.last_pair_t<=.5)):
                matches=sorted((abs(x-guess[side]),side,x) for x in xs for side in (0,1))
                distance,side,x=matches[0]
                if distance<w*.09 and (len(matches)==1 or matches[1][0]-distance>w*.025):
                    width=guess[1]-guess[0]
                    pair=(x,x+width) if side==0 else (x-width,x)
                    mode='left_only' if side==0 else 'right_only'
            if pair:
                a,b=pair
                # Inferred identities never refresh the last observed boundary memory.
                if mode=='double':new[index]=pair
                elif prior:new[index]=prior
                lower=pair
                p=ground((a+b)/2,y,w,h)
                if p:observed.append(dict(y=y,left=float(a),right=float(b),mode=mode,ground=p))
        doubles=sum(p['mode']=='double' for p in observed)
        if doubles>=5:self.last_pair_t=t
        self.previous=new
        # Offset a visible single boundary along its ground-plane normal.
        # Pixel width is used only for identity association, not metric centering.
        for mode,side,sign in [('left_only','left',1),('right_only','right',-1)]:
            single=[p for p in observed if p['mode']==mode]
            if len(single)<4:
                for p in single:p['ground']=None
                continue
            edge=[ground(p[side],p['y'],w,h) for p in single]
            if any(v is None for v in edge):
                for p in single:p['ground']=None
                continue
            e=np.array(edge);c=np.polyfit(e[:,1],e[:,0],2)
            for p,(x,z) in zip(single,edge):
                slope=2*c[0]*z+c[1];offset=sign*GEOMETRY['line_center_width_m']/2
                norm=math.hypot(1,slope)
                p['ground']=[float(x+offset/norm),float(z-offset*slope/norm)]
                p['inferred_boundary']=[float(x+2*offset/norm),float(z-2*offset*slope/norm)]
        pts=sorted((p['ground'] for p in observed if p['ground'] is not None),key=lambda p:p[1])
        fit=[];residual=None;curvature=None
        if len(pts)>=6 and pts[-1][1]-pts[0][1]>.15:
            arr=np.array(pts);coef=np.polyfit(arr[:,1],arr[:,0],2)
            residual=float(np.sqrt(np.mean((np.polyval(coef,arr[:,1])-arr[:,0])**2)))
            if residual<.08:
                fit=[[float(np.polyval(coef,z)),float(z)] for z in np.linspace(arr[0,1],arr[-1,1],30)]
                z=float(np.median(arr[:,1]))
                curvature=float(2*coef[0]/(1+(2*coef[0]*z+coef[1])**2)**1.5)
        valid=bool(fit) and doubles>=3
        # Single-only continuation is bounded by time and never treated as observed geometry.
        if fit and not valid and self.last_pair_t is not None and t-self.last_pair_t<=.5:valid=True
        return dict(valid=valid,rows=observed,path=fit if valid else [],double_rows=doubles,
                    single_rows=len(observed)-doubles,residual_m=residual,
                    path_curvature=curvature,
                    reason='ESTIMATED_PATH' if valid else 'INSUFFICIENT_OR_AMBIGUOUS_BOUNDARIES',
                    hardware_output=False)

def pursuit(path,lookahead=1.0):
    """Right-positive ground coordinates, rear axle origin; never extrapolate."""
    if not math.isfinite(lookahead) or not .2<=lookahead<=3:raise ValueError('lookahead')
    if not path:return None
    target=next((p for p in path if math.hypot(*p)>=lookahead),None)
    if target is None:return None
    x,z=target
    return dict(target=target,curvature=2*x/(x*x+z*z))
