"""Offline hypotheses only: normal offset and evidence-gated mode selection.

All metric outputs depend on the supplied, explicitly estimated camera geometry.
No controller, serial or network imports.
"""
import numpy as np
from curve_replay import ground, pixel

def anchored_identity(points,side):
    """Conservative visible anchor, not a claim of semantic side recognition.

    A curve wholly in the mid-distance cannot cold-initialize a side merely
    because it happens to cross that half of the image.
    """
    if not points:return False
    x,y=max(points,key=lambda p:p[1])
    return bool((y>=320 and (x<300 if side=='left' else x>340)) or
                (y>=180 and (x<30 if side=='left' else x>610)))

def offset_boundary(points,side,g,distance=.2525,diagnostic=None):
    """Near-to-far parametric curve, offset along its inward ground normal."""
    def fail(reason):
        if diagnostic is not None:diagnostic['reason']=reason
        return []
    if diagnostic is not None:diagnostic.update(reason='OK',observed_points=len(points))
    if not points:return fail('NO_OBSERVATION')
    pts=[ground(x,y,640,360,g) for x,y in sorted(points,key=lambda p:-p[1])]
    a=np.array([p for p in pts if p is not None],float)
    if diagnostic is not None:diagnostic['projected_points']=len(a)
    if len(a)<8:return fail('PROJECTION_OUT_OF_RANGE')
    # Avoid duplicate/projected jumps and unsupported tiny fragments.
    a=a[np.r_[True,np.linalg.norm(np.diff(a,axis=0),axis=1)>.005]]
    if len(a)<8:return fail('INSUFFICIENT_DISTINCT_POINTS')
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(a,axis=0),axis=1))]
    if arc[-1]<.2:return fail('INSUFFICIENT_ARC_LENGTH')
    count=min(100,max(8,int(arc[-1]/.025)))
    samples=np.linspace(0,arc[-1],count)
    a=np.column_stack([np.interp(samples,arc,a[:,k]) for k in range(2)])
    smooth=np.array([np.mean(a[max(0,i-2):i+3],axis=0) for i in range(len(a))])
    tangent=np.gradient(smooth,axis=0);norm=np.linalg.norm(tangent,axis=1)
    normal=np.column_stack([tangent[:,1],-tangent[:,0]])/np.maximum(norm[:,None],1e-9)
    center=smooth+normal*distance*(1 if side=='left' else -1)
    # Reject offset foldbacks / cusps rather than drawing a spurious loop.
    if np.any(np.sum(np.diff(center,axis=0)*np.diff(smooth,axis=0),axis=1)<=0):return fail('OFFSET_FOLDBACK')
    result=[[float(x),float(z)] for x,z in center if .33<z<3.33]
    return result if len(result)>=8 else fail('OFFSET_OUT_OF_RANGE')

class BoundaryIdentity:
    """Associate current skeleton observations, never reuse an old output path.

    Unwarped history is ONLY a bounded association proposal, not spatial memory.
    Association needs overlapping rows and consistent displacement. Expires on
    gaps, ambiguity, or missing observations; image half never relabels a track.
    """
    def __init__(self):
        self.previous={};self.time=None

    def step(self,mask,t):
        import cv2
        from temporal_lane import thin,graph_path,row_curve
        if self.time is None or not 0<t-self.time<=.5:self.previous={}
        sk=thin(cv2.resize(mask.astype(np.uint8),(640,360),interpolation=cv2.INTER_NEAREST))
        sk[:100]=0
        proposals=[]
        for side in ('left','right'):
            for seed in ([],self.previous.get(side,[])):
                if len(seed)==0 and proposals and side=='left':continue
                p=graph_path(sk,side,np.asarray(seed).reshape(-1,2))
                rows=row_curve(p)
                if len(rows)>=8:
                    curve=[[x,float(y)] for y,x in rows.items()]
                    if not any(self.distance(curve,q)<3 for q in proposals):proposals.append(curve)
        output={};diagnostics={}
        used=set()
        # Match existing identities before allowing cold initialization.
        for side,prior in self.previous.items():
            scores=sorted((self.distance(prior,p),i) for i,p in enumerate(proposals))
            if scores and scores[0][0]<22 and (len(scores)==1 or scores[1][0]-scores[0][0]>10):
                _,i=scores[0]
                if i not in used:
                    output[side]=proposals[i];used.add(i)
                    diagnostics[side]={'reason':'TRACK_ASSOCIATED','distance_px':scores[0][0]}
        for side in ('left','right'):
            if side in output:continue
            candidates=[i for i,p in enumerate(proposals) if i not in used and anchored_identity(p,side)]
            if len(candidates)==1:
                i=candidates[0];used.add(i);output[side]=proposals[i]
                diagnostics[side]={'reason':'VISIBLE_ANCHOR'}
            else:diagnostics[side]={'reason':'IDENTITY_AMBIGUOUS' if proposals else 'NO_OBSERVATION'}
        self.previous=output;self.time=t
        return {s:output.get(s,[]) for s in ('left','right')},diagnostics

    @staticmethod
    def distance(a,b):
        aa={int(y):x for x,y in a};bb={int(y):x for x,y in b}
        keys=aa.keys()&bb.keys()
        if len(keys)<8:return float('inf')
        d=np.array([bb[k]-aa[k] for k in keys])
        # Penalize inconsistent shape changes, not just a lucky intersection.
        return float(np.median(abs(d)) + np.percentile(abs(d-np.median(d)),80))

def image_curve(path,g):
    return [p for x,z in path if (p:=pixel(x,z,640,360,g)) and -30<p[0]<670 and 90<p[1]<390]

class PrioritySelector:
    """Confirmation hysteresis; never hold a previous frame's unaligned path."""
    def __init__(self):self.mode=None;self.pending=None;self.count=0
    def update(self,candidates):
        available={k:v for k,v in candidates.items() if len(v)>=8}
        # Prefer paired evidence. For single evidence retain identity if present.
        desired='double' if 'double' in available else self.mode if self.mode in available else max(available,key=lambda k:len(available[k]),default=None)
        if desired is None:
            self.mode=None;self.pending=None;self.count=0
            return dict(path=[],mode='no_evidence',pending=None)
        if desired!=self.mode:
            if desired==self.pending:self.count+=1
            else:self.pending=desired;self.count=1
            if self.count>=3:self.mode=desired;self.pending=None;self.count=0
        # Previous mode can continue only with CURRENT-frame observations.
        path=available.get(self.mode,[])
        return dict(path=path,mode=self.mode if path else 'confirming',pending=self.pending)
