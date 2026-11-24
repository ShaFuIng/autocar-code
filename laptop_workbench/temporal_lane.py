"""Offline visual lane memory. No hardware/network interface.

Current observations are skeleton graph paths. History is transformed with a
validated ground-image homography, expires by original observation time, and is
never promoted to an observation. Metric projection remains explicitly estimated.
"""
from collections import deque
import cv2
import numpy as np
from curve_replay import ground

def thin(mask):
    """Zhang-Suen thinning on a padded binary image."""
    a=np.pad((mask>0).astype(np.uint8),1)
    for _ in range(100):
        total=0
        for phase in (0,1):
            p=a[1:-1,1:-1]
            ns=[a[:-2,1:-1],a[:-2,2:],a[1:-1,2:],a[2:,2:],
                a[2:,1:-1],a[2:,:-2],a[1:-1,:-2],a[:-2,:-2]]
            b=sum(ns);changes=sum((ns[i]==0)&(ns[(i+1)%8]==1) for i in range(8))
            if phase==0:gate=(ns[0]*ns[2]*ns[4]==0)&(ns[2]*ns[4]*ns[6]==0)
            else:gate=(ns[0]*ns[2]*ns[6]==0)&(ns[0]*ns[4]*ns[6]==0)
            remove=(p==1)&(b>=2)&(b<=6)&(changes==1)&gate
            total+=int(remove.sum());p[remove]=0
        if not total:break
    return a[1:-1,1:-1]

def transform(points,H):
    if not len(points):return np.empty((0,2))
    return cv2.perspectiveTransform(np.asarray(points,np.float32).reshape(1,-1,2),H)[0]

def estimate_motion(previous,current):
    """Forward/backward LK plus RANSAC; reject poor spread and implausible warp."""
    diagnostic=dict(valid=False,reason='NO_PREVIOUS_FRAME',tracks=0,inliers=0)
    if previous is None:return None,diagnostic
    roi=np.zeros_like(previous);roi[int(len(roi)*.48):int(len(roi)*.94),20:-20]=255
    p=cv2.goodFeaturesToTrack(previous,250,.02,9,mask=roi)
    if p is None or len(p)<18:return None,dict(diagnostic,reason='TOO_FEW_FEATURES')
    q,ok,_=cv2.calcOpticalFlowPyrLK(previous,current,p,None,winSize=(21,21),maxLevel=3)
    if q is None:return None,dict(diagnostic,reason='NO_FORWARD_FLOW')
    back,ok2,_=cv2.calcOpticalFlowPyrLK(current,previous,q,None,winSize=(21,21),maxLevel=3)
    if back is None:return None,dict(diagnostic,reason='NO_BACKWARD_FLOW')
    keep=(ok.ravel()>0)&(ok2.ravel()>0)&(np.linalg.norm(p-back,axis=2).ravel()<1.2)
    a=p[keep].reshape(-1,2);b=q[keep].reshape(-1,2);diagnostic['tracks']=len(a)
    if len(a)<18:return None,dict(diagnostic,reason='FLOW_INCONSISTENT')
    H,inliers=cv2.findHomography(a,b,cv2.RANSAC,2.5)
    if H is None:return None,dict(diagnostic,reason='NO_HOMOGRAPHY')
    k=inliers.ravel()>0;n=int(k.sum());diagnostic['inliers']=n
    if n<16 or n/len(a)<.65:return None,dict(diagnostic,reason='LOW_INLIER_RATIO')
    span=np.ptp(a[k],axis=0)
    if span[0]<120 or span[1]<55:return None,dict(diagnostic,reason='POOR_SPATIAL_COVERAGE')
    error=float(np.median(np.linalg.norm(transform(a[k],H)-b[k],axis=1)))
    grid=np.array([[80,200],[560,200],[80,330],[560,330]],np.float32)
    moved=transform(grid,H)
    if not np.isfinite(moved).all() or np.max(np.linalg.norm(moved-grid,axis=1))>70:
        return None,dict(diagnostic,reason='IMPLAUSIBLE_WARP')
    area=abs(cv2.contourArea(moved[[0,1,3,2]].astype(np.float32)))/(480*130)
    if not .7<area<1.4:return None,dict(diagnostic,reason='IMPLAUSIBLE_SCALE')
    return H,dict(diagnostic,valid=True,reason='GROUND_IMAGE_FLOW',median_error_px=error)

def graph_path(skeleton,side,predicted):
    ys,xs=np.nonzero(skeleton);points=np.column_stack([xs,ys])
    if not len(points):return []
    if len(predicted):
        # Use the most recent near boundary location to retain identity.
        anchor=predicted[np.argmax(predicted[:,1])]
        distances=np.linalg.norm(points-anchor,axis=1);i=int(np.argmin(distances))
        if distances[i]>32:return []
        seed=tuple(points[i])
    else:
        candidates=points[(ys>210)&((xs<290) if side=='left' else (xs>350))]
        if not len(candidates):return []
        seed=tuple(candidates[np.argmax(candidates[:,1])])
    queue=deque([seed]);parent={seed:None};best=seed
    while queue:
        x,y=queue.popleft()
        if y<best[1]:best=(x,y)
        for dx,dy in [(-1,-1),(0,-1),(1,-1),(-1,0),(1,0),(-1,1),(0,1),(1,1)]:
            nx,ny=x+dx,y+dy
            if not (0<=nx<640 and 100<=ny<360 and ny<=seed[1]+6):continue
            key=(nx,ny)
            if key not in parent and skeleton[ny,nx]:
                parent[key]=(x,y);queue.append(key)
    path=[];cursor=best
    while cursor is not None:path.append(cursor);cursor=parent[cursor]
    if len(path)<25 or seed[1]-best[1]<25:return []
    path=np.asarray(path[::-1],float)
    # Graph can branch at a merged distant mask; stop returning toward the car.
    keep=[path[0]];lowest=path[0,1]
    for p in path[1:]:
        if p[1]>lowest+8:break
        keep.append(p);lowest=min(lowest,p[1])
    return np.asarray(keep)

def row_curve(points):
    """Resample an already associated graph curve; no cross-boundary row matching."""
    if not len(points):return {}
    points=np.asarray(points)
    out={}
    for y in range(100,351,4):
        near=points[np.abs(points[:,1]-y)<=2]
        if len(near):out[y]=float(np.median(near[:,0]))
    return out

class TemporalLane:
    def __init__(self):
        self.gray=None;self.time=None;self.memory={'left':[],'right':[]}
        self.horizon=.65

    def step(self,image,mask,t):
        image=cv2.resize(image,(640,360))
        mask=cv2.resize(mask.astype(np.uint8),(640,360),interpolation=cv2.INTER_NEAREST)
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        gap=self.time is None or t<=self.time or t-self.time>.5
        H,motion=estimate_motion(None if gap else self.gray,gray)
        # Without verified alignment, do not blend or carry historical positions.
        if H is None:self.memory={'left':[],'right':[]}
        else:
            for side,entries in self.memory.items():
                warped=transform([[e[0],e[1]] for e in entries],H)
                self.memory[side]=[[float(x),float(y),e[2]] for (x,y),e in zip(warped,entries)
                    if t-e[2]<=self.horizon and -40<x<680 and 100<y<410]
        sk=thin(mask);sk[:100]=0
        curves={};rejected=0
        for side in ('left','right'):
            predicted=np.array([[e[0],e[1]] for e in self.memory[side]]).reshape(-1,2)
            observed=graph_path(sk,side,predicted)
            # Retry cold initialization only when there is no historical identity.
            curves[side]=row_curve(observed)
        memrows={}
        for side in ('left','right'):
            prior=row_curve([[e[0],e[1]] for e in self.memory[side]])
            updated=[]
            for y,x in curves[side].items():
                if y in prior:
                    if abs(x-prior[y])>24:
                        rejected+=1;continue
                    x=.75*x+.25*prior[y]
                updated.append([x,float(y),t])
            current_y={int(e[1]) for e in updated}
            # Only keep original observation ages for unseen locations.
            updated.extend(e for e in self.memory[side] if int(round(e[1]/4)*4) not in current_y)
            self.memory[side]=updated[-400:]
            memrows[side]={}
            for y in range(100,351,4):
                nearby=[e for e in updated if abs(e[1]-y)<=2]
                if nearby:
                    fresh=[e for e in nearby if e[2]==t];use=fresh or nearby
                    memrows[side][y]=(float(np.median([e[0] for e in use])),max(e[2] for e in use))
        rows=[];centers=[];metric=[]
        for y in range(348,99,-4):
            l=memrows['left'].get(y);r=memrows['right'].get(y)
            if not l or not r:continue
            if r[0]-l[0]<16:rejected+=1;continue
            age=max(t-l[1],t-r[1])
            mode='double' if age<1e-6 else 'memory'
            x=(l[0]+r[0])/2
            rows.append(dict(y=y,left=l[0],right=r[0],mode=mode,age_s=age,
                             confidence=max(0,1-age/self.horizon)))
            centers.append([x,float(y)])
            g=ground(x,y,640,360)
            if g:metric.append(g)
        self.gray=gray;self.time=t
        doubles=sum(r['mode']=='double' for r in rows)
        # History may bridge brief missing observations, but cannot self-renew.
        valid=len(metric)>=6 and bool(curves['left'] or curves['right'])
        metric.sort(key=lambda p:p[1])
        return dict(valid=valid,path=metric if valid else [],image_path=centers,
                    boundaries={s:[[x,float(y)] for y,x in curves[s].items()] for s in curves},
                    memory={s:self.memory[s] for s in self.memory},rows=rows,
                    double_rows=doubles,single_rows=len(rows)-doubles,
                    motion=motion,rejected_points=rejected,
                    reason='OBSERVED_AND_ALIGNED_MEMORY' if valid else 'INSUFFICIENT_ASSOCIATED_BOUNDARIES',
                    hardware_output=False,metric_calibrated=False,
                    memory_kind='IMAGE_GROUND_HOMOGRAPHY_NOT_METRIC_ODOMETRY')
