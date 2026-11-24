"""K offline worker implementation; no experiment result or hardware dependencies.
Frame-count thresholds follow the reviewed K experiment (tested at 10 fps).
"""
import copy,time
import cv2
import numpy as np
import temporal_lane as temporal
from road_segments_v31 import RoadPath
from junction_straight import JunctionStraight
YS=np.array([180.,260.,330.])
KEYS=["left","right"]
def motion(previous,current):
    H,d=temporal.estimate_motion(previous,current)
    if H is not None or previous is None:return H,d
    roi=np.zeros_like(previous);roi[190:335,30:610]=255
    p=cv2.goodFeaturesToTrack(previous,300,.015,8,mask=roi)
    if p is None:return None,d
    q,ok,_=cv2.calcOpticalFlowPyrLK(previous,current,p,None)
    if q is None:return None,d
    back,ok2,_=cv2.calcOpticalFlowPyrLK(current,previous,q,None)
    if back is None:return None,d
    keep=(ok.ravel()>0)&(ok2.ravel()>0)&(np.linalg.norm(p-back,axis=2).ravel()<1.2)
    a=p[keep].reshape(-1,2);b=q[keep].reshape(-1,2)
    if len(a)<18:return None,d
    A,mask=cv2.estimateAffinePartial2D(a,b,method=cv2.RANSAC,ransacReprojThreshold=2.5)
    if A is None:return None,d
    good=mask.ravel()>0;scale=np.linalg.norm(A[:,0]);span=np.ptp(a[good],axis=0)
    H2=np.vstack([A,[0,0,1]]);error=float(np.median(np.linalg.norm(temporal.transform(a[good],H2)-b[good],axis=1)))
    if good.sum()<16 or good.mean()<.45 or min(span)<55 or span[0]<120 or not .9<scale<1.1 or error>1.5:return None,d
    grid=np.array([[80,200],[560,200],[80,330],[560,330]],np.float32)
    if np.max(np.linalg.norm(temporal.transform(grid,H2)-grid,axis=1))>40:return None,d
    return H2,dict(valid=True,reason='LOCAL_SIMILARITY_FALLBACK',original_reason=d['reason'],inliers=int(good.sum()),median_error_px=error)
def base_fusion(current,hist):
    fit=np.array(current['fits']);out=[];diagnostics=[];bad=False
    for side,key in enumerate(['left','right']):
        c=current[key];points=np.array(c['points']);ys=points[:,1];pivot=float(np.mean(ys));span=float(np.ptp(ys))
        distance=float(np.percentile(abs(np.polyval(fit[side]-hist[side],ys)),90))
        angle=float(abs(np.arctan(fit[side,0])-np.arctan(hist[side,0]))*180/np.pi)
        tolerance=6+min(c['paint_width'],8)
        compatible=distance<tolerance and angle<20
        bad|=not compatible
        # Location is supported at observed pivot; extrapolation slope is much
        # less certain for short far segments. Never gate on blank near pixels.
        slope_weight=float(np.clip(span/90,.15,.65))
        a=(1-slope_weight)*hist[side,0]+slope_weight*fit[side,0]
        x=.7*np.polyval(fit[side],pivot)+.3*np.polyval(hist[side],pivot)
        out.append([float(a),float(x-a*pivot)])
        diagnostics.append(dict(visible_error_px=distance,angle_difference_deg=angle,tolerance_px=tolerance,pivot_y=pivot,current_position_weight=.7,current_slope_weight=slope_weight,compatible=compatible))
    return np.array(out),diagnostics,bad
def detect(im, identity_prior=None):
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    mask = ((hsv[:,:,1]<65)&(hsv[:,:,2]>175)).astype('uint8')*255
    mask[:145]=0; mask[270:]=0
    edge=cv2.Canny(mask,30,80); edge[144:147]=0; edge[268:271]=0
    lines=cv2.HoughLinesP(edge,1,np.pi/720,10,minLineLength=16,maxLineGap=4)
    pools=[[],[]]
    for x,y,xx,yy in ([] if lines is None else lines.reshape(-1,4)):
        if abs(yy-y)<5: continue
        a=(xx-x)/(yy-y)
        if not .4<abs(a)<4: continue
        side=int(a>0); f=np.array([a,x-a*y]); pts=[]; widths=[]; wide_rows=0
        for row in range(min(y,yy),max(y,yy)+1):
            u=np.polyval(f,row); lo=max(0,int(u)-5); hi=min(640,int(u)+6)
            if hi<=lo: continue
            xs=np.flatnonzero(mask[row,lo:hi])+lo
            if not len(xs): continue
            q=int(xs[np.argmin(abs(xs-u))]); l=q; r=q
            while l>0 and mask[row,l-1] and q-l<36: l-=1
            while r<639 and mask[row,r+1] and r-q<36: r+=1
            if r-l>35:
                # Keep a real inward transition, never the capped search endpoint.
                boundary=r if side==0 else l
                outside=boundary+1 if side==0 else boundary-1
                if not 0<=outside<640 or mask[row,outside]: continue
                if abs(boundary-u)>6: continue
                wide_rows+=1
            pts.append([float(r if side==0 else l),row]); widths.append(r-l+1)
        if len(pts)<4: continue
        pts=np.array(pts); fit=np.polyfit(pts[:,1],pts[:,0],1)
        pts=pts[abs(pts[:,0]-np.polyval(fit,pts[:,1]))<3]
        if len(pts)<4 or np.ptp(pts[:,1])<3: continue
        fit=np.polyfit(pts[:,1],pts[:,0],1)
        if (side==0 and fit[0]>=-.4) or (side==1 and fit[0]<=.4): continue
        if any(max(abs(np.polyval(fit-np.array(c['fit']),[180,230])))<4 for c in pools[side]): continue
        span=float(np.ptp(pts[:,1])); top=float(pts[:,1].min())
        residual=float(np.sqrt(np.mean((pts[:,0]-np.polyval(fit,pts[:,1]))**2)))
        pools[side].append(dict(fit=fit.tolist(),points=pts.tolist(),span=span,top=top,
            paint_width=float(np.median(widths)),wide_fraction=min(1,wide_rows/len(pts)),residual=residual,
            score=min(span,45)-1.8*(top-145)-4*residual-12*min(1,wide_rows/len(pts))))
    pairs=[]
    for l in pools[0]:
        for r in pools[1]:
            lf,rf=np.array(l['fit']),np.array(r['fit'])
            vy=(lf[1]-rf[1])/(rf[0]-lf[0]); vx=np.polyval(lf,vy)
            # Geometric checks remain, but a line no longer needs to reach y=188.
            if not (95<vy<215 and 130<vx<510 and 20<np.polyval(rf-lf,260)<800): continue
            # Both observed arms must lie below their common vanishing point;
            # otherwise opposite branch arms form an X instead of the road.
            if vy>min(l['top'],r['top'])+3: continue
            if min(l['span'],r['span'])<5: continue
            score=l['score']+r['score']-2*abs(l['top']-r['top'])
            # A recent reference helps identify which paint segments belong to
            # this lane, even when it cannot be projected for path output. It
            # is a soft association cost, not a substitute current measurement.
            if identity_prior is not None:
                score-=sum(.8*float(np.median(abs(np.polyval(np.array(c['fit'])-identity_prior[k],np.array(c['points'])[:,1])))) for k,c in enumerate([l,r]))
            pairs.append(dict(left=l,right=r,fits=[lf.tolist(),rf.tolist()],score=score))
    pairs.sort(key=lambda c:-c['score'])
    return (pairs[0] if pairs else None),pools,mask
def warp(fits,H):
    result=[]
    for f in fits:
        pts=temporal.transform(np.column_stack([np.polyval(f,YS),YS]),H)
        result.append(np.polyfit(pts[:,1],pts[:,0],1))
    result=np.array(result)
    if not np.isfinite(result).all() or np.any(np.polyval(result[1]-result[0],[220,330])<=0): return None
    return result
def supported_reference(memory,pools):
    """Brief image-space translation checked by a clear side, not optical pose."""
    matches=[]
    for k,pool in enumerate(pools):
        for c in pool:
            if c['span']<15 or c['residual']>2 or c.get('wide_fraction',0)>.5: continue
            ys=np.array(c['points'])[:,1]
            delta=np.polyval(np.array(c['fit'])-memory[k],ys)
            error=float(np.percentile(abs(delta),90))
            angle=float(abs(np.arctan(c['fit'][0])-np.arctan(memory[k,0]))*180/np.pi)
            if error<14 and angle<10:
                matches.append((error,k,c,float(np.median(delta))))
    if not matches:return None,None,None
    error,k,c,delta=min(matches,key=lambda x:x[0]); ref=np.array(memory,dtype=float)
    shift=float(np.clip(delta*.5,-5,5));ref[:,1]+=shift
    return ref,(k,c),dict(source='CURRENT_SIDE_CHECKED_HISTORY',side=KEYS[k],
        visible_error_px=error,translation_px=shift,width_source='recent_history',
        limitation='image-space hypothesis; not recovered optical motion')
def conservative_fusion(current,reference):
    proposal,diag,bad=base_fusion(current,reference)
    for k,key in enumerate(KEYS):
        c=current[key];ys=np.array(c['points'])[:,1];pivot=float(np.mean(ys))
        # Short support calibrates position, not an entire extrapolated angle.
        weight=min(.4,(c['span']/80)**2)*(1-.7*min(1,c.get('wide_fraction',0)))
        f=np.array(c['fit']);a=(1-weight)*reference[k,0]+weight*f[0]
        x=.55*np.polyval(f,pivot)+.45*np.polyval(reference[k],pivot)
        proposed=np.array([a,x-a*pivot]);change=proposed-reference[k]
        max_change=float(np.max(abs(np.polyval(change,[180,330]))))
        scale=min(1,12/max_change) if max_change else 1
        proposal[k]=reference[k]+scale*change
        diag[k].update(current_slope_weight=weight,current_position_weight=.55,
                       correction_scale=scale,max_extrapolated_correction_px=12)
    return proposal,diag,bad

class VisibleTracker:
    def __init__(self):
        self.state=dict(memory=None,anchor_gray=None,anchor_index=None,last_observed=-1000,last_alignment=-1000,pending=None,streak=0)
        self.index=0
    def step(self,image):
        im=cv2.resize(image,(640,360));n=self.index;rows=[]
        memory=self.state["memory"]
        anchor_gray=self.state["anchor_gray"]
        anchor_index=self.state["anchor_index"]
        last_observed=self.state["last_observed"]
        last_alignment=self.state["last_alignment"]
        pending=self.state["pending"]
        streak=self.state["streak"]
        gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        current,pools,mask=detect(im,memory if n-last_observed<=10 else None)
        hist=None;bridge=None;bridge_side=None;bridge_diag=None;source=anchor_index
        md=dict(valid=False,reason='NO_SAVED_REFERENCE')
        if memory is not None and n-last_observed<=30:
            H,md=motion(anchor_gray,gray)
            if H is not None:
                hist=warp(memory,H)
                if hist is not None:last_alignment=n
        elif memory is not None:md=dict(valid=False,reason='REFERENCE_STALE_RETAINED')
        if hist is None and memory is not None and n-last_alignment<=3 and n-last_observed<=3:
            bridge,bridge_side,bridge_diag=supported_reference(memory,pools)
        reference=hist if hist is not None else bridge
        fused=reference;decision='HISTORY_ONLY' if hist is not None else 'NO_REFERENCE';diag=[];observed=False
        if bridge is not None:decision='SIDE_CHECKED_BRIDGE';diag=[bridge_diag]
        if current is not None:
            fits=np.array(current['fits'])
            if reference is None:
                fused=fits;decision='CURRENT_ONLY_NO_WARP';observed=True;streak=0
                if memory is None:last_alignment=n
            else:
                proposal,fd,bad=conservative_fusion(current,reference)
                stable=pending is not None and np.max(abs(np.array([np.polyval(fits[k]-pending[k],[180,220]) for k in range(2)])))<18
                streak=(streak+1 if stable else 1) if bad else 0
                diag+=fd
                if not bad or streak>=2:
                    fused=proposal;observed=True
                    decision='GEOMETRY_BRIDGE_FUSE' if bridge is not None else ('VISIBLE_FUSE' if not bad else 'CONFIRMED_VISIBLE_CORRECTION')
                else:decision='BRIDGE_CONFLICT_PENDING' if bridge is not None else 'LARGE_CONFLICT_PENDING'
            pending=fits
        elif bridge is not None:
            k,c=bridge_side
            current={KEYS[k]:c,KEYS[1-k]:dict(points=[],inferred=True),'fits':None,'single_side':KEYS[k]}
            # No observation age renewal: a brief bridge cannot sustain itself.
            pending=None;streak=0
        elif hist is not None:
            ref,side,sd=supported_reference(hist,pools)
            if ref is not None:
                fused=ref;k,c=side;observed=True
                current={KEYS[k]:c,KEYS[1-k]:dict(points=[],inferred=True),'fits':None,'single_side':KEYS[k]}
                decision='SINGLE_SIDE_POSITION_FUSE';diag=[sd]
            pending=None;streak=0
        else:pending=None;streak=0
        if observed:last_observed=n
        if fused is not None:
            memory=np.array(fused);anchor_gray=gray.copy();anchor_index=n
        rows.append(dict(index=n,current=current,history=None if hist is None else hist.tolist(),
            geometric_reference=None if bridge is None else bridge.tolist(),
            fused=None if fused is None else np.asarray(fused).tolist(),motion=md,decision=decision,diagnostics=diag,
            memory=dict(source_index=source,saved_index=anchor_index,last_observed=last_observed,
                evidence_age_frames=n-last_observed,retained=memory is not None,last_alignment=last_alignment,
                recovered=bool(hist is not None and source is not None and n-source>1)),
            candidates=[len(x) for x in pools],mask_pixels=int(np.count_nonzero(mask))))
        self.state={'memory':memory,'anchor_gray':anchor_gray,'anchor_index':anchor_index,'last_observed':last_observed,'last_alignment':last_alignment,'pending':pending,'streak':streak}
        self.index+=1
        return rows[0]
def evidence(a,v):
    pts=np.array([[p['x'],p['y']] for p in a['path_samples'] if p['source']=='double' and 230<=p['y']<=326]).reshape(-1,2)
    d=dict(double_rows=len(pts),span=0,center_error=None,width_error=None,angle_error=None,eligible=False)
    if len(pts)<2 or v is None:return d
    fits=np.array(v);ys=pts[:,1];d['span']=float(np.ptp(ys))
    d['center_error']=float(np.percentile(abs(pts[:,0]-np.polyval(fits.mean(axis=0),ys)),90))
    edges=[{int(y):x for x,y in edge} for edge in a.get('observed_edges',[])[:2]]
    if len(edges)!=2:return d
    common=[int(y) for y in ys if int(y) in edges[0] and int(y) in edges[1]]
    if len(common)<12:return d
    widths=np.array([edges[1][y]-edges[0][y] for y in common]);vw=np.polyval(fits[1]-fits[0],common)
    d['width_error']=float(np.percentile(abs(widths-vw)/np.maximum(abs(vw),20),90))
    angles=[]
    for k in range(2):
        f=np.polyfit(common,[edges[k][y] for y in common],1)
        angles.append(float(abs(np.arctan(f[0])-np.arctan(fits[k,0]))*180/np.pi))
    d['angle_error']=max(angles)
    d['eligible']=bool(len(pts)>=18 and d['span']>=64 and d['center_error']<18 and d['width_error']<.25 and d['angle_error']<15)
    return d
class Handoff:
    """Hysteresis for offline 10-fps clips; parameters are experimental.

    Strict current geometry starts confirmation. A short uncertainty band pauses
    confirmation instead of erasing it. Once A owns the path, judge current A
    support and continuity, not its distance from the old straight corridor.
    """
    def __init__(self):
        self.state='APPROACH';self.entry=0;self.loss=0;self.armed=False;self.confirm=0;self.alpha=0.;self.bad=0
        self.recent=[];self.reason='INITIAL'

    def step(self,entry,ev):
        good=ev['eligible'];supported=ev['supported'];hold=ev['hold_eligible']
        self.recent.append(good);self.recent=self.recent[-5:]
        self.reason='HOLD'
        if self.state=='APPROACH':
            self.entry=self.entry+1 if entry else 0
            if self.entry>=3:self.state='CROSS';self.reason='ENTRY_CONFIRMED'
        elif self.state=='CROSS':
            self.loss=self.loss+1 if not good else 0
            if self.loss>=3:self.armed=True
            if self.armed and good:self.state='CONFIRM';self.confirm=1;self.bad=0;self.recent=[True]
        elif self.state=='CONFIRM':
            self.bad=0 if hold else self.bad+1
            self.confirm=sum(self.recent)
            if self.bad>=3:self.state='CROSS';self.confirm=0;self.reason='SUSTAINED_REJOIN_FAILURE'
            elif len(self.recent)==5 and self.confirm>=4 and hold:
                self.state='BLEND';self.alpha=.2;self.bad=0;self.reason='FOUR_OF_FIVE_CURRENT_CONFIRMATIONS'
        elif self.state=='BLEND':
            if hold:
                self.bad=0;self.alpha=min(1,self.alpha+.2)
                if self.alpha>=.999:self.state='FOLLOW_A';self.alpha=1;self.reason='HANDOFF_COMPLETE'
            else:
                self.bad+=1
                if self.bad>=3:self.state='CROSS';self.alpha=0;self.confirm=0;self.reason='SUSTAINED_BLEND_FAILURE'
        elif self.state=='FOLLOW_A':
            self.bad=0 if ev['a_continuous'] and supported else self.bad+1
            if self.bad>=3:self.state='CROSS';self.alpha=0;self.confirm=0;self.armed=True;self.reason='CURRENT_A_LOST_OR_DISCONTINUOUS'
        return self.state

def center(path,y):
    if not path:return None
    p=np.array(path);order=np.argsort(p[:,1]);p=p[order]
    return float(np.interp(y,p[:,1],p[:,0])) if p[0,1]<=y<=p[-1,1] else None

class KPath:
    def __init__(self,row=.6):
        self.row=row
        self.base=RoadPath("road-memory-v3.1",None,row)
        self.hint=JunctionStraight("junction-straight-v1",None,row)
        self.visible=VisibleTracker();self.tracker=Handoff();self.previous=[];self.last_delta=None;self.index=0;self.last_time=None
    def step(self,image,mask,t):
        if self.last_time is not None and (t<=self.last_time or t-self.last_time>1):
            self.__init__(self.row)
        self.last_time=t
        a=self.base.step(image,mask,t);hint=self.hint.step(image,mask,t)["junction"]["active"];v=self.visible.step(image)
        tracker=self.tracker;rows=self.previous;last_delta=self.last_delta;n=self.index
        ev=evidence(a,v['fused'])
        # Current double support is independent of a possibly stale v3 reference.
        doubles=[p for p in a['path_samples'] if p['source']=='double' and 230<=p['y']<=326]
        ev['supported']=bool(len(doubles)>=18 and np.ptp([p['y'] for p in doubles])>=64)
        # Six pixels of ambiguity cover paint-edge/local fit uncertainty at 640px;
        # this is an offline experiment parameter, not a safety threshold.
        ev['hold_eligible']=bool(ev['supported'] and ev['center_error'] is not None and ev['center_error']<24 and ev['width_error'] is not None and ev['width_error']<.30 and ev['angle_error'] is not None and ev['angle_error']<18)
        current_a=(np.array(a['path'])/2).tolist()
        old_a=getattr(self,'last_a',[])
        changes=[abs(center(current_a,y)-center(old_a,y)) for y in [230,260,300,326] if center(current_a,y) is not None and center(old_a,y) is not None]
        ev['a_step_px']=max(changes) if changes else None
        ev['a_continuous']=bool(changes and max(changes)<24)
        self.last_a=current_a
        state=tracker.step(hint,ev)
        ap=(np.array(a['path'])/2).tolist();f=np.mean(v['fused'],axis=0) if v['fused'] is not None else None
        vp=[] if f is None else [[float(np.polyval(f,y)),float(y)] for y in range(160,335,2)]
        path=vp;source='v3';alpha=tracker.alpha
        if state=='APPROACH':path=ap;source='A'
        elif state=='FOLLOW_A' and ev['supported'] and ev['a_continuous']:
            path=ap;source='A'
            last_delta=None if f is None else [[float(x-np.polyval(f,y)),y] for x,y in ap]
        elif state=='BLEND' and ev['hold_eligible'] and f is not None:
            path=[[float((1-alpha)*np.polyval(f,y)+alpha*x),y] for x,y in ap]
            last_delta=[[float(alpha*(x-np.polyval(f,y))),y] for x,y in ap];source='blend'
        elif state in ['BLEND','FOLLOW_A'] and f is not None:
            # Brief evidence gaps do not output newly rejected A geometry.
            if last_delta:
                path=[[float(np.polyval(f,y)+dx),y] for dx,y in last_delta]
            source='temporary_v3_support'
        x=center(path,330);previous=center(rows[-1]['path'],330) if rows else None
        steps={}
        for y in [210,260,330]:
            now=center(path,y);prev=center(rows[-1]['path'],y) if rows else None
            steps[str(y)]=None if now is None or prev is None else now-prev
        rows.append(dict(index=n,state=state,source=source,alpha=alpha,evidence=ev,armed=tracker.armed,
            confirm=tracker.confirm,entry_hint=hint,path=path,a_path=ap,v3_path=vp,current=v['current'],
            near_x=x,raw_step=None if x is None or previous is None else x-previous,
            raw_steps=steps,
            handoff_offset=None if x is None or center(vp,330) is None else x-center(vp,330)))
        r=rows[-1];self.previous=[r];self.last_delta=last_delta;self.index+=1
        result=copy.deepcopy(a)
        result.update(path=[[x*2,y*2] for x,y in r['path']],valid=bool(r['path']),mode='K_'+r['state'],reason='K_'+r['source'],hardware_output=False,
            k=dict(state=r['state'],source=r['source'],alpha=r['alpha'],evidence=r['evidence'],raw_steps=r['raw_steps'],entry_hint=hint,visible_decision=v['decision'],motion=v['motion'],handoff_reason=tracker.reason,revision='k-stable-v2'))
        if r['source']!='A':
            result.update(quality=.35,path_samples=[dict(x=x,y=y,source='junction_inferred',confidence=.35,age_s=0) for x,y in r['path']],road_segments=[dict(source='junction_inferred',points=result['path'],confidence=.35,age_s=0)],history_segments=[],history_path=[])
        result['path_semantics']='estimated_road_center_not_vehicle_trajectory'
        return result
