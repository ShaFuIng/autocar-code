"""Offline junction v2. Pixel heuristics; no car, metric trajectory or safety claim.

Consumes frozen v3.1 evidence, never human labels. All timers use capture time.
"""
import copy
import cv2
import numpy as np
from junction_straight import white_runs
from temporal_lane import estimate_motion, transform

PARAMS=dict(confirm_s=.30,rejoin_s=.60,rejoin_gap_s=.30,track_hold_s=.45,association_px=35,
            width_ratio=(.35,2.2),max_slope=1.25,max_residual_px=8,max_turn=.9)

def candidates(mask,entry,width):
    chains=[]
    for y in range(150,331,4):
        runs=white_runs(mask,y); options=[]; expected=max(12,float(np.polyval(width,y)))
        for l,r in zip(runs,runs[1:]):
            w=r[0]-l[1];x=(l[1]+r[0])/2
            if 8<w and .35<w/expected<2.2:options.append((x,y,w))
        used=set()
        for point in options:
            x,_,w=point;matches=[]
            for i,c in enumerate(chains):
                if i in used or not 0<y-c[-1][1]<=8:continue
                slope=(c[-1][0]-c[-2][0])/(c[-1][1]-c[-2][1]) if len(c)>1 else 0
                dx=abs(x-(c[-1][0]+slope*(y-c[-1][1])))
                if dx<10 and abs(x-c[-1][0])/(y-c[-1][1])<1.5 and abs(w-c[-1][2])<max(15,w*.3):matches.append((dx,i))
            if matches:
                _,i=min(matches);chains[i].append(point);used.add(i)
            else:chains.append([point]);used.add(len(chains)-1)
    good=[];rejected=[]
    for chain in chains:
        if len(chain)<5:continue
        a=np.array(chain);fit=np.polyfit(a[:,1],a[:,0],1)
        residual=float(np.max(abs(a[:,0]-np.polyval(fit,a[:,1]))))
        slopes=np.diff(a[:,0])/np.diff(a[:,1]);turn=float(np.ptp(slopes))
        reason='NEAR_ONLY_NOT_EXIT' if a[0,1]>230 else 'SIDEWAYS' if abs(fit[0])>PARAMS['max_slope'] else 'BENT_OR_MIXED' if residual>8 or turn>.9 else None
        item=dict(points=a[:,:2].tolist(),slope=float(fit[0]),residual_px=residual,turn=turn)
        if reason:rejected.append(dict(item,reason=reason));continue
        deviation=float(np.median(abs(a[:,0]-np.polyval(entry,a[:,1]))))
        width_error=float(np.median(abs(a[:,2]-np.polyval(width,a[:,1]))/np.maximum(12,np.polyval(width,a[:,1]))))
        item.update(deviation_px=deviation,width_error=width_error,score=deviation+20*width_error+20*abs(fit[0]-entry[0])-min(15,(a[-1,1]-a[0,1])*.1))
        good.append(item)
    return sorted(good,key=lambda x:x['score'])[:12],rejected

def distance(a,b):
    a=np.array(a);b=np.array(b);lo=max(a[:,1].min(),b[:,1].min());hi=min(a[:,1].max(),b[:,1].max())
    if hi-lo<8:return None
    ys=np.linspace(lo,hi,8);a=a[np.argsort(a[:,1])];b=b[np.argsort(b[:,1])]
    return float(np.median(abs(np.interp(ys,a[:,1],a[:,0])-np.interp(ys,b[:,1],b[:,0]))))

class JunctionTracked:
    def __init__(self):
        self.state='FOLLOW';self.gray=None;self.entry=None;self.width=None;self.track=None;self.track_at=None
        self.suspect_at=None;self.rejoin_at=None;self.lost_near=False;self.track_id=0;self.last_t=None;self.rejoined=False
        self.rejoin_evidence_s=0.;self.last_recovered=None

    def step(self,image,mask,t,baseline):
        if self.last_t is not None and (t<=self.last_t or t-self.last_t>1):self.__init__()
        dt=0 if self.last_t is None else t-self.last_t
        self.last_t=t;r=copy.deepcopy(baseline)
        gray=cv2.cvtColor(cv2.resize(image,(640,360)),cv2.COLOR_BGR2GRAY);H,motion=estimate_motion(self.gray,gray);self.gray=gray
        m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
        if H is not None:
            if self.entry is not None:
                ys=np.array([230.,280.,330.]);a=transform(np.column_stack([np.polyval(self.entry,ys),ys]),H);self.entry=np.polyfit(a[:,1],a[:,0],1)
            if self.track is not None:self.track=transform(self.track,H).tolist()
        elif self.track is not None:
            # Unaligned coordinates are not silently treated as current evidence.
            self.track=None
        samples=r.get('path_samples',[]);near=[n for n in samples if n['source']=='double' and 250<=n['y']<=334]
        nearpts=[[n['x'],n['y']] for n in near]
        good_near=len(near)>=12 and np.ptp([n['y'] for n in near])>=44
        if good_near:
            nf=np.polyfit([n['y'] for n in near],[n['x'] for n in near],1)
            good_near=max(abs(n['x']-np.polyval(nf,n['y'])) for n in near)<8
        # Measure current adjacent boundary spacing independently of baseline width memory.
        widths=[]
        for y in range(250,335,4):
            runs=white_runs(m,y);center=float(np.interp(y,sorted(n['y'] for n in near),[n['x'] for n in sorted(near,key=lambda n:n['y'])])) if near else 320
            enclosing=[b[0]-a[1] for a,b in zip(runs,runs[1:]) if a[1]<center<b[0]]
            if enclosing:widths.append([y,min(enclosing)])
        if self.entry is None and good_near and len(widths)>=8:
            self.entry=nf;self.width=np.polyfit(*np.array(widths).T,1)
        cs=[];reject=[];selected=None;bridge=[];flags={};agreement=None;reason='FOLLOW_BASELINE';prior=self.state
        if self.entry is not None:
            branches=sum(len(white_runs(m,y))>=3 for y in range(170,251,4))>=3
            widened=bool(widths and np.median([w/max(12,np.polyval(self.width,y)) for y,w in widths])>1.35)
            far=[abs(n['x']-np.polyval(self.entry,n['y'])) for n in samples if n['y']<230]
            flags=dict(branches=branches,widened=widened,near_supported=bool(good_near),far_deviation=max(far,default=0))
            # Missing white lines alone cannot assert an intersection.
            evidence=branches and (widened or not good_near or max(far,default=0)>25)
            if self.rejoined:
                if widened or not good_near:self.rejoined=False
                else:evidence=False
            cs,reject=candidates(m,self.entry,self.width)
            if self.state in ('FOLLOW','SUSPECT'):
                if evidence:
                    if self.suspect_at is None:self.suspect_at=t
                    self.state='SUSPECT'
                    if t-self.suspect_at>=PARAMS['confirm_s'] and cs:
                        self.state='CROSS';self.lost_near=not good_near;self.track=None
                else:self.state='FOLLOW';self.suspect_at=None
            if self.state in ('CROSS','REJOIN'):
                self.lost_near|=not good_near
                if self.track_at is not None and t-self.track_at>PARAMS['track_hold_s']:self.track=None
                if self.track is not None:
                    matches=[(distance(c['points'],self.track),i) for i,c in enumerate(cs)]
                    matches=[(v,i) for v,i in matches if v is not None and v<PARAMS['association_px']]
                    if matches:selected=min(matches)[1]
                elif cs and cs[0]['deviation_px']<65:
                    selected=0;self.track_id+=1
                if selected is not None:self.track=cs[selected]['points'];self.track_at=t
                if self.track is not None and good_near:agreement=distance(nearpts,self.track)
                # Recovered current near corridor must agree with the tracked exit;
                # once rejoined, curves are left to baseline rather than old entry heading.
                # Recovery may use current directly observed paired boundaries after
                # motion tracking fails. This is explicit reacquisition, not old pixels.
                local_branches=sum(len(white_runs(m,y))>=3 for y in range(250,331,4))
                paired=np.array([[n['x'],n['y']] for n in samples if n['source']=='double' and 210<=n['y']<=334])
                residual=float(np.max(abs(paired[:,0]-np.polyval(np.polyfit(paired[:,1],paired[:,0],2),paired[:,1])))) if len(paired)>=24 else None
                # Other lanes outside the enclosing pair do not veto recovery.
                paired_near=good_near and len(near)>=18 and len(widths)>=18 and residual is not None and residual<10
                flags.update(near_reacquired=bool(paired_near),local_branch_rows=local_branches)
                recovered=self.lost_near and paired_near and (agreement is None or agreement<20)
                if recovered:
                    if self.rejoin_at is None:self.rejoin_at=t;self.rejoin_evidence_s=0.
                    elif self.last_recovered is not None:self.rejoin_evidence_s+=dt
                    self.last_recovered=t
                    self.state='REJOIN'
                    if self.rejoin_evidence_s>=PARAMS['rejoin_s']:
                        self.state='FOLLOW';self.suspect_at=None;self.track=None;self.rejoin_at=None;self.rejoined=True
                elif self.last_recovered is not None and t-self.last_recovered<=PARAMS['rejoin_gap_s'] and self.rejoin_at is not None:
                    self.state='REJOIN'
                else:self.rejoin_at=None;self.last_recovered=None;self.rejoin_evidence_s=0.;self.state='CROSS'
                if self.state in ('CROSS','REJOIN'):
                    age=t-self.track_at if self.track_at is not None else None
                    if self.track is not None and age<=PARAMS['track_hold_s']:
                        a=np.array(self.track);a=a[np.argsort(a[:,1])]
                        # Full exit shape checked; join to its near end with matching tangent.
                        target=a[-1];y1=float(target[1]);x1=float(target[0]);y0=334.
                        entry=nf if good_near else self.entry;x0=float(np.polyval(entry,y0))
                        slope=float(np.polyfit(a[-min(8,len(a)):,1],a[-min(8,len(a)):,0],1)[0])
                        bridge=a.tolist()
                        for y in np.arange(y1+4,y0+1,4):
                            u=(y-y1)/max(1,y0-y1);L=y0-y1
                            x=(2*u**3-3*u**2+1)*x1+(u**3-2*u**2+u)*L*slope+(-2*u**3+3*u**2)*x0+(u**3-u**2)*L*entry[0]
                            bridge.append([float(x),float(y)])
                        reason='TRACKED_EXIT' if selected is not None else 'ALIGNED_EXIT_HOLD'
                        if self.state=='REJOIN':
                            weight=min(1,(t-self.rejoin_at)/PARAMS['rejoin_s']);bp=np.array(r['path'])/2;bp=bp[np.argsort(bp[:,1])]
                            bridge=[[float(x*(1-weight)+np.interp(y,bp[:,1],bp[:,0])*weight) if bp[0,1]<=y<=bp[-1,1] else x,y] for x,y in bridge];reason='BLEND_TO_ROAD'
                    elif self.state=='REJOIN' and paired_near:
                        bridge=paired[np.argsort(paired[:,1])].tolist();reason='CURRENT_ROAD_REACQUIRED'
                    else:reason='EXIT_UNCONFIRMED';bridge=[]
                    r.update(path=[[x*2,y*2] for x,y in bridge],valid=bool(bridge),quality=.45 if bridge else 0,mode='junction_tracked' if bridge else 'junction_unconfirmed',reason=reason,
                        road_segments=[dict(source='junction_inferred',points=[[x*2,y*2] for x,y in bridge],confidence=.45,age_s=age or 0)] if bridge else [],history_segments=[],history_path=[],history_points=0,
                        coverage=dict(visible_rows=len(bridge),double_rows=0,components=1 if bridge else 0,visible_span_px=float(np.ptp(np.array(bridge)[:,1])*2) if bridge else 0,control_span_px=float(np.ptp(np.array(bridge)[:,1])*2) if bridge else 0))
            if self.state in ('FOLLOW','SUSPECT') and good_near and len(widths)>=8:
                self.entry=.8*self.entry+.2*nf;self.width=.8*self.width+.2*np.polyfit(*np.array(widths).T,1)
            if self.state=='SUSPECT':reason='SUSPECT_BASELINE'
        r['junction']=dict(intent='straight',active=self.state in ('CROSS','REJOIN'),state=self.state,reason=reason,entry_direction=[[float(np.polyval(self.entry,y)),y] for y in [334,280,230,180]] if self.entry is not None else [],candidates=cs,selected=selected,inferred_path=bridge,rejected=reject,track_id=self.track_id,track_age_s=t-self.track_at if self.track_at is not None else None,motion=motion,evidence=flags,rejoin_agreement_px=agreement,transition=prior+' → '+self.state if prior!=self.state else None,parameters=PARAMS)
        return r
