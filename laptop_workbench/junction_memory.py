"""Offline junction v4. Pixel heuristics; no car, metric trajectory or safety claim.

Consumes frozen v3.1 evidence, never human labels. All timers use capture time.
"""
import copy
import cv2
import numpy as np
from junction_straight import white_runs
from temporal_lane import estimate_motion, transform

PARAMS=dict(confirm_s=.30,rejoin_s=.60,rejoin_gap_s=.30,track_hold_s=.45,memory_hold_s=1.2,exit_confirm_frames=3,rejoin_confirm_s=.6,association_px=35,
            width_ratio=(.45,1.65),max_slope=1.1,max_residual_px=7,width_growth_ratio=(.25,2.8),min_chain_points=4,max_turn=.9)

def candidates(mask,entry,width):
    """Track both inner edges; reject opening pairs by perspective width support."""
    chains=[];rejected=[];counts=dict(scan_rows=0,no_pair_rows=0,width_rejected=0,short_chains=0)
    for y in range(132,311,2):
        runs=white_runs(mask,y);expected=max(6,float(np.polyval(width,y)));options=[];counts['scan_rows']+=1
        for l,r in zip(runs,runs[1:]):
            left=l[1];right=r[0];w=right-left
            if not 6<w or not .45<w/expected<1.65:
                counts['width_rejected']+=1;continue
            options.append((float((left+right)/2),float(y),float(left),float(right)))
        if not options:counts['no_pair_rows']+=1
        used=set()
        for point in options:
            matches=[]
            for i,c in enumerate(chains):
                dy=y-c[-1][1]
                if i in used or not 0<dy<=6:continue
                slope=(np.array(c[-1][2:])-np.array(c[-2][2:]))/(c[-1][1]-c[-2][1]) if len(c)>1 else np.zeros(2)
                err=max(abs(np.array(point[2:])-(np.array(c[-1][2:])+slope*dy)))
                if err<8 and max(abs(np.array(point[2:])-np.array(c[-1][2:])))/dy<3:matches.append((err,i))
            if matches:
                _,i=min(matches);chains[i].append(point);used.add(i)
            else:chains.append([point]);used.add(len(chains)-1)
    good=[]
    for chain in chains:
        if len(chain)<4:counts['short_chains']+=1;continue
        a=np.array(chain);lf=np.polyfit(a[:,1],a[:,2],1);rf=np.polyfit(a[:,1],a[:,3],1);fit=(lf+rf)/2
        residual=float(max(np.max(abs(a[:,2]-np.polyval(lf,a[:,1]))),np.max(abs(a[:,3]-np.polyval(rf,a[:,1])))))
        growth=float(rf[0]-lf[0]);expected_growth=max(.15,float(width[0]))
        ratio=float(np.median((a[:,3]-a[:,2])/np.maximum(6,np.polyval(width,a[:,1]))))
        reason='NEAR_ONLY_NOT_EXIT' if a[0,1]>230 else 'OPENING_NOT_EXIT' if not .25<growth/expected_growth<2.8 else 'BENT_BOUNDARY' if residual>7 else 'SIDEWAYS' if abs(fit[0]-entry[0])>1.1 else None
        item=dict(points=a[:,:2].tolist(),left_edge=a[:,[2,1]].tolist(),right_edge=a[:,[3,1]].tolist(),slope=float(fit[0]),residual_px=residual,width_growth=growth,width_ratio=ratio)
        if reason:rejected.append(dict(item,reason=reason));continue
        deviation=float(np.median(abs(a[:,0]-np.polyval(entry,a[:,1]))))
        item.update(deviation_px=deviation,width_error=abs(1-ratio),score=deviation+30*abs(1-ratio)+20*abs(fit[0]-entry[0])-min(12,(a[-1,1]-a[0,1])*.15))
        good.append(item)
    rejected.append(dict(reason='GENERATION_COUNTS',**counts))
    return sorted(good,key=lambda c:c['score'])[:12],rejected

def distance(a,b):
    a=np.array(a);b=np.array(b);lo=max(a[:,1].min(),b[:,1].min());hi=min(a[:,1].max(),b[:,1].max())
    if hi-lo<8:return None
    ys=np.linspace(lo,hi,8);a=a[np.argsort(a[:,1])];b=b[np.argsort(b[:,1])]
    return float(np.median(abs(np.interp(ys,a[:,1],a[:,0])-np.interp(ys,b[:,1],b[:,0]))))

class JunctionMemory:
    def __init__(self):
        self.state='FOLLOW';self.gray=None;self.entry=None;self.width=None;self.track=None;self.track_at=None
        self.suspect_at=None;self.rejoin_at=None;self.lost_near=False;self.track_id=0;self.last_t=None;self.rejoined=False
        self.pending_exit=None;self.pending_count=0;self.memory_at=None;self.memory_aligned=False;self.rejoin_evidence_s=0.;self.last_recovered=None;self.reference_gray=None;self.reference_entry=None;self.reference_width=None;self.reference_at=None

    def step(self,image,mask,t,baseline):
        if self.last_t is not None and (t<=self.last_t or t-self.last_t>1):self.__init__()
        dt=0 if self.last_t is None else t-self.last_t
        self.last_t=t;r=copy.deepcopy(baseline)
        gray=cv2.cvtColor(cv2.resize(image,(640,360)),cv2.COLOR_BGR2GRAY);H,motion=estimate_motion(self.gray,gray);self.gray=gray
        m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
        reference_motion={'valid':False,'reason':'NO_REFERENCE'}
        self.memory_aligned=False
        if H is not None and self.pending_exit is not None:self.pending_exit=transform(self.pending_exit,H).tolist()
        elif H is None:self.pending_exit=None;self.pending_count=0
        if self.reference_entry is not None:
            RH,reference_motion=estimate_motion(self.reference_gray,gray)
            if RH is None and H is not None and self.entry is not None:
                RH=H;reference_motion=dict(valid=True,reason='ADJACENT_MEMORY_FALLBACK')
                ref_entry=self.entry.copy();ref_width=self.width.copy()
            else:ref_entry=self.reference_entry;ref_width=self.reference_width
            if RH is not None:
                self.memory_aligned=True
                ys=np.array([220.,270.,330.]);xs=np.polyval(ref_entry,ys);ws=np.maximum(12,np.polyval(ref_width,ys))
                left=transform(np.column_stack([xs-ws/2,ys]),RH);right=transform(np.column_stack([xs+ws/2,ys]),RH)
                lf=np.polyfit(left[:,1],left[:,0],1);rf=np.polyfit(right[:,1],right[:,0],1)
                self.entry=(lf+rf)/2;self.width=rf-lf
            else:
                self.entry=self.reference_entry.copy();self.width=self.reference_width.copy()
        if H is not None and self.track is not None:self.track=transform(self.track,H).tolist()
        elif self.track is not None:self.track=None
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
        reference_updated=False
        if good_near and len(widths)>=12 and self.width is not None:
            measured=np.polyfit(*np.array(widths).T,1)
            ratios=[w/max(12,np.polyval(self.width,y)) for y,w in widths]
            if .75<np.median(ratios)<1.25 and measured[0]>.15:
                self.entry=.75*self.entry+.25*nf;self.width=.7*self.width+.3*measured
                self.memory_at=t;self.memory_aligned=True
                self.reference_gray=gray.copy();self.reference_entry=self.entry.copy();self.reference_width=self.width.copy();self.reference_at=t;reference_updated=True
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
                if selected is None and cs and cs[0]['deviation_px']<65:
                    delta=distance(cs[0]['points'],self.pending_exit) if self.pending_exit is not None else None
                    self.pending_count=self.pending_count+1 if delta is not None and delta<20 else 1
                    self.pending_exit=cs[0]['points']
                    if self.pending_count>=PARAMS['exit_confirm_frames']:
                        selected=0;self.track_id+=1
                elif selected is None:self.pending_count=0;self.pending_exit=None
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
                    self.state='REJOIN' if self.rejoin_evidence_s>=PARAMS['rejoin_confirm_s'] else 'CROSS'
                    if self.rejoin_evidence_s>=PARAMS['rejoin_confirm_s']+PARAMS['rejoin_s']:
                        self.state='FOLLOW';self.suspect_at=None;self.track=None;self.rejoin_at=None;self.rejoined=True
                elif self.last_recovered is not None and t-self.last_recovered<=PARAMS['rejoin_gap_s'] and self.rejoin_at is not None:
                    self.state='REJOIN' if self.rejoin_evidence_s>=PARAMS['rejoin_confirm_s'] else 'CROSS'
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
                            weight=min(1,max(0,self.rejoin_evidence_s-PARAMS['rejoin_confirm_s'])/PARAMS['rejoin_s']);bp=np.array(r['path'])/2;bp=bp[np.argsort(bp[:,1])]
                            bridge=[[float(x*(1-weight)+np.interp(y,bp[:,1],bp[:,0])*weight) if bp[0,1]<=y<=bp[-1,1] else x,y] for x,y in bridge];reason='BLEND_TO_ROAD'
                    elif self.state=='REJOIN' and paired_near:
                        bridge=paired[np.argsort(paired[:,1])].tolist();reason='CURRENT_ROAD_REACQUIRED'
                    elif self.memory_aligned and self.memory_at is not None and t-self.memory_at<=PARAMS['memory_hold_s']:
                        bridge=[[float(np.polyval(self.entry,y)),float(y)] for y in range(190,335,4)]
                        reason='STRAIGHT_MEMORY_PREDICTION'
                    else:reason='NO_EXIT_CANDIDATE' if not cs else 'EXIT_ASSOCIATION_FAILED';bridge=[]
                    r.update(path=[[x*2,y*2] for x,y in bridge],valid=bool(bridge),quality=.45 if bridge else 0,mode='junction_tracked' if bridge else 'junction_unconfirmed',reason=reason,
                        road_segments=[dict(source='junction_inferred',points=[[x*2,y*2] for x,y in bridge],confidence=.45,age_s=age or 0)] if bridge else [],history_segments=[],history_path=[],history_points=0,
                        coverage=dict(visible_rows=len(bridge),double_rows=0,components=1 if bridge else 0,visible_span_px=float(np.ptp(np.array(bridge)[:,1])*2) if bridge else 0,control_span_px=float(np.ptp(np.array(bridge)[:,1])*2) if bridge else 0))
            if self.state in ('FOLLOW','SUSPECT') and good_near and len(widths)>=8:
                self.entry=.8*self.entry+.2*nf;self.width=.8*self.width+.2*np.polyfit(*np.array(widths).T,1)
                self.memory_at=t
                self.reference_gray=gray.copy();self.reference_entry=self.entry.copy();self.reference_width=self.width.copy();self.reference_at=t
            if self.state=='SUSPECT':reason='SUSPECT_BASELINE'
        r['junction']=dict(intent='straight',active=self.state in ('CROSS','REJOIN'),state=self.state,reason=reason,entry_direction=[[float(np.polyval(self.entry,y)),y] for y in [334,280,230,180]] if self.entry is not None else [],candidates=cs,selected=selected,inferred_path=bridge,rejected=reject,track_id=self.track_id,track_age_s=t-self.track_at if self.track_at is not None else None,motion=motion,reference_motion=reference_motion,reference_updated=reference_updated,reference_age_s=t-self.reference_at if self.reference_at is not None else None,width_model=self.width.tolist() if self.width is not None else None,evidence=flags,rejoin_agreement_px=agreement,transition=prior+' → '+self.state if prior!=self.state else None,parameters=PARAMS,memory_age_s=t-self.memory_at if self.memory_at is not None else None,memory_aligned=self.memory_aligned,exit_confirmation_count=self.pending_count,rejoin_evidence_s=self.rejoin_evidence_s)
        return r
