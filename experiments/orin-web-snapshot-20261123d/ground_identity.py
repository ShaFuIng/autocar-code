"""Single-line identity r3 (ground-identity-r1). Deployed with a json switch (ground_identity.json, enabled false = old behaviour).
Matches each current white-line track to the previous frame's L/R line in GROUND coordinates
(camera-ground-20261109-r2 homography, cm), after moving the previous line by the car's own motion
(speed*dt forward, gz*dt yaw; gz and pitch from the IMU logger latest.json). A lane edge moves a few cm
between frames; the other edge is 46 cm away; a crossing line has a different heading. Pixel matching
(<=30 px) cannot tell these apart near the car. Thresholds are experiment values, not validated limits.
Flat ground only: |pitch| > pitch_max, no previous frame or no gyro -> the existing rule.
Valid only for the camera pose of the calibration; check with check_camera.py after any camera change.
"""
import json,math,time,numpy as np
from pathlib import Path
from offscreen_center import OffscreenEstimate
from standalone_line_path import LineEstimate
SLOPE_REVISION='slope-width-r1'   # slope-20261115
try:
    import crossing_source as CS   # crossing-source-r2 (missing module -> feature off)
except Exception:
    CS=None
REVISION='ground-identity-r1'
HERE=Path(__file__).resolve().parent
CAL=json.loads((HERE/'camera_ground_calibration.json').read_text(encoding='utf8'))
H=np.array(CAL['homography_image_to_ground_cm'])
DEFAULTS=dict(enabled=False,match_cm=8.0,heading_deg=25.0,z_min_cm=30.,z_max_cm=140.,speed_mps=0.0,pitch_max_deg=4.0,dy_px=0.0,
              imu_path='/home/jetson/orin_interface_v1/logs/imu/latest.json',imu_max_age_s=0.3,flat_pitch_deg=2.6)
def load_config():
    c=dict(DEFAULTS)
    try:c.update(json.loads((HERE/'ground_identity.json').read_text(encoding='utf8')))
    except (OSError,ValueError):pass
    cs=dict(CS.DEFAULTS) if CS else dict(enabled=False)   # crossing-source-r2: read with ground_identity.json
    try:cs.update(json.loads((HERE/'crossing_source.json').read_text(encoding='utf8')))
    except (OSError,ValueError):pass
    if not CS:cs['enabled']=False
    c['crossing_source']=cs
    return c
MATCH_CM=8.0;HEADING_DEG=25.0;Z_RANGE=(30.,140.);YAW_SIGN=1.0

def ground(pts,dy):
    if not pts:return np.zeros((0,2))
    ys=np.array(sorted(pts),float);xs=np.array([pts[int(y)] for y in ys],float)
    p=np.stack([xs,ys+dy,np.ones_like(xs)],1)@H.T;g=p[:,:2]/p[:,2:3]
    return g[(g[:,1]>=Z_RANGE[0])&(g[:,1]<=Z_RANGE[1])]

def move(g,d_cm,yaw_rad):
    # previous-frame points into the current car frame: forward travel then yaw
    q=g-np.array([0.,d_cm]);c,s=math.cos(yaw_rad),math.sin(yaw_rad)
    return np.stack([c*q[:,0]+s*q[:,1],-s*q[:,0]+c*q[:,1]],1)

def poly_dist(a,b):
    # median distance of points a to polyline b (vectorised)
    if len(a)<4 or len(b)<2:return None
    u=b[:-1][None];w=(b[1:]-b[:-1])[None];L=np.maximum((w*w).sum(-1),1e-12)
    t=np.clip(((a[:,None]-u)*w).sum(-1)/L,0.,1.)
    d=np.hypot(*np.moveaxis(u+t[...,None]*w-a[:,None],-1,0)).min(1)
    return float(np.median(d))

def heading(g):
    if len(g)<4 or np.ptp(g[:,1])<10:return None
    return math.degrees(math.atan(np.polyfit(g[:,1],g[:,0],1)[0]))

class GroundIdentityEstimate(OffscreenEstimate):
    dy=0.;motion=None;pitch=None;yaw_sign=YAW_SIGN;match_cm=MATCH_CM;heading_deg=HEADING_DEG;pitch_max=4.0
    amb_one_side=False;one_side_n=0   # crossings-worker-r1 (set by JunctionLine from crossings_worker.json; False = unchanged)
    def configure(self,c):
        global Z_RANGE
        self.cs=c.get('crossing_source') or dict(enabled=False);self._cs_ref=None   # crossing-source-r2
        self.c=c;self.dy=float(c['dy_px']);self.match_cm=float(c['match_cm']);self.heading_deg=float(c['heading_deg']);self.pitch_max=float(c['pitch_max_deg']);Z_RANGE=(float(c['z_min_cm']),float(c['z_max_cm']))
        return self
    def read_imu(self):
        # worker-side: gz / pitch straight from the IMU logger (same file live_pid reads); stale or missing -> no ground matching this frame
        c=self.c;self.motion=None;self.pitch=None;self.imu_note='NO_IMU'
        try:
            d=json.loads(Path(c['imu_path']).read_text(encoding='utf8'));t=float(d['t']);g=float(d['gyro_dps'][2]);p=float(d['pitch_deg'])
            if not(math.isfinite(t) and math.isfinite(g)) or not -.05<=time.monotonic()-t<=float(c['imu_max_age_s']):self.imu_note='STALE_IMU';return
            self.motion=(float(c['speed_mps']),g);self.pitch=p-float(c['flat_pitch_deg']);self.imu_note='OK'
        except Exception:pass
    def step(self,mask,t):
        self._dt=t-self.t if self.t is not None else None;self._assign=None;self.ground_log=dict(used=False)
        self.cs_log=None
        r=super().step(mask,t);self.ground_log.update(revision=REVISION,imu=getattr(self,'imu_note',None));r['ground_identity']=self.ground_log
        if self.cs_log is not None:r['crossing_source']=self.cs_log   # crossing-source-r2: only on zone frames with the switch on
        return r

    _gi_step=step
    slope_width=None;_sw_since=None;_sw_k=None;_sw_kt=None   # slope-width-r1 (set by JunctionLine from crossings_worker.json; None = unchanged)
    def step(self,mask,t):
        if self.slope_width is None:return self._gi_step(mask,t)
        # slope-width-r1 (slope-20261115): on the slope the two-line lane width at the control rows is ~0.85 x the estimator width
        # (same row profile as on the flat upper section), but the rate-limited width memory (0.3 /s) enters the slope with whatever
        # ratio the upper section left (0.85-1.40, median 1.05), so every double <-> single switch stepped the centre. While steep,
        # the single-line ratio starts at the slope prior and follows the ratios observed on this slope at rate_per_s; when the
        # slope ends the memory continues from that value (normal 0.3 /s limiter), so there is no step at the landing either.
        c=self.slope_width;thr=float(c.get('pitch_deg',-8.));rate=float(c.get('rate_per_s',1.))
        steep=getattr(self,'imu_note',None)=='OK' and self.pitch is not None and self.pitch<=thr
        log=dict(revision=SLOPE_REVISION,applied=False,pitch=None if self.pitch is None else round(float(self.pitch),1))
        reset=self.t is not None and (t<=self.t or t-self.t>1)
        if not steep or reset:
            if self._sw_since is not None and not reset and self._sw_k is not None:   # hand over: memory continues from the slope value
                self.k_used,self.k_t=self._sw_k,self._sw_kt
            self._sw_since=self._sw_k=self._sw_kt=None
            r=self._gi_step(mask,t);log['reason']='RESET' if steep else 'NOT_STEEP';r['slope_width']=log;return r
        if self._sw_since is None:self._sw_since=t;self._sw_k=float(c.get('ratio',.85));self._sw_kt=t
        obs=self.ratio if (self.ratio is not None and self.ratio_t is not None and self.ratio_t>=self._sw_since) else None
        if obs is not None:
            lim=rate*max(0.,min(.5,t-self._sw_kt));self._sw_k+=max(-lim,min(lim,float(obs)-self._sw_k))
        self._sw_kt=t;ks=self._sw_k
        self.k_used,self.k_t=ks,t
        r=self._gi_step(mask,t)
        log.update(applied=True,k=round(ks,3),k_source='slope_observation' if obs is not None else 'slope_prior')
        r['slope_width']=log
        return r

    def _pair(self,trs):
        l,r=self._pair_r2(trs)
        if self.amb_one_side and l is None and r is None and self.pair_diagnostic.get('reason')=='AMBIGUOUS_CURRENT_PAIRS':l,r=self._one_side(trs,l,r)
        return l,r

    def _one_side(self,trs,l,r):
        # crossings-worker-r1 (crossings-20261114, ac4f 182.10 / 264.37 s): several current pairs pass the shape gate (two left
        # tracks, e.g. the lane edge and a circle marking, each pairing with one far right track) while the previous frame had
        # only one side, so the two-side identity test cannot pass and the frame had no path (-> junction fallback corridor).
        # When exactly one side has a previous-frame reference and one current track of that side continues it (the same
        # 4-row / 24-row / 30 px test as the pair identity check), that side alone is used, i.e. the estimate continues as before.
        ref={s:self.identity_reference.get(s) for s in ('L','R')};have=[s for s in ('L','R') if ref[s]]
        if len(have)!=1:return l,r
        s=have[0];best=None
        for tr in trs:
            if tr['side']!=s:continue
            ys=sorted(set(ref[s])&set(tr['pts']))
            if len(ys)<4 or ys[-1]-ys[0]<24:continue
            d=float(np.median([abs(ref[s][y]-tr['pts'][y]) for y in ys]))
            if d<=30 and (best is None or d<best[0]):best=(d,tr)
        if best is None:return l,r
        self.one_side_n+=1;self.pair_diagnostic.update(reason='AMBIGUOUS_ONE_SIDE_HISTORY',one_side=dict(side=s,dx=round(best[0],1)))
        return (best[1],None) if s=='L' else (None,best[1])

    def _pair_r2(self,trs):
        cs=getattr(self,'cs',None)
        if not cs or not cs.get('enabled'):return super()._pair(trs)
        return self._cs_pair(trs,cs)

    def _cs_pair(self,trs,cs):
        # crossing-source-r2: ground kink cut + lane-direction continuity on the tracks before pairing, ONLY on frames whose
        # worker request carried zone in cs['zones'] (live_pid: circle-crossing stretch after curve 2). Elsewhere the
        # output is unchanged; the lane-direction reference is kept warm from the accepted edges (tracks only read).
        zr=(float(cs['z_min_cm']),float(cs['z_max_cm']));span=float(cs['min_span_cm'])
        inzone=getattr(self,'cs_zone',None) in tuple(cs.get('zones') or ())
        gate=getattr(self,'imu_note',None)=='OK' and self.motion is not None and not(self.straight_started and not self.straight_finished) and not(self.pitch is not None and abs(self.pitch)>self.pitch_max)
        dt=self._dt if self._dt is not None and 0<self._dt<=1. else None
        ref=self._cs_ref
        if ref is not None and (not gate or dt is None or self.t-ref['t']>float(cs['ref_max_s'])):ref=None
        if ref is not None:
            yaw=math.radians(self.motion[1]*dt)*self.yaw_sign
            ref['g']=[move(g,0.,yaw) for g in ref['g']]
        self._cs_ref=ref
        if not gate:
            if inzone:self.cs_log=dict(revision=CS.REVISION,zone=self.cs_zone,gate='NO_IMU' if getattr(self,'imu_note',None)!='OK' or self.motion is None else 'STRAIGHT' if self.straight_started and not self.straight_finished else 'PITCH')
            return super()._pair(trs)
        if not inzone:
            l,r=super()._pair(trs)
            gs=[]
            for x in (l,r):
                if x is None:continue
                rows,g=CS.track_ground(H,x['pts'],self.dy,zr)
                if cs.get('kink_rule',True):
                    k=CS.kink_index(g,float(cs['kink_deg']),float(cs['base_cm']),float(cs['seg_cm']))
                    if k is not None:g=g[:k]
                if CS.track_heading(g,span) is not None:gs.append(g)
            if gs:self._cs_ref=dict(t=self.t,g=gs)
            return l,r
        log=dict(revision=CS.REVISION,zone=self.cs_zone,gate='OK',cut=[],rejected=[]);self.cs_log=log
        hs=[h for h in (CS.track_heading(g,span) for g in (ref['g'] if ref else [])) if h is not None]
        href=float(np.mean(hs)) if hs else None
        dir_on=bool(cs.get('dir_rule',True)) and href is not None and abs(self.motion[1])<=float(cs['gz_max_dps'])
        log.update(ref_deg=None if href is None else round(href,1),dir_on=dir_on)
        keep=[];geo={}
        for i,tr in enumerate(trs):
            if tr['side'] not in ('L','R'):keep.append(tr);continue
            rows,g=CS.track_ground(H,tr['pts'],self.dy,zr)
            if cs.get('kink_rule',True):
                k=CS.kink_index(g,float(cs['kink_deg']),float(cs['base_cm']),float(cs['seg_cm']))
                if k is not None:
                    lim=int(rows[k-1]);kept={y:x for y,x in tr['pts'].items() if y>=lim}
                    log['cut'].append(dict(i=i,side=tr['side'],row=lim,z_cm=round(float(g[k-1,1]),1),dropped=len(tr['pts'])-len(kept)))
                    if len(kept)<4:log['rejected'].append(dict(i=i,side=tr['side'],why='KINK_SHORT'));continue
                    tr['pts']=kept;tr['ymin']=min(kept);tr['ymax']=max(kept);g=g[:k]
            if dir_on:
                h=CS.track_heading(g,span)
                if h is not None and CS.adiff(h,href)>float(cs['dir_deg']):
                    log['rejected'].append(dict(i=i,side=tr['side'],why='DIRECTION',deg=round(h,1)));continue
            geo[id(tr)]=g;keep.append(tr)
        l,r=super()._pair(keep)
        gs=[geo[id(x)] for x in (l,r) if x is not None and id(x) in geo and CS.track_heading(geo[id(x)],span) is not None]
        # in the zone the reference is renewed only from edges that passed the direction check (or when there is none):
        # unchecked edges while |gz| > gz_max must not re-seed it (68f9 196.8 s: an oblique line re-seeded it at -19 deg)
        if gs and (dir_on or ref is None):self._cs_ref=dict(t=self.t,g=gs)
        log['kept_ref']=bool(gs) and (dir_on or ref is None)
        return l,r

    def _plan(self,trs):
        self._assign={}
        if self.straight_started or not self.identity_reference or self.motion is None or self._dt is None:return
        if self.pitch is not None and abs(self.pitch)>self.pitch_max:return
        v,gz=self.motion;d=v*100*self._dt;yaw=math.radians(gz*self._dt)*self.yaw_sign
        refs={s:move(ground(p,self.dy),d,yaw) for s,p in self.identity_reference.items() if p}
        cand=[]
        for tr in trs:
            g=ground(tr['pts'],self.dy);h=heading(g)
            for s,rg in refs.items():
                dist=poly_dist(g,rg);hr=heading(rg)
                if dist is None:continue
                if h is not None and hr is not None and abs(h-hr)>self.heading_deg:continue
                if dist<=self.match_cm:cand.append((dist,s,id(tr),tr))
        cand.sort(key=lambda c:c[0]);used=set()
        for dist,s,i,tr in cand:
            if s in self._assign or i in used:continue
            self._assign[s]=(tr,dist);used.add(i)
        self.ground_log=dict(used=True,d_cm=round(d,1),yaw_deg=round(math.degrees(yaw),1),assign={s:round(x[1],1) for s,x in self._assign.items()})

    def _pick(self,trs,side):
        if self._assign is None:self._plan(trs)
        if not self.ground_log.get('used'):return super()._pick(trs,side)
        if side in self._assign:
            tr,dist=self._assign[side]
            if id(tr) not in self.identity_used:
                self.identity_used.add(id(tr))
                if tr['side']!=side:self.identity_repairs.append(dict(from_side=tr['side'],to_side=side,ground_cm=dist))
                tr['side']=side;return tr
        # no ground match for this side: never steal the other side's matched line
        other='R' if side=='L' else 'L'
        if other in self._assign:self.identity_used.add(id(self._assign[other][0]))
        tr=LineEstimate._pick(self,[x for x in trs if id(x) not in self.identity_used],side)
        if other in self._assign:self.identity_used.discard(id(self._assign[other][0]))
        if tr:self.identity_used.add(id(tr))
        return tr
