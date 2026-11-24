import json,sys,collections
from rp import *
MPC=BASE['m_per_count']
def vser(fr):
    # speed from encoder steps: pairs of frames where enc changes
    pts=[];last=None
    for i,f in enumerate(fr):
        if f['enc'] is None: continue
        if last is None or f['enc']!=last[1]: pts.append((f['t'],f['enc'],i)); last=(f['t'],f['enc'])
    v=[None]*len(fr); k=0
    for j in range(1,len(pts)):
        t0,e0,i0=pts[j-1]; t1,e1,i1=pts[j]
        vv=(e1-e0)*MPC/max(1e-3,t1-t0)
        nxt=pts[j+1][2] if j+1<len(pts) else len(fr)
        for i in range(i1,nxt): v[i]=vv
    return v
RUNS=sys.argv[1:]
rows=[]
for r in RUNS:
    fr=load(r); lp,out=run(fr,BASE); v=vser(fr)
    c1=[e['d_cross'] for e in lp.events if e['name']=='C1s']
    c4e=[e['d_cross'] for e in lp.events if e['name']=='C4e']
    for k,dc in enumerate(c1):
        lap0 = (k==0)
        prev=[x for x in c4e if x<dc and x>dc-15]
        lo = 0.0 if lap0 else (prev[-1]+0.0 if prev else dc-10.7)
        fc_first=None; band=None; ca=[]; farhi=[]
        for i,(f,o) in enumerate(zip(fr,out)):
            d=o['odo_m']
            if not (lo<=d<=dc+0.3): continue
            x=d-dc
            if f['rsn'] in ('ERROR_LARGE','CURVE_AHEAD') and (f['err'] or 0)>0: ca.append(round(x,2))
            if f['valid'] and f['far'] is not None and f['far']>=0.4 and f['near'] is not None and f['near']>=-0.05: farhi.append(round(x,2))
            if f['lfc']=='c1' and band is None: band=round(x,2)
        # entry speed: v at C1s (first frame odo>=dc) and at -1.5
        def vat(xx):
            for i,o in enumerate(out):
                if o['odo_m']>=dc+xx: return v[i]
        def pwmat(xx):
            for i,o in enumerate(out):
                if o['odo_m']>=dc+xx: return fr[i]['pwm']
        rows.append(dict(run=r,lap=k+1,lap0=lap0,lo=round(lo-dc,2),ca_first=ca[0] if ca else None,ca_n=len(ca),ca_early=[x for x in ca if x<-1.0][:8],
            far_first=farhi[0] if farhi else None,far_early=[x for x in farhi if x<-1.0][:8],band=band,
            v_m2=vat(-2.0),v_m1=vat(-1.0),v0=vat(0.0),v05=vat(0.5),pwm_m15=pwmat(-1.5),pwm_m05=pwmat(-0.5)))
for x in rows:
    print(json.dumps({k:(round(v,2) if isinstance(v,float) else v) for k,v in x.items()}))
