"""Estimate steering play (backlash width h), straight point and gain from lap logs: curvature (-gz/v) vs the SENT command
passed through a backlash model, on flat moving frames. usage: python play_now.py run-xxxx-2026.... ..."""
import json,sys,numpy as np
ENC=2.22e-4
def load(run):
    T=[];U=[];G=[];E=[];P=[]
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        if '"kind": "sample"' not in l[:40]:continue
        p=json.loads(l).get('proposal') or {}
        if 't' not in p:continue
        i=p.get('inputs') or {};g=i.get('gz')
        if g is None:g=(p.get('gyro_damp') or {}).get('gz')
        e=i.get('enc_b')
        if g is None or p.get('steer_us') is None:continue
        T.append(p['t']);U.append((p.get('play_comp') or {}).get('steer_us',p['steer_us']));G.append(g);E.append(e if e is not None else np.nan);P.append((p.get('imu_boost') or {}).get('pitch_rel_deg') or 0.)
    return map(np.array,(T,U,G,E,P))
def wheel(U,h):
    w=np.empty_like(U);x=U[0]
    for i,u in enumerate(U):
        x=min(max(x,u-h/2),u+h/2);w[i]=x
    return w
for run in sys.argv[1:]:
    T,U,G,E,P=load(run)
    if len(T)<500:print(run,'too short');continue
    t=np.arange(T[0],T[-1],0.02)
    u=U[np.clip(np.searchsorted(T,t,side='right')-1,0,len(U)-1)];g=np.interp(t,T,G);p=np.interp(t,T,P)
    ok=~np.isnan(E);te=T[ok];ee=E[ok]*ENC
    if len(te)<50:v=np.full_like(t,np.nan)
    else:
        s=np.interp(t,te,ee);v=np.gradient(s,0.02);k=np.ones(25)/25;v=np.convolve(v,k,'same')
    if np.isnan(v).all():print(run,'no encoder');continue
    kap=-g/np.maximum(v,0.3)
    flat=(v>0.6)&(np.abs(p)<3)
    best=None;tab={}
    for lag_s in (0.10,0.16,0.20,0.26,0.32):
        n=int(lag_s/0.02)
        for h in (0,60,100,130,155,180,210,250,300):
            x=wheel(u,h)[:-n];y=kap[n:];m=flat[n:]&flat[:-n]
            A=np.c_[x[m],np.ones(m.sum())];c,_,_,_=np.linalg.lstsq(A,y[m],rcond=None);r=y[m]-A@c;r2=1-r.var()/y[m].var()
            tab[(lag_s,h)]=(r2,c[0],-c[1]/c[0])
            if best is None or r2>best[0]:best=(r2,lag_s,h,c[0],-c[1]/c[0])
    L=best[1]
    # reversal stats of the sent command on flat frames
    du=np.diff(u[flat]);sg=np.sign(du[du!=0]);rev=(np.diff(sg)!=0).sum()/(flat.sum()*0.02)
    print('%-22s flat %.0fs v %.2f | best h %3d lag %.2fs R2 %.3f gain %.3f deg/m per us centre %4.0f | R2 at h=0/100/155/210/300: %s | sent reversals %.1f/s'%(run,flat.sum()*0.02,np.median(v[flat]),best[2],L,best[0],best[3],best[4],' '.join('%.3f'%tab[(L,h)][0] for h in (0,100,155,210,300)),rev))
