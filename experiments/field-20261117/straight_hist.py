"""Straight-line weave across runs: per run, main straight (first 3 m after C4e vs later part), diagonal, circle straight."""
import json,sys,glob,os,numpy as np,collections
ENC=2.22e-4
def load(r):
    Q=[]
    for l in open(r+'/pid.jsonl',encoding='utf8'):
        if '"kind": "sample"' not in l[:40]:continue
        p=json.loads(l)['proposal']
        if 't' in p:Q.append(p)
    return Q
runs=sys.argv[1:]
print('run   fps | main straight 0-3m after C4e: gz_sd  | main straight >3m: gz_sd heading_p2p(deg) v pwm rev/s e|med| | diagonal(C1e+1..5m): gz_sd v | circle(C2e+0.5..2.6m): gz_sd v')
for r in runs:
    try:Q=load(r)
    except Exception as e:print(r,e);continue
    if len(Q)<300:continue
    T=np.array([q['t'] for q in Q]);hz=len(Q)/(T[-1]-T[0])
    pts=[]
    for q in Q:
        e=(q.get('inputs') or {}).get('enc_b')
        if e is not None and (not pts or e!=pts[-1][1]):pts.append((q['t'],e))
    def v(t0,t1):
        a=[p for p in pts if t0<=p[0]<=t1];return (a[-1][1]-a[0][1])*ENC/(a[-1][0]-a[0][0]) if len(a)>1 and a[-1][0]>a[0][0] else np.nan
    B=collections.defaultdict(list)
    for q in Q:
        lp=q.get('lap_pos') or {};a=lp.get('anchor');x=lp.get('since_anchor_m') or 0;lap=lp.get('lap')
        if a=='C4e' and x<3:B[('m0',lap)].append(q)
        elif a=='C4e' and 3<=x<=8:B[('m1',lap)].append(q)
        elif a=='C1e' and 1<=x<=5:B[('d',lap)].append(q)
        elif a=='C2e' and 0.5<=x<=2.6:B[('c',lap)].append(q)
    def st(key):
        out=[]
        for (k,lap),X in B.items():
            if k!=key or len(X)<8:continue
            gz=np.array([(x.get('inputs') or {}).get('gz') or 0 for x in X],float);t=np.array([x['t'] for x in X]);e=np.array([x.get('image_error') or 0 for x in X])
            u=np.array([x['steer_us'] for x in X]);du=np.diff(u);sg=np.sign(du[du!=0]);rev=(np.diff(sg)!=0).sum()/max(t[-1]-t[0],1e-3)
            h=np.concatenate([[0],np.cumsum((gz[1:]+gz[:-1])/2*np.diff(t))]);h=h-np.polyval(np.polyfit(t,h,1),t)
            out.append((gz.std(),np.ptp(h),v(t[0],t[-1]),np.mean([x.get('pwm') or 0 for x in X]),rev,np.median(np.abs(e))))
        return np.array(out) if out else None
    m0=st('m0');m1=st('m1');d=st('d');c=st('c')
    f=lambda A,i:('%5.1f'%np.median(A[:,i]) if A is not None else '   - ')
    rng=lambda A,i:('(%.0f-%.0f)'%(A[:,i].min(),A[:,i].max()) if A is not None and len(A)>1 else '')
    print('%s %4.0f | %s %s | %s %s  %s  %s %s %s %s | %s %s | %s %s'%(r[4:8]+r[-4:],hz,f(m0,0),rng(m0,0),f(m1,0),rng(m1,0),f(m1,1),f(m1,2),f(m1,3),f(m1,4),f(m1,5),f(d,0),f(d,2),f(c,0),f(c,2)))
