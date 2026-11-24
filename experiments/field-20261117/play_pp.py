"""Backlash width from straight-line oscillation windows: wheel swing = command swing - h.
Per 1.2 s window on flat straights: pp of sent command (after play comp) vs pp of curvature (-gz/v). Fit pp_kappa = g*(pp_cmd - h)."""
import json,sys,numpy as np
ENC=2.22e-4
def load(r):
    Q=[]
    for l in open(r+'/pid.jsonl',encoding='utf8'):
        if '"kind": "sample"' not in l[:40]:continue
        p=json.loads(l)['proposal']
        if 't' in p:Q.append(p)
    return Q
for r in sys.argv[1:]:
    Q=load(r);T=np.array([q['t'] for q in Q])
    pts=[]
    for q in Q:
        e=(q.get('inputs') or {}).get('enc_b')
        if e is not None and (not pts or e!=pts[-1][1]):pts.append((q['t'],e))
    te=np.array([p[0] for p in pts]);se=np.array([p[1] for p in pts])*ENC
    seg=[];last=None
    for q in Q:
        lp=q.get('lap_pos') or {};a=lp.get('anchor');x=lp.get('since_anchor_m') or 0
        ok=(a=='C4e' and 1.5<=x<=8) or (a=='C1e' and 1<=x<=5) or (a=='C2e' and 0.5<=x<=2.6)
        seg.append(ok)
    seg=np.array(seg);X=[];Y=[]
    t=T[0]
    while t<T[-1]:
        m=(T>=t)&(T<t+1.2)
        if m.sum()>=8 and seg[m].all():
            W=[q for q,k in zip(Q,m) if k]
            u=np.array([(q.get('play_comp') or {}).get('steer_us',q['steer_us']) for q in W],float);gz=np.array([(q.get('inputs') or {}).get('gz') or 0 for q in W],float)
            v=(np.interp(t+1.2,te,se)-np.interp(t,te,se))/1.2
            if v>0.7:X.append(np.ptp(u));Y.append(np.ptp(gz)/v)
        t+=0.6
    X=np.array(X);Y=np.array(Y)
    if len(X)<8:print(r[4:8],'too few windows',len(X));continue
    # robust line through the upper part (large swings are past the backlash)
    big=X>=np.percentile(X,40)
    g,c=np.polyfit(X[big],Y[big],1);h=-c/g
    small=X<120
    print('%s windows %3d | cmd pp median %3.0f (p10 %3.0f p90 %3.0f) | curvature pp median %4.1f deg/m | fit: gain %.3f deg/m per us, h (cmd-axis intercept) %4.0f us | response ratio (deg/m per us) by cmd pp: <120: %.3f  120-200: %.3f  >200: %.3f'%(r[4:8]+r[-4:],len(X),np.median(X),np.percentile(X,10),np.percentile(X,90),np.median(Y),g,h,np.median(Y[small]/X[small]) if small.sum()>2 else np.nan,np.median(Y[(X>=120)&(X<200)]/X[(X>=120)&(X<200)]) if ((X>=120)&(X<200)).sum()>2 else np.nan,np.median(Y[X>=200]/X[X>=200]) if (X>=200).sum()>2 else np.nan))
