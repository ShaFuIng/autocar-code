import json,sys,numpy as np
t_from=float(sys.argv[1]) if len(sys.argv)>1 else 0
R=[json.loads(l) for l in open('poses.jsonl',encoding='utf8') if l.strip()]
R=[r for r in R if 'fail' not in r and r.get('gyro')]
t0=R[0]['t'];T=np.array([r['t']-t0 for r in R]);g=np.array([max(abs(x) for x in r['gyro']) for r in R])
still=g<1.0;segs=[];s=None
for i,ok in enumerate(still):
    if ok and s is None: s=i
    if (not ok or i==len(R)-1) and s is not None:
        if T[i-1]-T[s]>=3.5: segs.append((s,i-1))
        s=None
print('last t %.0f s, states %s'%(T[-1],set(r['state'] for r in R[-50:])))
py=None
for a,b in segs:
    if T[b]<t_from: continue
    w=R[a+3:b-1] if b-a>8 else R[a:b]
    f=lambda k:np.array([x[k] for x in w if x.get(k) is not None],float)
    m=lambda k:(np.median(f(k)) if len(f(k)) else float('nan'))
    y=m('yaw');v=np.mean([bool(x.get('valid')) for x in w])
    print('t %5.0f-%5.0f (%3.0f s) yaw %9.1f d %+6.1f pitch %+5.2f roll %+5.2f | near %+.2f sd %.2f far %+.2f ctl %+.2f | valid %3.0f%% %s %s'%(T[a],T[b],T[b]-T[a],y,(y-py) if py is not None else 0,m('pitch'),m('roll'),m('err'),f('err').std() if len(f('err')) else 0,m('far'),m('ctl'),100*v,w[len(w)//2].get('mode'),w[len(w)//2].get('src')))
    py=y
