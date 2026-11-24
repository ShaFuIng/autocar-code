"""arc_lat y as a position measurement in curve 3: share of frames with a value per 0.3 m, frame-to-frame noise, longest gap; reasons when missing."""
import numpy as np, p3, lib, collections
for r in ['6f61','c6f3','d60e']:
    P=p3.passes([r]); E=np.arange(0.6,4.81,0.3)
    ok=[];nz=[];gap=[]
    for a,b in zip(E[:-1],E[1:]):
        v=np.concatenate([q['A']['y'][(q['A']['xc']>=a)&(q['A']['xc']<b)] for q in P]); ok.append(np.isfinite(v).mean())
    for q in P:
        A=q['A']; m=(A['xc']>=0.6)&(A['xc']<4.4); y=A['y'][m]; t=A['t'][m]
        d=np.diff(y); nz.append(np.nanstd(d[np.isfinite(d)])); 
        g=0;gm=0
        for i in range(len(y)):
            g = g+1 if not np.isfinite(y[i]) else 0; gm=max(gm,g)
        gap.append(gm)
    print(r,'valid share per 0.3 m from 0.6:',' '.join('%.2f'%v for v in ok),'| frame-to-frame sd of y %s cm | longest gap (frames) %s'%(' '.join('%.1f'%v for v in nz),' '.join(str(v) for v in gap)))
    Q=lib.load(r); c=collections.Counter()
    for p in Q:
        lp=p.get('lap_pos') or {}; al=p.get('arc_lat') or {}
        if lp.get('anchor')=='C3s' and 0.6<=(lp.get('since_anchor_m') or 0)<4.4: c[(al.get('ok'),al.get('why'),al.get('gate_rej'),al.get('reason'))]+=1
    print('   ',dict(c))
