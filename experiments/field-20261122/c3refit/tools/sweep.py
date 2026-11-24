import json, sys, numpy as np, io, contextlib
import calib, sim
E=calib.E; xb=json.load(open(sys.argv[1],encoding='utf8'))['c3']['xband']
f=(E[:-1]>=0.6)&(E[:-1]<4.4)
def score(r, xbd, plant, x0=0.6):
    k0=int(round((x0-0.3)/0.3)); st=dict(x0=x0, h0=float(calib.med(r,'h')[k0]), y0=float(calib.med(r,'y')[k0]), v_of_x=calib.vfun(r))
    R=np.concatenate([sim.run(xbd, seed=s, noise=0.05, plant=plant, **st) for s in range(6)])
    c=lambda a,b: np.sqrt(np.nanmean((a-b)**2))
    return c(sim.binned(R,E,1)[f],calib.med(r,'steer')[f]), c(sim.binned(R,E,2)[f],calib.med(r,'y')[f]), c(sim.binned(R,E,5)[f],calib.med(r,'h')[f]), np.nanmin(sim.binned(R,E,2)[3:7]), np.nanmax(sim.binned(R,E,1)[9:11])
for G in (0.20,0.215,0.23):
    for tau,tl in ((0.10,0.15),(0.10,0.08),(0.07,0.08),(0.13,0.05)):
        pl=dict(G=G,U0=0.,tau=tau,tl=tl); a=score('6f61',None,pl); b=score('c6f3',xb,pl)
        print('G %.3f tau %.2f tl %.2f | 6f61 chain: cmd %3.0f y %.1f h %.1f ymin(1.2-2.4) %+.0f | c6f3 band: cmd %3.0f y %.1f h %.1f ymin %+.0f cmdmax(3.0-3.6 bins) %+.0f'%(G,tau,tl,a[0],a[1],a[2],a[3],b[0],b[1],b[2],b[3],b[4]))
