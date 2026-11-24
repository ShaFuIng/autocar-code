import json,sys
from rp import *
def vwin(fr,out,d,h=0.3):
    # average speed between odometer d-h and d+h (first frames reaching these)
    ta=tb=None
    for f,o in zip(fr,out):
        if ta is None and o['odo_m']>=d-h: ta=(f['t'],o['odo_m'])
        if o['odo_m']>=d+h: tb=(f['t'],o['odo_m']); break
    if ta and tb and tb[0]>ta[0]: return (tb[1]-ta[1])/(tb[0]-ta[0])
for r in sys.argv[1:]:
    fr=load(r); lp,out=run(fr,BASE)
    c1=[e['d_cross'] for e in lp.events if e['name']=='C1s']
    c4e=[e['d_cross'] for e in lp.events if e['name']=='C4e']
    for k,dc in enumerate(c1):
        prev=[x for x in c4e if dc-15<x<dc]
        lo=0.0 if k==0 else prev[-1]
        hits={'ca':[], 'far':[]}
        for f,o in zip(fr,out):
            d=o['odo_m']
            if not (lo<=d<=dc-1.0): continue
            x=round(d-dc,2); xs=round(d-lo,2)
            if f['rsn'] in ('ERROR_LARGE','CURVE_AHEAD') and (f['err'] or 0)>0: hits['ca'].append((xs,x))
            if f['valid'] and f['far'] is not None and f['far']>=0.4 and f['near'] is not None and f['near']>=-0.05: hits['far'].append((xs,x,round(f['far'],2)))
        def summ(L):
            if not L: return '-'
            return 'n%d sinceC4e/start %.2f..%.2f (toC1s %.2f..%.2f)'%(len(L),min(a[0] for a in L),max(a[0] for a in L),min(a[1] for a in L),max(a[1] for a in L))
        vs=[vwin(fr,out,dc+xx) for xx in (-1.5,-0.5,0.0,0.3)]
        print(r,'lap',k+1,'len %.2f'%(dc-lo),'CA/EL',summ(hits['ca']),'| FAR',summ(hits['far']),'| v(-1.5,-0.5,0,+0.3)',[None if x is None else round(x,2) for x in vs])
