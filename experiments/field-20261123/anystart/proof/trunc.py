import json,sys,copy
from rp import *
MPC=BASE['m_per_count']
def cfgmod(v):
    c=copy.deepcopy(BASE); c['skip_after_by']=dict(c['skip_after_by'],C1s=v); return c
XS=[2.0,3.44,5.0,7.0,9.0,10.5]
res=[]
for r in ['e69d','9af6','2728','7658','54cb']:
    fr=load(r); lpf,outf=run(fr,BASE)
    enc0=next(f['enc'] for f in fr if f['enc'] is not None)
    full=[e for e in lpf.events if 'd_cross' in e]
    c1=[e for e in full if e['name']=='C1s' and e['lap']>=2]
    for e in c1:
        dc=e['d_cross']
        # next C1s after this
        nxt=[x for x in full if x['name']=='C1s' and x['d_cross']>dc+10]
        dend=nxt[0]['d_cross']+0.5 if nxt else 1e9
        ref=[(x['name'],x['d_cross']) for x in full if dc-0.01<=x['d_cross']<=dend]
        for X in XS:
            i0=next((i for i,o in enumerate(outf) if o['odo_m']>=dc-X),None)
            if i0 is None: continue
            for tag,cfg in (('base',BASE),('fix',cfgmod(9.0))):
                lp=lap_pos.LapPos(copy.deepcopy(cfg)); e0=None
                for f in fr[i0:]:
                    if e0 is None and f['enc'] is not None: e0=f['enc']
                    lp.update(f['t'],f['gz'],f['pr'],f['enc'])
                    if lp.D+ (e0-enc0)*MPC > dend: break
                off=(e0-enc0)*MPC
                ev=[(x['name'],round(x['d_cross']+off,3) if 'd_cross' in x else None) for x in lp.events]
                sk=[x for x in lp.events if x['name'] in ('SKIPPED','LOST')]
                c1t=[x for x in lp.events if x['name']=='C1s' and 'd_cross' in x]
                c1err=round(c1t[0]['d_cross']+off-dc,3) if c1t else None
                # match landmarks
                got={}
                for n,d in ev:
                    if d is not None: got.setdefault(n,[]).append(d)
                miss=0; maxerr=0
                for n,d in ref:
                    cand=[abs(x-d) for x in got.get(n,[])]
                    if not cand or min(cand)>0.5: miss+=1
                    else: maxerr=max(maxerr,min(cand))
                res.append(dict(run=r,lap=e["lap"],X=X,cfg=tag,c1err=c1err,maxerr=round(maxerr,3),skips=[(x['name'],x.get('at') or x.get('why')) for x in sk],miss=miss,nref=len(ref),
                     first_anchor=lp.events[0]['name'] if lp.events else None))
json.dump(res,open('trunc_res.json','w'))
import collections
agg=collections.defaultdict(list)
for x in res: agg[(x['cfg'],x['X'])].append(x)
for k in sorted(agg):
    L=agg[k]; ok=sum(1 for x in L if x['c1err'] is not None and abs(x['c1err'])<0.3 and x['miss']==0 and not x['skips'])
    ce=[x['c1err'] for x in L if x['c1err'] is not None]
    print(k,'n',len(L),'ok',ok,'c1err range',(min(ce),max(ce)) if ce else None,'noC1',sum(1 for x in L if x['c1err'] is None),'skips',collections.Counter(str(x['skips']) for x in L).most_common(3),'miss',dict(collections.Counter(x['miss'] for x in L)),'maxerr',max(x['maxerr'] for x in L))
