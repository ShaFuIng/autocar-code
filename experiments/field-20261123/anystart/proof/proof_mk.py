import json,sys,copy,collections
from pathlib import Path
from rp import *
import brake_zones
sys.path.insert(0,r'D:/OneDrive/Desktop/自走車/Orin_Interface_v1/laptop_workbench/data/experiments/field-20261123/anystart/tools')
import mkstart
B=Path('base')
res=[]
for r in ['e69d','9af6','2728','7658','54cb']:
    fr=load(r); lpf,outf=run(fr,BASE)
    full=[e for e in lpf.events if 'd_cross' in e]
    for e in [x for x in full if x['name']=='C1s' and x['lap']>=2]:
        dc=e['d_cross']
        for D in (2.5,3.44,6.0,9.0,10.5):
            for err in (-1.0,-0.5,0.0,0.5,1.0,2.0):
                Dtrue=D+err
                if Dtrue<2.0 or Dtrue>11.2: continue
                i0=next(i for i,o in enumerate(outf) if o['odo_m']>=dc-(Dtrue-0.44)); off=outf[i0]['odo_m']
                for skip in (None,9.0):
                    cf=mkstart.build(B,D,skip)
                    lp=lap_pos.LapPos(copy.deepcopy(cf['lap_pos.json'][1])); bz=brake_zones.BrakeZones(copy.deepcopy(cf['brake_zones.json'][1]))
                    st=cf['lap_ff.json'][1]['curves']['c1']['start']; lo=st['gap_m']+st['win_m'][0]; hi=st['gap_m']+st['win_m'][1]
                    z1=None; z1e=None; band=None; farfirst=None
                    for f in fr[i0:]:
                        s=lp.update(f['t'],f['gz'],f['pr'],f['enc'])
                        cap,info=bz.update({'enc_b':{'forward':f['enc'],'reverse':0}},f['rsn'],f['err'],f['far'],f['pr'],lap=lp)
                        d=s['odo_m']
                        zn=((info or {}).get('zone') or {}).get('name','')
                        if zn.startswith('curve 1 on lap 1'):
                            if z1 is None: z1=d
                            z1e=d
                        okf=f['valid'] and (f['far'] or 0)>=0.4 and (f['near'] if f['near'] is not None else -1)>=-0.05
                        if okf and farfirst is None and d>0.5: farfirst=d
                        if band is None and s['lap']==0 and s['anchor'] is None and lo<=d<=hi and okf: band=d
                        if d>Dtrue+12: break
                    ev=[x for x in lp.events]
                    c1=[x for x in ev if x['name']=='C1s']
                    res.append(dict(run=r,lap=e['lap'],D=D,err=err,skip=skip,c1err=None if not c1 else round(c1[0]['d_cross']+off-dc,2),
                        bad=[x['name']+':'+str(x.get('at') or x.get('why')) for x in ev if x['name'] in ('SKIPPED','LOST')],
                        z1=None if z1 is None else (round(z1+off-dc,2),round(z1e+off-dc,2)),band=None if band is None else round(band+off-dc,2),
                        far1=None if farfirst is None else round(farfirst+off-dc,2)))
json.dump(res,open('proof_mk.json','w'))
agg=collections.defaultdict(list)
for x in res: agg[(x['skip'],x['D'],x['err'])].append(x)
for k in sorted(agg,key=lambda k:(k[0] or 0,k[1],k[2])):
    L=agg[k]
    ok=sum(1 for x in L if x['c1err'] is not None and abs(x['c1err'])<0.35 and not x['bad'])
    z=[x['z1'][0] for x in L if x['z1']]; b=[x['band'] for x in L if x['band'] is not None]
    late=sum(1 for x in L if x['band'] is not None and x['far1'] is not None and x['band']-x['far1']>0.1 and x['far1']>-1.5)
    mis=sum(1 for x in L if x['band'] is not None and x['band']<-1.2)
    print('skip',k[0],'D',k[1],'err%+.1f'%k[2],'n',len(L),'locOK',ok,'bad',collections.Counter(str(x['bad']) for x in L if x['bad']).most_common(1),
          'zone1 start rel C1s',(min(z),max(z)) if z else None,'band',(min(b),max(b),len(b)) if b else None,'band late>0.1m',late,'misfire<-1.2',mis)
