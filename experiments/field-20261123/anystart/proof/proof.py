import json,sys,copy,collections
from rp import *
import brake_zones
A=r'D:/OneDrive/Desktop/自走車/Orin_Interface_v1/laptop_workbench/data/experiments/field-20261123/anystart/on/anystart/'
NLP=json.load(open(A+'lap_pos.json',encoding='utf8')); NBZ=json.load(open(A+'brake_zones.json',encoding='utf8'))
OBZ=json.load(open('base/brake_zones.json',encoding='utf8'))
MPC=BASE['m_per_count']
def sim(fr,i0,lpcfg,bzcfg,stop_d=None):
    lp=lap_pos.LapPos(copy.deepcopy(lpcfg)); bz=brake_zones.BrakeZones(copy.deepcopy(bzcfg)); rows=[]; e0=None
    for f in fr[i0:]:
        if f['enc'] is None: continue
        if e0 is None: e0=f['enc']
        st=lp.update(f['t'],f['gz'],f['pr'],f['enc'])
        cap,info=bz.update({'enc_b':{'forward':f['enc'],'reverse':0}},f['rsn'],f['err'],f['far'],f['pr'],lap=lp)
        rows.append((f,st,cap,(info or {}).get('zone') and info['zone'].get('name','')[:40]))
        if stop_d is not None and lp.D>stop_d: break
    return lp,bz,rows
def band_trig(rows,lo=1.0,hi=4.6):
    for f,st,cap,zn in rows:
        if st['lap']!=0 or st['anchor'] is not None: return None
        d=st['odo_m']
        if lo<=d<=hi and f['valid'] and f['far'] is not None and f['far']>=0.4 and f['near'] is not None and f['near']>=-0.05: return d
    return None
out=collections.defaultdict(list)
XS=[2.0,3.0,3.44,5.0,7.0,9.0,10.5]
for r in ['e69d','9af6','2728','7658','54cb']:
    fr=load(r); lpf,outf=run(fr,BASE)
    full=[e for e in lpf.events if 'd_cross' in e]
    for e in [x for x in full if x['name']=='C1s' and x['lap']>=2]:
        dc=e['d_cross']; top=[x['d_cross'] for x in full if x['name']=='TOP' and x['d_cross']>dc][:1]; dash=[x['d_cross'] for x in full if x['name']=='DASHe' and x['d_cross']>dc][:1]
        c1e=[x['d_cross'] for x in full if x['name']=='C1e' and x['d_cross']>dc][:1]
        for X in XS:
            i0=next((i for i,o in enumerate(outf) if o['odo_m']>=dc-X),None)
            off=outf[i0]['odo_m']
            lp,bz,rows=sim(fr,i0,NLP,NBZ,stop_d=(dash[0]-off+5) if dash else None)
            fc=bz.anchors.get('first_curve'); c1=[x for x in lp.events if x['name']=='C1s']
            bt=band_trig(rows)
            # zones in absolute
            zon=collections.OrderedDict()
            for f,st,cap,zn in rows:
                if zn: zon.setdefault(zn,[st['odo_m']+off,st['odo_m']+off]); zon[zn][1]=st['odo_m']+off
            def rel(z,ref):
                k=[q for q in zon if q[:28]==z[:28]]
                return None if not k else (round(zon[k[0]][0]-ref,2),round(zon[k[0]][1]-ref,2))
            rec=dict(run=r,lap=e['lap'],X=X,fc_toC1s=None if fc is None else round(fc+off-dc,2),c1err=round(c1[0]['d_cross']+off-dc,2) if c1 else None,
                     skips=[x['name']+':'+str(x.get('at') or x.get('why')) for x in lp.events if x['name'] in ('SKIPPED','LOST')],
                     band_toC1s=None if bt is None else round(bt+off-dc,2),
                     c1zone24=rel('anystart: curve 1 on lap 1, from the first',dc),c1zone36=rel('anystart: curve 1 on lap 1, then 36 (= th',dc),
                     c2lap1=rel('curve 2 on lap 1 (FLOOR 17.75-18.0 m afte',c1e[0]) if c1e else None,
                     preturn_fb=rel('pre-turn lap 1 fallback (TOP +1.5 m is a',top[0]) if top else None, crest_fb=rel('crest lap 1 fallback (DASHe +1.8 m is abo',dash[0]) if dash else None)
            out['trunc'].append(rec)
# real short starts + lap1 of multi-lap: old vs new
for r in ['3f22','fd22','b23a','2b72','b619','1377','1cd7','e69d','9af6','2728','7658','54cb']:
    fr=load(r)
    i0=0
    lpo,bzo,ro=sim(fr,0,BASE,OBZ,stop_d=60); lpn,bzn,rn=sim(fr,0,NLP,NBZ,stop_d=60)
    c1=[x['d_cross'] for x in lpn.events if x['name']=='C1s'][0]
    diffs=[]
    for (f,so,co,zo),(_,sn,cn,zn) in zip(ro,rn):
        pw=f['pwm'] or 0
        # pre-cap pwm unknown; compare caps applied relative to logged pwm
        if co!=cn: diffs.append((round(so['odo_m']-c1,2),co,cn,round(pw,1),zo,zn))
    # summarise differing stretches
    segs=[]
    for d in diffs:
        if segs and abs(d[0]-segs[-1][1])<0.35 and (d[1],d[2])==segs[-1][2]: segs[-1][1]=d[0]
        else: segs.append([d[0],d[0],(d[1],d[2])])
    out['fixed'].append(dict(run=r,fc_old=bzo.anchors.get('first_curve'),fc_new=bzn.anchors.get('first_curve'),C1s=c1,band_new=band_trig(rn),band_old=band_trig(ro,2.0,4.2),
        diff_segments=[(round(a,2),round(b,2),'old cap %s -> new cap %s'%k) for a,b,k in segs],samelap=[x['name'] for x in lpo.events if x['name'] in('SKIPPED','LOST')]==[x['name'] for x in lpn.events if x['name'] in('SKIPPED','LOST')]))
json.dump(out,open('proof_out.json','w'),ensure_ascii=False,indent=0)
agg=collections.defaultdict(list)
for x in out['trunc']: agg[x['X']].append(x)
for X in XS:
    L=agg[X]
    def rng(k): v=[x[k] for x in L if x[k] is not None]; return (min(v),max(v),len(v)) if v else None
    def rng2(k,i): v=[x[k][i] for x in L if x[k]]; return (min(v),max(v),len(v)) if v else None
    print('X',X,'n',len(L),'c1err',rng('c1err'),'skips',sum(1 for x in L if x['skips']),'fc',rng('fc_toC1s'),'band',rng('band_toC1s'),'z24 start',rng2('c1zone24',0),'z36 end',rng2('c1zone36',1),
          'c2lap1 start(C1e+)',rng2('c2lap1',0),'preturn_fb(TOP+)',rng2('preturn_fb',0),rng2('preturn_fb',1),'crest_fb(DASHe+)',rng2('crest_fb',0),rng2('crest_fb',1))

