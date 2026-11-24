"""C：每輪每圈四個彎的帶狀／前饋層是否觸發（RAMP/HOLD）、結束原因；彎 3 另記 C3s 地標出現時第一格的 since_anchor_m。"""
import pickle,glob,os,json,collections
from flagsim import TMP
LAY={'curve1':('lap_ff','c1'),'curve2':('lap_ff','c2'),'curve3':('arc2_c3','c3'),'curve4':('lap_ff4','c4')}
res=[]
for fn in sorted(glob.glob(os.path.join(TMP,'*.pkl'))):
    run=os.path.basename(fn)[:4];rows=pickle.load(open(fn,'rb'))['rows']
    active={k:any((r.get(v[0]) or {}).get('curve')==v[1] for r in rows) for k,v in LAY.items()}
    last={}
    for r in rows:
        if r['lseg'] is not None:last=dict(lap=r['lap'],lseg=r['lseg'])
        elif last:r['lap'],r['lseg']=last['lap'],last['lseg']
    by=collections.defaultdict(list)
    for i,r in enumerate(rows):by[r['lap']].append(i)
    for lap,idx in by.items():
        if lap is None:continue
        for cv,(lk,cn) in LAY.items():
            if not active[cv]:continue
            pas=[i for i in idx if rows[i]['lseg']==cv]
            if not pas:continue
            recs=[rows[i][lk] for i in idx if (rows[i].get(lk) or {}).get('curve')==cn]
            states=collections.Counter(a.get('state') for a in recs)
            trig=bool(states.get('RAMP') or states.get('HOLD'))
            ends=collections.Counter((a.get('end'),a.get('cancel')) for a in recs if a.get('state') in('END','FADE'))
            d=dict(run=run,lap=lap,curve=cv,trig=trig,states=dict(states),ends=[list(k) for k in ends],n_pass=len(pas),
                   x_trig=next((a.get('x_trig') for a in recs if a.get('x_trig') is not None),None),anchor=next((a.get('anchor') for a in recs if a.get('anchor')),None),
                   kind=next((a.get('kind') for a in recs if a.get('kind')),None),x_first=next((a.get('x_m') for a in recs if a.get('state') in ('RAMP','HOLD')),None))
            if cv=='curve3':
                f=next((rows[i] for i in idx if rows[i]['anchor']=='C3s' and rows[i]['since'] is not None),None)
                d['c3s_first_since']=None if f is None else f['since']
            res.append(d)
json.dump(res,open(os.path.join(TMP,'..','proof','C_curves.json'),'w'),ensure_ascii=False,indent=0)
for cv in LAY:
    R=[r for r in res if r['curve']==cv];nt=[r for r in R if not r['trig']]
    print(cv,'passes(層有開的輪)',len(R),'未觸發',len(nt))
    for r in nt:print('   ',r['run'],'lap',r['lap'],r['states'],r.get('c3s_first_since',''))
    ends=collections.Counter(tuple(map(tuple,r['ends'])) for r in R if r['trig']);print('   結束原因',dict(ends))
R=[r for r in res if r['curve']=='curve3']
print('C3s 第一格 since 分布（觸發/未觸發）:',sorted((r['c3s_first_since'],r['trig'],r['run'],r['lap']) for r in R if r['c3s_first_since'] is not None)[-12:])
