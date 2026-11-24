"""round-1 check: lap/segment times, curve 1 (incl. C1e+1 m), curve 3 trigger per pass. usage: python field-20261122/r1chk.py run..."""
import sys, json, collections, numpy as np
sys.path.insert(0,'field-20261118')
from ld import load
S=['curve1','diagonal_v','curve2','circle_straight','left_bend','curve3','ramp','top','dashed_turn','upper_straight','downhill','curve4','main_straight']
for run in sys.argv[1:]:
    Q=load(run); nl=max(p['_lap'] for p in Q); print('==',run,'dur %.1f'%(Q[-1]['t']-Q[0]['t']))
    T=collections.defaultdict(dict)
    for lap in range(0,nl+1):
        for s in S:
            c=[p['t'] for p in Q if p['_lap']==lap and p['_s']==s]
            if len(c)>5: T[s][lap]=c[-1]-c[0]
    laps=[sum(T[s].get(l,0) for s in S) for l in range(1,nl+1) if all(l in T[s] for s in S)]
    print('  圈(各段加總)',' '.join('%.1f'%x for x in laps))
    for s in S: print('  %-16s'%s,' '.join('%5.2f'%v for v in T[s].values()))
    # curve 1 passes
    idx=[i for i,p in enumerate(Q) if p['_s']=='curve1']; G=[];cur=[]
    for i in idx:
        if cur and i-cur[-1]>30: G.append(cur);cur=[]
        cur.append(i)
    if cur: G.append(cur)
    for g in G:
        if len(g)<20: continue
        i0,i1=g[0],g[-1]; te=Q[i1]['t']
        def al(ps,k): 
            v=[p['arc_lat'][k] for p in ps if (p.get('arc_lat') or {}).get('ok')]; return min(v) if v else float('nan')
        cur=Q[i0:i1+1]; ex=[p for p in Q[i1+1:i1+120] if ((p.get('lap_pos') or {}).get('anchor')=='C1e' and ((p.get('lap_pos') or {}).get('since_anchor_m') or 9)<=1.0)]
        ex2=[p for p in Q[i1+1:i1+200] if p["t"]-te<=2.5]
        if not ex2 or not ex: continue
        print('  彎1 lap%d %.2fs 彎內 內min %+5.1f 外min %+5.1f | C1e+0～1m 內min %+5.1f 外min %+5.1f | 出彎2.5s 誤差 %+.2f..%+.2f 舵 %+4.0f..%+4.0f | 誤差max %.2f 舵max %+4.0f'%(Q[i0]['_lap'],te-Q[i0]['t'],al(cur,'clr_in_cm'),al(cur,'clr_out_cm'),al(ex,'clr_in_cm'),al(ex,'clr_out_cm'),min(p.get('image_error') or 0 for p in ex2),max(p.get('image_error') or 0 for p in ex2),min((p.get('steer_us') or 1475)-1475 for p in ex2),max((p.get('steer_us') or 1475)-1475 for p in ex2),max(p.get('image_error') or 0 for p in cur),max((p.get('steer_us') or 1475)-1475 for p in cur)))
    # raw: curve3 trigger per pass, c1 band e_us 110-150
    P=[]; E=collections.defaultdict(list); k=0; last=None
    for n,l in enumerate(open(run+'/pid.jsonl',encoding='utf8')):
        if '"arc2_c3"' in l or '"lap_ff"' in l:
            p=json.loads(l).get('proposal') or {}
            a=p.get('arc2_c3')
            if isinstance(a,dict): P.append((n,a.get('state'),bool(a.get('xband')),a.get('x_m') if a.get('x_m') is not None else a.get('x_trig')))
            f=p.get('lap_ff')
            if isinstance(f,dict) and f.get('curve')=='c1' and isinstance(f.get('band'),dict) and f['band'].get('own'):
                if last is None or n-last>300: k+=1
                last=n; td=f['band'].get('turned_deg') or 0
                if 110<=td<=150 and f['band'].get('e_us') is not None: E[k].append(f['band']['e_us'])
    G=[];cur=[]
    for r in P:
        if cur and r[0]-cur[-1][0]>300: G.append(cur);cur=[]
        cur.append(r)
    if cur: G.append(cur)
    print('  彎3 每次通過: '+' | '.join('%s xband%d 首x%s'%('啟動' if any(r[1] in('RAMP','HOLD') for r in g) else '未啟動',sum(1 for r in g if r[2]),next(('%.2f'%r[3] for r in g if r[1]=='RAMP' and r[3] is not None),'-')) for g in G))
    print('  彎1 帶狀 110～150° 誤差項中位(µs): '+' '.join('%+.0f'%np.median(v) for v in E.values() if v))
