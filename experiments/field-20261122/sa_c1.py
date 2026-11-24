import json, collections, numpy as np, sys
shown=False
for run in sys.argv[1:]:
    rows=[]
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        if '"lap_pos"' not in l: continue
        p=json.loads(l).get('proposal') or {}
        lp=p.get('lap_pos'); sa=p.get('speed_adapt') or {}
        if not isinstance(lp,dict) or lp.get('odo_m') is None: continue
        if not shown and sa: shown=True; print('speed_adapt sample',json.dumps(sa)[:420])
        bz=p.get('brake_zones') or {}
        rows.append((lp['odo_m'],lp.get('anchor'),lp.get('since_anchor_m'),sa.get('level'),sa.get('reason'),sa.get('target'),p.get('pwm'),(p.get('arc_lat') or {}).get('v'),p.get('image_error'),(p.get('turn_preview') or {}).get('far_projected_error'),bool(bz.get('zone')),sa.get('weave_slow')))
    cs=[rows[i][0]-(rows[i][2] or 0) for i in range(1,len(rows)) if rows[i][1]=='C1s' and rows[i-1][1]!='C1s']
    print(run,'C1s crossings',len(cs))
    for k,dc in enumerate(cs):
        b=collections.OrderedDict()
        for r in rows:
            if dc-4.0<=r[0]<=dc+0.6: b.setdefault(round(float(np.floor((r[0]-dc)/0.4)*0.4),1),[]).append(r)
        out=[]
        for x,v in b.items():
            mc=lambda j: collections.Counter(str(r[j]) for r in v).most_common(1)[0][0]
            med=lambda j: (lambda a: ('%.2f'%np.median(a)) if a else '-')([r[j] for r in v if isinstance(r[j],(int,float))])
            out.append('%+.1f %s/%s 目標%s 實送%s v%s e%s far%s%s'%(x,mc(3)[:4],mc(4)[:12],med(5),med(6),med(7),med(8),med(9),' 區' if any(r[10] for r in v) else ''))
        print('  #%d '%k+' | '.join(out))
