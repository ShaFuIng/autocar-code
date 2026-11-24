"""ramp swing per pass: python rampsw.py run centre [...]"""
import json, sys, collections, numpy as np
A=sys.argv[1:]
for run,c0 in zip(A[0::2],map(float,A[1::2])):
    L=collections.defaultdict(list)
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        if not l.startswith('{"kind": "sample"'): continue
        p=json.loads(l).get('proposal') or {}; lp=p.get('lap_pos') or {}
        if lp.get('seg')!='ramp': continue
        ls=p.get('lap_speed') or {}; i=p.get('inputs') or {}
        L[lp.get('lap')].append((p.get('t'),p.get('pwm') or 0,p.get('image_error'),(i.get('steer_cmd') or c0)-c0,ls.get('blocked'),i.get('gz') or 0,lp.get('odo_m')))
    for lap,v in L.items():
        e=[x[2] for x in v if x[2] is not None]; g=np.array([x[5] for x in v]); t=np.array([x[0] for x in v])
        zc=int(np.sum(np.diff(np.sign(g[np.abs(g)>8]))!=0))
        h=[x for x in v if x[0]-t[0]>1.0]   # after the first second (upper half of the slope)
        print(run[4:8],'圈%s'%lap,'%.2fs'%(t[-1]-t[0]),'坡末車速 %.2f'%((v[-1][6]-v[-8][6])/max(1e-6,v[-1][0]-v[-8][0])),'誤差 max %+.2f min %+.2f'%(max(e),min(e)),'gz 標準差 全程 %.0f 後段 %.0f 換向 %d'%(np.std(g),np.std([x[5] for x in h]) if h else -1,zc),'後段誤差幅 %.2f'%(max(x[2] for x in h if x[2] is not None)-min(x[2] for x in h if x[2] is not None)) if h else '','油門 max %.0f ≥90 %d格'%(max(x[1] for x in v),sum(x[1]>=90 for x in v)),'被擋',dict(collections.Counter(x[4] for x in v if x[4])))
