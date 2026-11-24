import json,sys,numpy as np,collections
names=['curve1','diagonal_v','curve2','circle_straight','left_bend','curve3','ramp','top','dashed_turn','upper_straight','downhill','curve4','main_straight']
def load(path):
    S=[]
    for l in open(path+'/pid.jsonl',encoding='utf8'):
        d=json.loads(l)
        if d.get('kind')=='sample' and 't' in d['proposal']: S.append(d['proposal'])
    return S
out={}
for path in sys.argv[1:]:
    S=load(path);r=path.split('run-')[1][:4]
    laps=collections.defaultdict(list)
    for p in S:
        lp=p.get('lap_pos') or {}
        if lp.get('lap') is not None: laps[lp['lap']].append(p)
    full=[]
    for L,X in sorted(laps.items()):
        segs=[(x.get('lap_pos') or {}).get('seg') for x in X]
        if all(n in segs for n in names) and L>=1: full.append(L)
    T=collections.defaultdict(list);V=collections.defaultdict(list);P=collections.defaultdict(list);tot=[];p50=[];p45=[];pm=[]
    for L in full:
        X=laps[L];t=np.array([x['t'] for x in X]);dt=np.diff(t,append=t[-1]).clip(0,.3);pw=np.array([x['pwm'] for x in X]);sg=np.array([(x.get('lap_pos') or {}).get('seg') for x in X]);od=np.array([x['lap_pos']['odo_m'] for x in X])
        tot.append(dt.sum());p50.append(dt[pw>=49.5].sum()/dt.sum());p45.append(dt[pw>=45].sum()/dt.sum());pm.append((pw*dt).sum()/dt.sum())
        for n in names:
            m=sg==n
            if m.any(): T[n].append(dt[m].sum());P[n].append((pw[m]*dt[m]).sum()/dt[m].sum())
    if not full: print(r,'no full laps');continue
    print('%s laps %s | lap(seg-sum) %.1f | pwm mean %.1f | time at PWM>=50 %.0f%% >=45 %.0f%%'%(r,full,np.mean(tot),np.mean(pm),100*np.mean(p50),100*np.mean(p45)))
    out[r]=({n:np.mean(T[n]) for n in names},{n:np.mean(P[n]) for n in names})
rs=list(out)
print('%-16s'%'segment time s'+''.join('%8s'%r for r in rs)+'   | mean pwm '+''.join('%6s'%r for r in rs))
for n in names: print('%-16s'%n+''.join('%8.2f'%out[r][0][n] for r in rs)+'   |          '+''.join('%6.0f'%out[r][1][n] for r in rs))
flat=['curve1','diagonal_v','curve2','circle_straight','left_bend','curve3','curve4','main_straight'];up=['ramp','top','dashed_turn','upper_straight','downhill']
print('%-16s'%'FLAT total'+''.join('%8.2f'%sum(out[r][0][n] for n in flat) for r in rs));print('%-16s'%'UPPER+slopes'+''.join('%8.2f'%sum(out[r][0][n] for n in up) for r in rs))
