"""arc_lat per-zone profile by turned angle. usage: python field-20261119/lat.py run-xxxx [zones]"""
import sys,json,numpy as np,collections;sys.path.insert(0,'field-20261118')
from ld import load
gz=lambda p:(p.get('inputs') or {}).get('gz') or 0
sent=lambda p:(p.get('play_comp') or {}).get('steer_us') or 0
Q=load(sys.argv[1]);zs=(sys.argv[2].split(',') if len(sys.argv)>2 else ['c1','c2','c3','c4'])
passes=collections.defaultdict(list);cur=None
for p in Q:
    a=p.get('arc_lat') or {};z=a.get('zone')
    if z!=cur:
        cur=z;h=0
        if z: passes[z].append([])
    if z:
        if passes[z][-1]: h+=gz(p)*(p['t']-passes[z][-1][-1][0]['t'])
        passes[z][-1].append((p,h))
for z in zs:
    P=passes.get(z,[]);rows=[x for ps in P for x in ps]
    if not rows: continue
    st=collections.Counter(x[0]['arc_lat'].get('state') for x in rows).most_common(5);ap=np.mean([bool(x[0]['arc_lat'].get('applied')) for x in rows])*100
    print('== %s passes %d frames %d applied %.0f%% states %s why %s'%(z,len(P),len(rows),ap,st,collections.Counter(x[0]['arc_lat'].get('why') for x in rows if not x[0]['arc_lat'].get('ok')).most_common(3)))
    step=20 if z!='c3' else 30
    for a0 in range(0,280,step):
        R=[x for x in rows if a0<=-x[1]<a0+step];S=[x[0]['arc_lat'] for x in R if x[0]['arc_lat'].get('ok')]
        if len(S)<6:continue
        y=[s['y_cm'] for s in S];c=[s['clr_in_cm'] for s in S]
        print('   %3d-%3d y %+5.1f (%+5.1f..%+5.1f) clr_in %+5.1f/min %+5.1f clr_out %+5.1f | pre %4.0f pid %4.0f sent max %4.0f gz %+4.0f e %+.2f u_i %s psi %+.1f kap %s n %d'%(a0,a0+step,np.median(y),np.percentile(y,10),np.percentile(y,90),np.median(c),min(c),np.median([s['clr_out_cm'] for s in S]),np.median([x[0].get('steer_us') or 0 for x in R]),np.median([x[0]['arc_lat'].get('pid_steer_us') or 0 for x in R]),max(sent(x[0]) for x in R),np.median([gz(x[0]) for x in R]),np.median([x[0].get('image_error') or 0 for x in R]),'%+.0f'%np.median([s.get('u_i') or 0 for s in S]) if any('u_i' in s for s in S) else '-',np.median([s.get('psi_deg') or 0 for s in S]),'%+.0f'%np.median([s.get('kap_dpm') or 0 for s in S]),len(S)))
    for k,ps in enumerate(P):
        ys=[x[0]['arc_lat']['y_cm'] for x in ps if x[0]['arc_lat'].get('ok')]
        if ys: print('   pass %d: y min %+.1f max %+.1f | sent max %.0f min %.0f | gz sd %.1f | turned %.0f'%(k+1,min(ys),max(ys),max(sent(x[0]) for x in ps),min(sent(x[0]) for x in ps),np.std([gz(x[0]) for x in ps]),-ps[-1][1]))
