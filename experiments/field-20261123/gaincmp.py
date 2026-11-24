"""curvature per sent steer inside curves: python gaincmp.py run centre [run centre ...]"""
import json, sys, numpy as np, collections
A=sys.argv[1:]; LAG=0.26
for run,c0 in zip(A[0::2],map(float,A[1::2])):
    T=[];G=[];S=[];O=[];SEG=[]
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        if not l.startswith('{"kind": "sample"'): continue
        p=json.loads(l).get('proposal') or {}; lp=p.get('lap_pos') or {}; i=p.get('inputs') or {}
        if p.get('t') is None or lp.get('odo_m') is None or i.get('steer_cmd') is None: continue
        T.append(p['t']);G.append(i.get('gz') or 0);S.append(i['steer_cmd']-c0);O.append(lp['odo_m']);SEG.append(lp.get('seg'))
    T=np.array(T);G=np.array(G);S=np.array(S);O=np.array(O)
    v=np.array([ (O[min(len(O)-1,k+8)]-O[max(0,k-8)])/max(1e-6,T[min(len(T)-1,k+8)]-T[max(0,k-8)]) for k in range(len(T))])
    Sl=np.interp(T-LAG,T,S)   # steer sent LAG earlier
    Sm=np.array([np.mean(Sl[max(0,k-4):k+5]) for k in range(len(T))]); Gm=np.array([np.mean(G[max(0,k-4):k+5]) for k in range(len(T))])
    kap=-Gm/np.maximum(v,0.3)   # deg/m, + = right
    print('==',run[4:8],'中心',c0)
    for seg in ('curve1','curve2','curve4'):
        m=np.array([s==seg for s in SEG])&(v>0.6)
        row=[]
        for lo,hi in ((100,150),(150,200),(200,250),(250,300),(300,380)):
            mm=m&(Sm>=lo)&(Sm<hi)
            row.append('%d～%d: %s'%(lo,hi,'%.1f (n%d, v%.2f)'%(np.median(kap[mm]),mm.sum(),np.median(v[mm])) if mm.sum()>=25 else '—'))
        mm=m&(Sm>=100)
        g=np.median(kap[mm]/Sm[mm]) if mm.sum()>50 else float('nan')
        print('  %s 實送右舵→曲率 deg/m | '%seg+' | '.join(row)+' | 曲率/舵量 中位 %.3f'%g)
