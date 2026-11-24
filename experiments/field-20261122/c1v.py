import sys,numpy as np
sys.path.insert(0,'field-20261118')
from ld import load
for a in sys.argv[1:]:
    grp,runs=a.split('=',1);T=[]
    for r in runs.split():
        Q=load('run-%s-20261122'%r);c=[i for i,p in enumerate(Q) if p['_lap']==1 and p['_s']=='curve1']
        if not c: print(grp,r,'no curve1');continue
        s,e=c[0],c[-1]
        def v(a,b):
            en=[(Q[i]['t'],(Q[i].get('inputs') or {}).get('enc_b')) for i in range(a,b) if (Q[i].get('inputs') or {}).get('enc_b')]
            return (en[-1][1]-en[0][1])*0.000222/(en[-1][0]-en[0][0]) if len(en)>2 and en[-1][0]>en[0][0] else float('nan')
        n=len(c);pre=[i for i in range(s) if Q[s]['t']-Q[i]['t']<=0.8];post=[i for i in range(e+1,len(Q)) if Q[i]['t']-Q[e]['t']<=1.5]
        ep=[Q[i].get('image_error') or 0 for i in post];sp=[(Q[i].get('steer_us') or 1475)-1475 for i in post];pw=[Q[i].get('pwm') or 0 for i in c]
        T.append(Q[e]['t']-Q[s]['t'])
        print('%s %s 彎1 %.2fs | 入彎前 %.2f 前 %.2f 中 %.2f 後 %.2f | 入彎油門 %.0f 彎內中位 %.0f | 出彎後1.5s 誤差 %+.2f..%+.2f 舵 %+4.0f..%+4.0f 舵sd %.0f (%d格)'%(grp,r,T[-1],v(pre[0],s+1) if pre else float('nan'),v(s,s+n//3),v(s+n//3,s+2*n//3),v(s+2*n//3,e+1),Q[s].get('pwm') or 0,np.median(pw),min(ep) if ep else 0,max(ep) if ep else 0,min(sp) if sp else 0,max(sp) if sp else 0,np.std(sp) if sp else 0,len(post)))
    if T: print('   彎1 時間中位 %.2f'%np.median(T))
