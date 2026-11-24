"""Per-run summary for centre comparison. usage: cmp.py runid ..."""
import sys,json,numpy as np,collections;sys.path.insert(0,'field-20261118')
from ld import load
gz=lambda p:(p.get('inputs') or {}).get('gz') or 0
J=lambda p:((p['_v'].get('path_diagnostic') or {}).get('junction') or {})
for r in sys.argv[1:]:
    Q=load('run-%s-20261118'%r)
    c1=[];last=None
    for p in Q:
        if p['_s']=='curve1' and last!='curve1':c1.append(p['t'])
        last=p['_s']
    print('=====',r,'laps',[round(b-a,1) for a,b in zip(c1,c1[1:])],'start centre',None)
    print(' gaps',[(round(a['t'],1),round(b['t']-a['t'],2),a['_s'],a['_lap']) for a,b in zip(Q,Q[1:]) if b['t']-a['t']>0.25])
    # corridor offset periods
    off=[(p['t'],p['_lap'],p['_s']) for p in Q if J(p).get('baseline_x') is not None and J(p).get('corridor_x') is not None and J(p)['corridor_x']-J(p)['baseline_x']>90 and p['_s'] in('main_straight','diagonal_v','circle_straight')]
    print(' corridor offset frames on straights:',len(off),sorted({(l,s) for _,l,s in off}))
    S=[p for p in Q if p['_s'] in('main_straight','diagonal_v','circle_straight') and abs(gz(p))<15 and p.get('valid',True)]
    for s in ('main_straight','diagonal_v','circle_straight'):
        W=[p for p in S if p['_s']==s]
        print('  %-16s e mean %+.3f med %+.3f | I med %+.0f |I|>=38: %2.0f%% | raw-centre med %+.0f'%(s,np.mean([p.get('image_error') or 0 for p in W]),np.median([p.get('image_error') or 0 for p in W]),np.median([p.get('i') or 0 for p in W]),100*np.mean([abs(p.get('i') or 0)>=38 for p in W]),np.median([((p.get('play_comp') or {}).get('raw_us') or 0) for p in W])-(1510 if r in sys.argv[-1:] and len(sys.argv)>2 else 1480)))
    for lp in sorted({p['_lap'] for p in Q}):
        L=[p for p in Q if p['_lap']==lp];out=['  lap%d'%lp]
        D=[p for p in L if p['_s'] in('downhill','curve4')]
        i0=next((i for i,p in enumerate(D) if (p.get('lap_pos') or {}).get('anchor')=='LAND8'),None)
        if i0 is not None:
            tl=D[i0]['t']-((D[i0]['lap_pos'].get('since_anchor_m') or 0)/1.0)
            pre=[p for p in D if -0.5<=p['t']-tl<-0.05];po=[p for p in D if 0<=p['t']-tl<2.5];l3=[p for p in D if -0.15<=p['t']-tl<=0.3]
            dt=np.median(np.diff([p['t'] for p in po])) if len(po)>3 else .035
            out.append('land: pre0.5s sent med %d min %d | gz@land %.0f..%.0f | e>=.95 %.2fs | emin %.2f | peak %d'%(np.median([p['steer_us'] for p in pre]) if pre else 0,min([p['steer_us'] for p in pre]) if pre else 0,min(map(gz,l3)) if l3 else 0,max(map(gz,l3)) if l3 else 0,sum((p.get('image_error') or 0)>=.95 for p in po)*dt,min(p.get('image_error') or 0 for p in po) if po else 0,max(p['steer_us'] for p in po) if po else 0))
        for s,k in (('curve1','c1'),('curve2','c2')):
            C=[p for p in L if p['_s']==s]
            if len(C)>20:
                dt=np.median(np.diff([p['t'] for p in C]))
                out.append('%s: e med %.2f min %.2f, e>=.95 %.2fs, st max %d'%(k,np.median([p.get('image_error') or 0 for p in C]),min(p.get('image_error') or 0 for p in C),sum((p.get('image_error') or 0)>=.95 for p in C)*dt,max(p['steer_us'] for p in C)))
        print(' | '.join(out))
