"""Per lap, per lap_pos segment: gz sd (high-passed 1 s), yaw_inner term sd, share of frames curve_map not IDLE, no_hold share, pwm mean, |e| median."""
import json,sys,numpy as np,collections
SEGS=['main_straight','diagonal_v','circle_straight','ramp','curve4','upper_straight','downhill']
for run in sys.argv[1:]:
    Q=[]
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        if '"kind": "sample"' not in l[:40]:continue
        p=json.loads(l)['proposal']
        if 't' in p:Q.append(p)
    lap=0;last=None;G=collections.defaultdict(list)
    for p in Q:
        s=(p.get('lap_pos') or {}).get('seg') or last
        if s=='curve1' and last!='curve1':lap+=1
        last=s;G[(lap,s)].append(p)
    print('=====',run[4:8],'  cols: gz_sd(hp)  term_sd  cm_notidle%  no_hold%  pwm  |e|med  dur')
    for s in SEGS:
        print(' ',s)
        for lp in sorted({k[0] for k in G}):
            P=G.get((lp,s))
            if not P or len(P)<20:continue
            t=np.array([p['t'] for p in P]);gz=np.array([(p.get('inputs') or {}).get('gz') or 0. for p in P])
            # remove 1 s moving mean
            hp=np.array([gz[i]-gz[(t>t[i]-.5)&(t<t[i]+.5)].mean() for i in range(len(t))])
            yi=[p.get('yaw_inner') or {} for p in P]
            term=np.array([y.get('term_us') or 0. for y in yi])
            cm=np.mean([(p.get('curve_map') or {}).get('reason') not in (None,'IDLE') for p in P])*100
            nh=np.mean([bool(y.get('no_hold')) for y in yi])*100
            pw=np.mean([p.get('pwm') or 0 for p in P]);e=np.median([abs(p.get('image_error') or 0) for p in P])
            print('    lap %d  %5.1f  %5.1f  %4.0f  %4.0f  %4.1f  %.2f  %.1fs'%(lp,hp.std(),term.std(),cm,nh,pw,e,t[-1]-t[0]))
