"""ramp power: python rampchk.py run [run ...]  -> per pass: time, speed, pwm, pitch, battery if logged"""
import json, sys, numpy as np
for run in sys.argv[1:]:
    P=[]
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        if not l.startswith('{"kind": "sample"'): continue
        p=json.loads(l).get('proposal') or {}; lp=p.get('lap_pos') or {}; i=p.get('inputs') or {}
        if p.get('t') is None or lp.get('odo_m') is None: continue
        P.append((p['t'],lp.get('seg'),lp.get('odo_m'),p.get('pwm'),i.get('pitch_rel') if i.get('pitch_rel') is not None else i.get('pitch'),lp.get('lap'),i.get('vbat') or i.get('battery_v')))
    T=np.array([x[0] for x in P]);O=np.array([x[2] for x in P])
    v=np.array([(O[min(len(O)-1,k+6)]-O[max(0,k-6)])/max(1e-6,T[min(len(T)-1,k+6)]-T[max(0,k-6)]) for k in range(len(P))])
    out=[];k=0
    while k<len(P):
        if P[k][1]=='ramp':
            j=k
            while j<len(P) and P[j][1]=='ramp': j+=1
            if j-k>20:
                pw=[x[3] for x in P[k:j] if x[3] is not None]; vv=v[k:j]
                out.append('圈%s %.2fs 入坡車速 %.2f 最低 %.2f 末 %.2f 油門中位 %.0f 最大 %.0f 坡長 %.2fm'%(P[k][5],T[j-1]-T[k],np.median(v[max(0,k-5):k+1]),vv.min(),vv[-1],np.median(pw),max(pw),O[j-1]-O[k]))
            k=j
        else: k+=1
    print(run[4:8],'電壓欄位',P[0][6]); [print('   ',o) for o in out]
