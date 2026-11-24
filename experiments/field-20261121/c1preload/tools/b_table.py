import json,numpy as np
def load(p):
    f=open(p,encoding='utf8');json.loads(f.readline());return [json.loads(l) for l in f]
def P(r): return r.get('proposal') or {}
print('B. open-loop replay, preload on vs off (same recorded vision / IMU / encoder; the car does not react). us = microseconds.')
print('   dt = seconds relative to the curve-map FLOOR start of lap-1 curve 1; command = proposal.steer_us (before the output stage); sent = after the output stage (play compensation).')
for tgt in (20,15,25):
    print('\n=== target %d us ==='%tgt)
    for run in ('9934','9284','69a8','bab2'):
        A=load('old_%s.jsonl'%run);B=load('on%d_%s.jsonl'%(tgt,run));assert len(A)==len(B)
        t=[];last=0.
        for a in A:
            v=P(a).get('t');last=v if isinstance(v,(int,float)) else last;t.append(last)
        t=np.array(t);fl=next(i for i,a in enumerate(A) if (P(a).get('curve_map') or {}).get('reason')=='FLOOR');t0=t[fl]
        pre=[i for i,b in enumerate(B) if ((P(b).get('i_ramp') or {}).get('preload'))]
        states=[P(B[i])['i_ramp']['preload']['state'] for i in pre]
        num=lambda x:x if isinstance(x,(int,float)) else 0.
        dI=np.array([num(P(b).get('i'))-num(P(a).get('i')) for a,b in zip(A,B)])
        dR=np.array([num(P(b).get('steer_us'))-num(P(a).get('steer_us')) for a,b in zip(A,B)])
        dS=np.array([num(b.get('sent_steer'))-num(a.get('sent_steer')) for a,b in zip(A,B)])
        dP=np.array([num(b.get('pwm'))-num(a.get('pwm')) for a,b in zip(A,B)])
        diff=[i for i,(a,b) in enumerate(zip(A,B)) if {k:v for k,v in P(a).items() if k!='i_ramp'}!={k:v for k,v in P(b).items() if k!='i_ramp'} or a.get('sent_steer')!=b.get('sent_steer') or a.get('pwm')!=b.get('pwm')]
        lap=[(P(a).get('lap_pos') or {}).get('lap') for a in A]
        lap2=[i for i in diff if isinstance(lap[i],int) and lap[i]>=2];cmd2=[i for i in lap2 if dR[i]!=0 or dS[i]!=0 or dP[i]!=0];only=sorted({k for i in lap2 for k in P(A[i]) if k!='i_ramp' and P(A[i]).get(k)!=P(B[i]).get(k)})
        cmd=[i for i in range(len(A)) if dR[i]!=0 or dS[i]!=0]
        print('run %s: frames %d | trigger dt %+.2f s (I before %.1f us) | preload frames %d (%s) | target reached dt %+.2f s'%(run,len(A),t[pre[0]]-t0,P(B[pre[0]])['i_ramp']['preload']['i_us_before'],len(pre),'/'.join(sorted(set(states))),t[pre[-1]]-t0))
        print('   frames whose record differs (any field): %d, dt %+.2f .. %+.2f s | frames whose command differs: %d, last dt %+.2f s | lap >= 2: record differs on %d frames (fields: %s), command / PWM differs on %d frames | PWM differs anywhere: %d frames'%(len(diff),t[diff[0]]-t0,t[diff[-1]]-t0,len(cmd),(t[cmd[-1]]-t0) if cmd else float('nan'),len(lap2),','.join(only) or '-',len(cmd2),int((dP!=0).sum())))
        for a0,a1 in ((-0.6,-0.3),(-0.3,0.0),(0.0,0.3),(0.3,0.6),(0.6,0.9),(0.9,1.2),(1.2,1.8),(1.8,2.6),(2.6,4.0)):
            m=(t-t0>=a0)&(t-t0<a1);ix=np.flatnonzero(m)
            if not len(ix):continue
            Io=np.array([num(P(A[i]).get('i')) for i in ix]);Ro=np.array([num(P(A[i]).get('steer_us')) for i in ix]);er=np.array([num(P(A[i]).get('error')) for i in ix])
            print('   dt %+.1f..%+.1f  error %.2f | I off %+6.1f on %+6.1f (d %+5.1f) | command off %6.0f on %6.0f (d mean %+5.1f, max %+5.1f, min %+5.1f) | sent d mean %+5.1f max %+5.1f'%(a0,a1,er.mean(),Io.mean(),Io.mean()+dI[m].mean(),dI[m].mean(),Ro.mean(),Ro.mean()+dR[m].mean(),dR[m].mean(),dR[m].max(),dR[m].min(),dS[m].mean(),dS[m].max()))
