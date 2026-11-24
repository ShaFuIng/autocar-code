import json,sys,numpy as np,collections
def load(r):
    A=[json.loads(l) for l in open(r+'/pid.jsonl',encoding='utf8')];return [d for d in A if d.get('kind')=='sample' and 't' in d['proposal']]
for r in sys.argv[1:]:
    D=load(r);Q=[d['proposal'] for d in D];t=np.array([q['t'] for q in Q]);dt=np.diff(t)
    age=np.array([d['vision'].get('control_age_s') or np.nan for d in D]);inf=np.array([d['vision'].get('inference_ms') or np.nan for d in D])
    print('=====',r[4:8],'fps %.2f'%(len(Q)/(t[-1]-t[0])),'dt p50/p90 %.3f/%.3f'%tuple(np.percentile(dt,[50,90])),'age p50/p90/max %.3f/%.3f/%.3f'%(np.nanmedian(age),np.nanpercentile(age,90),np.nanmax(age)),'worker p50 %.1f'%np.nanmedian(inf),'gaps>0.25',int((dt>.25).sum()),'ri',(Q[len(Q)//2].get('rate_indep') or Q[len(Q)//2].get('ri')))
    seg=collections.defaultdict(list);last=None;lap=0;pl=None
    for q in Q:
        lp=q.get('lap_pos') or {};s=lp.get('seg') or last
        if s=='curve1' and last!='curve1':lap+=1
        last=s;seg[(s,lap)].append(q)
    for s in ['main_straight','diagonal_v','circle_straight','left_bend','upper_straight','downhill']:
        row=[]
        for (ss,l),X in sorted(seg.items(),key=lambda k:k[0][1]):
            if ss!=s or len(X)<15:continue
            gz=np.array([(x.get('inputs') or {}).get('gz') or 0 for x in X],float);st=np.array([(x.get('play_comp') or {}).get('steer_us',x['steer_us']) for x in X]);e=np.array([x.get('image_error') or 0 for x in X]);pw=np.array([x.get('pwm') or 0 for x in X])
            row.append('L%d gz_sd %4.1f st_sd %3d |e|med %.2f emin %+.2f emax %+.2f pwm %2.0f'%(l,gz.std(),st.std(),np.median(abs(e)),e.min(),e.max(),pw.mean()))
        print(' %-15s'%s,' || '.join(row))
