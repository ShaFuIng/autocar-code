"""dashed turn start: python dashchk.py run centre [run centre ...]"""
import json, sys, numpy as np
A=sys.argv[1:]
for run,c0 in zip(A[0::2],map(float,A[1::2])):
    R=[]
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        if not l.startswith('{"kind": "sample"'): continue
        j=json.loads(l); p=j.get('proposal') or {}; lp=p.get('lap_pos') or {}
        if lp.get('anchor') not in ('TOP','DASHs','RAMP'): R.append(None); continue
        dt=p.get('dash_turn') or {}; up=p.get('upper_path') or {}; d=(j.get('vision') or {}).get('dash_route') or {}
        R.append(dict(t=p.get('t'),a=lp.get('anchor'),x=lp.get('since_anchor_m'),ap=bool(dt.get('applied')),xr=(dt.get('vision') or {}).get('x_row'),tu=dt.get('turn_us'),s=up.get('s_m'),gz=(p.get('inputs') or {}).get('gz') or 0,e=p.get('image_error'),why=d.get('reason'),mode=d.get('mode')))
    i=0;n=0
    while i<len(R):
        if R[i] and R[i]['ap'] and not (R[i-1] and R[i-1]['ap']):
            j=i
            while j<len(R) and R[j] and (R[j]['ap'] or (j+1<len(R) and R[j+1] and R[j+1]['ap'])): j+=1
            seg=[r for r in R[i:j] if r]; pre=[r['e'] for r in R[max(0,i-45):i] if r and r['e'] is not None]
            k=i;acc=0;t60=None;emax=0
            while k+1<len(R) and R[k] and R[k+1] and R[k+1]['t']-R[i]['t']<4:
                acc+=R[k]['gz']*(R[k+1]['t']-R[k]['t'])
                if acc>=60 and t60 is None: t60=R[k+1]['t']-R[i]['t']
                if R[k]['e'] is not None and R[k]['t']-R[i]['t']>0.8: emax=max(emax,abs(R[k]['e']))
                k+=1
            n+=1
            xs=[r['xr'] for r in seg if r['xr'] is not None]
            print(run[4:8],'#%d'%n,'起轉 TOP+%.2f m (s %.2f)'%(seg[0]['x'] if seg[0]['a']=='TOP' else -1, seg[0]['s'] or -1),'起轉原因',seg[0]['why'],'x_row 首 %.0f 最小 %.0f'%(xs[0],min(xs)),'最大左 %+.0f'%(min(r['tu'] for r in seg if r['tu'])-c0),'起轉前誤差 %+.2f'%(np.median(pre) if pre else 9),'60° %s'%('%.2f s'%t60 if t60 else '未到'),'4 s 內轉 %.0f° 轉後誤差峰 %.2f'%(acc,emax))
            i=j
        else: i+=1
