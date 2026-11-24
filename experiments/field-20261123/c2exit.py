"""curve-2 exit -> circle straight -> crossing, per lap. usage: python field-20261123/c2exit.py run [run...]"""
import json, sys, collections, numpy as np
for run in sys.argv[1:]:
    R=[]
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        if not l.startswith('{"kind": "sample"'): continue
        p=json.loads(l).get('proposal') or {}; lp=p.get('lap_pos') or {}
        a=lp.get('anchor'); s=lp.get('since_anchor_m')
        if a not in ('C2s','C2e') or s is None: continue
        f=p.get('lap_ff') or {}; b=f.get('band') or {}; ls=p.get('lap_speed') or {}; tl=ls.get('two_level') or {}
        R.append(dict(lap=lp.get('lap'),a=a,s=s,t=p.get('t'),e=p.get('image_error'),st=(p.get('steer_us') or 1475)-1475,gz=(p.get('inputs') or {}).get('gz') or 0,pwm=p.get('pwm') or 0,owner=str(p.get('owner')),valid=p.get('valid'),reason=str(p.get('reason')),
                      td=b.get('turned_deg') if f.get('curve')=='c2' else None,fs=f.get('state') if f.get('curve')=='c2' else None,tls=tl.get('state'),tlw=tl.get('why'),dev=lp.get('dev_deg'),src=((p.get('junction') or {}).get('source')),v=(p.get('arc_lat') or {}).get('v'),
                      xp=bool(p.get('crossing_pass') or p.get('xp_anchor') or p.get('arc2_blind') ), keys=[k for k in p if any(x in k for x in ('cross','xp','blind','junction'))]))
    print('==',run[4:8],'路口相關欄位',sorted({k for r in R for k in r['keys']}))
    for lap in sorted({r['lap'] for r in R if r['lap']}):
        c2=[r for r in R if r['lap']==lap and r['a']=='C2s']; ce=[r for r in R if r['lap']==lap and r['a']=='C2e' and r['s']<=6.5]
        if len(ce)<20: continue
        end=[r for r in c2 if r['td'] is not None]; tdend=end[-1]['td'] if end else None; send=end[-1]['s'] if end else None
        vex=[r['v'] for r in c2[-8:] if r['v']]
        seg=lambda a,b:[r for r in ce if a<=r['s']<b]
        def stat(a,b):
            v=seg(a,b)
            if not v: return ' - '
            e=[r['e'] for r in v if r['e'] is not None]
            return '%+.2f/%+.2f st%+4.0f gz%+3.0f pwm%3.0f%s'%(np.median(e) if e else float('nan'),(max(e,key=abs) if e else float('nan')),np.median([r['st'] for r in v]),np.median([r['gz'] for r in v]),np.median([r['pwm'] for r in v]),'*' if any(r['tls']=='TROUBLE' for r in v) else '')
        settle=next((r['s'] for i,r in enumerate(ce) if all(abs(x['e'] or 0)<=0.15 for x in ce[i:i+12]) and len(ce[i:i+12])==12),None)
        inval=sum(1 for r in ce if r['reason']!='PROPOSAL_ONLY')
        print(' L%d 帶狀結束 轉%s°@C2s+%s 出彎車速%s | C2e後 0-1: %s | 1-2: %s | 2-3: %s | 3-4.5: %s | 4.5-6.5: %s | 誤差回到±0.15: %s m | 非一般格 %d | 兩級制TROUBLE %d格'%(lap,('%.0f'%tdend) if tdend is not None else '-',('%.2f'%send) if send is not None else '-',('%.2f'%np.median(vex)) if vex else '-',stat(0,1),stat(1,2),stat(2,3),stat(3,4.5),stat(4.5,6.5),('%.1f'%settle) if settle is not None else '未',inval,sum(1 for r in ce if r['tls']=='TROUBLE')))
