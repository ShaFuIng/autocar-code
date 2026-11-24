"""any-start check for the first lap of a run. usage: python field-20261123/startchk.py run..."""
import json, sys, numpy as np, collections
for run in sys.argv[1:]:
    rows=[]
    for n,l in enumerate(open(run+'/pid.jsonl',encoding='utf8')):
        if n>4500: break
        p=json.loads(l).get('proposal') or {}
        lp=p.get('lap_pos')
        if not isinstance(lp,dict) or lp.get('odo_m') is None: continue
        bz=(p.get('brake_zones') or {}); z=bz.get('zone') or {}; f=p.get('lap_ff') or {}
        rows.append(dict(D=lp['odo_m'],a=lp.get('anchor'),s=lp.get('since_anchor_m'),lost=lp.get('lost'),lap=lp.get('lap'),seg=lp.get('seg'),pwm=p.get('pwm') or 0,v=(p.get('arc_lat') or {}).get('v'),e=p.get('image_error') or 0,
                         zone=(z.get('name') or '')[:16],cap=bz.get('cap_pwm'),c1=(f.get('curve')=='c1'),own=((f.get('band') or {}).get('own') if f.get('curve')=='c1' else None),st=(p.get('steer_us') or 1475)-1475,
                         clin=(p.get('arc_lat') or {}).get('clr_in_cm') if (p.get('arc_lat') or {}).get('ok') else None,clout=(p.get('arc_lat') or {}).get('clr_out_cm') if (p.get('arc_lat') or {}).get('ok') else None,sb=(p.get('start_bias') or {}).get('bias_us')))
    anc={}
    for i in range(1,len(rows)):
        if rows[i]['a']!=rows[i-1]['a'] and rows[i]['a'] and rows[i]['a'] not in anc: anc[rows[i]['a']]=rows[i]['D']-(rows[i]['s'] or 0)
    dc=anc.get('C1s'); print('==',run[4:8],'總里程 %.1f m | 地標(起跑後里程):'%rows[-1]['D'],{k:round(v,2) for k,v in anc.items()},'| lost 格',sum(1 for r in rows if r['lost']),'| laps',sorted({r['lap'] for r in rows}))
    if dc is None: print('   沒認到 C1s'); continue
    z1=[r['D'] for r in rows if r['zone'].startswith('curve 1 on lap 1')]; b1=[r['D'] for r in rows if r['c1']]; ow=[r['D'] for r in rows if r['own']]
    sb=[r['D'] for r in rows if r['sb']]
    vat=[r['v'] for r in rows if r['v'] and abs(r['D']-dc)<0.25]; vmax=[r['v'] for r in rows if r['v'] and r['D']<dc]
    c1=[r for r in rows if r['a']=='C1s']; ex=[r for r in rows if r['a']=='C1e' and (r['s'] or 9)<=1.0]
    mn=lambda rs,k: min([r[k] for r in rs if r[k] is not None]+[99])
    print('   彎1起點在起跑後 %.2f m | 第1圈彎1減速區 C1s%+.2f..%+.2f m | 帶狀首格 C1s%+.2f、全權自 C1s%+.2f | 起步左舵到 C1s%+.2f | 直線最高車速 %s、C1s 處車速 %s'%(dc,(z1[0]-dc) if z1 else float('nan'),(z1[-1]-dc) if z1 else float('nan'),(b1[0]-dc) if b1 else float('nan'),(ow[0]-dc) if ow else float('nan'),(sb[-1]-dc) if sb else float('nan'),('%.2f'%max(vmax)) if vmax else '-',('%.2f'%np.median(vat)) if vat else '-'))
    print('   彎1 誤差max %.2f 舵max %+d 內min %+.1f 外min %+.1f | C1e+1m 外min %+.1f | 直線油門最高 %.0f'%(max(r['e'] for r in c1),max(r['st'] for r in c1),mn(c1,'clin'),mn(c1,'clout'),mn(ex,'clout'),max(r['pwm'] for r in rows if r['D']<dc-1.8)))
    d2=anc.get('C2s')
    if d2:
        z2=collections.OrderedDict()
        for r in rows:
            if r['zone'] and d2-3<r['D']<d2+0.5: z2.setdefault(r['zone'],[]).append(r['D']-d2)
        v2=[r['v'] for r in rows if r['v'] and abs(r['D']-d2)<0.25]
        print('   彎2 入口前作用的區:',{k:(round(v[0],2),round(v[-1],2)) for k,v in z2.items()},'C2s 車速',('%.2f'%np.median(v2)) if v2 else '-')
