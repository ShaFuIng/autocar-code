import json,sys
ENC=2.22e-4
for run in sys.argv[1:]:
    Q=[]
    for l in open(run+'/pid.jsonl',encoding='utf8'):
        d=json.loads(l)
        if d.get('kind')=='sample' and 't' in d['proposal']:
            v=d.get('vision') or {}; Q.append((d['proposal'],v.get('dash_route') or {}))
    pts=[]
    for p,_ in Q:
        e=(p.get('inputs') or {}).get('enc_b')
        if e is not None and (not pts or e!=pts[-1][1]): pts.append((p['t'],e))
    def v_at(t):
        a=[x for x in pts if t-0.5<=x[0]<=t+0.1]
        return (a[-1][1]-a[0][1])*ENC/(a[-1][0]-a[0][0]) if len(a)>1 else 0
    def odo(t):
        a=[x for x in pts if x[0]<=t]; return a[-1][1]*ENC if a else 0
    tops=[]; prev=None
    for p,_ in Q:
        s=(p.get('lap_pos') or {}).get('seg')
        if s=='top' and prev!='top': tops.append(p['t'])
        prev=s
    print('=====',run)
    for n,t0 in enumerate(tops):
        W=[x for x in Q if t0<=x[0]['t']<=t0+14]
        ts=te=None; pos=None; tdn=None; rows=[]; early=0
        for p,dr in W:
            lp=p.get('lap_pos') or {}; us=p.get('upper_single') or {}; one=us.get('one') or {}
            if dr.get('reason')=='TURN_TOO_EARLY': early+=1
            if ts is None and dr.get('mode')=='TURN': ts=p['t']; pos=lp.get('since_anchor_m'); vin=v_at(p['t']); y0=(p.get('inputs') or {}).get('yaw')
            if ts and te is None and dr.get('mode')!='TURN': te=p['t']; yte=us.get('yaw_deg')
            if te and tdn is None and lp.get('seg')=='downhill': tdn=p['t']
            if te and tdn is None: rows.append((p['t'],us.get('state'),one.get('phi'),one.get('X'),one.get('xm'),one.get('tgt'),us.get('e_src'),p.get('image_error'),p.get('steer_us'),(p.get('inputs') or {}).get('gz'),dr.get('reason'),(us.get('edge') or {}).get('ok'),us.get('h_target_deg'),us.get('yaw_deg'),p.get('valid')))
        if not ts or not rows: print(' pass',n+1,'no turn'); continue
        st={}
        for r in rows: st[r[1]]=st.get(r[1],0)+1
        ndash=sum(1 for r in rows if r[6]=='dash'); nedge=sum(1 for r in rows if r[11]); ninv=sum(1 for r in rows if not r[14])
        phis=[r[2] for r in rows if r[2] is not None]; Xs=[r[3] for r in rows if r[3] is not None]
        ht=[r[12] for r in rows if r[12] is not None]
        print(' pass %d: gate-blocked %d, TURN at TOP+%.2f m v_in %.2f, TURN %.2f s (odo %.2f m), after-turn %.2f s (odo %.2f m), frames %d states %s dashframes %d edge_ok %d invalid %d'%(n+1,early,pos or 0,vin,te-ts,odo(te)-odo(ts),(tdn or rows[-1][0])-te,odo(tdn or rows[-1][0])-odo(te),len(rows),st,ndash,nedge,ninv))
        print('    h_target first %.1f last %.1f (yaw at TURN start %s) ; phi first %.1f max %.1f min-after-max %.1f last %.1f ; X first %.1f min %.1f max %.1f last %.1f'%(ht[0],ht[-1],y0,phis[0],max(phis),min(phis[phis.index(max(phis)):]),phis[-1],Xs[0],min(Xs),max(Xs),Xs[-1]))
        for dt in (0.1,0.5,1.0,1.5,2.0,2.5,3.0,3.5,4.0,4.5):
            c=[r for r in rows if r[0]>=te+dt]
            if not c: break
            r=c[0]; f=lambda x,fm='%.1f':'-' if x is None else fm%x
            print('      +%.1fs %-6s phi %6s X %6s xm %6s tgt %5s src %-5s e %6s steer %5s gz %5s dash %-16s edge %s v %.2f'%(dt,r[1],f(r[2]),f(r[3]),f(r[4]),f(r[5]),r[6],f(r[7],'%.2f'),f(r[8],'%.0f'),f(r[9],'%.0f'),r[10],r[11],v_at(r[0])))
