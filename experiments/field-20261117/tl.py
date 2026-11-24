import json,sys
ENC=2.22e-4
r=sys.argv[1]; t0=float(sys.argv[2]); t1=float(sys.argv[3]); step=int(sys.argv[4]) if len(sys.argv)>4 else 1
Q=[]
for l in open('run-%s-20261117/pid.jsonl'%r,encoding='utf8'):
    d=json.loads(l)
    if d.get('kind')=='sample' and 't' in d['proposal']: Q.append(d)
pts=[]
for d in Q:
    q=d['proposal']; e=(q.get('inputs') or {}).get('enc_b')
    if e is not None and (not pts or e!=pts[-1][1]): pts.append((q['t'],e))
def v_at(t):
    a=[p for p in pts if t-0.6<=p[0]<=t+0.1]
    return (a[-1][1]-a[0][1])*ENC/(a[-1][0]-a[0][0]) if len(a)>1 and a[-1][0]>a[0][0] else 0
n=0
for d in Q:
    q=d['proposal']; t=q['t']
    if not (t0<=t<=t1): continue
    n+=1
    if n%step: continue
    lp=q.get('lap_pos') or {}; sa=q.get('speed_adapt') or {}; bz=(q.get('brake_zones') or {}).get('zone') or {}; cm=q.get('curve_map') or {}; v=d.get('vision') or {}
    xp=q.get('crossing_pass') or {}; xc=q.get('xp_corridor') or {}
    print('%.2f %-9s %s e%+.2f pre%5.0f sent%5.0f pwm%2.0f v%.2f gz%4.0f pit%5s | %-12s | sa %-14s %s | bz %s | cm %s %s | lp %s+%.1f lost %s | xp %s %s | src %s'%(t,(lp.get('seg') or '-')[:9],'V' if q.get('valid') else 'x',q.get('image_error') or 0,q.get('steer_us') or 0,(q.get('play_comp') or {}).get('steer_us') or 0,q.get('pwm') or 0,v_at(t),(q.get('inputs') or {}).get('gz') or 0,('%.1f'%sa['pitch_rel']) if sa.get('pitch_rel') is not None else '-',(q.get('owner') or '')[:12],str(sa.get('reason'))[:14],sa.get('level') or '',(bz.get('name') or '')[:14],cm.get('reason') or cm.get('phase'),cm.get('turned_deg'),lp.get('anchor'),lp.get('since_anchor_m') or 0,lp.get('lost'),xp.get('state') or xp.get('mode') or '',xc.get('state') or '',str(v.get('selected_source'))[:8]))
