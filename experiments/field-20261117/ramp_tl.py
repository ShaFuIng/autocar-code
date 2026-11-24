import json,sys
ENC=2.22e-4
run=sys.argv[1]; which=int(sys.argv[2]); step=int(sys.argv[3]) if len(sys.argv)>3 else 3
Q=[]
for l in open(run+'/pid.jsonl',encoding='utf8'):
    d=json.loads(l)
    if d.get('kind')=='sample' and 't' in d['proposal']:
        v=d.get('vision') or {}; le=v.get('line_estimate') or {}
        Q.append((d['proposal'],v.get('selected_source'),le.get('sources') if isinstance(le,dict) else None,(v.get('junction_switch') or {}).get('reason') if isinstance(v.get('junction_switch'),dict) else None))
pts=[]
for p,*_ in Q:
    e=(p.get('inputs') or {}).get('enc_b')
    if e is not None and (not pts or e!=pts[-1][1]): pts.append((p['t'],e))
def v_at(t):
    a=[x for x in pts if t-0.5<=x[0]<=t+0.1]
    return (a[-1][1]-a[0][1])*ENC/(a[-1][0]-a[0][0]) if len(a)>1 else 0
ramps=[]; prev=None
for p,*_ in Q:
    s=(p.get('lap_pos') or {}).get('seg')
    if s=='ramp' and prev!='ramp': ramps.append([p['t'],None])
    if s=='top' and prev=='ramp' and ramps: ramps[-1][1]=p['t']
    prev=s
a,b=ramps[which]
n=0
for p,src,srcs,js in Q:
    t=p['t']
    if not (a-1.2<=t<=b+0.3): continue
    n+=1
    if n%step: continue
    yi=p.get('yaw_inner') or {}; sa=p.get('speed_adapt') or {}; lp=p.get('lap_pos') or {}; cm=p.get('curve_map') or {}; pid=p.get('pid') or {}
    f=lambda x,fm='%.0f':'-' if x is None else fm%x
    print('%6.2f %-6s e%+.2f far%6s %s pre%5.0f sent%5.0f gz%4.0f pit%5s v%.2f pw%2.0f %-12s| yi term%5s tgt%5s %s | P%5s D%5s I%4s | cm %s %s | own %-12s src %-10s %s'%(t-a,(lp.get('seg') or '-')[:6],p.get('image_error') or 0,f(p.get('far_error'),'%+.2f'),'V' if p.get('valid') else 'x',p.get('steer_us') or 0,(p.get('play_comp') or {}).get('steer_us') or 0,(p.get('inputs') or {}).get('gz') or 0,f(sa.get('pitch_rel'),'%.1f'),v_at(t),p.get('pwm') or 0,str(sa.get('reason'))[:12],f(yi.get('term_us')),f(yi.get('target_dps')),''.join(k[0] for k in ('no_hold','term_rate_limited','oppose_capped') if yi.get(k)),f(pid.get('p_us') if pid else p.get('p_us')),f(pid.get('d_us') if pid else p.get('d_us')),f(pid.get('i_us') if pid else p.get('i_us')),cm.get('phase') or cm.get('reason'),f(cm.get('turned_deg')),(p.get('owner') or '')[:12],str(src)[:10],str(srcs)[:30]))
