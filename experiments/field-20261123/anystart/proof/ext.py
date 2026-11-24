import json,sys,os
E=r'D:/OneDrive/Desktop/自走車/Orin_Interface_v1/laptop_workbench/data/experiments'
OUT=os.path.dirname(os.path.abspath(__file__))
def g(d,*ks):
    for k in ks:
        if not isinstance(d,dict): return None
        d=d.get(k)
    return d
for r in sys.argv[1:]:
    fo=open(os.path.join(OUT,r+'.jsonl'),'w')
    for line in open(os.path.join(E,'run-%s-20261122'%r,'pid.jsonl'),encoding='utf8'):
        if '"proposal"' not in line: continue
        try: d=json.loads(line)
        except: continue
        p=d.get('proposal') or {}
        lp=p.get('lap_pos') or {}; sa=p.get('speed_adapt') or {}; bz=p.get('brake_zones') or {}; lf=p.get('lap_ff') or {}
        tp=p.get('turn_preview') or {}; cm=p.get('curve_map') or {}; inp=p.get('inputs') or {}
        o=dict(t=p.get('t'),valid=p.get('valid'),lap=lp.get('lap'),an=lp.get('anchor'),sa_m=lp.get('since_anchor_m'),odo=lp.get('odo_m'),s=lp.get('s_m'),lost=lp.get('lost'),
          far=tp.get('far_projected_error'),near=tp.get('near_error'),err=p.get('image_error'),pwm=p.get('pwm'),st=p.get('steer_us'),
          lev=sa.get('level'),rsn=sa.get('reason'),tgt=sa.get('target_pwm'),bzd=bz.get('d_m'),bza=bz.get('anchors'),bzz=(bz.get('zone') or {}).get('name','')[:30] if bz.get('zone') else None,bzc=bz.get('cap_pwm'),
          lfc=lf.get('curve'),lfs=lf.get('state'),lfk=lf.get('kind'),lfx=lf.get('x_m'),gz=inp.get('gz'),enc=inp.get('enc_b'),pr=g(p,'imu_boost','pitch_rel_deg'),
          sb=g(p,'start_bias','bias_us'),cmr=cm.get('reason'),cms=cm.get('seg'),cmi=cm.get('idx'),reason=p.get('reason'))
        fo.write(json.dumps(o)+'\n')
    fo.close(); print(r,'done')
