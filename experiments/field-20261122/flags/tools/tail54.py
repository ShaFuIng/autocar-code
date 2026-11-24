import json,sys
run=sys.argv[1]; N=int(sys.argv[2]) if len(sys.argv)>2 else 250
P='D:/OneDrive/Desktop/自走車/Orin_Interface_v1/laptop_workbench/data/experiments/'
rows=[]
for l in open(P+f'run-{run}-20261122/pid.jsonl',encoding='utf-8'):
    rows.append(l)
rows=rows[-N:]
for l in rows:
    d=json.loads(l)
    if d.get('kind')!='sample':print('KIND',d.get('kind'),str(d)[:300]);continue
    v=d['vision'];p=d['proposal'];dr=v.get('dash_route') or {}
    flags=[k for k,x in d.items() if x is True and k not in('command_dispatched',)]
    l4=p.get('land4') or {}
    ib=p.get('imu_boost') or {}
    print(f"t={p.get('t')} cap={v.get('capture_at'):.3f} con={v.get('consumed_at'):.3f} age={v.get('control_age_s'):.3f} fs={v.get('frame_seq')} inf={v.get('inference_ms')} rt={dr.get('route')} dpr={dr.get('pitch_rel_deg')} ibp={ib.get('pitch_rel_deg')} ibmode={ib.get('mode')} imut={ib.get('imu_t')} fdg={v.get('flat_dip_guard')} rel={v.get('downhill_release')} rs={p.get('reason')} own={p.get('owner')} e={p.get('image_error')} st={p.get('steer_us')} pwm={p.get('pwm')} l4={l4.get('state')},{l4.get('u_m')},{l4.get('x_m')},{l4.get('d_mouth_m')} fl={flags}")
