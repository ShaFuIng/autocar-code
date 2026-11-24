# -*- coding: utf-8 -*-
"""speedplan：逐行串流 pid.jsonl，只取 proposal 中與油門／車速／餘裕有關的欄位，存成精簡 pickle（暫存夾）。
用法：python extract.py <暫存夾> <run 資料夾> [...]
每格欄位見 rec()。車速另由 inputs.veh_ms／enc_b（MCU 狀態時間＋後輪計數）重算，避開 SpeedMeter 單格尖峰（chatter 報告）。"""
import json, sys, os, pickle
from multiprocessing import Pool

def short(name):
    if not name: return None
    n = name
    for k, v in (('curve 1 on lap 1', 'BZ_c1_lap1'), ('curve 1 after the slope', 'BZ_c1'), ('curve 2 on lap 1', 'BZ_c2_lap1'),
                 ('slope bottom', 'BZ_bottom'), ('circle-crossing guard lap 1', 'BZ_xing_lap1'), ('circle-crossing guard after', 'BZ_xing'),
                 ('crest lap 1', 'BZ_mouth_lap1'), ('crest:', 'BZ_mouth'), ('pre-turn lap 1', 'BZ_preturn_lap1'), ('pre-turn:', 'BZ_preturn'),
                 ('curve 2 entry', 'BZ_c2'), ('ramp last metre', 'BZ_ramp50'), ('curve 3 entry', 'BZ_c3entry'), ('curve 3 tail', 'BZ_c3tail'),
                 ('A main straight', 'LS_main100'), ('A diagonal', 'LS_diag100'), ('A circle', 'LS_circ100'), ('A ramp', 'LS_ramp100'),
                 ('c4 back half', 'LS_c4_58'), ('main straight', 'LS_main50'), ('diagonal', 'LS_diag50'), ('circle straight', 'LS_circ50'),
                 ('crest (RAMP', 'LS_crest27'), ('c3 front', 'LS_c3front48')):
        if n.startswith(k): return v
    return n[:30]

def rec(d):
    p = d.get('proposal') or {}
    if 't' not in p: return None
    lp = p.get('lap_pos') or {}; sa = p.get('speed_adapt') or {}; bz = p.get('brake_zones') or {}; ls = p.get('lap_speed') or {}
    inp = p.get('inputs') or {}; al = p.get('arc_lat') or {}; cm = p.get('curve_map') or {}; pc = p.get('play_comp') or {}
    sg = p.get('speed_guard') or {}; uw = p.get('upper_wide') or {}; tp = p.get('turn_pwm') or {}
    z = bz.get('zone') or {}
    cpb = sa.get('curve_pwm_by') or {}
    dh = sg.get('dh') or {}
    r = dict(t=p['t'], lap=lp.get('lap'), seg=lp.get('seg'), anc=lp.get('anchor'), sam=lp.get('since_anchor_m'), s_m=lp.get('s_m'),
             odo=lp.get('odo_m'), lost=lp.get('lost'), nxt=lp.get('next'), tonext=lp.get('to_next_m'),
             ms=inp.get('veh_ms'), enc=inp.get('enc_b'), gz=inp.get('gz'),
             pwm=p.get('pwm'), e=p.get('image_error'), valid=p.get('valid'), reason=p.get('reason'),
             st=pc.get('steer_us', p.get('steer_us')), st_raw=p.get('steer_us'), lim=p.get('limited'), owner=p.get('owner'), cseg=p.get('seg'),
             sa_r=sa.get('reason'), sa_l=sa.get('level'), sa_t=sa.get('target_pwm'), sa_p=sa.get('pwm'), sa_c=cpb.get('curve'),
             sa_land=bool(sa.get('landing')), sa_up=sa.get('upper_pwm'), sa_dhe=bool(sa.get('downhill_entry')), sa_ws=bool(sa.get('weave_slow')),
             sa_cp=sa.get('curve_pwm'), sa_ulp=sa.get('uphill_low_pwm'), sa_crest=bool(sa.get('crest_drop')), derr=sa.get('derr'),
             bz=short(z.get('name')), bz_cap=bz.get('cap_pwm'), bz_rel=z.get('rel_m'), bz_src=z.get('src'), bz_bef=bz.get('pwm_before'),
             ls=short(ls.get('zone')), ls_rel=ls.get('rel_m'), ls_blk=ls.get('blocked'), ls_p=ls.get('pwm'), ls_bef=ls.get('pwm_before'), ls_tp=bool(ls.get('taper')),
             sg_dh=bool(dh) and bool(dh.get('active') or dh.get('applied') or dh.get('state') in ('ON', 'HOLD')), sg_raw=json.dumps(dh)[:200] if dh else None,
             uw=uw.get('kind'), tpc=tp.get('cap'), tpb=tp.get('pwm_before'), llp=p.get('lane_lock_pwm'),
             pitch=(p.get('imu_boost') or {}).get('pitch_rel_deg'), ib=(p.get('imu_boost') or {}).get('applied'),
             cin=al.get('clr_in_cm'), cout=al.get('clr_out_cm'), y=al.get('y_cm'), alz=al.get('zone'), alv=al.get('v'),
             cm_r=cm.get('reason'), cm_s=cm.get('seg'), cm_i=cm.get('idx'),
             land4=(p.get('land4') or {}).get('state'), l4x=(p.get('land4') or {}).get('u_m'),
             ff4=((p.get('lap_ff4') or {}).get('band') or {}).get('state'), ffv=(p.get('lap_ff') or {}).get('v'))
    return r

def one(args):
    out, run = args
    R = []
    for l in open(os.path.join(run, 'pid.jsonl'), encoding='utf8'):
        if '"kind": "sample"' not in l[:40]: continue
        try: d = json.loads(l)
        except ValueError: continue
        x = rec(d)
        if x is not None: R.append(x)
    name = os.path.basename(run.rstrip('/\\'))[4:8]
    with open(os.path.join(out, name + '.pkl'), 'wb') as f: pickle.dump(R, f)
    return name, len(R)

if __name__ == '__main__':
    out = sys.argv[1]; runs = sys.argv[2:]
    with Pool(min(6, len(runs))) as P:
        for n, k in P.imap_unordered(one, [(out, r) for r in runs]): print(n, k, flush=True)
