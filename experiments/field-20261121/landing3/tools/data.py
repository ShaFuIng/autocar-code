"""landing3-20261121: per-frame records around the downhill / landing / curve-4 entry of a run (read-only, from pid.jsonl)."""
import json, pickle, sys
from pathlib import Path
EXP = Path(__file__).resolve().parents[3]
CACHE = Path(__file__).resolve().parent / '_cache'
M = 2.22e-4

def load(run):
    CACHE.mkdir(exist_ok=True); c = CACHE / (run + '.pkl')
    if c.exists(): return pickle.load(open(c, 'rb'))
    out = []
    for l in open(str(EXP / run / 'pid.jsonl'), encoding='utf8'):
        if '"kind": "sample"' not in l[:40]: continue
        x = json.loads(l); p = x.get('proposal') or {}
        if p.get('t') is None: continue
        lp = p.get('lap_pos') or {}; i = p.get('inputs') or {}; sa = p.get('speed_adapt') or {}; bz = p.get('brake_zones') or {}; bo = p.get('bottom_owner') or {}
        cm = p.get('curve_map') or {}; al = p.get('arc_lat') or {}; ls = p.get('lap_speed') or {}; a2 = p.get('arc2_land') or {}; sg = (p.get('speed_guard') or {}).get('dh') or {}
        out.append(dict(t=p['t'], lap=lp.get('lap'), seg=lp.get('seg'), anc=lp.get('anchor'), s=lp.get('since_anchor_m'), odo=lp.get('odo_m'), lost=lp.get('lost'),
                        gz=i.get('gz'), enc=i.get('enc_b'), sent=i.get('steer_cmd'), raw=p.get('steer_us'), e=p.get('image_error'), pwm=p.get('pwm'), valid=p.get('valid'), reason=p.get('reason'),
                        owner=p.get('owner'), pitch=(p.get('imu_boost') or {}).get('pitch_rel_deg'), sa_reason=sa.get('reason'), sa_level=sa.get('level'), sa_pwm=sa.get('pwm'),
                        landing=sa.get('landing'), curve_by=(sa.get('curve_pwm_by') or {}).get('curve'), bz_cap=bz.get('cap_pwm'), bz_zone=str(bz.get('zone'))[:40] if bz.get('zone') else None,
                        bo=bool(bo), bo_phase=bo.get('phase'), bo_cap=bo.get('right_cap_us'), cm=(cm.get('seg'), cm.get('idx'), cm.get('reason')),
                        clr_out=al.get('clr_out_cm') if al.get('ok') else None, clr_in=al.get('clr_in_cm') if al.get('ok') else None, al_zone=al.get('zone'),
                        ls_zone=(ls.get('zone') or '')[:18] if ls else None, ls_blocked=ls.get('blocked') if ls else None, pulse=a2.get('pulse'), a2=bool(a2), a2_keys={k: a2[k] for k in a2 if k != 'pulse' and not isinstance(a2[k], (dict, list))},
                        guard=sg.get('applied'), guard_armed=sg.get('armed') if 'armed' in sg else sg.get('state'), i_us=p.get('i')))
    pickle.dump(out, open(c, 'wb')); return out

def landings(Q):
    """index of the first LAND8 frame of each lap (complete passes only)"""
    out = []; prev = None
    for k, q in enumerate(Q):
        if q['anc'] == 'LAND8' and prev != 'LAND8': out.append(k)
        prev = q['anc']
    return out
