"""c3front-20261121: per-frame table of curve 3 (from C3s-2 m to the ramp) for the allowed runs. Read-only on the logs.
x = metres after the C3s landmark, from lap_pos.odo_m interpolated in time between its 5 Hz steps (odo_m - since_anchor_m = crossing)."""
import json, sys
from pathlib import Path
import numpy as np
EXP = Path(__file__).resolve().parents[3]
RUNS = ['run-e1b4-20261121', 'run-b93e-20261121', 'run-9934-20261121']


def load(run):
    F = []
    for l in open(str(EXP / run / 'pid.jsonl'), encoding='utf8'):
        if '"kind": "sample"' not in l[:40]: continue
        x = json.loads(l); p = x.get('proposal') or {}
        if p.get('t') is None: continue
        v = x.get('vision') or {}; pd = v.get('path_diagnostic') or {}
        F.append((p, pd.get('arc_fit') or {}, v))
    return F


def laps(run):
    """list of dict(run, lap, rows) ; rows = per-frame dicts with x"""
    F = load(run); T = np.array([p['t'] for p, _, _ in F])
    odo = np.array([(p.get('lap_pos') or {}).get('odo_m') if (p.get('lap_pos') or {}).get('odo_m') is not None else np.nan for p, _, _ in F], float)
    # odometer steps at 5 Hz: use the frames where it changes as knots
    k = [i for i in range(1, len(F)) if not np.isnan(odo[i]) and odo[i] != odo[i - 1]]
    od = np.interp(T, T[k], odo[k]) if len(k) > 2 else odo
    out = {}
    last = max(((p.get('lap_pos') or {}).get('lap') or 0) for p, _, _ in F)
    cross = {}
    for i, (p, af, v) in enumerate(F):
        lp = p.get('lap_pos') or {}
        if lp.get('anchor') == 'C3s' and lp.get('since_anchor_m') is not None and lp.get('lap') not in cross:
            cross[lp['lap']] = lp['odo_m'] - lp['since_anchor_m']
    for lap, d0 in cross.items():
        if lap is None or lap < 1 or lap >= last: continue
        rows = []
        for i, (p, af, v) in enumerate(F):
            x = od[i] - d0
            if not -2.5 <= x <= 7.5: continue
            lp = p.get('lap_pos') or {}
            if lp.get('lap') not in (lap, lap - 1): continue
            inp = p.get('inputs') or {}; yi = p.get('yaw_inner') or {}; sa = p.get('speed_adapt') or {}; al = p.get('arc_lat') or {}; cf = p.get('curve_ff') or {}
            lf = p.get('lap_ff') or {}; cm = p.get('curve_map') or {}; bz = p.get('brake_zones') or {}; ls = p.get('lap_speed') or {}
            L = af.get('L') or {}
            rows.append(dict(i=i, t=p['t'], x=x, gz=inp.get('gz'), e=p.get('image_error'), ce=p.get('error'), P=p.get('p'), I=p.get('i'), D=p.get('d'),
                             desired=p.get('desired_us'), steer=p.get('steer_us'), sent=inp.get('steer_cmd'), pwm=p.get('pwm'), owner=p.get('owner'),
                             yi_u=yi.get('u_us'), yi_gz_lp=yi.get('gz_lp'), yi_target=yi.get('target_dps'), yi_reason=yi.get('reason'), yi_floor=yi.get('curve_floor'),
                             sa_reason=sa.get('reason'), sa_level=sa.get('level'), sa_pwm=sa.get('pwm'), sa_target=sa.get('target_pwm'), derr=sa.get('derr'),
                             bz_cap=bz.get('cap_pwm'), bz_zone=bz.get('zone'), ls=ls.get('pwm'), ls_zone=(ls.get('zone') or '')[:14] if ls else None,
                             af_ok=af.get('ok'), af_k=af.get('kappa_dpm'), af_z=(L.get('z_cm') or [None, None]), af_psi=L.get('psi_car_deg'), af_psi_near=L.get('psi_near_deg'), af_y=af.get('y_l_cm'),
                             al_kap=al.get('kap_dpm'), al_y=al.get('y_cm'), al_psi=al.get('psi_deg'), al_clr_in=al.get('clr_in_cm'), al_ok=al.get('ok'), al_v=al.get('v'),
                             cff=cf.get('reason') if cf else None, cff_floor=cf.get('floor_us') if cf else None, lff=lf.get('reason') if lf else None,
                             cm_reason=cm.get('reason'), cm_idx=cm.get('idx'), cm_turned=cm.get('turned_deg'), seg=lp.get('seg'), anchor=lp.get('anchor'), since=lp.get('since_anchor_m'),
                             c2e=lp.get('c2e_m'), odo=od[i], valid=p.get('valid'), enc=inp.get('enc_b'), center=(p.get('single_centre') or {}).get('center_us')))
        if len(rows) > 60: out[(run, lap)] = rows
    return out


def all_laps(runs=RUNS):
    o = {}
    for r in runs: o.update(laps(r))
    return o


def speed(rows):
    """m/s per row from the interpolated odometer over +-0.25 s"""
    t = np.array([r['t'] for r in rows]); x = np.array([r['x'] for r in rows]); v = np.zeros(len(rows))
    for i in range(len(rows)):
        a = np.searchsorted(t, t[i] - .25); b = min(len(rows) - 1, np.searchsorted(t, t[i] + .25))
        v[i] = (x[b] - x[a]) / max(1e-3, t[b] - t[a])
    return v


if __name__ == '__main__':
    A = all_laps()
    for k, rows in A.items(): print(k, len(rows), 'x %.2f..%.2f' % (rows[0]['x'], rows[-1]['x']), 'c2e at C3s:', [r['c2e'] for r in rows if r['anchor'] == 'C3s'][:1], [r['since'] for r in rows if r['anchor'] == 'C3s'][:1])
