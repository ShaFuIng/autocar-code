"""straightfast-20261121 common: frame loader (recorded), zone names, first-order speed model (same table / tau as c4fast/tools/analyse.py)."""
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
EXP = HERE.parents[2]
SNAP = EXP / 'orin-web-snapshot-20261121c'
RUNS = ('run-e1b4-20261121', 'run-b93e-20261121', 'run-9934-20261121')
M = 2.22e-4
ZONES = (('main', 'A main straight', 'C4e'), ('diag', 'A diagonal', 'C1e'), ('circle', 'A circle straight', 'C2e'))
SG = json.loads((SNAP / 'speed_guard.json').read_text(encoding='utf8'))['overspeed_watch']
TAB = [tuple(x) for x in SG['table']]; V50 = TAB[-1][1]
HI = [(55., 1.034 * V50), (60., 1.057 * V50), (65., 1.079 * V50), (70., 1.098 * V50), (100., 6492.)]
PW = np.array([x[0] for x in TAB] + [x[0] for x in HI]); CP = np.array([x[1] for x in TAB] + [x[1] for x in HI])
TAU_UP, TAU_DN = float(SG['tau_up_s']), float(SG['tau_down_s'])


def vss(pwm): return float(np.interp(pwm, PW, CP)) * M


def sim(t, pwm, v0, k=1.0):
    """first-order speed model: dv/dt = (k * vss(pwm) - v) / tau (tau_up when accelerating, tau_down when slowing)"""
    v = [v0]
    for i in range(1, len(t)):
        tgt = k * vss(pwm[i - 1]); dt = min(0.2, max(0., t[i] - t[i - 1])); tau = TAU_UP if tgt > v[-1] else TAU_DN
        v.append(v[-1] + (tgt - v[-1]) * (1. - np.exp(-dt / tau)))
    return np.array(v)


def zone_of(name):
    for key, pre, anc in ZONES:
        if isinstance(name, str) and name.startswith(pre): return key
    return None


def load(run):
    """recorded sample frames of a run: list of dicts (one per frame)"""
    out = []
    for l in open(str(EXP / run / 'pid.jsonl'), encoding='utf8'):
        if '"kind": "sample"' not in l[:40]: continue
        x = json.loads(l); p = x.get('proposal') or {}
        if p.get('t') is None: continue
        lp = p.get('lap_pos') or {}; sa = p.get('speed_adapt') or {}; ls = p.get('lap_speed') if isinstance(p.get('lap_speed'), dict) else None
        bz = p.get('brake_zones') or {}; i = p.get('inputs') or {}
        out.append(dict(t=p['t'], lap=lp.get('lap'), anchor=lp.get('anchor'), since=lp.get('since_anchor_m'), seg=lp.get('seg'), lost=lp.get('lost'),
                        pwm=p.get('pwm'), sa_pwm=sa.get('pwm'), sa_reason=sa.get('reason'), sa_level=sa.get('level'), weave=bool(sa.get('weave_slow')), derr=sa.get('derr'),
                        stable_s=sa.get('stable_s'), ls=ls, zone=zone_of((ls or {}).get('zone')), bz=bz.get('cap_pwm'), e=p.get('image_error'), steer=p.get('steer_us'),
                        gz=i.get('gz'), enc=i.get('enc_b'), owner=p.get('owner'), valid=p.get('valid'), reason=p.get('reason'),
                        k_gs=(p.get('gain_schedule') or {}).get('k')))
    return out


def meas_v(t, enc, i, w=0.3):
    j = int(np.searchsorted(t, t[i] - w)); k = min(len(t) - 1, int(np.searchsorted(t, t[i] + w)))
    if enc[j] is None or enc[k] is None or t[k] - t[j] < 0.3: return np.nan
    return abs(enc[k] - enc[j]) * M / (t[k] - t[j])
