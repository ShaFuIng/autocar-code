"""lap from the start, curve 1: module-level open loop of the patched lap_ff.py with on/c1_step3_lap1/lap_ff.json on the logged lap-1 passes.
(The replay harness cannot do this lap: its reconstructed absolute IMU heading has no reference before the first upper section; here the IMU heading is the integrated logged gyro.)
Inputs per frame = the logged lap_pos state, near / far error, gyro; speed = encoder slope; chain command = the logged final pre-play command (only used while the band ramps in / fades out).
single_centre is applied as on the car (c_eff = centre).   python lap1_open.py [runs]"""
import sys, os, json, importlib.util, numpy as np, lib
from lib import g
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(lib.EXP / 'orin-web-snapshot-20261122'))
sp = importlib.util.spec_from_file_location('lap_ff_new', str(HERE.parent / 'candidate' / 'lap_ff.py')); LF = importlib.util.module_from_spec(sp); sp.loader.exec_module(LF)
CFG = HERE.parent / 'on' / 'c1_step3_lap1' / 'lap_ff.json'
runs = sys.argv[1:] or lib.MAIN + lib.MORE + lib.SHORT
C = 1475.
print('lap from the start with %s + candidate/lap_ff.py (open loop: the car reaction is not modelled)' % CFG.parent.name)
print(' run  | trigger: D m  tb   ref n | band frames | pre median by tb 20-45 / 45-100 / 100-125 / 125-150: car | band | late peak pre (last 1.2 s before C1e) car -> band | 0..1 s after C1e: pre min/max car -> band | end')
rows = []
for run in runs:
    try: Q = lib.load(run)
    except Exception: continue
    c = LF.load_config(CFG); c['c_eff_us'] = C   # single_centre on the car
    L = LF.LapFF(c)
    t = np.array([p['t'] for p in Q]); gz = np.array([g(p, 'inputs', 'gz') for p in Q], float); enc = np.array([g(p, 'inputs', 'enc_b') for p in Q], float); v = lib.speed(t, enc)
    yaw = np.concatenate([[0], np.cumsum(np.nan_to_num(gz[1:]) * np.diff(t))])
    out = []; first = None; ke = None
    for i, p in enumerate(Q):
        lpz = p.get('lap_pos') or {}
        if lpz.get('lap') is None: out.append(None); continue
        if lpz.get('lap', 9) > 0 and lpz.get('anchor') not in ('C1s', 'C1e'):
            if ke is not None and t[i] - t[ke] > 2.6: break
        an = lpz.get('anchor'); since = lpz.get('since_anchor_m'); odo = lpz.get('odo_m')
        lp = dict(lap=lpz.get('lap'), anchor=an, anchor_d=None if an is None or since is None else round(odo - since, 3), since_anchor_m=since, lost=bool(lpz.get('lost')), odo_m=odo, yaw_imu=float(yaw[i]), gz_imu=float(gz[i]) if np.isfinite(gz[i]) else None)
        if ke is None and an == 'C1e': ke = i
        pre = g(p, 'steer_us', default=C); near = p.get('image_error'); far = g(p, 'turn_preview', 'far_projected_error', default=None)
        if not p.get('valid') or near is None: out.append(None); continue
        st, inf = L.step(lp, far, near, True, float(v[i]) if np.isfinite(v[i]) else None, 0., float(yaw[i]), pre, C, 780., 2050., True)
        if inf and inf.get('curve') == 'c1' and first is None: first = (i, inf, lp)
        out.append((st, inf, pre))
    if first is None or ke is None: print(' %s | not triggered' % run); continue
    i0, inf0, lp0 = first; ks = [i for i, o in enumerate(out) if o and o[1] and o[1].get('curve') == 'c1' and o[1].get('band')]
    tb = {i: out[i][1]['band']['turned_deg'] for i in ks}
    def med(a, b, k):
        x = [out[i][k] - C for i in ks if a <= tb[i] < b]; return np.median(x) if x else np.nan
    A = [i for i in range(len(out)) if out[i] and 0 < t[ke] - t[i] <= 1.2]; B = [i for i in range(len(out)) if out[i] and 0 <= t[i] - t[ke] <= 1.0]
    R = L.pref.get('c1'); endw = [out[i][1].get('end') for i in ks if out[i][1].get('end')]
    r = dict(D=lp0['odo_m'], tb=inf0['band']['turned_deg'], n=len(R[1]) if R else 0, pk_old=max(out[i][2] for i in A) - C, pk_new=max(out[i][0] for i in A) - C,
             mn_old=min(out[i][2] for i in B) - C, mx_old=max(out[i][2] for i in B) - C, mn_new=min(out[i][0] for i in B) - C, mx_new=max(out[i][0] for i in B) - C)
    rows.append(r)
    print(' %s | %5.2f %+5.1f %3d | %4d | %+4.0f %+4.0f %+4.0f %+4.0f | %+4.0f %+4.0f %+4.0f %+4.0f | %+4.0f -> %+4.0f | %+4.0f/%+4.0f -> %+4.0f/%+4.0f | %s' % (
        run, r['D'], r['tb'], r['n'], len(ks), med(20, 45, 2), med(45, 100, 2), med(100, 125, 2), med(125, 150, 2), med(20, 45, 0), med(45, 100, 0), med(100, 125, 0), med(125, 150, 0),
        r['pk_old'], r['pk_new'], r['mn_old'], r['mx_old'], r['mn_new'], r['mx_new'], endw[0] if endw else '-'))
print('\n%d of %d lap-1 passes triggered | trigger odometer %.2f..%.2f m (median %.2f) | band heading at the trigger %+.1f..%+.1f deg | reference frames %d..%d | late peak pre: car median %+.0f max %+.0f -> band median %+.0f max %+.0f' % (
    len(rows), len(runs), min(r['D'] for r in rows), max(r['D'] for r in rows), np.median([r['D'] for r in rows]), min(r['tb'] for r in rows), max(r['tb'] for r in rows), min(r['n'] for r in rows), max(r['n'] for r in rows),
    np.median([r['pk_old'] for r in rows]), max(r['pk_old'] for r in rows), np.median([r['pk_new'] for r in rows]), max(r['pk_new'] for r in rows)))
