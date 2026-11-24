"""c3refit: curve-3 passes with the x-band log and a CONTINUOUS distance.
x   = odometer distance since C3s as the controller saw it (lap_pos odo: a 5 Hz staircase, steps ~0.2-0.25 m)
xc  = the same odometer, linearly interpolated in time between the encoder updates (offline, non-causal) = where the car really was
Runs: 6f61 (x-band shadow), c6f3 (x-band owns), d60e / 84f2 (no x-band, no trim), ccf1 (trim 30)."""
import os, tempfile
from pathlib import Path
import numpy as np
os.environ.setdefault('C34_CACHE', str(Path(os.environ.get('C3R_SCRATCH', Path(tempfile.gettempdir()) / 'c3refit')) / 'cache'))
import lib
from lib import g
STR = ('owner', 'a3state', 'topstate', 'seg')
C = lib.CENTRE


def passes(runs):
    out = []
    for r in runs:
        Q = lib.load(r); t, gz, enc = lib.arrays(Q); v = lib.speed(t, enc)
        odo = np.array([g(p.get('lap_pos') or {}, 'odo_m') for p in Q], float)
        # continuous odometer: value at each change instant, linear in between
        ch = np.where(np.diff(odo) > 1e-6)[0] + 1
        odc = np.interp(t, t[ch], odo[ch]) if len(ch) > 3 else odo.copy()
        gzf = gz.copy()
        for k in range(1, len(gzf)):
            if not np.isfinite(gzf[k]): gzf[k] = gzf[k - 1]          # lap_pos holds the last gyro value on frames without one
        gzf = np.nan_to_num(gzf); yawi = np.concatenate([[0.], np.cumsum((gzf[1:] + gzf[:-1]) / 2 * np.diff(t))])   # lap_pos-style heading (deg, left +)
        anc = np.array([(p.get('lap_pos') or {}).get('anchor') for p in Q], dtype=object); sin = np.array([g(p.get('lap_pos') or {}, 'since_anchor_m') for p in Q], float)
        i = 0; n = len(Q)
        while i < n:
            lp = Q[i].get('lap_pos') or {}
            if lp.get('anchor') != 'C3s': i += 1; continue
            i0 = i; rows = []; d0 = lp.get('odo_m') - lp.get('since_anchor_m'); j = max(0, i0 - 90)
            while j < n:
                p = Q[j]; lp = p.get('lap_pos') or {}
                if j >= i0 and lp.get('anchor') not in ('C3s', 'C3e'): break
                x = odo[j] - d0
                if x > 8.2: break
                cf = p.get('curve_ff') or {}; a3 = p.get('arc2_c3') or {}; tp = a3.get('top') or {}; al = p.get('arc_lat') or {}; ok = bool(al.get('ok')); yi = p.get('yaw_inner') or {}; xb = a3.get('xband') or {}
                rows.append(dict(i=j, t=t[j], x=x, xc=odc[j] - d0, gz=gz[j], v=v[j], err=p.get('image_error') if p.get('image_error') is not None else np.nan, valid=float(bool(p.get('valid'))),
                                 steer=(p.get('steer_us') or np.nan) - C, sent=g(p.get('play_comp') or {}, 'steer_us') - C, des=(p.get('desired_us') or np.nan) - C, P=p.get('p') or 0., I=p.get('i') or 0., D=p.get('d') or 0.,
                                 cff_ap=float(bool(cf.get('applied'))), cff_floor=g(cf, 'floor_us') - C, cff_in=g(cf, 'pid_steer_us') - C, cff_out=g(cf, 'steer_us') - C,
                                 a3_in=g(a3, 'pid_steer_us') - C, a3_out=g(a3, 'steer_us') - C, a3_floor=g(a3, 'floor_us') - C, a3_a=g(a3, 'a'), a3state=a3.get('state'), turned=g(a3, 'turned_deg'), gtab=g(a3, 'g'),
                                 xb_x=g(xb, 'x_m'), xb_w=g(xb, 'w'), xb_own=g(xb, 'own'), xb_kap=g(xb, 'kappa'), xb_prof=g(xb, 'prof_us') - C, xb_ec=g(xb, 'e_c'), xb_e=g(xb, 'e_us'), xb_gz=g(xb, 'gz_us'),
                                 xb_tgt=g(xb, 'target_us') - C, xb_in=g(xb, 'in_us') - C, xb_out=g(xb, 'out_us') - C,
                                 topstate=tp.get('state'), top_us=g(tp, 'top_us') - C, yi_out=g(yi, 'steer_us') - C,
                                 y=al.get('y_cm') if ok else np.nan, cin=al.get('clr_in_cm') if ok else np.nan, cout=al.get('clr_out_cm') if ok else np.nan, psi=al.get('psi_deg') if ok and al.get('psi_deg') is not None else np.nan,
                                 kpath=al.get('kap_dpm') if ok and al.get('kap_dpm') is not None else np.nan, pwm=p.get('pwm') or 0., owner=p.get('owner'), seg=lp.get('seg'),
                                 pitch=g(p.get('imu_boost') or {}, 'pitch_rel_deg')))
                j += 1
            i = max(j, i0 + 1)
            if len(rows) > 120:
                A = {k: np.array([q[k] for q in rows], dtype=object if k in STR else float) for k in rows[0]}
                gzz = np.nan_to_num(A['gz']); h = -np.concatenate([[0], np.cumsum((gzz[1:] + gzz[:-1]) / 2 * np.diff(A['t']))])
                A['h'] = h - h[int(np.argmin(np.abs(A['xc'])))]   # car heading since the C3s crossing (right +), gyro integral
                A['curv'] = -A['gz'] / np.where(A['v'] > 0.3, A['v'], np.nan)
                # H = heading turned right of the circle-straight plateau (lap_pos base = median heading over C2e + 3.5 .. 6.5 m; C3s is its -10 deg crossing)
                mb = (anc[max(0, i0 - 600):i0] == 'C2e') & (sin[max(0, i0 - 600):i0] >= 3.5) & (sin[max(0, i0 - 600):i0] <= 6.5)
                base = np.median(yawi[max(0, i0 - 600):i0][mb]) if mb.sum() > 10 else np.nan
                A['H'] = base - yawi[A['i'].astype(int)]
                out.append(dict(run=r, lap=Q[i0]['_lap'], A=A))
    return out


def binmed(P, key, edges, xk='xc', fn=np.nanmedian):
    r = []
    for a, b in zip(edges[:-1], edges[1:]):
        vals = np.concatenate([q['A'][key][(q['A'][xk] >= a) & (q['A'][xk] < b)] for q in P]) if P else np.array([])
        vals = vals.astype(float); r.append(fn(vals) if np.isfinite(vals).any() else np.nan)
    return np.array(r)


if __name__ == '__main__':
    import sys
    for r in sys.argv[1:] or ['6f61', 'c6f3', 'd60e', '84f2', 'ccf1']:
        P = passes([r]); print(r, len(P), 'passes', [(q['lap'], len(q['A']['x']), round(float(np.nanmax(q['A']['x'])), 2)) for q in P])
