"""curve-4 passes: one row per frame from the LAND8 anchor to the main straight. -> list of dict(run, lap, arrays)."""
import numpy as np, lib
from lib import g


def passes(runs=None):
    out = []
    for r in runs or lib.RUNS:
        Q = lib.load(r); t, gz, enc = lib.arrays(Q); v = lib.speed(t, enc)
        i = 0; n = len(Q)
        while i < n:
            lp = Q[i].get('lap_pos') or {}
            if lp.get('anchor') != 'LAND8': i += 1; continue
            i0 = i; e0 = None; rows = []
            while i < n:
                p = Q[i]; lp = p.get('lap_pos') or {}
                if lp.get('anchor') not in ('LAND8', 'C4e'): break
                xl = lp.get('since_anchor_m')
                if lp.get('anchor') == 'C4e':
                    if xl is not None and xl > 3.0: break
                f4 = p.get('lap_ff4') or {}; bd = f4.get('band') or {}; al = p.get('arc_lat') or {}; ok = bool(al.get('ok'))
                if e0 is None and lp.get('anchor') == 'LAND8' and np.isfinite(enc[i]): e0 = enc[i] - (xl or 0) / lib.M_PER_COUNT
                xe = (enc[i] - e0) * lib.M_PER_COUNT if e0 is not None and np.isfinite(enc[i]) else np.nan
                rows.append(dict(i=i, t=t[i], xe=xe, x4=g(f4, 'x_m'), tb=g(bd, 'turned_deg'), gz=gz[i], v=v[i], err=p.get('image_error') if p.get('image_error') is not None else np.nan,
                                 valid=bool(p.get('valid')), far=g(p.get('turn_preview') or {}, 'far_projected_error'), cerr=p.get('error') if p.get('error') is not None else np.nan, steer=(p.get('steer_us') or np.nan) - lib.CENTRE, pid=g(f4, 'pid_steer_us') - lib.CENTRE, des=(p.get('desired_us') or np.nan) - lib.CENTRE,
                                 bstate=bd.get('state'), kap=g(bd, 'kappa'), prof=g(bd, 'prof_us') - lib.CENTRE, ec=g(bd, 'e_c'), eus=g(bd, 'e_us'), gzus=g(bd, 'gz_us'), btgt=g(bd, 'target_us') - lib.CENTRE,
                                 btop=g(bd, 'top_us') - lib.CENTRE, f4state=f4.get('state'), a=g(f4, 'a'), kx=g(f4, 'kappa'), gg=g(f4, 'g'), v4=g(f4, 'v'),
                                 y=al.get('y_cm') if ok else np.nan, cin=al.get('clr_in_cm') if ok else np.nan, cout=al.get('clr_out_cm') if ok else np.nan, kpath=al.get('kap_dpm') if ok else np.nan, psi=al.get('psi_deg') if ok and al.get('psi_deg') is not None else np.nan, yf=al.get('y_front_cm') if ok and al.get('y_front_cm') is not None else np.nan,
                                 pwm=p.get('pwm') or 0, owner=p.get('owner'), I=p.get('i') if p.get('i') is not None else np.nan, cm=g(p.get('curve_map') or {}, 'reason', default=None), anchor=lp.get('anchor'), xl=xl if xl is not None else np.nan,
                                 land4=g(p.get('land4') or {}, 'state', default=None)))
                i += 1
            if len(rows) > 100:
                A = {k: np.array([q[k] for q in rows], dtype=object if k in ('bstate', 'f4state', 'owner', 'cm', 'anchor', 'land4') else float) for k in rows[0]}
                out.append(dict(run=r, lap=Q[i0]['_lap'], A=A))
    return out


if __name__ == '__main__':
    P = passes(); print(len(P), 'passes')
    for q in P:
        A = q['A']; m = (A['xe'] > 1.5) & (A['xe'] < 9)
        print(q['run'], q['lap'], 'n', len(A['t']), 'x4-xe med %.2f' % np.nanmedian(A['x4'] - A['xe']), 'cin min %+.1f cout min %+.1f' % (np.nanmin(A['cin'][m]), np.nanmin(A['cout'][m])),
              'err 1.5-2.25 %+.2f 2.25-3 %+.2f max %.2f' % (np.nanmedian(A['err'][(A['xe'] > 1.5) & (A['xe'] < 2.25)]), np.nanmedian(A['err'][(A['xe'] > 2.25) & (A['xe'] < 3)]), np.nanmax(A['err'][m])),
              'steer sd %.0f' % np.nanstd(A['steer'][(A['xe'] > 1.5) & (A['xe'] < 8)]), 'tb end %.0f' % np.nanmax(A['tb']))
