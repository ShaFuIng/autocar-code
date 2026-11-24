"""curve-3 passes: frames while the lap_pos anchor is C3s (x = metres since C3s), plus the 60 frames before it (negative x from the odometer)."""
import numpy as np, lib
from lib import g
STR = ('owner', 'cm', 'a3state', 'topstate', 'cffby', 'seg')


def passes(runs=None):
    out = []
    for r in runs or lib.RUNS:
        Q = lib.load(r); t, gz, enc = lib.arrays(Q); v = lib.speed(t, enc)
        i = 0; n = len(Q)
        while i < n:
            lp = Q[i].get('lap_pos') or {}
            if lp.get('anchor') != 'C3s': i += 1; continue
            i0 = i; rows = []
            d0 = lp.get('odo_m') - lp.get('since_anchor_m')
            j = max(0, i0 - 60)
            while j < n:
                p = Q[j]; lp = p.get('lap_pos') or {}
                if j >= i0 and lp.get('anchor') != 'C3s': break
                x = (lp.get('odo_m') - d0) if lp.get('odo_m') is not None else np.nan
                if x > 8.0: break
                cf = p.get('curve_ff') or {}; a3 = p.get('arc2_c3') or {}; tp = a3.get('top') or {}; al = p.get('arc_lat') or {}; ok = bool(al.get('ok')); ct = p.get('curve_trim') or {}; yi = p.get('yaw_inner') or {}; cm = p.get('curve_map') or {}
                rows.append(dict(i=j, t=t[j], x=x, gz=gz[j], v=v[j], err=p.get('image_error') if p.get('image_error') is not None else np.nan, cerr=p.get('error') if p.get('error') is not None else np.nan,
                                 valid=float(bool(p.get('valid'))), steer=(p.get('steer_us') or np.nan) - lib.CENTRE, des=(p.get('desired_us') or np.nan) - lib.CENTRE, P=p.get('p') or 0., I=p.get('i') or 0., D=p.get('d') or 0.,
                                 cff_ap=float(bool(cf.get('applied'))), cff_floor=g(cf, 'floor_us') - lib.CENTRE, cff_in=g(cf, 'pid_steer_us') - lib.CENTRE, cff_side=g(cf, 'side', default=0), cffby=cf.get('by_curve'),
                                 a3_ap=float(bool(a3.get('applied'))), a3_floor=g(a3, 'floor_us') - lib.CENTRE, a3_tgt=g(a3, 'target_us') - lib.CENTRE, a3_in=g(a3, 'pid_steer_us') - lib.CENTRE, a3_a=g(a3, 'a'), a3state=a3.get('state'),
                                 a3_turned=g(a3, 'turned_deg'), a3_ahead=g(a3, 'ahead_deg'),
                                 topstate=tp.get('state'), top_us=g(tp, 'top_us') - lib.CENTRE, trim=g(ct, 'trim_us', default=0.), trim_ap=float(bool(ct.get('applied'))),
                                 yi_in=g(yi, 'pid_steer_us') - lib.CENTRE, yi_out=g(yi, 'steer_us') - lib.CENTRE, yi_floor=float(bool(yi.get('curve_floor'))), cm=cm.get('reason'), cm_turned=g(cm, 'turned_deg'),
                                 y=al.get('y_cm') if ok else np.nan, cin=al.get('clr_in_cm') if ok else np.nan, cout=al.get('clr_out_cm') if ok else np.nan, psi=al.get('psi_deg') if ok and al.get('psi_deg') is not None else np.nan,
                                 kpath=al.get('kap_dpm') if ok and al.get('kap_dpm') is not None else np.nan, pwm=p.get('pwm') or 0., owner=p.get('owner'), seg=lp.get('seg'), sent=g(p.get('play_comp') or {}, 'steer_us') - lib.CENTRE,
                                 far=g(p.get('turn_preview') or {}, 'far_projected_error'), pitch=g(p.get('imu_boost') or {}, 'pitch_rel_deg')))
                j += 1
            i = max(j, i0 + 1)
            if len(rows) > 120:
                A = {k: np.array([q[k] for q in rows], dtype=object if k in STR else float) for k in rows[0]}
                out.append(dict(run=r, lap=Q[i0]['_lap'], A=A))
    return out


if __name__ == '__main__':
    P = passes(); print(len(P), 'passes')
    for q in P:
        A = q['A']; f = lambda lo, hi, k, fn: fn(A[k][(A['x'] >= lo) & (A['x'] < hi)]) if ((A['x'] >= lo) & (A['x'] < hi)).any() else np.nan
        print(q['run'], q['lap'], 'xmax %.2f' % np.nanmax(A['x']), 'front cin min %+5.1f | corner cout min %+5.1f err max %.2f steer max %+4.0f | foot err max %.2f steer max %+4.0f cin min %+5.1f | trim %2.0f' % (
            f(0.6, 4.2, 'cin', np.nanmin), f(4.2, 5.6, 'cout', np.nanmin), f(4.2, 5.6, 'err', np.nanmax), f(4.2, 5.6, 'steer', np.nanmax), f(5.6, 7.2, 'err', np.nanmax), f(5.6, 7.2, 'steer', np.nanmax), f(5.6, 7.2, 'cin', np.nanmin),
            f(1.2, 3.8, 'trim', np.nanmax)))
