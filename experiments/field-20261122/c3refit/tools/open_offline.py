"""proof C2b: what the candidate band would send on the car's own logged frames (open loop), computed with the CANDIDATE CODE itself:
candidate/lap_ff.py LapFF4._xb_tick / _xb2 are called frame by frame with the logged inputs of each curve-3 pass
  distance the band saw (arc2_c3.x_m, the 5 Hz staircase), camera near error, wheel speed, gain g, gyro rate, clock t, arc_lat y of the previous frame,
  heading since the C3s encoder step (gyro integral of the log; on the car = turned_deg + the step offset), command entering the band.
This replaces the harness replay for the c3 numbers: in the harness the rebuilt IMU heading is wrong on part of the curve-3 passes (see C2_open_loop.txt, last lines).
Open loop = the car's path is the logged one (chain-driven in 6f61 / d60e, r1-band-driven in c6f3); it shows the size and smoothness of the command, not where the car would go.
usage: python open_offline.py <step arc2.json> [...]"""
import importlib.util, json, sys, numpy as np, p3
from pathlib import Path
ROOT = p3.lib.HERE.parent; sys.path.insert(0, str(p3.lib.EXP / 'orin-web-snapshot-20261122')); sys.dont_write_bytecode = True
sp = importlib.util.spec_from_file_location('lap_ff_new', ROOT / 'candidate' / 'lap_ff.py'); NEW = importlib.util.module_from_spec(sp); sp.loader.exec_module(NEW)
LFF = p3.lib.HERE / 'base' / 'lap_ff.car_052be3dc.json'
E = np.arange(0.6, 4.51, 0.3)


def drive(arc2, q):
    A = q['A']; F = NEW.LapFF4(NEW.load_block(arc2['c3'], NEW.load_config(LFF)), 'c3'); act = dict(d_land=0., h_off=0.); rows = []; alp = None
    for i in range(len(A['t'])):
        xs = A['xb_x'][i] if np.isfinite(A['xb_x'][i]) else (A['x'][i - 1] if i else np.nan)
        lp = dict(odo_m=float(xs) if np.isfinite(xs) else None, gz_imu=float(A['gz'][i]) if np.isfinite(A['gz'][i]) else None, al=alp)
        F._xb_tick(lp, float(A['t'][i]))
        cin = A['xb_in'][i] if np.isfinite(A['xb_in'][i]) else A['a3_out'][i]
        if np.isfinite(xs) and xs >= 0.5 and np.isfinite(cin) and np.isfinite(A['err'][i]) and np.isfinite(A['gtab'][i]):
            v = A['v'][i] if np.isfinite(A['v'][i]) else 1.0
            r = F._xb2(act, lp, float(xs), float(A['err'][i]), True, float(np.clip(v, 0.6, 1.4)), float(A['gtab'][i]), 1475. + float(cin), 1475., 2050., float(A['t'][i]), float(A['h'][i]))
            if r is not None:
                o = r['info']; rows.append((A['xc'][i], o['x_m'], o['target_us'] - 1475., o['prof_us'] - 1475., o['h_us'], o['y_us'], o['e_us'], o['gz_us'], o['own'], A['steer'][i], A['xb_tgt'][i], (o['h_deg'] - o['h_ref']) if o['h_deg'] is not None else np.nan,
                                    o['y_cm'] if o['y_cm'] is not None else np.nan, o['raw_us'] - 1475., i, cin))
        alp = (float(A['t'][i]), float(A['y'][i]) if np.isfinite(A['y'][i]) else None)
    return np.array(rows, float)


for path in sys.argv[1:]:
    arc2 = json.load(open(path, encoding='utf8')); X = arc2['c3']['xband']
    print('=' * 20, Path(path).parent.name, '(k_e %s k_h %s k_y %s rate %s cap +%s)' % (X['k_e_us'], X['k_h_us'], X.get('k_y_us'), X['rate_us_s'], X['cap_rel_us']))
    for run in ('6f61', 'c6f3', 'd60e'):
        R = [drive(arc2, q) for q in p3.passes([run])]; R = [r for r in R if len(r) > 30]; Z = np.concatenate(R)
        print('%s (%d passes, %d band frames): %s' % (run, len(R), len(Z), {'6f61': 'the car was driven by the chain (r1 band shadow)', 'c6f3': 'the car was driven by the r1 band', 'd60e': 'the car was driven by the chain (no band)'}[run]))
        print('   x (dead-reckoned)  n | car cmd med | new target med = feed-forward + heading + position + camera + gyro | heading - plan (deg) | arc_lat y | band share | r1 target med (log)')
        for a in E[:-1]:
            m = (Z[:, 1] >= a) & (Z[:, 1] < a + 0.3)
            if m.sum() < 3: continue
            md = lambda c: np.nanmedian(Z[m, c]) if np.isfinite(Z[m, c]).any() else np.nan
            print('      %.1f-%.1f  %4d | %+5.0f | %+5.0f = %+5.0f %+4.0f %+4.0f %+4.0f %+4.0f | %+5.1f | %+5.1f | %.2f | %+5.0f' % (a, a + 0.3, m.sum(), md(9), md(2), md(3), md(4), md(5), md(6), md(7), md(11), md(12), np.mean(Z[m, 8]), md(10)))
        st = lambda c: np.concatenate([np.abs(np.diff(r[:, c]))[(np.diff(r[:, 14]) == 1) & (r[1:, 1] >= 0.9) & (r[1:, 1] < 4.05)] for r in R])
        own = (Z[:, 1] >= 0.9) & (Z[:, 1] < 4.05)
        r1 = st(10); r1 = r1[np.isfinite(r1)]
        print('   0.9-4.05 m frame-to-frame step: new target median %.0f P95 %.0f max %.0f us | car command median %.0f P95 %.0f max %.0f us%s' % (np.median(st(2)), np.percentile(st(2), 95), st(2).max(), np.median(st(9)), np.percentile(st(9), 95), st(9).max(),
              (' | r1 target (log) median %.0f P95 %.0f max %.0f us' % (np.median(r1), np.percentile(r1, 95), r1.max())) if len(r1) else ''))
        print('   0.9-4.05 m range: new target %+.0f..%+.0f us (car command %+.0f..%+.0f) | frames at the cap %.1f %%, at the floor %.1f %% | frames with band share < 1 (near error > 0.92): %.1f %% | heading term at its limit %.1f %%, position term at its limit %.1f %% | no arc_lat value %.1f %%' % (
            Z[own, 2].min(), Z[own, 2].max(), np.nanmin(Z[own, 9]), np.nanmax(Z[own, 9]), 100 * np.mean(Z[own, 13] >= X['cap_rel_us'] - 0.5), 100 * np.mean(Z[own, 13] <= -X['floor_below_us'] + 0.5), 100 * np.mean(Z[own, 8] < 0.999),
            100 * np.mean(np.abs(Z[own, 4]) >= X['h_lim_us'] - 0.5), 100 * np.mean(np.abs(Z[own, 5]) >= X.get('y_lim_us', 80.) - 0.5), 100 * np.mean(~np.isfinite(Z[own, 12]))))
        print('   distance: dead-reckoned x minus the continuous odometer: median %+.3f m, P5 %+.3f, P95 %+.3f (the staircase the r1 band used: median %+.3f, P5 %+.3f)' % (np.median(Z[:, 1] - Z[:, 0]), np.percentile(Z[:, 1] - Z[:, 0], 5), np.percentile(Z[:, 1] - Z[:, 0], 95),
              np.nanmedian(np.concatenate([q['A']['xb_x'] - q['A']['xc'] for q in p3.passes([run])])) if run != 'd60e' else np.nan, np.nanpercentile(np.concatenate([q['A']['xb_x'] - q['A']['xc'] for q in p3.passes([run])]), 5) if run != 'd60e' else np.nan))
