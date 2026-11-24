"""c3front A: throttle zone for the curve-3 front part. Uses c4fast/tools/emul.py (the car's REAL lap_speed.py fed with the recorded
frames) + two things emul does not model, added here and labelled MODEL:
  (1) after the zone speed_adapt continues from min(zone PWM, hard_cap 50) and falls at down_rate (speed_adapt.json, 30 PWM/s) to its
      own value; recorded later caps (brake zones) still cap;
  (2) the first-order speed model of c4fast (dv/dt = (k*vss(pwm) - v)/tau), k fitted per lap on the recorded PWM.
usage: python thr.py <car lap_speed.json> <candidate lap_speed.json> [...]"""
import collections, json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / 'c4fast' / 'tools'))
import emul, analyse   # c4fast-20261121 tools
EXP = emul.EXP
RUNS = ('run-e1b4-20261121', 'run-b93e-20261121', 'run-9934-20261121')
SA = json.loads((emul.SNAP / 'speed_adapt.json').read_text(encoding='utf8'))
DOWN = float(SA['down_rate']); HARD = float(SA['hard_cap'])
ZN = 'c3 front'
import os
BRK = [float(v) for v in os.environ['C3BRAKE'].split(',')] if os.environ.get('C3BRAKE') else None   # MODEL: x0,x1,cap


def xcoord(R):
    """odometer interpolated in time, and the C3s crossing odometer per lap"""
    t = np.array([r['t'] for r in R]); odo = np.array([r['odo'] if r['odo'] is not None else np.nan for r in R], float)
    k = [i for i in range(1, len(R)) if not np.isnan(odo[i]) and odo[i] != odo[i - 1]]
    od = np.interp(t, t[k], odo[k]); cross = {}
    for r in R:
        if r['anchor'] == 'C3s' and r['since'] is not None and r['lap'] not in cross: cross[r['lap']] = r['odo'] - r['since']
    return od, cross


def inzone(r):
    z = r['new_ls'] if isinstance(r['new_ls'], dict) else None
    return z is not None and str(z.get('zone', '')).startswith(ZN)


def with_decay(A, B):
    """recorded-side PWM, module candidate PWM, candidate PWM with the MODEL decay after the zone"""
    pa = np.array([r['new_pwm'] if isinstance(r['new_pwm'], (int, float)) else np.nan for r in A])
    pb = np.array([r['new_pwm'] if isinstance(r['new_pwm'], (int, float)) else np.nan for r in B])
    out = pb.copy(); last = None
    for i, r in enumerate(B):
        if inzone(r):
            if pb[i] > pa[i] + 1e-6: last = (r['t'], min(pb[i], HARD))
            continue
        if last is not None:
            v = last[1] - DOWN * (r['t'] - last[0])
            if not np.isnan(pa[i]) and v > pa[i] + 1e-6 and r['t'] - last[0] < 2.0:
                cap = A[i].get('bz_cap'); out[i] = min(v, cap) if isinstance(cap, (int, float)) else v
            else: last = None
    return pa, pb, out


def main():
    car = json.loads(Path(sys.argv[1]).read_text(encoding='utf8'))
    for cp in sys.argv[2:]:
        cand = json.loads(Path(cp).read_text(encoding='utf8'))
        zc = next(z for z in cand['zones'] if str(z.get('name', '')).startswith(ZN))
        print('=' * 118); print('candidate', Path(cp).name, '| zone:', json.dumps({k: v for k, v in zc.items() if k != 'name'}))
        tot = collections.Counter(); G = []
        for run in RUNS:
            A = emul.run(car, str(EXP / run)); B = emul.run(cand, str(EXP / run)); assert len(A) == len(B)
            raw = [p for p, _ in emul.frames(str(EXP / run))]
            for r, p in zip(A, raw): r['bz_cap'] = (p.get('brake_zones') or {}).get('cap_pwm')
            pa, pb, pc = with_decay(A, B); od, cross = xcoord(A); t = np.array([r['t'] for r in A]); enc = [r['enc'] for r in A]
            if BRK:   # MODEL only: an extra brake-zone cap between x = BRK[0] and BRK[1] m after C3s (brake zones act after lap_speed)
                for i in range(len(A)):
                    for l in (A[i]['lap'], (A[i]['lap'] or 0) - 1):
                        if l in cross and BRK[0] <= od[i] - cross[l] <= BRK[1] and not np.isnan(pc[i]): pc[i] = min(pc[i], BRK[2]); break
            last_lap = max(r['lap'] for r in A if r['lap'] is not None)

            def xof(i):
                lap = A[i]['lap']
                for l in (lap, (lap or 0) - 1):
                    if l in cross and -3 < od[i] - cross[l] < 8: return od[i] - cross[l]
                return None
            ch = np.flatnonzero(np.abs(np.nan_to_num(pb - pa)) > 1e-6); chm = np.flatnonzero(np.abs(np.nan_to_num(pc - pa)) > 1e-6)
            xs = [xof(i) for i in ch]; xm = [xof(i) for i in chm]
            outside = sum(1 for x in xs if x is None or not 0.2 <= x <= float(zc['to_m']) + 0.30)
            outside_m = sum(1 for x in xm if x is None or not 0.2 <= x <= 4.25)
            corner = sum(1 for x in xm if x is not None and x >= 4.3)
            low = int(np.sum(pb[ch] < pa[ch] - 1e-6)) + (0 if BRK else int(np.sum(pc[chm] < pa[chm] - 1e-6)))
            tot.update(frames=len(A), changed=len(ch), outside=outside, lowered=low, changed_model=len(chm), outside_model=outside_m, corner=corner)
            print('%s: frames %d | module (real lap_speed.py): PWM changed on %d frames, outside C3s+0.2 .. zone end+0.30 m (lap_speed reads the 5 Hz lap_pos odometer: up to ~0.25 m late): %d, lowered: %d | + MODEL decay: %d frames, outside C3s+0.2..4.25 m: %d, at/after the corner (x >= 4.3): %d' % (
                run[4:8], len(A), len(ch), outside, low, len(chm), outside_m, corner))
            for lap in sorted(k for k in cross if k is not None):
                if lap < 1 or lap >= last_lap: continue
                idx = [i for i in range(len(A)) if -1.0 <= od[i] - cross[lap] <= 6.0 and A[i]['lap'] in (lap, lap - 1)]
                if len(idx) < 50: continue
                x = od[idx] - cross[lap]; tt = t[idx]
                zl = [B[i]['new_ls'] for i in idx if inzone(B[i])]
                blk = collections.Counter(z.get('blocked') for z in zl if z.get('blocked'))
                c = [n for n, i in enumerate(idx) if abs(np.nan_to_num(pc[i] - pa[i])) > 1e-6]
                vm = np.array([analyse.meas_v(t, enc, i) for i in idx]); v0 = vm[np.isfinite(vm)][0] if np.isfinite(vm).any() else 1.0
                best = min(np.arange(0.80, 1.201, 0.01), key=lambda k: np.nanmean((analyse.sim(tt, pa[idx], v0, k) - vm) ** 2))
                va = analyse.sim(tt, pa[idx], v0, best); vb = analyse.sim(tt, pc[idx], v0, best); rms = float(np.sqrt(np.nanmean((va - vm) ** 2)))
                da = np.concatenate([[0.], np.cumsum(0.5 * (va[1:] + va[:-1]) * np.diff(tt))]); db = np.concatenate([[0.], np.cumsum(0.5 * (vb[1:] + vb[:-1]) * np.diff(tt))])
                Told = tt[-1] - tt[0]; Tnew = float(np.interp(da[-1], db, tt - tt[0])) if db[-1] >= da[-1] else float('nan')

                def v_at(xq):   # same place of the track on both sides (the new side gets there earlier)
                    n = int(np.argmin(np.abs(x - xq))); return va[n], float(np.interp(da[n], db, vb))
                v32 = v_at(3.2); v43 = v_at(4.3); v48 = v_at(4.8)
                G.append((Told - Tnew, v32[1] - v32[0], v43[1] - v43[0], v48[1] - v48[0], best, rms, v43[0], v48[0], len(zl), sum(blk.values())))
                print('   lap %d: zone frames %d, blocked %s | PWM differs x %+.2f .. %+.2f (%d frames), max %.0f | MODEL k %.2f rms %.3f: time x -1..6 m %.2f -> %.2f s (%+.2f) | v at x 3.2: %.2f -> %.2f ; 4.3 (corner entry): %.2f -> %.2f ; 4.8 (corner): %.2f -> %.2f' % (
                    lap, len(zl), dict(blk), x[c[0]] if c else float('nan'), x[c[-1]] if c else float('nan'), len(c), np.nanmax(pc[idx]), best, rms, Told, Tnew, Tnew - Told,
                    v32[0], v32[1], v43[0], v43[1], v48[0], v48[1]))
        g = np.array(G)
        print('TOTAL frames %d | module: PWM changed %d, outside the window %d, lowered %d | + MODEL decay: changed %d, outside C3s+0.2..4.25 %d, at/after the corner (x >= 4.3) %d' % (
            tot['frames'], tot['changed'], tot['outside'], tot['lowered'], tot['changed_model'], tot['outside_model'], tot['corner']))
        print('MODEL (%d laps): time saved over x -1..6 m median %.2f s (min %.2f, max %.2f) | speed change at x 3.2: %+.2f ; at 4.3: %+.3f (max %+.3f) ; at 4.8: %+.3f (max %+.3f) m/s | recorded-side model speed at 4.3 / 4.8: %.2f / %.2f | zone frames blocked %d of %d | k median %.2f, rms %.3f m/s' % (
            len(g), np.median(g[:, 0]), g[:, 0].min(), g[:, 0].max(), np.median(g[:, 1]), np.median(g[:, 2]), g[:, 2].max(), np.median(g[:, 3]), g[:, 3].max(),
            np.median(g[:, 6]), np.median(g[:, 7]), int(g[:, 9].sum()), int(g[:, 8].sum()), np.median(g[:, 4]), np.median(g[:, 5])))


if __name__ == '__main__':
    main()
