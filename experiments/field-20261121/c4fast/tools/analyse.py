"""c4fast-20261121: open-loop tables for the curve-4 back-half zone (uses tools/emul.py = the car's real lap_speed.py on recorded frames).
usage: python analyse.py <car lap_speed.json> <candidate lap_speed.json> [...]   (runs e1b4 / b93e / 9934 hard-wired)"""
import json, sys, collections
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import emul

EXP = emul.EXP
RUNS = ('run-e1b4-20261121', 'run-b93e-20261121', 'run-9934-20261121')
M = 2.22e-4
# PWM -> steady encoder speed (counts/s) on the flat: the car's own table (speed_guard.json overspeed_watch.table, <= 50) and the
# measured ratios above 50 (AGENTS.md: wheels-up 55/60/65/70 % = 1.034/1.057/1.079/1.098 x the 50 % speed; 100 % on the ground 6492 cps)
SG = json.loads((emul.SNAP / 'speed_guard.json').read_text(encoding='utf8'))['overspeed_watch']
TAB = [tuple(x) for x in SG['table']]
V50 = TAB[-1][1]
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


def meas_v(t, enc, i, w=0.3):
    j = int(np.searchsorted(t, t[i] - w)); k = min(len(t) - 1, int(np.searchsorted(t, t[i] + w)))
    if enc[j] is None or enc[k] is None or t[k] - t[j] < 0.3: return np.nan
    return abs(enc[k] - enc[j]) * M / (t[k] - t[j])


def pos(r):
    return '%s%+.2f' % (r['anchor'], r['since']) if r['anchor'] is not None and r['since'] is not None else '?'


def laps_windows(R):
    """per lap: frame indices from LAND8+2.0 m to C4e+3.0 m"""
    out = collections.OrderedDict()
    for i, r in enumerate(R):
        if r['lap'] is None or r['since'] is None: continue
        if (r['anchor'] == 'LAND8' and r['since'] >= 2.0) or (r['anchor'] == 'C4e' and r['since'] <= 3.0): out.setdefault(r['lap'], []).append(i)
    return out


def main():
    car = json.loads(Path(sys.argv[1]).read_text(encoding='utf8'))
    for cand_path in sys.argv[2:]:
        cand = json.loads(Path(cand_path).read_text(encoding='utf8'))
        print('=' * 110); print('candidate', Path(cand_path).name)
        tot = collections.Counter(); gains = []
        for run in RUNS:
            A = emul.run(car, str(EXP / run)); B = emul.run(cand, str(EXP / run)); assert len(A) == len(B)
            last_lap = max(r['lap'] for r in A if r['lap'] is not None)
            t = np.array([r['t'] for r in A]); enc = [r['enc'] for r in A]
            pa = np.array([r['new_pwm'] if isinstance(r['new_pwm'], (int, float)) else np.nan for r in A]); pb = np.array([r['new_pwm'] if isinstance(r['new_pwm'], (int, float)) else np.nan for r in B])
            ch = np.flatnonzero(np.abs(np.nan_to_num(pb - pa)) > 1e-6)
            inwin = [i for i in ch if (A[i]['anchor'] == 'LAND8' and A[i]['since'] is not None and A[i]['since'] >= 3.0) or (A[i]['anchor'] == 'C4e' and A[i]['since'] is not None and A[i]['since'] <= 3.2)]
            lower = [i for i in ch if pb[i] < pa[i] - 1e-6]
            tot['frames'] += len(A); tot['changed'] += len(ch); tot['outside'] += len(ch) - len(inwin); tot['lower'] += len(lower)
            print('%s: frames %d | PWM changed on %d frames, outside LAND8+3.0 .. C4e+3.2: %d, lowered: %d | steering: not an output of this layer (0 frames)' % (run[4:8], len(A), len(ch), len(ch) - len(inwin), len(lower)))
            for lap, idx in laps_windows(A).items():
                if lap == last_lap and A[idx[-1]]['anchor'] != 'C4e': continue
                c = [i for i in idx if abs(np.nan_to_num(pb[i] - pa[i])) > 1e-6]
                if not c: print('   lap %d: no change' % lap); continue
                zl = [B[i]['new_ls'] for i in idx if isinstance(B[i]['new_ls'], dict) and str(B[i]['new_ls'].get('zone', '')).startswith('c4')]
                blk = collections.Counter(z.get('blocked') for z in zl)
                seg = [i for i in idx if A[i]['anchor'] == 'LAND8' and A[i]['since'] >= 3.4]
                # speed model on this lap window: k fitted on the recorded PWM, then the candidate PWM
                i0 = idx[0]; tt = t[idx]; vm = np.array([meas_v(t, enc, i) for i in idx]); v0 = vm[np.isfinite(vm)][0] if np.isfinite(vm).any() else 1.0
                best = min(np.arange(0.80, 1.101, 0.01), key=lambda k: np.nanmean((sim(tt, pa[idx], v0, k) - vm) ** 2))
                va = sim(tt, pa[idx], v0, best); vb = sim(tt, pb[idx], v0, best); rms = float(np.sqrt(np.nanmean((va - vm) ** 2)))
                dist = np.concatenate([[0.], np.cumsum(0.5 * (va[1:] + va[:-1]) * np.diff(tt))])   # old-side distance along the window
                db = np.concatenate([[0.], np.cumsum(0.5 * (vb[1:] + vb[:-1]) * np.diff(tt))])
                T_old = tt[-1] - tt[0]; T_new = float(np.interp(dist[-1], db, tt - tt[0])) if db[-1] >= dist[-1] else float('nan')
                ic4e = next((n for n, i in enumerate(idx) if A[i]['anchor'] == 'C4e'), len(idx) - 1)
                gains.append((T_old - T_new, vb[ic4e] - va[ic4e], best, rms))
                print('   lap %d: PWM differs %s .. %s (%d frames) | c4-zone frames %d, blocked %s | PWM in LAND8>=3.4: old mean %.1f -> new mean %.1f (max %.0f) | model k %.2f rms %.3f m/s: window time %.2f -> %.2f s (%+.2f), speed at C4e %.2f -> %.2f m/s, at window end %.2f -> %.2f' % (
                    lap, pos(A[c[0]]), pos(A[c[-1]]), len(c), len(zl), dict(blk), np.nanmean(pa[seg]) if seg else float('nan'), np.nanmean(pb[seg]) if seg else float('nan'), np.nanmax(pb[idx]),
                    best, rms, T_old, T_new, T_new - T_old, va[ic4e], vb[ic4e], va[-1], vb[-1]))
        g = np.array(gains)
        print('TOTAL frames %d | PWM changed %d | changed outside the window %d | lowered %d' % (tot['frames'], tot['changed'], tot['outside'], tot['lower']))
        if len(g): print('MODEL (first order, open loop, %d laps): time saved LAND8+2.0 .. C4e+3.0 median %.2f s (min %.2f, max %.2f); speed at C4e median %+.2f m/s; fitted curve factor k median %.2f; model rms vs encoder median %.3f m/s' % (
            len(g), np.median(g[:, 0]), g[:, 0].min(), g[:, 0].max(), np.median(g[:, 1]), np.median(g[:, 2]), np.median(g[:, 3])))


if __name__ == '__main__':
    main()
