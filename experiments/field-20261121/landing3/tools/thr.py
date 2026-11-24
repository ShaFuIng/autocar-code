"""landing3-20261121: what a different speed_adapt landing_pwm (and landing_s) does to the PWM after the landing, frame by frame.

Emulation of the speed_adapt 'landing' rule on the recorded frames (NOT the module itself: its other inputs - far error, wheel speed,
weave flag - are not all in the log). Per recorded frame the log gives level / reason / target_pwm / pwm / landing{pwm,since_s,target}.
  target' = L                      when the frame has a landing record, level LOW with a path reason, and the recorded target is the
                                   landing value (recorded target == the car's landing_pwm); else the recorded target
  frames with since_s > landing_s' lose the landing rule (target' = max(min_pwm, curve_pwm) when the landing value had set it)
  cur'    = target' at once when a landing record is present and target' > cur' (the module's jump); else +-30 PWM/s (up_rate / down_rate)
After the window cur' converges to the recorded value at 30 PWM/s. Then the recorded brake-zone cap is re-applied (min). Frames whose
recorded final PWM came from another layer (lap_speed zone, guard) keep the recorded value.
Checked first with L = the car's value: must give the recorded speed_adapt PWM back.
usage: python thr.py   (runs e1b4 / b93e / 9934; tables to stdout)"""
import json, sys, collections
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from data import load, landings, M, EXP
SNAP = EXP / 'orin-web-snapshot-20261121c'
SA = json.loads((SNAP / 'speed_adapt.json').read_text(encoding='utf8'))
SG = json.loads((SNAP / 'speed_guard.json').read_text(encoding='utf8'))['overspeed_watch']
TAB = [tuple(x) for x in SG['table']]; V50 = TAB[-1][1]
HI = [(55., 1.034 * V50), (60., 1.057 * V50), (65., 1.079 * V50), (70., 1.098 * V50), (100., 6492.)]
PW = np.array([x[0] for x in TAB] + [x[0] for x in HI]); CP = np.array([x[1] for x in TAB] + [x[1] for x in HI])
TAU_UP, TAU_DN = float(SG['tau_up_s']), float(SG['tau_down_s'])
PATH = ('ERROR_LARGE', 'CURVE_AHEAD', 'STEERING_LARGE', 'ERROR_RATE_LARGE', 'NO_ERROR')
RUNS = ('run-e1b4-20261121', 'run-b93e-20261121', 'run-9934-20261121')


def vss(p): return float(np.interp(p, PW, CP)) * M


def sim(t, pwm, v0, k=1.):
    v = [v0]
    for i in range(1, len(t)):
        tg = k * vss(pwm[i - 1]); dt = min(.2, max(0., t[i] - t[i - 1])); tau = TAU_UP if tg > v[-1] else TAU_DN
        v.append(v[-1] + (tg - v[-1]) * (1. - np.exp(-dt / tau)))
    return np.array(v)


def emulate(Q, L, ls_s=None):
    """returns arrays (speed_adapt PWM, final PWM). L = landing_pwm, ls_s = landing_s (None = as recorded)."""
    L0 = float(SA['landing_pwm'])
    sa = np.array([q['sa_pwm'] if q['sa_pwm'] is not None else np.nan for q in Q]); fin = np.array([q['pwm'] if q['pwm'] is not None else np.nan for q in Q])
    new_sa = sa.copy(); new_fin = fin.copy(); cur = None; active = False
    for i, q in enumerate(Q):
        ld = q['landing']
        if ld is None:
            if active and cur is not None and not np.isnan(sa[i]) and abs(cur - sa[i]) > 1e-6 and q['sa_reason'] != 'DOWNHILL':
                dt = max(0., min(.5, q['t'] - Q[i - 1]['t']))
                cur = min(sa[i], cur + 30. * dt) if sa[i] > cur else max(sa[i], cur - 30. * dt); new_sa[i] = cur
                if abs(cur - sa[i]) < 1e-6: active = False
            else:
                active = False; cur = None
        else:
            dt = max(0., min(.5, q['t'] - Q[i - 1]['t'])) if i else 0.
            if cur is None: cur = sa[i - 1] if i and not np.isnan(sa[i - 1]) else sa[i]
            tgt = ld.get('target'); inwin = ls_s is None or ld.get('since_s', 0.) <= ls_s
            land_set = q['sa_level'] == 'LOW' and q['sa_reason'] in PATH and tgt is not None and abs(tgt - L0) < 1e-6
            if land_set: tgt = L if inwin else max(float(SA['min_pwm']), float(SA.get('curve_pwm') or 0.))
            if tgt is None: tgt = sa[i]
            if inwin and tgt > cur: cur = tgt
            elif tgt > cur: cur = min(tgt, cur + 30. * dt)
            else: cur = max(tgt, cur - 30. * dt)
            new_sa[i] = cur; active = True
        if not np.isnan(new_sa[i]) and abs(new_sa[i] - sa[i]) > 1e-9:
            f = new_sa[i]
            if q['bz_cap'] is not None: f = min(f, q['bz_cap'])
            same_layer = abs(fin[i] - sa[i]) < .05 or (q['bz_cap'] is not None and abs(fin[i] - min(sa[i], q['bz_cap'])) < .05)
            if not same_layer: f = fin[i]   # a later layer set the recorded final PWM (lap_speed zone, guard): keep it
            new_fin[i] = f
    return new_sa, new_fin


def xpos(Q, k):
    enc0 = Q[k]['enc']; s0 = Q[k]['s'] or 0.
    return lambda i: None if Q[i]['enc'] is None else s0 + (Q[i]['enc'] - enc0) * M


def main():
    print('car: landing_pwm %s, landing_s %s, curve_pwm %s, min_pwm %s, mid_pwm %s' % (SA['landing_pwm'], SA['landing_s'], SA.get('curve_pwm'), SA['min_pwm'], SA['mid_pwm']))
    for run in RUNS:
        Q = load(run); a, b = emulate(Q, float(SA['landing_pwm']))
        rs = np.array([q['sa_pwm'] if q['sa_pwm'] is not None else np.nan for q in Q]); rf = np.array([q['pwm'] if q['pwm'] is not None else np.nan for q in Q])
        ld = np.array([q['landing'] is not None for q in Q]); d = np.abs(np.nan_to_num(a - rs))
        print('fidelity %s: landing frames %d | emulated speed_adapt PWM vs recorded (car value): max |diff| %.3f, frames > 0.05: %d (of them in landing frames %d) | final PWM frames changed: %d' % (
            run[4:8], ld.sum(), d.max(), int((d > .05).sum()), int((d[ld] > .05).sum()), int((np.abs(np.nan_to_num(b - rf)) > 1e-6).sum())))
    for name, L, ls in (('s1 landing_pwm 40', 40., None), ('s2 landing_pwm 46', 46., None), ('probe landing_pwm 50', 50., None), ('probe landing_s 2.0 (landing_pwm 33)', 33., 2.0)):
        print('=' * 100); print('candidate:', name); tot = collections.Counter(); gains = []
        for run in RUNS:
            Q = load(run); T = np.array([q['t'] for q in Q]); a, b = emulate(Q, L, ls)
            rf = emulate(Q, float(SA['landing_pwm']))[1]   # old side = the same emulator with the car's value (both sides through one emulator)
            ch = np.flatnonzero(np.abs(np.nan_to_num(b - rf)) > 1e-6)
            Ls = landings(Q); win = set()
            for k in Ls:
                x = xpos(Q, k); end = next((i for i in range(k, len(Q)) if Q[i]['anc'] != 'LAND8'), len(Q))
                win.update(i for i in range(max(0, k - 12), end) if x(i) is not None and -0.3 <= x(i) <= 4.2)
            out = [i for i in ch if i not in win]; low = [i for i in ch if b[i] < rf[i] - 1e-6]
            tot['frames'] += len(Q); tot['changed'] += len(ch); tot['outside'] += len(out); tot['lower'] += len(low)
            print('%s: frames %d | final PWM changed %d | outside LAND8 -0.3..+4.2 m: %d | lowered: %d' % (run[4:8], len(Q), len(ch), len(out), len(low)))
            for k in Ls:
                x = xpos(Q, k); end = next((i for i in range(k, len(Q)) if Q[i]['anc'] != 'LAND8'), len(Q))
                idx = [i for i in range(k, end) if x(i) is not None and x(i) <= 6.0]
                if len(idx) < 60 or x(idx[-1]) < 5.: continue
                c = [i for i in idx if abs(np.nan_to_num(b[i] - rf[i])) > 1e-6]
                if not c:
                    print('   lap %s: no change' % Q[k]['lap']); continue
                tt = T[idx]; pa = rf[idx]; pb = b[idx]; n = len(Q)
                vm = np.array([np.nan if Q[min(n - 1, i + 5)]['enc'] is None or Q[max(0, i - 5)]['enc'] is None else (Q[min(n - 1, i + 5)]['enc'] - Q[max(0, i - 5)]['enc']) * M / max(1e-3, T[min(n - 1, i + 5)] - T[max(0, i - 5)]) for i in idx])
                v0 = vm[np.isfinite(vm)][0] if np.isfinite(vm).any() else 1.
                best = min(np.arange(.80, 1.101, .01), key=lambda kk: np.nanmean((sim(tt, pa, v0, kk) - vm) ** 2))
                va = sim(tt, pa, v0, best); vb = sim(tt, pb, v0, best); rms = float(np.sqrt(np.nanmean((va - vm) ** 2)))
                da = np.concatenate([[0.], np.cumsum(.5 * (va[1:] + va[:-1]) * np.diff(tt))]); db = np.concatenate([[0.], np.cumsum(.5 * (vb[1:] + vb[:-1]) * np.diff(tt))])
                Tn = float(np.interp(da[-1], db, tt - tt[0])); i34 = next((m for m, i in enumerate(idx) if x(i) >= 3.4), len(idx) - 1); i20 = next((m for m, i in enumerate(idx) if x(i) >= 2.0), 0)
                gains.append(((tt[-1] - tt[0]) - Tn, vb[i34] - va[i34], vb[i20] - va[i20], rms, va[i34]))
                seg = [m for m, i in enumerate(idx) if 1.5 <= x(i) <= 3.4]
                print('   lap %s: PWM differs x %.2f .. %.2f m (%d frames) | PWM x 1.5-3.4: old mean %.1f -> new %.1f (max %.0f) | model k %.2f rms %.3f: time to x 6.0 %.2f -> %.2f s (%+.2f) | speed at x 2.0 %+.2f, at x 3.4 %.2f -> %.2f' % (
                    Q[k]['lap'], x(c[0]), x(c[-1]), len(c), np.nanmean(pa[seg]), np.nanmean(pb[seg]), np.nanmax(pb), best, rms, tt[-1] - tt[0], Tn, Tn - (tt[-1] - tt[0]), vb[i20] - va[i20], va[i34], vb[i34]))
        g = np.array(gains)
        print('TOTAL frames %d | PWM changed %d | outside the window %d | lowered %d' % (tot['frames'], tot['changed'], tot['outside'], tot['lower']))
        if len(g): print('MODEL (%d landings): time saved to x 6.0 m median %.2f s (min %.2f max %.2f) | speed at x 3.4 m median %+.2f m/s (max %+.2f) | at x 2.0 median %+.2f | model rms median %.3f m/s' % (
            len(g), np.median(g[:, 0]), g[:, 0].min(), g[:, 0].max(), np.median(g[:, 1]), g[:, 1].max(), np.median(g[:, 2]), np.median(g[:, 3])))


if __name__ == '__main__':
    main()
