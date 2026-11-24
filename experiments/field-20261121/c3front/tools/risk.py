"""c3front: (1) what the existing candidates curves-20261120/on/speed_adapt.c3_36.json / c3_40.json would change (MODEL: the frames whose
recorded speed_adapt PWM is the curve-3 value 32 get 36 / 40; recorded brake caps still cap; first-order speed model of c4fast);
(2) risk data of the 10 laps: margins in the front part, and how the corner (x 4.3..5.5 m after C3s) went against the speed it was entered with."""
import collections, json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parents[1] / 'c4fast' / 'tools'))
import c3data, analyse
A = c3data.all_laps()
print('1. existing candidates speed_adapt.c3_36 / c3_40 (curve_pwm_by c3 32 -> 36 / 40), MODEL')
for new in (36., 40.):
    G = []; span = []
    for k, rows in A.items():
        t = np.array([r['t'] for r in rows]); x = np.array([r['x'] for r in rows]); pa = np.array([r['pwm'] if r['pwm'] is not None else np.nan for r in rows], float)
        pb = pa.copy(); ch = []
        for i, r in enumerate(rows):
            # speed_adapt gave the curve-3 PWM and nothing after it lowered it on this frame
            if r['sa_pwm'] is not None and abs(r['sa_pwm'] - 32.) < 0.01 and r['pwm'] is not None and abs(r['pwm'] - 32.) < 0.01 and r['ls'] is None:
                pb[i] = new; ch.append(i)
        if not ch: continue
        span.append((x[ch[0]], x[ch[-1]]))
        v = c3data.speed(rows); v0 = v[0]
        best = min(np.arange(0.80, 1.201, 0.01), key=lambda kk: np.nanmean((analyse.sim(t, pa, v0, kk) - v) ** 2))
        va = analyse.sim(t, pa, v0, best); vb = analyse.sim(t, pb, v0, best)
        da = np.concatenate([[0.], np.cumsum(0.5 * (va[1:] + va[:-1]) * np.diff(t))]); db = np.concatenate([[0.], np.cumsum(0.5 * (vb[1:] + vb[:-1]) * np.diff(t))])
        m = (x >= -1.0) & (x <= 6.0); i0 = np.argmax(m); i1 = len(x) - 1 - np.argmax(m[::-1])
        Told = t[i1] - t[i0]; Tnew = float(np.interp(da[i1], db, t)) - float(np.interp(da[i0], db, t))
        def at(xq):
            n = int(np.argmin(np.abs(x - xq))); return float(np.interp(da[n], db, vb)) - va[n]
        G.append((Told - Tnew, at(4.3), at(4.8)))
    g = np.array(G); sp = np.array(span)
    print('   c3 %.0f: frames changed from x %+.2f..%+.2f (first) to %+.2f..%+.2f (last) | time saved x -1..6 m median %.2f s | speed at x 4.3: %+.3f (max %+.3f), at 4.8: %+.3f (max %+.3f) m/s' % (
        new, sp[:, 0].min(), sp[:, 0].max(), sp[:, 1].min(), sp[:, 1].max(), np.median(g[:, 0]), np.median(g[:, 1]), g[:, 1].max(), np.median(g[:, 2]), g[:, 2].max()))
print('   (the last changed frame is where the C2e-anchored tail zone cap 28 starts: C2e+14.7 m = x 3.8..4.5 depending on the lap, the corner peak is at x 4.66..5.11)')

print('\n2. margins in the front part (x 0.6..3.3 m), per lap: max |steer - centre| us, |image error| P90 / max, frames over err 0.6, speed m/s; yaw-rate sd')
for k, rows in A.items():
    R = [r for r in rows if 0.6 <= r['x'] <= 3.3]; v = c3data.speed(rows); vv = [v[i] for i, r in enumerate(rows) if 0.6 <= r['x'] <= 3.3]
    st = np.array([abs(r['steer'] - 1475.) for r in R]); e = np.array([abs(r['e'] or 0.) for r in R]); gz = np.array([r['gz'] or 0. for r in R])
    print('   %s L%d  steer max %3.0f  |e| P90 %.2f max %.2f  frames > 0.6: %d of %d  speed %.2f  gz sd %.0f  inner clearance min %s cm' % (
        k[0][4:8], k[1], st.max(), np.percentile(e, 90), e.max(), int((e > 0.6).sum()), len(R), np.mean(vv), gz.std(), min([r['al_clr_in'] for r in R if r['al_clr_in'] is not None] or [float('nan')])))

print('\n3. the corner against its entry speed, per lap: speed at x 4.3 (interpolated odometer +-0.25 s), then in x 4.3..5.5: max image error, frames with |e| >= 0.95,')
print('   max command - centre (proposal) and sent, frames with the proposal >= centre+400, min yaw rate dps, min inner clearance cm (arc_lat), speed at x 5.5')
T = []
for k, rows in A.items():
    v = c3data.speed(rows); x = np.array([r['x'] for r in rows]); n43 = int(np.argmin(np.abs(x - 4.3))); n55 = int(np.argmin(np.abs(x - 5.5)))
    R = [r for r in rows if 4.3 <= r['x'] <= 5.5]
    e = np.array([r['e'] or 0. for r in R]); st = np.array([r['steer'] - 1475. for r in R]); se = np.array([(r['sent'] or 1475.) - 1475. for r in R]); gz = np.array([r['gz'] or 0. for r in R])
    cl = [r['al_clr_in'] for r in R if r['al_clr_in'] is not None]
    T.append((v[n43], e.max(), int((np.abs(e) >= .95).sum()), st.max(), se.max(), int((st >= 400).sum()), gz.min(), min(cl) if cl else np.nan, v[n55]))
    print('   %s L%d  v(4.3) %.2f | e max %.2f, sat frames %d | cmd max %+4.0f, sent max %+4.0f, frames >= +400: %2d | gz min %4.0f | clr_in min %5.1f | v(5.5) %.2f' % ((k[0][4:8], k[1]) + T[-1]))
T = np.array(T)
for j, nm in ((1, 'max error'), (3, 'max command'), (5, 'frames >= +400'), (6, 'min yaw rate'), (7, 'min inner clearance')):
    ok = ~np.isnan(T[:, j]); print('   correlation of the entry speed with %-20s r = %+.2f (n=%d)' % (nm, np.corrcoef(T[ok, 0], T[ok, j])[0, 1], ok.sum()))
print('   entry speed x 4.3 over the 10 laps: min %.2f median %.2f max %.2f m/s' % (T[:, 0].min(), np.median(T[:, 0]), T[:, 0].max()))
