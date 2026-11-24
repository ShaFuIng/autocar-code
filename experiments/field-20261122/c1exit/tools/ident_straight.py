"""camera error sensitivity on straights from the gyro (no arc_lat needed):
  on a straight lane  e(t) = a0 + a1*dist + c*Yaw(t) + b*1.745*J(t),  Yaw = integral gz dt (deg, left +),  J = integral v*Yaw dt (deg*m)
  (psi = psi0 + Yaw, y = y0 + integral v*sin(psi) dt; a0, a1 per segment absorb y0 / psi0).  Pooled least squares for c [/deg] and b [/cm].
segments: the diagonal 0.9 .. 3.6 s after C1e and the main straight 1.0 .. 4.5 s after C4e.   python ident_straight.py [runs]"""
import sys, numpy as np, lib, passes
runs = sys.argv[1:] or lib.MAIN + lib.MORE
for an, a, b_ in (('C1e', 0.9, 3.6), ('C4e', 1.0, 4.5), ('C2e', 1.2, 3.2)):
    rows = []; Y = []; nseg = 0; segs = []
    for run in runs:
        Q, A = passes.run_arrays(run)
        for i in passes.exits(A, an):
            t = A['t']; m = (t >= t[i] + a) & (t <= t[i] + b_) & np.isfinite(A['e']) & np.isfinite(A['gz']) & np.isfinite(A['v']) & (A['valid'] > 0)
            if m.sum() < 40: continue
            tt = t[m]; gz = A['gz'][m]; v = A['v'][m]; e = A['e'][m]
            dt = np.diff(tt, prepend=tt[0]); yaw = np.cumsum(gz * dt); dist = np.cumsum(v * dt); J = np.cumsum(v * yaw * dt)
            segs.append((e, dist, yaw, J)); nseg += 1
    n = sum(len(s[0]) for s in segs); M = np.zeros((n, 2 * nseg + 2)); y = np.zeros(n); k = 0
    for j, (e, dist, yaw, J) in enumerate(segs):
        L = len(e); M[k:k + L, 2 * j] = 1; M[k:k + L, 2 * j + 1] = dist; M[k:k + L, -2] = yaw; M[k:k + L, -1] = J; y[k:k + L] = e; k += L
    co, *_ = np.linalg.lstsq(M, y, rcond=None); res = y - M @ co
    M0 = M[:, :-2]; c0, *_ = np.linalg.lstsq(M0, y, rcond=None); r0 = y - M0 @ c0
    print('%s +%.1f..%.1f s: %d segments %d frames | c %.4f /deg  b %.4f /cm (= %.3f per deg*m) | resid sd %.3f (without the two terms %.3f)' % (an, a, b_, nseg, n, co[-2], co[-1] / 1.745, co[-1], res.std(), r0.std()))
