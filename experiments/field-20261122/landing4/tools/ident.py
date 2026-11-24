"""steering response in the landing zone (x 0..2 m) and on the lower slope, from the 23 landings: curvature = -gz / v (deg/m, right +) against the
pre-compensation command delayed by tau; least squares kappa = g * (cmd - c) per tau."""
from desc import *
import bisect
R = [d for d in all_desc() if d['i_land'] is not None]


def fit(zone, name):
    best = None
    for tau in (0.10, 0.15, 0.20, 0.25, 0.30, 0.35):
        xs = []; ys = []
        for d in R:
            F = d['F']; t = d['t']; il = d['i_land'] - d['a']
            tt = [x for x in t]; cm = [p.get('steer_us') for p in F]
            for n, p in enumerate(F):
                x = d['x'][n]
                if x is None or not (zone[0] <= x <= zone[1]): continue
                gz = g(p, 'inputs', 'gz'); v = g(p, 'yaw_inner', 'speed_mps') or g(p, 'lap_ff4', 'v') or 1.1
                if gz is None or abs(gz) > 150: continue
                k = bisect.bisect_right(tt, t[n] - tau) - 1
                if k < 0 or cm[k] is None: continue
                xs.append(cm[k] - C0); ys.append(-gz / v)
        n = len(xs); mx = sum(xs) / n; my = sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs); sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys)); syy = sum((y - my) ** 2 for y in ys)
        gch = sxy / sxx; c = mx - my / gch; r2 = sxy * sxy / (sxx * syy)
        if best is None or r2 > best[3]: best = (tau, gch, c, r2, n)
        print('  %s tau %.2f s: gain %.3f deg/m per us, zero at centre %+.0f us, r2 %.2f, n %d' % (name, tau, gch, c, r2, n))
    return best


b1 = fit((0.0, 1.6), 'landing x 0..1.6 m')
b2 = fit((-1.2, -0.1), 'lower slope x -1.2..-0.1 m')
b3 = fit((2.0, 4.5), 'curve 4 x 2.0..4.5 m')
print('best:', b1, b2, b3)
