"""curve 4: SIMPLE closed-loop model of the band (estimate only, see the limits below).
  car: curvature (deg/m) = G * (u_delayed + U0), pure delay TD + first-order lag TAU; psi = lane heading - car heading (deg, + = car points out);
       dy/ds = sin(psi) (cm, + = outside).   lane: uniform arc KL deg/m (odometer units) from the hand-over.
  camera: near error = f(s) + B*y + C*psi  (f, B, C = least-squares fit of the 26 logged passes, c4err.py; f is attached to the lane position
          through the median turned-angle-vs-distance of the passes, so the camera 'features' (dips at ~50 / ~105 / ~160 deg) stay on the lane).
  controller: the band target only (logged: final command = band target on 72 % of the frames):
          u = kappa(tb)/g(v) + clip(k_e*(e - ec(tb)), -250, +150) + clip(1.5*(gz + v*kappa), +-60), blended in over x 1.2..1.5 from the land4 value.
  start (median of the 26 passes at x 1.2 m): tb 44.7, psi -8.3 deg, y -2.8 cm, command +85.
Limits: no PID chain / curve_map floor / top, no backlash, no slip, linear camera model (residual sd 0.10), one lane shape. It is used to COMPARE tables on the same
model and to see whether the model reproduces the logged pattern with the car's present tables; it is not a prediction of the real car.
usage: python sim_c4.py <lap_ff.json> [<lap_ff.json> ...]"""
import json, sys, math, numpy as np, lib

G = 0.18; U0 = 8.; TD = 0.12; TAU = 0.12; DT = 1 / 30.
KL = 23.5; TT0 = 36.4; S0 = 1.2
GT = [[0.8, 0.192], [0.93, 0.167], [1.06, 0.159], [1.22, 0.15], [1.37, 0.13]]
TBS = [(1.2, 45.1), (1.5, 49.5), (1.8, 53.8), (2.1, 58.3), (2.4, 65.1), (2.7, 72.1), (3.0, 79.6), (3.3, 87.7), (3.6, 95.0), (3.9, 104.2), (4.2, 111.3), (4.5, 117.9), (4.8, 124.8), (5.1, 134.1), (5.4, 142.3),
       (5.7, 150.7), (6.0, 157.6), (6.3, 163.5), (6.6, 169.4), (6.9, 174.7), (7.2, 180.1), (7.5, 187.4)]   # median turned angle vs lap_ff4 x (c4track.py)
E = json.load(open(lib.HERE / 'c4err.json')); B = E['b']; C = E['c']; F = E['f']


def ip(t, x):
    t = sorted((float(a), float(b)) for a, b in t)
    if x <= t[0][0]: return t[0][1]
    if x >= t[-1][0]: return t[-1][1]
    for (x0, y0), (x1, y1) in zip(t, t[1:]):
        if x0 <= x <= x1: return y0 + (y1 - y0) * (x - x0) / max(1e-9, x1 - x0)


def vel(s): return 0.98 if s < 2.4 else min(1.21, 0.98 + (s - 2.4) * 0.075)


def run(band, psi0=-8.3, y0=-2.8, tb0=44.7, u0=85., noise=0., seed=0, kl=KL, xerr=0.):
    rng = np.random.default_rng(seed)
    s = S0; tb = tb0; y = y0; ttl = tb0 + psi0; u_lag = u0; buf = [u0] * max(1, int(round(TD / DT))); out = []; kr = float(band.get('kappa_ref', 22.))
    while s < 7.6 and tb < 186:
        v = vel(s); psi = ttl - tb
        e = ip(F, ip(TBS, s)) + B * y + C * psi + (rng.normal(0, noise) if noise else 0.)
        kap = ip(band['kappa_turn'], tb); g = ip(GT, v); prof = kap / g
        ect = band.get('e_c_turn'); ec = ip(ect, tb) if ect else float(band['e_c']) * kap / kr
        et = max(-250., min(150., float(band['k_e_us']) * (e - ec)))
        kc = G * (u_lag + U0); gz = -kc * v
        gt = max(-60., min(60., 1.5 * (gz + v * kap)))
        H = band.get('head'); ht = 0.
        if H: ht = max(-float(H['lim_us']), min(float(H['lim_us']), float(H['k_us_deg']) * ip(H['gain_x'], s + xerr) * (ip(H['lane_turn'], s + xerr) - tb)))   # c4-head-r1 (model of the candidate term)
        tgt = max(0., min(305., prof + et + gt + ht)); w = max(0., min(1., (s - S0) / 0.3)); u = u0 + w * (tgt - u0)
        buf.append(u); ud = buf.pop(0); u_lag += (ud - u_lag) * (1 - math.exp(-DT / TAU))
        out.append((s, tb, psi, y, e, ec, u, kc, prof, et))
        ds = v * DT; tb += kc * ds; ttl += kl * ds; y += 100. * math.sin(math.radians(psi)) * ds; s += ds
    return np.array(out)


def show(name, R):
    print(name); print('   x     tb   psi    y    err   ec   cmd  curv  prof  e_us')
    for x0 in np.arange(1.35, 7.6, 0.3):
        m = (R[:, 0] >= x0 - 0.15) & (R[:, 0] < x0 + 0.15)
        if m.any(): print('  %4.2f %5.1f %+5.1f %+5.1f %+.2f %.2f %+5.0f %5.1f %+5.0f %+5.0f' % ((x0,) + tuple(np.median(R[m, k]) for k in range(1, 10))))
    m = R[:, 0] >= 1.5
    print('   summary x>=1.5: y min %+.1f max %+.1f | cmd min %+.0f max %+.0f sd %.0f | curvature min %.1f max %.1f sd %.1f | largest cmd change over 0.6 m %+.0f' % (
        R[m, 3].min(), R[m, 3].max(), R[m, 6].min(), R[m, 6].max(), R[m, 6].std(), R[m, 7].min(), R[m, 7].max(), R[m, 7].std(),
        max((R[j, 6] - R[i, 6] for i in range(len(R)) for j in range(i, len(R)) if R[i, 0] >= 1.5 and 0.55 < R[j, 0] - R[i, 0] < 0.65), key=abs)))


if __name__ == '__main__':
    for p in sys.argv[1:]:
        band = json.load(open(p, encoding='utf8'))['c4']['band']
        show(p, run(band))
        ys = []; cs = []
        for k in range(40):   # scatter of the hand-over state (sd from the 26 passes: tb 2.5 deg, psi ~3 deg, y ~3 cm) + camera noise 0.05
            r = np.random.default_rng(100 + k); R = run(band, psi0=-8.3 + r.normal(0, 3.), y0=-2.8 + r.normal(0, 3.), tb0=44.7 + r.normal(0, 2.5), noise=0.05, seed=k); m = R[:, 0] >= 1.5
            ys.append((R[m, 3].min(), R[m, 3].max())); cs.append(R[m, 6].std())
        ys = np.array(ys); print('   40 scattered starts + noise: y min median %+.1f worst %+.1f | y max median %+.1f worst %+.1f | cmd sd median %.0f' % (np.median(ys[:, 0]), ys[:, 0].min(), np.median(ys[:, 1]), ys[:, 1].max(), np.median(cs)))
