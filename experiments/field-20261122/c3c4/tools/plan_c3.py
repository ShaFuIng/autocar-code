"""curve 3 front section (C3s + 0.6 .. 4.4 m): plan for the distance band (x-band-r1) and a SIMPLE closed-loop model of it (estimate only).
Lane (proof/B2): heading rate kT(x) of the lane seen from the car (gyro heading + arc_lat relative heading, 20 passes): a polyline, bend 1 at 1.2-2.2 m,
straight 2.3-3.1 m, bend 2 3.2-4.0 m, corner 4.5-5.1 m.
Plan: the car's path curvature = the lane's heading rate smoothed over SMOOTH_M (a car cannot follow the kinks; the smoothing cuts each bend by a few cm),
started from the logged median state at 0.6 m. From the plan:  y*(x), psi*(x) (position / heading relative to the lane), then
  kappa_x(x) = command the plan needs, in the controller's units, LEAD_M earlier (plant delay)       [us = kappa / g(v)]
  e_c_x(x)   = f(x) + B*y*(x) + C*psi*(x)   (f, B, C = camera fit of the logged passes, c3err.py)   = what the camera reads on the plan
Model for the check: car curvature = G*(u delayed + U0); camera e = f(x) + B*y + C*psi (+ noise); band: u = kappa_x/g + clip(k_e*(e - e_c_x)) + gyro term,
blended by w_x; outside w_x the logged median command + K*(e - logged median e) stands in for the car's present chain.
Limits: one lane shape (medians), linear camera model (residual sd 0.115), no backlash / slip, the present chain is a one-gain stand-in, C3s position scatter
is only tested as a fixed x offset. NOT a prediction of the real car.
usage: python plan_c3.py   (prints the plan, the tables and the model runs; mkcfg.py writes the on/ files from the same functions)"""
import copy, json, math, sys, numpy as np, lib

G = 0.17; U0 = 22.   # proof/A5: logged fit 0.16 * (cmd + 38) (regression, biased toward a low gain / high offset); straight point 1467 = offset 8 with gain ~0.18
DELAY_M = 0.13; LAG_M = 0.13; DS = 0.01; LEAD_M = 0.25; SMOOTH_M = 0.7
KT = [(-0.6, 26.1), (-0.3, 35.5), (0.0, 27.0), (0.3, 21.8), (0.6, 25.1), (0.9, 13.5), (1.2, 37.5), (1.5, 49.3), (1.8, 42.4), (2.1, 42.0), (2.4, 13.3), (2.7, -3.2), (3.0, 7.7), (3.3, 44.8), (3.6, 43.4),
      (3.9, 33.1), (4.2, 12.9), (4.5, 31.5), (4.8, 87.2), (5.1, 28.0)]
# logged medians (proof B1 / B2): x, command - centre, y cm, psi deg, near error, wheel speed
MED = [(0.45, 115, -3.3, -1.4, 0.29, 0.89), (0.6, 99, -4.8, -5.5, 0.24, 0.90), (0.9, 71, -7.8, -6.1, 0.18, 0.92), (1.2, 115, -13.2, -7.9, 0.35, 0.96), (1.5, 173, -14.7, -0.8, 0.48, 1.02), (1.8, 204, -13.7, 1.5, 0.54, 1.07),
       (2.1, 172, -11.9, 6.4, 0.44, 1.10), (2.4, 137, -8.6, 4.0, 0.27, 1.13), (2.7, 72, -9.1, -2.8, -0.05, 1.12), (3.0, 31, -13.9, -9.9, -0.13, 1.08), (3.3, 97, -19.3, -3.0, 0.23, 1.00), (3.6, 144, -18.4, 7.1, 0.38, 0.94),
       (3.9, 187, -14.9, 8.3, 0.49, 0.92), (4.2, 189, -10.2, 7.1, 0.47, 0.94), (4.5, 286, -9.2, -1.6, 0.70, 0.96)]
GT = [[0.8, 0.192], [0.93, 0.167], [1.06, 0.159], [1.22, 0.15], [1.37, 0.13]]
E = json.load(open(lib.HERE / 'c3err.json')); B = E['b']; C = E['c']; KX = E['Kx']
FX = [(0.15, .52), (0.45, .39), (0.75, .33), (1.05, .54), (1.35, .69), (1.65, .70), (1.95, .66), (2.25, .39), (2.55, .18), (2.85, .22), (3.15, .35), (3.45, .52), (3.75, .56), (4.05, .51), (4.5, .51)]   # c3err.py f(x)
X0 = 0.6; X1 = 4.4
W_X = [[0.6, 0.0], [0.9, 1.0], [4.05, 1.0], [4.35, 0.0]]   # where the band owns


def ip(t, x):
    if x <= t[0][0]: return float(t[0][1])
    for (x0, y0), (x1, y1) in zip(t, t[1:]):
        if x <= x1: return float(y0) + (float(y1) - float(y0)) * (x - float(x0)) / max(1e-9, float(x1) - float(x0))
    return float(t[-1][1])


def med(k, x): return ip([(m[0], m[k]) for m in MED], x)


def plan(smooth=SMOOTH_M, y_t=-8.0, om=2.5, zeta=0.8, y0=None, psi0=None):
    """-> rows (x, k_plan, psi*, y*): an ideal (no delay) well-damped tracker run on the lane: path curvature = smoothed lane heading rate
    + a*psi + b*(y - y_t)  (natural wavelength 2*pi/om m, damping zeta), from the logged median state at 0.6 m toward y_t cm from the lane centre."""
    xs = np.arange(X0, X1 + 1e-9, 0.05); rows = []; psi = med(3, X0) if psi0 is None else psi0; y = med(2, X0) if y0 is None else y0
    a = 2 * zeta * om; b = om * om / 1.745
    for x in xs:
        kp = np.mean([ip(KT, x + d) for d in np.linspace(-smooth / 2, smooth / 2, 15)]) + a * psi + b * (y - y_t)
        kp = max(0., min(55., kp))
        rows.append((x, kp, psi, y))
        psi += (ip(KT, x) - kp) * 0.05; y += 100. * math.sin(math.radians(psi)) * 0.05
    return rows


def tables(rows):
    kx = []; ec = []
    for x, kp, psi, y in rows:
        xl = min(X1, x + LEAD_M); kpl = ip([(r[0], r[1]) for r in rows], xl)
        kx.append([round(float(x), 2), round(max(0., (kpl / G - U0) * ip(GT, med(5, x))), 1)])
        ec.append([round(float(x), 2), round(ip(FX, x) + B * y + C * psi, 3)])
    return kx[::3] + ([kx[-1]] if (len(kx) - 1) % 3 else []), ec[::3] + ([ec[-1]] if (len(ec) - 1) % 3 else [])


def sim(xb, xerr=0., g_scale=1., y0=None, psi0=None, noise=0., seed=0, k_chain=1., plant=None):
    G, U0 = plant or (globals()['G'], globals()['U0'])
    rng = np.random.default_rng(seed); x = 0.45; y = med(2, x) if y0 is None else y0; psi = med(3, x) if psi0 is None else psi0
    buf = [med(1, x)] * int(round(DELAY_M / DS)); ul = med(1, x); out = []; n = 0; e = med(4, x)
    while x < 4.5:
        if n % 3 == 0: e = ip(FX, x) + B * y + C * psi + (rng.normal(0, noise) if noise else 0.)   # ~30 Hz at 1 m/s
        n += 1; xm = x + xerr; v = med(5, x); g = ip(GT, v)
        chain = min(396., med(1, x) + k_chain * ip(KX, x) * (e - (ip(FX, x) + B * med(2, x) + C * med(3, x))))
        u = chain; w = ip(xb['w_x'], xm) if xb else 0.
        if w > 0.:
            kap = ip(xb['kappa_x'], xm); kc = G * g_scale * (ul + U0)
            tgt = kap / g + max(-xb['e_lo_us'], min(xb['e_hi_us'], xb['k_e_us'] * (e - ip(xb['e_c_x'], xm)))) + max(-xb['gz_lim_us'], min(xb['gz_lim_us'], xb['k_gz_us'] * (-kc * v + v * kap)))
            tgt = max(-xb['floor_below_us'], min(xb['cap_rel_us'], tgt)); we = max(0., min(1., (xb['e_max'] - e) / xb['e_band'])); u = chain + w * we * (tgt - chain)
        buf.append(u); ud = buf.pop(0); ul += (ud - ul) * (1 - math.exp(-DS / LAG_M))
        out.append((x, u, y, psi, e))
        psi += (ip(KT, x) - G * g_scale * (ul + U0)) * DS; y += 100. * math.sin(math.radians(psi)) * DS; x += DS
    return np.array(out)


def xband(rows, k_e=300., k_gz=1.0):
    kx, ec = tables(rows)
    return dict(enabled=True, kappa_x=kx, e_c_x=ec, w_x=W_X, k_e_us=k_e, e_lo_us=250., e_hi_us=250., k_gz_us=k_gz, gz_lim_us=60., cap_rel_us=330., floor_below_us=60., e_max=0.9, e_band=0.1)


def report(name, R):
    g_ = lambda x, k: R[int(np.argmin(np.abs(R[:, 0] - x))), k]; m = (R[:, 0] >= 0.9) & (R[:, 0] <= 4.2)
    return '%-34s y min %+5.1f max %+5.1f (0.9-4.2 m) | y / psi at 3.3 m %+5.1f / %+5.1f | at 4.35 m %+5.1f / %+5.1f | cmd max %+4.0f min %+4.0f | err max %.2f' % (
        name, R[m, 2].min(), R[m, 2].max(), g_(3.3, 2), g_(3.3, 3), g_(4.35, 2), g_(4.35, 3), R[m, 1].max(), R[m, 1].min(), R[m, 4].max())


if __name__ == '__main__':
    PARAMS = dict(c3_own_step1_in10=dict(smooth=0.7, y_t=-10.0), c3_own_step2_in7=dict(smooth=0.7, y_t=-7.0))
    print('model check with NO band (the stand-in chain only; should stay on the logged medians: y 3.3 m -19.3, 4.35 m about -9.7 / +2.8):'); print('  ' + report('no band', sim(None)))
    for name, pr in PARAMS.items():
        rows = plan(**pr); xb = xband(rows)
        print('\n==== %s  (smooth %.1f m, target %+.0f cm)' % (name, pr['smooth'], pr['y_t']))
        print('  x     lane kT  plan k   psi*    y*    | logged median y / psi | kappa_x (table)  e_c_x   logged median error')
        for x, kp, psi, y in rows[::6]: print('  %.2f  %6.1f  %6.1f  %+5.1f  %+5.1f  |  %+5.1f / %+5.1f      |  %5.1f          %+.2f    %+.2f' % (x, ip(KT, x), kp, psi, y, med(2, x), med(3, x), ip(xb['kappa_x'], x), ip(xb['e_c_x'], x), med(4, x)))
        print('  ' + report('nominal', sim(xb)))
        for lab, kw in (('C3s 0.2 m early (x reads +0.2)', dict(xerr=0.2)), ('C3s 0.2 m late (x reads -0.2)', dict(xerr=-0.2)), ('steering gain x 0.85', dict(g_scale=0.85)), ('steering gain x 1.15', dict(g_scale=1.15)), ('plant 0.16 * (cmd + 38)', dict(plant=(0.16, 38.))), ('plant 0.18 * (cmd + 8)', dict(plant=(0.18, 8.))),
                        ('start 5 cm further out, +4 deg', dict(y0=med(2, 0.45) + 5, psi0=med(3, 0.45) + 4)), ('start 5 cm further in, -4 deg', dict(y0=med(2, 0.45) - 5, psi0=med(3, 0.45) - 4))):
            print('  ' + report(lab, sim(xb, **kw)))
        S = [sim(xb, noise=0.06, seed=i, xerr=np.random.default_rng(i).normal(0, 0.15), g_scale=1 + np.random.default_rng(50 + i).normal(0, 0.1), y0=med(2, 0.45) + np.random.default_rng(90 + i).normal(0, 3), psi0=med(3, 0.45) + np.random.default_rng(130 + i).normal(0, 3)) for i in range(60)]
        f = lambda x, k: np.array([s[int(np.argmin(np.abs(s[:, 0] - x))), k] for s in S]); mn = np.array([s[(s[:, 0] >= 0.9) & (s[:, 0] <= 4.2), 2].min() for s in S]); mx = np.array([s[(s[:, 0] >= 0.9) & (s[:, 0] <= 4.2), 2].max() for s in S])
        print('  60 scattered runs (x sd 0.15 m, gain sd 10 %%, start sd 3 cm / 3 deg, camera noise 0.06): y min median %+.1f worst %+.1f | y max median %+.1f worst %+.1f | at 4.35 m y median %+.1f (p5 %+.1f p95 %+.1f) psi median %+.1f (p5 %+.1f p95 %+.1f)' % (
            np.median(mn), mn.min(), np.median(mx), mx.max(), np.median(f(4.35, 2)), np.percentile(f(4.35, 2), 5), np.percentile(f(4.35, 2), 95), np.median(f(4.35, 3)), np.percentile(f(4.35, 3), 5), np.percentile(f(4.35, 3), 95)))
