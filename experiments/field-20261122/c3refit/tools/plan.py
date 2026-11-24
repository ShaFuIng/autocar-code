"""c3refit planner: a SMOOTH path through the front of curve 3 (C3s + 0.6 .. 4.4 m) and the x-band tables that drive it.
Path = the car's curvature k*(x), piecewise linear on knots every KNOT m, found by linear least squares:
     minimise  ws * sum (second difference of k*)^2            (smooth steering: no step at each kink of the lane)
             + wy * sum (y*(x) - y_t)^2 for x >= Y_FROM        (stay near the target distance from the lane centre; soft: the path may round the kinks)
             + we * [(psi*(X1) - psi_end)^2 + (y*(X1) - y_end)^2]   (the state handed back before the corner)
     with  h*' = k*,  psi* = T(x) - h*,  y*' = 1.745 * psi*   (small angles; T = lane heading of model.json), started from the logged median state at X0.
Tables (all on the controller's distance axis, 0.15 m steps):
  u_x   = k*(x + LEAD) / G - U0   feed-forward in us (LEAD = plant dead time + lag at the local speed)
  kappa_x = k*(x)                 planned curvature (deg/m), only for the yaw-rate term
  h_x   = h*(x)                   planned heading since C3s (deg), for the heading term
  e_c_x = F(x) + B_z*y* + C_z*psi*   what the camera reads on the planned path (zone-wise camera fit of all five runs)
usage: python plan.py            prints the plans; mkcfg.py writes the on/ files from the same functions"""
import json, math, numpy as np, sim

M = sim.M; G = M['plant']['G']; U0 = M['plant']['U0']; LEAD_S = M['plant']['tau'] + M['plant']['tl']
X0 = 0.6; X1 = 4.4; DX = 0.05; KNOT = 0.2
bm = M['base_med']; BX = np.array(bm['x']); BH = np.array([np.nan if q is None else q for q in bm['h']]); BY = np.array([np.nan if q is None else q for q in bm['y']])
VMED = [[0.3, 0.92], [0.9, 0.93], [1.5, 1.02], [1.8, 1.10], [2.4, 1.15], [3.0, 1.07], [3.6, 0.95], [4.5, 0.97], [5.4, 0.90], [6.6, 0.80]]   # wheel speed medians (d0.py)
W_X = [[0.6, 0.0], [0.9, 1.0], [4.05, 1.0], [4.35, 0.0]]


def Tf(x): return sim.ip(M['T'], x)


def cam(x, y, psi):
    c = M['cam2']; z = max(0, min(len(c['B']) - 1, sum(1 for q in c['Z'] if q <= x) - 1))
    return sim.ip(c['F'], x) + c['B'][z] * y + c['C'][z] * psi


def plan(y_t=-7.0, psi_end=1.0, y_end=-9.0, ws=1.0, wy=0.15, we=0.5, y_from=1.2, h0=None, y0=None, kmax=48.):
    xs = np.arange(X0, X1 + 1e-9, DX); n = len(xs); kn = np.arange(X0, X1 + 1e-9, KNOT); m = len(kn)
    h0 = float(np.interp(X0, BX, BH)) if h0 is None else h0; y0 = float(np.interp(X0, BX, BY)) if y0 is None else y0
    Ik = np.zeros((n, m))
    for i, x in enumerate(xs):
        j = min(m - 2, int((x - X0) / KNOT)); a = (x - kn[j]) / KNOT; Ik[i, j] = 1 - a; Ik[i, j + 1] = a
    Cs = np.tril(np.ones((n, n)), -1) * DX            # integral up to (not including) the current point
    Hk = Cs @ Ik                                      # h = h0 + Hk k
    Tv = np.array([Tf(x) for x in xs]); Yk = -1.745 * (Cs @ Hk); ybase = y0 + 1.745 * (Cs @ (Tv - h0))   # y = ybase + Yk k
    rows = []; rhs = []
    D2 = np.zeros((m - 2, m))
    for j in range(m - 2): D2[j, j] = 1; D2[j, j + 1] = -2; D2[j, j + 2] = 1
    rows.append(math.sqrt(ws) * D2); rhs.append(np.zeros(m - 2))
    my = xs >= y_from; rows.append(math.sqrt(wy) * Yk[my]); rhs.append(math.sqrt(wy) * (y_t - ybase[my]))
    rows.append(math.sqrt(we) * (-Hk[-1:])); rhs.append(math.sqrt(we) * np.array([psi_end - (Tv[-1] - h0)]))
    rows.append(math.sqrt(we) * Yk[-1:]); rhs.append(math.sqrt(we) * np.array([y_end - ybase[-1]]))
    A = np.vstack(rows); b = np.concatenate(rhs); k = np.linalg.lstsq(A, b, rcond=None)[0]
    for _ in range(30):   # 0 <= k <= kmax by clamping the offenders and re-solving the rest
        lo = k < 0.; hi = k > kmax
        if not (lo.any() or hi.any()): break
        fix = np.where(lo, 0., np.where(hi, kmax, np.nan)); free = ~np.isfinite(fix)
        kk = np.linalg.lstsq(A[:, free], b - A[:, ~free] @ fix[~free], rcond=None)[0]; k = fix.copy(); k[free] = kk
        k = np.clip(k, 0., kmax) if _ == 29 else k
    kg = Ik @ k; h = h0 + Cs @ kg; psi = Tv - h; y = np.empty(n); y[0] = y0
    for i in range(1, n): y[i] = y[i - 1] + 100. * math.sin(math.radians(psi[i - 1])) * DX
    return dict(x=xs, k=kg, h=h, psi=psi, y=y, T=Tv)


def tables(pl, step=0.15):
    xs = pl['x']; kf = lambda x: float(np.interp(x, xs, pl['k'])); out = dict(u_x=[], kappa_x=[], h_x=[], e_c_x=[], y_x=[])
    for x in np.arange(X0, X1 + 1e-9, step):
        lead = LEAD_S * sim.ip(VMED, x)
        out['u_x'].append([round(float(x), 2), round(kf(min(X1, x + lead)) / G - U0, 1)]); out['kappa_x'].append([round(float(x), 2), round(kf(x), 1)])
        out['h_x'].append([round(float(x), 2), round(float(np.interp(x, xs, pl['h'])), 1)]); out['y_x'].append([round(float(x), 2), round(float(np.interp(x, xs, pl['y'])), 1)])
        out['e_c_x'].append([round(float(x), 2), round(max(-1., min(1., cam(x, float(np.interp(x, xs, pl['y'])), float(np.interp(x, xs, pl['psi']))))), 3)])
    return out


def xband(pl, k_e=150., e_lim=110., k_h=5., h_lim=100., k_y=0., y_lim=80., k_gz=1.0, gz_lim=40., rate=900., cap=300., floor=40., x_dr=True, w_x=None, shadow=False, e_max=1.0, e_band=0.08):
    t = tables(pl); xb = dict(enabled=True)
    if shadow: xb['shadow'] = True
    xb.update(kappa_x=t['kappa_x'], e_c_x=t['e_c_x'], w_x=w_x or W_X, u_x=t['u_x'], h_x=t['h_x'], k_e_us=k_e, e_lo_us=e_lim, e_hi_us=e_lim, k_h_us=k_h, h_lim_us=h_lim, k_gz_us=k_gz, gz_lim_us=gz_lim,
              rate_us_s=rate, cap_rel_us=cap, floor_below_us=floor, e_max=e_max, e_band=e_band, x_dr=x_dr, dr_max_s=0.3)
    if k_y: xb.update(y_x=t['y_x'], k_y_us=k_y, y_lim_us=y_lim, y_hold_s=0.6, y_age_s=0.2)
    return xb


VARIANTS = dict(   # name -> (plan kwargs, band kwargs)
    conservative=(dict(y_t=-9.0, psi_end=2.0, y_end=-10.0, wy=0.10), dict(k_e=80., e_lim=80., k_h=6., h_lim=80., k_y=5., y_lim=60., rate=700., cap=280.)),
    standard=(dict(y_t=-7.0, psi_end=1.0, y_end=-9.0, wy=0.15), dict(k_e=80., e_lim=80., k_h=8., h_lim=100., k_y=8., y_lim=80., rate=900., cap=300.)),
    prerotated=(dict(y_t=-7.0, psi_end=-3.0, y_end=-11.0, wy=0.15), dict(k_e=80., e_lim=80., k_h=8., h_lim=100., k_y=8., y_lim=80., rate=900., cap=300.)),   # optional: 4 deg more turned / 2 cm further in at the hand-back (a milder form of what run c6f3 did before the corner)
)

if __name__ == '__main__':
    for name, (pk, bk) in VARIANTS.items():
        pl = plan(**pk); t = tables(pl)
        print('==== %s  plan %s  band %s' % (name, pk, bk))
        print('   x    lane T  kT   | plan k*   h*   psi*   y*  | logged median h / y | u_x (us)  e_c_x')
        for x in np.arange(X0, X1 + 1e-9, 0.3):
            f = lambda k: float(np.interp(x, pl['x'], pl[k]))
            print('  %.1f  %6.1f %5.1f | %5.1f %6.1f %+5.1f %+5.1f | %6.1f / %+5.1f     | %+5.0f    %+.2f' % (x, Tf(x), (Tf(x + 0.2) - Tf(x - 0.2)) / 0.4, f('k'), f('h'), f('psi'), f('y'), np.interp(x, BX, BH), np.interp(x, BX, BY), sim.ip(t['u_x'], x), sim.ip(t['e_c_x'], x)))
        u = np.array([q[1] for q in t['u_x']]); print('   feed-forward: min %+.0f max %+.0f us, largest change over 0.3 m %.0f us | y* min %+.1f max %+.1f (1.2-4.2 m)' % (u.min(), u.max(), np.abs(u[2:] - u[:-2]).max(), pl['y'][(pl['x'] >= 1.2) & (pl['x'] <= 4.2)].min(), pl['y'][(pl['x'] >= 1.2) & (pl['x'] <= 4.2)].max()))
