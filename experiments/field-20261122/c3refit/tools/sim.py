"""c3refit: SIMPLE closed-loop model of curve 3 (C3s + 0.3 .. 6.6 m). An estimate, not a prediction: see the calibration (calib.py) for what it does and does not reproduce.
Pieces (all from ident.py -> model.json):
  lane      T(x): lane heading since C3s (deg) on the controller's distance axis; psi = T(x) - car heading (+ = car pointing outward); dy/dx = 100 sin(psi)  [cm/m, + = outward]
  plant     car curvature (deg/m) = G * (u delayed by tau s, first-order lag tl s, + U0), u = command - centre before the backlash stage (us)
  camera    near error = clip(F(x) + B*y + C*psi + noise, -1, 1), a new value every 1/30 s
  chain     outside the band: command = a(x) + K(x) * error (per 0.3 m bin, fitted on the logged final command), limited to [-200, +396]
  band      the x-band law of lap_ff.py (r1 = adcde919; r2 = c3refit keys x_dr / u_x / h_x / y_x / rate_us_s), same formula as the code (tests/ compares the two)
  arc_lat   y of the previous frame (+ 1 cm noise), no value outside +-20 cm (the car's Y_RANGE), then the last one is held y_hold_s
  distance  the controller's x is the 5 Hz encoder staircase (x_dr false) or staircase + speed * time since the last step (x_dr true)"""
import json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
M = json.load(open(HERE / 'model.json'))
GT = [[0.8, 0.192], [0.93, 0.167], [1.06, 0.159], [1.22, 0.15], [1.37, 0.13]]   # lap_ff gain table as seen in the log (g vs wheel speed)
DT = 1. / 150.
CAM = 'cam2'   # 'cam' = one B / C for the whole curve, 'cam2' = zone-wise slopes (ident2.py)


def ip(t, x):
    if x <= t[0][0]: return float(t[0][1])
    for (x0, y0), (x1, y1) in zip(t, t[1:]):
        if x <= x1: return float(y0) + (float(y1) - float(y0)) * (x - float(x0)) / max(1e-9, float(x1) - float(x0))
    return float(t[-1][1])


def band_target(X, x, near, v, g, gz, turned, last_tgt, dt, ym=None):
    """the x-band target relative to the centre (us): r1 formula + the r2 terms when their keys are present (same arithmetic as lap_ff.py _xb / _xb2). -> (target, parts)"""
    kap = ip(X['kappa_x'], x); prof = ip(X['u_x'], x) if X.get('u_x') else kap / g; ec = ip(X['e_c_x'], x)
    et = max(-X['e_lo_us'], min(X['e_hi_us'], X['k_e_us'] * (near - ec)))
    gt = max(-X['gz_lim_us'], min(X['gz_lim_us'], X['k_gz_us'] * (gz + v * kap))) if X.get('k_gz_us') else 0.
    ht = 0.; yt = 0.
    if X.get('h_x') and X.get('k_h_us') and turned is not None and abs(ip(X['h_x'], x) - turned) <= X.get('h_dev_max', 25.): ht = max(-X.get('h_lim_us', 80.), min(X.get('h_lim_us', 80.), X['k_h_us'] * (ip(X['h_x'], x) - turned)))
    if X.get('y_x') and X.get('k_y_us') and ym is not None: yt = max(-X.get('y_lim_us', 80.), min(X.get('y_lim_us', 80.), X['k_y_us'] * (ym - ip(X['y_x'], x))))
    raw = max(-X['floor_below_us'], min(X['cap_rel_us'], prof + et + gt + ht + yt)); tgt = raw
    if X.get('rate_us_s') and last_tgt is not None: r = X['rate_us_s'] * dt; tgt = max(last_tgt - r, min(last_tgt + r, raw))
    return tgt, dict(prof=prof, et=et, gt=gt, ht=ht, yt=yt, ec=ec, raw=raw)


def run(xb=None, x0=0.3, h0=None, y0=None, v_of_x=None, x_end=6.6, seed=0, noise=0., g_scale=1., u0_shift=0., xerr=0., cam_bias=None, chain_scale=1., T=None, plant=None, stair=True, h_bias=0., y_bias=0., y_noise=1.0, y_off=False):
    """-> array rows (x, u, y, psi, e, h, kcar, share, x_ctrl). xb = x-band dict or None (chain only). xerr = the controller's x reads this much more than the true distance."""
    T = T or M['T']; P = dict(M['plant']); P.update(plant or {}); cam = M[CAM]; ch = M['chain']; rng = np.random.default_rng(seed)
    bm = M['base_med']; bx = bm['x']
    if h0 is None: h0 = float(np.interp(x0, bx, [np.nan if q is None else q for q in bm['h']]))
    if y0 is None: y0 = float(np.interp(x0, bx, [np.nan if q is None else q for q in bm['y']]))
    vf = v_of_x or (lambda x: 1.0)
    x = x0; h = h0; y = y0; t = 0.; e = None; ul = None; buf = []; out = []; tick_t = -rng.uniform(0, 0.2); xs = x0; t_e = -1.; last_tgt = None; u = 0.; kc = ip(T, x0 + 0.05) * 0 + 25.
    nd = max(1, int(round(P['tau'] / DT))); t_c = -1.; share = 0.; ts = 0.; ym = None; ym_t = -9.; y_prev = None
    Fx = cam['F']; cha = [[c[0], c[1]] for c in ch]; chk = [[c[0], c[2]] for c in ch]
    while x < x_end:
        v = vf(x); psi = ip(T, x) - h
        if t - t_e >= 1. / 30. - 1e-9:   # a camera frame: new error, new command
            dtc = t - t_c if t_c >= 0 else 1. / 30.; t_c = t; t_e = t
            if 'Z' in cam:
                z = max(0, min(len(cam['B']) - 1, sum(1 for q in cam['Z'] if q <= x) - 1)); Bz = cam['B'][z]; Cz = cam['C'][z]
            else: Bz = cam['B']; Cz = cam['C']
            e = ip(Fx, x) + Bz * y + Cz * psi + (rng.normal(0, noise) if noise else 0.) + (ip(cam_bias, x) if cam_bias else 0.)
            e = max(-1., min(1., e))
            if t - tick_t >= 0.2: tick_t += 0.2 * math.floor((t - tick_t) / 0.2); xs = x; ts = t   # encoder step
            xc_ = (xs if stair else x) + xerr
            if xb is not None and xb.get('x_dr') and stair: xc_ = xs + xerr + v * min(t - ts, float(xb.get('dr_max_s', 0.3)))
            if y_prev is not None and abs(y_prev) <= 20. and not y_off: ym = y_prev + y_bias + (rng.normal(0, y_noise) if noise else 0.); ym_t = t   # arc_lat: previous frame, +-20 cm range
            y_prev = y
            chain = max(-200., min(396., chain_scale * (ip(cha, x) + ip(chk, x) * e)))
            u = chain; share = 0.
            if xb is not None:
                w = ip(xb['w_x'], xc_)
                if w > 0.:
                    g = ip(GT, v); tgt, _ = band_target(xb, xc_, e, v, g, -kc * v, h + h_bias, last_tgt, dtc, ym if t - ym_t <= float(xb.get('y_hold_s', 0.6)) else None); last_tgt = tgt
                    we = max(0., min(1., (xb['e_max'] - e) / max(1e-3, xb['e_band']))); share = w * we; u = chain + share * (tgt - chain)
                else: last_tgt = None
        buf.append(u); ud = buf.pop(0) if len(buf) > nd else buf[0]
        ul = ud if ul is None else ul + (ud - ul) * (1 - math.exp(-DT / P['tl']))
        kc = P['G'] * g_scale * (ul + P['U0'] + u0_shift)
        out.append((x, u, y, psi, e, h, kc, share, xc_ if xb is not None else x))
        dx = v * DT; h += kc * dx; y += 100. * math.sin(math.radians(psi)) * dx; x += dx; t += DT
    return np.array(out)


def binned(R, edges, col):
    return np.array([np.median(R[(R[:, 0] >= a) & (R[:, 0] < b), col]) if ((R[:, 0] >= a) & (R[:, 0] < b)).any() else np.nan for a, b in zip(edges[:-1], edges[1:])])
