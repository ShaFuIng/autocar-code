"""curve 3 front section: PERTURBATION model around the logged median pass (estimate only).
  state = deviation from the median pass of the 20 passes without the curve-3 trim: d_psi (deg, + = car points more to the outside),
  d_y (cm, + = further outside).   d(d_psi)/ds = -G * d_u(delayed), d(d_y)/ds = 100 * sin(d_psi).
  camera:  d_e = B * d_y + C * d_psi            (B, C = least squares of the logged passes, c3err.py)
  chain :  d_u = K * d_e - trim(x) + floor change   (K = logged slope of the final command against the error inside an x bin, c3err.py;
           trim only while the command is above centre + min_rel_us, as curve_trim does; the arc2 c3 floor is 'raise only':
           u += a * max(0, frac * kappa(x) / g - u), a = 0.9 * the layer's inside-yield of the model error; lowering the table removes the logged mean push of the old floor)
  command limited to +396 (the logged ceiling in curve 3).
Limits: linear camera model (residual sd 0.115), one gain K for the whole chain (the real chain has the PID D term, the curve_ff step at
0.55, curve_smooth, yaw_inner), no backlash, lane shape and median pass taken as fixed. The model reproduces nothing by itself (it starts
from the logged median); it only estimates how far a schedule change moves the car and how much the feedback gives back.
usage: python sim_c3.py <curve_trim.json> <arc2.json>   (files from on/; '-' = the car's file of the snapshot)"""
import json, sys, math, numpy as np, lib

G = 0.18; DELAY_M = 0.13; LAG_M = 0.13; DS = 0.01
E = json.load(open(lib.HERE / 'c3err.json')); B = E['b']; C = E['c']; KX = E['Kx']
SNAP = lib.EXP / 'orin-web-snapshot-20261122'
GT = [[0.8, 0.192], [0.93, 0.167], [1.06, 0.159], [1.22, 0.15], [1.37, 0.13]]
# logged medians per 0.3 m (proof B1 / B2 / B3): x, final command - centre, y cm, psi deg, near error, speed, median a of the c3 floor, mean us the old floor added over all frames
MED = [(0.15, 182, -1.3, 2.0, 0.54, 0.87, 0.0, 0.0), (0.45, 115, -3.3, -4.9, 0.29, 0.89, 0.0, 0.0), (0.75, 74, -6.4, -5.7, 0.17, 0.90, 0.48, 0.6), (1.05, 100, -11.1, -7.4, 0.31, 0.93, 1.0, 0.6),
       (1.35, 163, -14.2, -3.8, 0.43, 0.99, 1.0, 13.2), (1.65, 214, -14.1, 1.5, 0.53, 1.06, 1.0, 13.2), (1.95, 204, -13.6, 2.7, 0.52, 1.08, 1.0, 19.0), (2.25, 169, -10.3, 5.8, 0.36, 1.13, 1.0, 19.0),
       (2.55, 89, -8.9, -1.4, 0.02, 1.13, 0.78, 14.4), (2.85, 30, -11.2, -8.5, -0.12, 1.11, 0.29, 14.4), (3.15, 48, -16.0, -10.2, -0.10, 1.04, 0.16, 0.5), (3.45, 125, -18.5, 2.4, 0.30, 0.97, 0.89, 0.5),
       (3.75, 174, -16.6, 7.8, 0.48, 0.92, 1.0, 2.7), (4.05, 183, -12.8, 8.3, 0.47, 0.92, 1.0, 2.7), (4.35, 195, -8.8, 7.1, 0.53, 0.97, 1.0, 3.8)]


def ip(t, x):
    if x <= t[0][0]: return float(t[0][1])
    for (x0, y0), (x1, y1) in zip(t, t[1:]):
        if x <= x1: return float(y0) + (float(y1) - float(y0)) * (x - float(x0)) / max(1e-9, float(x1) - float(x0))
    return float(t[-1][1])


def col(k): return [(m[0], m[k]) for m in MED]


def run(trim_cfg, arc2_new, arc2_old, K_scale=1.0):
    c3t = ((trim_cfg or {}).get('curves') or {}).get('c3') if trim_cfg else None
    if c3t is not None and not c3t.get('enabled'): c3t = None
    kn = sorted((float(a), float(b)) for a, b in arc2_new['c3']['kappa_table']); ko = sorted((float(a), float(b)) for a, b in arc2_old['c3']['kappa_table']); fr = float(arc2_new['c3'].get('frac', 0.7))
    x = 0.0; dpsi = 0.0; dy = 0.0; n_delay = int(round(DELAY_M / DS)); buf = [0.0] * n_delay; du_lag = 0.0; rel = False; out = []
    while x < 4.5:
        de = B * dy + C * dpsi; u0 = ip(col(1), x); e = ip(col(4), x) + de
        u = u0 + K_scale * ip(KX, x) * de
        if ip(ko, x) > ip(kn, x) + 1e-9: u -= ip(col(7), x) * min(1., (ip(ko, x) - ip(kn, x)) / max(1e-6, ip(ko, x)))   # the old floor's logged mean push, removed in proportion to the lowering
        if ip(kn, x) > ip(ko, x) + 1e-9:   # raised floor: u + a * max(0, target - u), a = the layer's inside-yield (1 at near >= +0.10 .. 0 at <= -0.15; the logged heading guard was ~0.8-1 here)
            ay = max(0., min(1., (e + 0.15) / 0.25)); u = u + 0.9 * ay * max(0., fr * ip(kn, x) / ip(GT, ip(col(5), x)) - u)
        tr = 0.
        if c3t is not None and x >= c3t['table'][0][0] and x <= c3t['table'][-1][0]:
            if e >= float(c3t.get('e_release', 0.9)): rel = True
            if not rel:
                fl = float(c3t.get('min_rel_us', 60.)); tr = ip(c3t['table'], x)
                un = u if u <= fl else max(fl, u - tr); tr = u - un; u = un
        u = min(396., u); du = u - u0
        buf.append(du); dd = buf.pop(0); du_lag += (dd - du_lag) * (1 - math.exp(-DS / LAG_M))
        out.append((x, u, du, tr, ip(col(2), x) + dy, ip(col(3), x) + dpsi, e, dy, dpsi))
        dpsi += -G * du_lag * DS; dy += 100. * math.sin(math.radians(dpsi)) * DS; x += DS
    return np.array(out)


def show(name, R):
    print(name); print('    x     cmd (was)   trim   y cm (was)    psi (was)    err (was)')
    for m in MED:
        i = int(np.argmin(np.abs(R[:, 0] - m[0]))); r = R[i]
        print('   %.2f  %+5.0f (%+4.0f)  %4.0f  %+6.1f (%+5.1f)  %+5.1f (%+5.1f)  %+.2f (%+.2f)' % (m[0], r[1], m[1], r[3], r[4], m[2], r[5], m[3], r[6], m[4]))
    f = lambda lo, hi, k, fn: fn(R[(R[:, 0] >= lo) & (R[:, 0] < hi), k])
    print('   front y min %+.1f (was -18.5) | y at 4.35 m %+.1f (was -8.8), psi %+.1f (was +7.1) | error max 1.2-2.4 m %.2f (was 0.53), 3.3-4.4 m %.2f (was 0.53)' % (
        f(0.6, 4.2, 4, np.min), R[int(np.argmin(np.abs(R[:, 0] - 4.35))), 4], R[int(np.argmin(np.abs(R[:, 0] - 4.35))), 5], f(1.2, 2.4, 6, np.max), f(3.3, 4.4, 6, np.max)))


if __name__ == '__main__':
    ld = lambda p, d: json.load(open(SNAP / d if p == '-' else p, encoding='utf8'))
    old = json.load(open(SNAP / 'arc2.json', encoding='utf8'))
    tr = ld(sys.argv[1], 'curve_trim.json'); a2 = ld(sys.argv[2], 'arc2.json')
    if sys.argv[1] == '-': tr['curves']['c3']['enabled'] = False   # the comparison base is the car WITHOUT the curve-3 trim (run d60e)
    for ks in (1.0, 0.6, 1.5):
        show('%s + %s | chain gain x %.1f' % (sys.argv[1], sys.argv[2], ks), run(tr, a2, old, ks))
