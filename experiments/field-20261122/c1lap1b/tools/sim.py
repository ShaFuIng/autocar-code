"""c1lap1b: kinematic OPEN-LOOP estimate (NOT a validated closed-loop model) of the curve-1 band path, for comparing variants by difference.
Lane: straight to x=0, right arc R 1.275 m over ARC_DEG (= the diagonal's heading in band turned-angle terms, 152.5), then straight.
Car (rear axle): curvature kappa_act(s) = GAIN * (u(s - D) + 6) deg/m (c1exit identification: 0.19 deg/m per us, delay 0.20 s = D m).
u = prof(tb) + OFF(tb): prof = kappa_turn(tb) / g(v_ref) (band feed-forward); OFF(tb) = recorded later-lap median of the band's camera +
    gyro terms per 20-deg bin (A2_bins: e_us + gz_us), held FIXED (open loop: the camera term would react to a changed path, so the
    estimated changes are upper bounds); before the band onset u = U_PRE (recorded ~ +40 us).
The onset (metres relative to the arc start) is calibrated so that the baseline reproduces the recorded later-lap y at 60 deg (-9.2 cm);
the model's other points are listed next to the recorded medians as a plausibility check. Only differences are used."""
import math, json
from pathlib import Path
EXP = Path(__file__).resolve().parents[3]
CFG = json.load(open(EXP / 'orin-web-snapshot-20261122c' / 'lap_ff.json', encoding='utf8'))
R = 1.275; ARC_DEG = 152.5; D = 0.23; U_PRE = 40.; GAIN = 0.19
OFF = [[0., 42.], [20., -36.], [40., -18.], [60., -3.], [80., 27.], [100., 10.], [120., 32.], [140., 22.], [160., 22.]]   # bin starts (+10 = centre used below)
OFF = [[a + 10., b] for a, b in OFF]
REC = dict(y60=-9.2, y80=-8.45, y100=-1.35, y120=1.43, y140=8.3, y_exit03=9.3, y_min=-10.9)


def interp(t, x):
    if x <= t[0][0]: return t[0][1]
    for (a, b), (c, d) in zip(t, t[1:]):
        if a <= x <= c: return b + (d - b) * (x - a) / (c - a)
    return t[-1][1]


def lane(x, yv):
    """(offset cm, + = left/outside; lane heading turned; along-lane metres from the arc start)."""
    xc, yc = 0., -R
    ang = math.degrees(math.atan2(x - xc, yv - yc)) % 360.
    if x < 0 and ang > 270: return yv * 100., 0., x
    if ang <= ARC_DEG: return (math.hypot(x - xc, yv - yc) - R) * 100., ang, math.radians(ang) * R
    a = math.radians(ARC_DEG); ex, ey = xc + R * math.sin(a), yc + R * math.cos(a)
    hx, hy = math.cos(a), -math.sin(a); nx, ny = -hy, hx
    return ((x - ex) * nx + (yv - ey) * ny) * 100., ARC_DEG, a * R + (x - ex) * hx + (yv - ey) * hy


def run(table, s_on, y0=0., psi0=0., ramp=0.2, ds=0.004, v_ref=1.10, off=OFF):
    gv = interp(CFG['gain_table'], v_ref)
    x, yv, th, s = -1.5, y0 / 100., psi0, -1.5
    n = int(round(D / ds)); hist = [GAIN * (U_PRE + 6)] * (n + 1); out = []
    while s < 5.0:
        o, lth, sl = lane(x, yv)
        a = 0. if s < s_on else min(1., (s - s_on) / ramp)
        u = U_PRE + a * (interp(table, th) / gv + interp(off, th) - U_PRE)
        hist.append(GAIN * (u + 6)); k = hist[-n - 1]
        th += k * ds; x += math.cos(math.radians(th)) * ds; yv -= math.sin(math.radians(th)) * ds; s += ds
        out.append((s, o, th, lth, sl))
    return out


def summary(o):
    def at_th(deg):
        p = min(o, key=lambda p: abs(p[2] - deg)); return round(p[1], 1)
    end = next((p for p in o if p[3] >= ARC_DEG), None)
    r = dict(y_min=round(min(p[1] for p in o if p[2] > 5), 1), y60=at_th(60), y80=at_th(80), y100=at_th(100), y120=at_th(120), y140=at_th(140))
    if end:
        se = end[4]
        e3 = min(o, key=lambda p: abs(p[4] - (se + 0.3))); e9 = min(o, key=lambda p: abs(p[4] - (se + 0.9)))
        r.update(y_exit03=round(e3[1], 1), y_exit09=round(e9[1], 1), y_exit_max=round(max(p[1] for p in o if se <= p[4] <= se + 2.0), 1),
                 head_exit09=round(e9[2], 1), out_over=round(max(p[1] for p in o if p[4] >= 0), 1))
    return r


TAILS = dict(
    tail_a=[[0.0, 36.2], [28.0, 36.2], [46.0, 40.9], [100.0, 40.9], [113.0, 39.0], [120.0, 35.0], [130.0, 25.0], [140.0, 16.0], [147.0, 8.0], [153.0, -1.1], [165.0, -15.0]],
    tail_b=[[0.0, 36.2], [28.0, 36.2], [46.0, 40.9], [100.0, 40.9], [115.0, 40.9], [125.0, 32.5], [135.0, 22.0], [145.0, 10.0], [153.0, -1.1], [165.0, -15.0]])


def shift_tail(table, from_deg, by):
    return [[a + (by if a >= from_deg else 0.), b] for a, b in table]


if __name__ == '__main__':
    base = CFG['curves']['c1']['band']['kappa_turn']
    L = [__doc__, '']
    best = min(((k / 100., summary(run(base, k / 100.))) for k in range(-80, 41)), key=lambda z: abs(z[1]['y60'] - REC['y60']))
    so0 = best[0]
    L.append(f'calibration onset {so0:+.2f} m (rel. arc start): model {best[1]}')
    L.append(f'recorded later-lap medians:              {REC}')
    L.append('')
    L.append('A. band onset shifted (metres, + = later), baseline table -> estimated change vs calibration:')
    b0 = best[1]
    for d in (-0.15, -0.10, -0.05, 0.05, 0.10, 0.15):
        s = summary(run(base, so0 + d)); L.append(f'  onset {d:+.2f}: {s}   d(y_min) {s["y_min"] - b0["y_min"]:+.1f}  d(y_exit03) {s.get("y_exit03", 0) - b0.get("y_exit03", 0):+.1f}')
    L.append('')
    L.append('B. kappa_turn tail (points >= from_deg) shifted later by +by deg:')
    for fr in (105., 110.):
        for by in (3., 5., 8.):
            s = summary(run(shift_tail(base, fr, by), so0)); L.append(f'  from {fr:.0f} +{by:.0f}: {s}   d(y_exit03) {s.get("y_exit03", 0) - b0.get("y_exit03", 0):+.1f}  d(y140) {s["y140"] - b0["y140"]:+.1f}')
    L.append('')
    L.append('C. tail raised, zero crossing kept at 153 (candidate tails):')
    for name, t in TAILS.items():
        s = summary(run(t, so0)); L.append(f'  {name}: {s}   d(y_exit03) {s.get("y_exit03", 0) - b0.get("y_exit03", 0):+.1f}  d(y140) {s["y140"] - b0["y140"]:+.1f}  d(head_exit09) {s.get("head_exit09", 0) - b0.get("head_exit09", 0):+.1f}')
    (Path(__file__).resolve().parents[1] / 'proof' / 'E1_kinematic_estimate.txt').write_text('\n'.join(L), encoding='utf8')
    print('\n'.join(L))
