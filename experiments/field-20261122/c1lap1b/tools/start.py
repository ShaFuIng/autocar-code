"""c1lap1b: start straight of lap 1 (start -> curve-1 trigger). Heading relative to the pose at the start (hd = 0 at standstill),
IMU lateral drift = integral of sin(hd) over the continuous odometer (cm, + = left, assumes the start pose square to the lane),
camera near error / arc_lat y, start_bias, the steer command; reference heading of the band (plat) relative to the start pose."""
import math, statistics as st, sys
from pathlib import Path
from lib import load, num
from passes import curve_passes, med
from trig import cont
OUT = Path(__file__).resolve().parents[1] / 'proof'
RUNS = ['3f22', 'fd22', 'b23a', '2b72', 'b619', '1377', 'e69d', '54cb', '1cd7', '9af6', 'c6f3', '6f61', 'e0cd', '2728', '7658']


def one(run):
    F = load(run); P = curve_passes(F); so = cont(F, 'odo')
    p = P[0]
    q0 = p['band'][0] if p['band'] else p['j']
    b = [q for q in p['band'] if num(F[q]['tb'])]
    plat = med([F[q]['hd'] + F[q]['tb'] for q in b]) if b else None
    # rest bias: gz median over the first frames (standstill)
    rest = [F[q]['gz'] for q in range(0, q0) if num(F[q]['gz']) and (not num(so[q]) or so[q] < 0.005)]
    bias = med(rest[:20]) if rest else 0.
    # re-integrate heading with bias removed, from t of the first frame
    hd = 0.; tp = None; gp = None; H = []
    for q in range(len(F)):
        f = F[q]
        if num(f['t']) and tp is not None and num(gp): hd += (gp - bias) * (f['t'] - tp)
        if num(f['t']): tp = f['t']
        if num(f['gz']): gp = f['gz']
        H.append(hd)
    # lateral drift
    lat = 0.; sp = None; L = []
    for q in range(len(F)):
        s = so[q]
        if num(s) and sp is not None and s > sp: lat += math.sin(math.radians(H[q])) * (s - sp) * 100.
        if num(s): sp = s
        L.append(lat)
    rows = []
    for target in (0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0, 2.25):
        q = next((q for q in range(q0) if num(so[q]) and so[q] >= target), None)
        if q is None: continue
        f = F[q]
        rows.append((target, H[q], L[q], f['near'], f['y'], f['sb'], f['pre'], f['sent']))
    r = dict(run=run, banded=bool(p['band']), bias=bias, s_trig=so[q0], hd_trig=H[q0], lat_trig=L[q0], near_trig=F[q0]['near'],
             near_rest=med([F[q]['near'] for q in range(0, min(q0, 15)) if num(F[q]['near'])]), y_trig=F[q0]['y'],
             plat_vs_start=None if plat is None else plat - 0. + (H[q0] - F[q0]['hd']),   # plat in the bias-corrected frame (approx.)
             hd_max=max(H[:q0]), hd_min=min(H[:q0]), lat_max=max(L[:q0]), sb_on=any(num(F[q]['sb']) and F[q]['sb'] != 0 for q in range(q0)), rows=rows)
    # entry window (band) y_min
    cur = [F[q] for q in (p['band'] or p['cur'])]
    ys = [f['y'] for f in cur if num(f['y'])]; r['y_min'] = min(ys) if ys else None
    return r


if __name__ == '__main__':
    L = ['start straight of lap 1. hd = heading since the start pose (deg, + = left, gyro bias at rest removed); lat = IMU lateral drift (cm, + = left; assumes the start pose square to the lane);',
         'near = camera near error (+ = lane centre to the right of the car = car left of centre / pointing left); y = arc_lat lateral (cm, + = left; only from ~1.8 m).']
    S = []
    for run in RUNS:
        try: r = one(run)
        except Exception as e: L.append(f'{run}: ERROR {e}'); continue
        S.append(r)
        L.append(f"== {run} band={r['banded']} gyro_bias={r['bias']:.2f} s_trig={r['s_trig']:.2f} hd_trig={r['hd_trig']:.1f} hd_max={r['hd_max']:.1f} hd_min={r['hd_min']:.1f} lat_trig={r['lat_trig']:.1f} lat_max={r['lat_max']:.1f} "
                 f"near_rest={r['near_rest']} near_trig={r['near_trig']} y_trig={r['y_trig']} plat_vs_start={None if r['plat_vs_start'] is None else round(r['plat_vs_start'], 1)} y_min={r['y_min']}")
        for t, h, la, n, y, sb, pre, sent in r['rows']:
            L.append(f"   s={t:4.2f} hd={h:5.1f} lat={la:5.1f} near={None if n is None else round(n, 3)} y={y} sb={sb} pre={None if pre is None else round(pre)} sent={sent}")
    (OUT / 'B2_start_straight.txt').write_text('\n'.join(L), encoding='utf8')
    print('\n'.join(L))
