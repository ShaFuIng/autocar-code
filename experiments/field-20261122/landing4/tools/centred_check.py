"""cross-check of the land4 tables against the hand-placed centred poses (field-20261122/centred/poses.jsonl, 11-22; car placed by hand on the
lane centre, body along the lane, still >= 10 s per point; +-2 cm / +-3 deg; distances along the lane were NOT measured; the preview error
does not include the driving visual_center_offset of 18 px). Used only as an independent check: nothing in tables.json comes from it.
For each still pose after the upper straight: heading relative to the upper straight; for the poses after the landing the table position u
whose theta*(u) equals that heading, and the table's camera reference e_ref(u) next to the centred near error."""
import json, os, sys
from pathlib import Path
import numpy as np

EXP = Path(os.environ.get('L4_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))
T = json.load(open(Path(__file__).with_name('tables.json')))
R = [json.loads(l) for l in open(EXP / 'field-20261122' / 'centred' / 'poses.jsonl', encoding='utf8') if l.strip()]
R = [r for r in R if 'fail' not in r and r.get('gyro')]
t0 = R[0]['t']; tt = np.array([r['t'] - t0 for r in R]); gm = np.array([max(abs(x) for x in r['gyro']) for r in R])
segs = []; s = None
for i, ok in enumerate(gm < 1.0):
    if ok and s is None: s = i
    if (not ok or i == len(R) - 1) and s is not None:
        if tt[i - 1] - tt[s] >= 3.5: segs.append((s, i - 1))
        s = None
P = []
for a, b in segs:
    if tt[b] < 800: continue          # the curve-4 walk starts at the upper straight (t >= 800 s of this file)
    w = R[a + 3:b - 1] if b - a > 8 else R[a:b]
    md = lambda k: float(np.median([x[k] for x in w if x.get(k) is not None])) if any(x.get(k) is not None for x in w) else float('nan')
    P.append(dict(yaw=md('yaw'), pitch=md('pitch'), near=md('err'), valid=float(np.mean([bool(x.get('valid')) for x in w]))))
up = float(np.median([p['yaw'] for p in P[:3]]))
th = T['theta']; er = T['e_ref']


def interp(t, x):
    if x <= t[0][0]: return t[0][1]
    if x >= t[-1][0]: return t[-1][1]
    for (x0, y0), (x1, y1) in zip(t, t[1:]):
        if x0 <= x <= x1: return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def u_of(h):   # first u >= 0 with theta*(u) <= h
    u = 0.0
    while u <= 3.0:
        if interp(th, u) <= h: return u
        u += 0.01
    return None


print('upper-straight yaw (median of the first three poses) %.1f' % up)
print('pose | pitch | heading vs upper straight | centred near error (valid %) | table: u where theta* = heading | theta* on the slope at that pitch stage / e_ref(u)')
landed = False
for k, p in enumerate(P):
    h = p['yaw'] - up
    if p['pitch'] < -5: zone = 'slope'
    elif k < 3: zone = 'upper'
    else: zone = 'after landing'; landed = True
    extra = ''
    if zone == 'after landing' and h > -80:
        u = u_of(h)
        if u is not None: extra = 'u %.2f m | e_ref %.2f (difference centred - table %+.2f)' % (u, interp(er, u), p['near'] - interp(er, u))
    elif zone == 'after landing': extra = '(beyond the table, 3.0 m)'
    print('%2d %-13s | %+6.1f | %+7.1f | %+5.2f (%3.0f%%) | %s' % (k, zone, p['pitch'], h, p['near'], 100 * p['valid'], extra))
print('table on the slope: theta* -2.8 deg at u -1.2, -5.6 at -0.8, -8.4 at -0.4, -10.9 at the landing; e_ref +0.17..+0.20 at u -1.4..-0.7, +0.10 at -0.4')
