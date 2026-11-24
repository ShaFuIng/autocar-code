"""c1lap1b: build the candidate lap_ff.json files from a base lap_ff.json (default: snapshot-20261122c 6576d4df = car at 20:2x).
Only curves.c1.start.win_m and / or curves.c1.band.kappa_turn are changed; everything else is copied byte-for-byte in value.
Usage: python mkcfg.py [--base PATH] --out DIR"""
import json, sys, copy, hashlib
from pathlib import Path
EXP = Path(__file__).resolve().parents[3]
TAIL_A = [[0.0, 36.2], [28.0, 36.2], [46.0, 40.9], [100.0, 40.9], [113.0, 39.0], [120.0, 35.0], [130.0, 25.0], [140.0, 16.0], [147.0, 8.0], [153.0, -1.1], [165.0, -15.0]]
TAIL_B = [[0.0, 36.2], [28.0, 36.2], [46.0, 40.9], [100.0, 40.9], [115.0, 40.9], [125.0, 32.5], [135.0, 22.0], [145.0, 10.0], [153.0, -1.1], [165.0, -15.0]]
WIN = [-0.82, 1.0]
NOTE_W = ('c1lap1b (field-20261122/c1lap1b, offline, 2026-11-22): lap-1 window start -1.2 -> -0.82 (armed when the odometer reading lap_ff sees is >= 2.38 m '
          'from the start). The camera curvature marker of curve 1 sits at 1.93 m from the start (sd 0.026, 9 starts) but the FAR trigger fired 0.13 m earlier '
          'on lap 1 (car left of centre after the start bias, near error +0.12 -> far reaches 0.4 early) and the band started its turn 0.08-0.21 m earlier '
          'than on the later laps -> curve-1 lap-1 inner clearance -7.4 (median) vs -3.9. Off = start.win_m back to [-1.2, 1.0]. Later laps unaffected.')
NOTE_T = ('c1lap1b tail: kappa_turn 105-150 deg raised (the zero crossing at 153 kept): the band camera term was +30..+66 us over 80-150 deg on all laps and '
          'the car drifted from -8 cm (80 deg) to +9 cm outside at C1e+0.3 (clr_out -1.4..-4.6 in 10/10 later passes, at C1e+0.5). Back = the previous table.')


def build(base, win=False, tail=None):
    c = copy.deepcopy(base); c1 = c['curves']['c1']
    if win: c1['start']['win_m'] = list(WIN); c1['start']['note_c1lap1b'] = NOTE_W
    if tail is not None: c1['band']['kappa_turn'] = copy.deepcopy(tail); c1['band']['note_c1lap1b_tail'] = NOTE_T
    return c


def dump(c, p):
    p.parent.mkdir(parents=True, exist_ok=True)
    s = json.dumps(c, ensure_ascii=False, indent=1); p.write_text(s, encoding='utf8')
    return hashlib.sha256(p.read_bytes()).hexdigest()[:8]


if __name__ == '__main__':
    a = sys.argv[1:]; base_p = Path(a[a.index('--base') + 1]) if '--base' in a else EXP / 'orin-web-snapshot-20261122c' / 'lap_ff.json'
    out = Path(a[a.index('--out') + 1])
    base = json.load(open(base_p, encoding='utf8'))
    for name, kw in (('c1_lap1_win', dict(win=True)), ('c1_tail_a', dict(tail=TAIL_A)), ('c1_tail_b', dict(tail=TAIL_B)),
                     ('combo_lap1_win__tail_a', dict(win=True, tail=TAIL_A))):
        h = dump(build(base, **kw), out / name / 'lap_ff.json'); print(name, h)
