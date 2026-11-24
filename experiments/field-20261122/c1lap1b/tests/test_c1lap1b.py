"""c1lap1b tests (offline; python tests/test_c1lap1b.py). Uses the car's lap_ff.py from orin-web-snapshot-20261122c."""
import json, sys, hashlib, importlib.util, copy
from pathlib import Path
HERE = Path(__file__).resolve().parents[1]; EXP = HERE.parents[1]; SNAP = EXP / 'orin-web-snapshot-20261122c'
sys.path.insert(0, str(SNAP)); sys.path.insert(0, str(HERE / 'tools'))
spec = importlib.util.spec_from_file_location('lap_ff_snap', SNAP / 'lap_ff.py'); LF = importlib.util.module_from_spec(spec); spec.loader.exec_module(LF)
import mkcfg
BASE = json.load(open(SNAP / 'lap_ff.json', encoding='utf8'))
N = 0


def ok(c, m):
    global N
    assert c, m
    N += 1


def flat(d, p=''):
    out = {}
    if isinstance(d, dict):
        for k, v in d.items(): out.update(flat(v, f'{p}.{k}' if p else k))
    else: out[p] = d
    return out


def test_base_hashes():
    ok(hashlib.sha256((SNAP / 'lap_ff.json').read_bytes()).hexdigest().startswith('6576d4df'), 'base lap_ff.json is the car file 6576d4df')
    ok(hashlib.sha256((SNAP / 'lap_ff.py').read_bytes()).hexdigest().startswith('80976389'), 'base lap_ff.py 80976389')


def test_only_listed_keys():
    allowed = {'c1_lap1_win': {'curves.c1.start.win_m', 'curves.c1.start.note_c1lap1b'},
               'c1_tail_a': {'curves.c1.band.kappa_turn', 'curves.c1.band.note_c1lap1b_tail'},
               'c1_tail_b': {'curves.c1.band.kappa_turn', 'curves.c1.band.note_c1lap1b_tail'},
               'combo_lap1_win__tail_a': {'curves.c1.start.win_m', 'curves.c1.start.note_c1lap1b', 'curves.c1.band.kappa_turn', 'curves.c1.band.note_c1lap1b_tail'}}
    fb = flat(BASE)
    for name, al in allowed.items():
        c = json.load(open(HERE / 'on' / name / 'lap_ff.json', encoding='utf8')); fc = flat(c)
        changed = {k for k in set(fb) | set(fc) if fb.get(k, '<absent>') != fc.get(k, '<absent>')}
        ok(changed <= al and changed, f'{name}: changed keys {changed} within {al}')
        ok(mkcfg.build(BASE, win='win' in name, tail=(mkcfg.TAIL_A if 'tail_a' in name else mkcfg.TAIL_B if 'tail_b' in name else None)) == c, f'{name} reproducible by mkcfg')


def test_tail_shape():
    for t in (mkcfg.TAIL_A, mkcfg.TAIL_B):
        old = BASE['curves']['c1']['band']['kappa_turn']
        ok([p for p in t if p[0] <= 100] == [p for p in old if p[0] <= 100], 'table up to 100 deg unchanged')
        ok([p for p in t if p[0] >= 153] == [p for p in old if p[0] >= 153], 'zero crossing at 153 and the 165 point unchanged')
        for x in range(101, 153):
            ok(LF.interp(t, x) >= LF.interp(old, x) - 1e-9, f'tail never lower than the old table at {x}')
        ok(all(a[1] >= b[1] for a, b in zip(t, t[1:]) if a[0] >= 100), 'monotone non-increasing from 100 deg')


def trig_of(cfg):
    c = dict(LF.DEFAULTS); c.update(cfg); return LF.Trigger(c['curves'], c['far_on'], c['near_min'])


def test_lap1_window():
    for name, first_arm in (('base', 2.0), ('c1_lap1_win', 2.38)):
        cfg = BASE if name == 'base' else json.load(open(HERE / 'on' / name / 'lap_ff.json', encoding='utf8'))
        L = LF.LapFF(dict(LF.DEFAULTS, **cfg))
        # lap 0, no anchor -> START pseudo anchor; far already high everywhere: trigger at the first armed reading
        fired = None
        for i in range(0, 60):
            odo = round(i * 0.06, 3)
            lp = dict(lap=0, anchor=None, anchor_d=None, since_anchor_m=None, lost=False, odo_m=odo, yaw_imu=0., gz_imu=0.)
            st, inf = L.step(lp, 0.6, 0.1, True, 1.1, 0., 0., 1500., 1475., 780., 2050., True)
            if inf is not None and fired is None: fired = odo
        ok(fired is not None and abs(fired - first_arm) < 0.061, f'{name}: lap-1 trigger at odo {fired} (expected first armed reading ~{first_arm})')
    # later laps: the C4e-anchored window is identical in both configs
    for name in ('base', 'c1_lap1_win'):
        cfg = BASE if name == 'base' else json.load(open(HERE / 'on' / name / 'lap_ff.json', encoding='utf8'))
        T = trig_of(cfg)
        a = T.armed(dict(lap=2, anchor='C4e', anchor_d=60., since_anchor_m=8.95, lost=False, odo_m=70.))
        b = T.armed(dict(lap=2, anchor='C4e', anchor_d=60., since_anchor_m=8.93, lost=False, odo_m=70.))
        ok(a is not None and b is None, f'{name}: later-lap window starts at C4e+8.94 m (unchanged)')


if __name__ == '__main__':
    for k, v in list(globals().items()):
        if k.startswith('test_'): v(); print('PASS', k)
    print('checks', N)
