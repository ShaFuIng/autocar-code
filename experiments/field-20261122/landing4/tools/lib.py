"""shared loader for the landing4 tools (pickles written by mk.py)."""
import pickle, os, tempfile
CACHE = os.environ.get('L4_CACHE', os.path.join(tempfile.gettempdir(), 'l4'))
RUNS = ['e1b4', '65ea', '3197', '8350', '46c9']
C0 = 1475.


def load(r):
    start, Q = pickle.load(open(os.path.join(CACHE, r + '.pkl'), 'rb'))
    lap = 0; last = None
    for i, p in enumerate(Q):
        s = (p.get('lap_pos') or {}).get('seg') or last
        if s == 'curve1' and last != 'curve1': lap += 1
        p['_lap'] = lap; p['_s'] = s; last = s; p['_i'] = i
    return start, Q


def g(p, *ks, d=None):
    x = p
    for k in ks:
        if not isinstance(x, dict): return d
        x = x.get(k)
        if x is None: return d
    return x


def pitch(p): return g(p, '_v', 'dash_route', 'pitch_rel_deg')


def sent(p):
    pc = p.get('play_comp')
    if isinstance(pc, dict) and pc.get('steer_us') is not None: return float(pc['steer_us'])
    return p.get('steer_us')
