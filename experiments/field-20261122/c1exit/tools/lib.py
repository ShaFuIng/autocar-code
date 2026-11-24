"""c1exit tools: shared loader (laptop only; read-only on run folders).
env C1X_EXP = experiments folder, C1X_CACHE = per-run pickle cache (default: system temp / c1exit; delete after use)."""
import json, os, pickle, tempfile
from pathlib import Path
import numpy as np

EXP = Path(os.environ.get('C1X_EXP', r'D:\OneDrive\Desktop\自走車\Orin_Interface_v1\laptop_workbench\data\experiments'))
HERE = Path(__file__).resolve().parent
CACHE = Path(os.environ.get('C1X_CACHE', Path(tempfile.gettempdir()) / 'c1exit'))
MAIN = ['e0cd', '6f61', 'c6f3']                 # same curve-1 settings; curve 2 slightly different (LOG 17:35 / 18:01 / 18:16)
MORE = ['4bbe', '1eab', 'ccf1', 'd60e']         # more curve-1 samples (same curve-1 settings since 14:22)
SHORT = ['de46', '5aa9', '8019', '676f', 'e757', '370e', '8ae8', 'b6e9', 'bcb9', '6ba6', 'f126']
CENTRE = 1475.
M_PER_COUNT = 0.000222
KEEP_V = ('frame_seq', 'capture_at', 'consumed_at', 'valid')


def load(run):
    CACHE.mkdir(parents=True, exist_ok=True); pk = CACHE / (run + '.pkl')
    if pk.exists(): return pickle.loads(pk.read_bytes())
    Q = []
    with open(EXP / ('run-%s-20261122' % run) / 'pid.jsonl', encoding='utf8') as f:
        for l in f:
            if '"kind": "sample"' not in l[:40]: continue
            d = json.loads(l); p = d['proposal']
            if 't' not in p: continue
            v = d.get('vision') or {}; p['_v'] = {k: v.get(k) for k in KEEP_V}
            pd_ = v.get('path_diagnostic') or {}
            p['_le'] = pd_.get('line_estimate') if isinstance(pd_, dict) else None
            Q.append(p)
    lap = 0; last = None
    for p in Q:
        s = (p.get('lap_pos') or {}).get('seg') or last
        if s == 'curve1' and last != 'curve1': lap += 1
        p['_lap'] = lap; p['_s'] = s; last = s
    pk.write_bytes(pickle.dumps(Q)); return Q


def g(d, *ks, default=np.nan):
    for k in ks:
        d = d.get(k) if isinstance(d, dict) else None
        if d is None: return default
    return d


def speed(t, enc, win=0.5):
    v = np.full(len(t), np.nan); j0 = 0; j1 = 0
    for i in range(len(t)):
        while t[j0] < t[i] - win / 2: j0 += 1
        while j1 + 1 < len(t) and t[j1 + 1] <= t[i] + win / 2: j1 += 1
        if j1 > j0 and t[j1] - t[j0] > 0.2 and np.isfinite(enc[j0]) and np.isfinite(enc[j1]): v[i] = (enc[j1] - enc[j0]) * M_PER_COUNT / (t[j1] - t[j0])
    return v
