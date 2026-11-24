"""Straight-line test support for page 04 (laptop only, read-only on records).

Reads PID runs already synchronised from Orin (data/orin/records/manual/<id>/
<version>/pid.jsonl, verified by the existing one-way sync), computes whole-run
and straight-segment metrics, returns per-sample series for plotting, and can
save a preset parameter set as a new visual-pid revision (same validation and
revision check as page 04). It never talks to the vehicle.

A frame counts as "straight" when the selected path's quadratic coefficient of
x(y) over image rows 140-340 (640x360) is below CURV_MAX*1e-3 and the visible
span is >= 80 rows; a segment needs >= MIN_SEG_S seconds. These are analysis
choices for comparing runs, not safety limits.
"""
import json, time, math, statistics, threading, hashlib
from pathlib import Path

# Standard library only: the workbench server runs on Python 3.12 without numpy.


def _mean(v): return sum(v) / len(v)
def _std(v):
    m = _mean(v); return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))
def _ptp(v): return max(v) - min(v)


def _quad_coef(ys, xs):
    """Least-squares x = a*y^2 + b*y + c; returns a (centred y for conditioning)."""
    m = _mean(ys); y = [v - m for v in ys]; n = len(y)
    s1, s2, s3, s4 = sum(y), sum(v * v for v in y), sum(v ** 3 for v in y), sum(v ** 4 for v in y)
    t0, t1, t2 = sum(xs), sum(a * b for a, b in zip(y, xs)), sum(a * a * b for a, b in zip(y, xs))
    A = [[s4, s3, s2], [s3, s2, s1], [s2, s1, n]]; B = [t2, t1, t0]
    det = lambda M: (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]))
    D = det(A)
    if abs(D) < 1e-12: return None
    Aa = [[B[i], A[i][1], A[i][2]] for i in range(3)]
    return det(Aa) / D

CURV_MAX = 1.0
MIN_SEG_S = 3.0
PARAM_KEYS = ('kp', 'ki', 'kd', 'integral_limit', 'row_ratio', 'steer_rate', 'pwm', 'left_gain', 'right_gain', 'derivative_tau')

# Candidates from the 2026-10-31 straight-line review: change one item per run.
PRESETS = {
    'rev93-like': dict(label='基準：Kp240／Ki0／Kd150（rev93 同組）', params=dict(kp=240, ki=0, kd=150, integral_limit=.3)),
    'rev95-like': dict(label='目前候選：Kp170／Ki60／Kd150（rev95 同組）', params=dict(kp=170, ki=60, kd=150, integral_limit=1.5)),
    'row-0.55': dict(label='看遠一點：取樣列 0.60 → 0.55（需重新載入預覽）', params=dict(row_ratio=.55)),
    'steer-500': dict(label='舵機速率 300 → 500 µs/s', params=dict(steer_rate=500)),
    'pwm-12': dict(label='開慢一點：PWM 15 → 12', params=dict(pwm=12)),
}


def _records(app):
    out = []
    for cur in (app.orin.root / 'records' / 'manual').glob('*/current.json'):
        try:
            item = json.loads(cur.read_text()); folder = cur.parent / item['version']
        except Exception:
            continue
        if (folder / 'pid.jsonl').exists(): out.append((cur.parent.name, folder))
    return out


def _load(folder):
    start, samples, stop, stopdiag = None, [], None, None
    for line in (folder / 'pid.jsonl').read_text(encoding='utf8').splitlines():
        try: r = json.loads(line)
        except Exception: continue
        k = r.get('kind')
        if k == 'start': start = r
        elif k == 'sample': samples.append(r)
        elif k == 'stop': stop = r
        elif k == 'stop_diagnostic': stopdiag = r
    return start, samples, stop, stopdiag


def _series(start, samples):
    t0 = start['started']; rows = []
    for x in samples:
        p = x.get('proposal') or {}
        if 'p' not in p or p.get('error') is None: continue
        v = x.get('vision') or {}; j = (v.get('path_diagnostic') or {}).get('junction') or {}
        q = [(x / 2, y / 2) for x, y in (v.get('path') or []) if 140 <= y / 2 <= 340]; curv = None
        if len(q) >= 6 and _ptp([y for _, y in q]) >= 80:
            a = _quad_coef([y for _, y in q], [x for x, _ in q]); curv = None if a is None else abs(a) * 1e3
        rows.append(dict(t=round(x['wall'] - t0, 3), e=float(p['error']), u=float(p['steer_us']), P=float(p['p']), I=float(p.get('i', 0) or 0), D=float(p['d']),
                         curv=curv, src=j.get('source'), lim=bool(p.get('limited'))))
    return rows


def _segments(rows):
    segs, cur = [], []
    for r in rows:
        straight = r['curv'] is not None and r['curv'] < CURV_MAX
        if straight and (not cur or r['t'] - cur[-1]['t'] < .4): cur.append(r); continue
        if cur and cur[-1]['t'] - cur[0]['t'] >= MIN_SEG_S: segs.append(cur)
        cur = [r] if straight else []
    if cur and cur[-1]['t'] - cur[0]['t'] >= MIN_SEG_S: segs.append(cur)
    return segs


def _metrics(rows):
    if len(rows) < 3: return None
    e = [r['e'] for r in rows]; u = [r['u'] for r in rows]; t = [r['t'] for r in rows]
    sign = lambda v: (v > 0) - (v < 0)
    zc = [t[i] for i in range(1, len(e)) if sign(e[i]) != sign(e[i - 1])]
    gaps = [b - a for a, b in zip(zc, zc[1:])]
    return dict(seconds=round(t[-1] - t[0], 1), n=len(rows), e_mean=round(_mean(e), 3), e_std=round(_std(e), 3), e_pp=round(_ptp(e), 3),
                e_over_half=round(_mean([abs(v) > .5 for v in e]), 2), steer_mean=round(_mean(u)), steer_std=round(_std(u)),
                cross_interval_s=round(statistics.median(gaps), 2) if len(zc) > 2 else None,
                d_over_p=round(_mean([abs(r['D']) for r in rows]) / max(1e-9, _mean([abs(r['P']) for r in rows])), 2),
                rate_limited=round(_mean([r['lim'] for r in rows]), 2))


def _summary(rid, folder, start, samples, stop, stopdiag, rows):
    segs = _segments(rows); st = [_metrics(s) for s in segs]
    straight = _metrics([r for s in segs for r in s])
    par = start.get('parameters') or {}
    last = (stopdiag or (samples[-1] if samples else None))
    return dict(id=rid, version=folder.name, started=start.get('started'), seconds_set=start.get('seconds'), mode=start.get('execution_mode'),
                path_algorithm=start.get('path_algorithm'), parameter_revision=start.get('parameter_revision'), params={k: par.get(k) for k in PARAM_KEYS},
                ran_s=round(last['wall'] - start['started'], 1) if last else None, stop=(stop or {}).get('reason'),
                whole=_metrics(rows), straight=straight, straight_segments=st, segment_spans=[[s[0]['t'], s[-1]['t']] for s in segs])


class StraightTest:
    def __init__(self, app):
        self.app = app; self.cache = {}; self.lock = threading.Lock()
        self.root = app.store.root / 'straight_test'; self.root.mkdir(parents=True, exist_ok=True)
        # run summaries persist across server restarts (parsing every pid.jsonl takes ~90 s); keyed by this file's hash
        self.code = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()[:16]; self.disk = self.root / 'summary_cache.json'
        try: d = json.loads(self.disk.read_text(encoding='utf8')); self.summaries = d['summaries'] if d.get('code') == self.code else {}
        except (OSError, ValueError, KeyError): self.summaries = {}

    def _analyse(self, rid, folder):
        key = (rid, folder.name)
        if key not in self.cache:
            start, samples, stop, stopdiag = _load(folder)
            if not start: return None
            rows = _series(start, samples)
            self.cache[key] = (_summary(rid, folder, start, samples, stop, stopdiag, rows), rows)
        return self.cache[key]

    def runs(self, limit=30):
        out = []
        with self.lock:
          n = len(self.summaries)
          for rid, folder in _records(self.app):
              key = rid + '/' + folder.name
              if key in self.summaries:
                  if self.summaries[key]: out.append(self.summaries[key])
                  continue
              try: a = self._analyse(rid, folder)
              except Exception as e: a = (dict(id=rid, error=type(e).__name__), [])
              if not a: self.summaries[key] = None
              elif 'error' not in a[0]: self.summaries[key] = a[0]
              if a: out.append(a[0])
          if len(self.summaries) != n:
              tmp = self.disk.with_suffix('.tmp')
              for _ in range(10):  # OneDrive may briefly lock the file; a failed write only costs a re-parse after restart
                  try: tmp.write_text(json.dumps(dict(code=self.code, summaries=self.summaries), ensure_ascii=False), encoding='utf8'); tmp.replace(self.disk); break
                  except OSError: time.sleep(.3)
        out.sort(key=lambda r: r.get('started') or 0, reverse=True)
        return dict(runs=out[:limit], total=len(out), curvature_max=CURV_MAX, min_segment_s=MIN_SEG_S, presets=PRESETS,
                    note='直線段由路徑形狀自動判定，用於跨輪比較，不是安全門檻；資料來自已同步並驗證SHA256的Orin紀錄。')

    def detail(self, rid):
        for r, folder in _records(self.app):
            if r == rid:
                a = self._analyse(r, folder)
                if not a: break
                summary, rows = a
                return dict(summary=summary, series=[dict(t=x['t'], e=round(x['e'], 4), u=round(x['u'], 1), I=round(x['I'], 1), straight=x['curv'] is not None and x['curv'] < CURV_MAX, src=x['src']) for x in rows])
        raise ValueError('找不到已同步的PID紀錄')

    def apply_preset(self, p):
        name = p.get('preset'); preset = PRESETS.get(name)
        if not preset: raise ValueError('未知的預設參數組')
        doc = self.app.store.get('visual-pid')
        if not doc.get('rev'): raise ValueError('請先在04保存PID參數')
        if p.get('expected_rev') != doc['rev']: raise ValueError('PID參數已被其他頁更新，請重新整理後再套用')
        data = json.loads(json.dumps(doc['data'])); before = dict(data['parameters'])
        data['parameters'].update(preset['params'])
        changed = {k: [before.get(k), v] for k, v in preset['params'].items() if before.get(k) != v}
        if not changed: return dict(rev=doc['rev'], changed={}, note='目前保存值已相同，未建立新版本')
        self.app.validate_document(dict(key='visual-pid', rev=doc['rev'], data=data))
        saved = self.app.store.save('visual-pid', doc['rev'], data)
        entry = dict(at=time.time(), preset=name, from_rev=doc['rev'], to_rev=saved['rev'], changed=changed)
        with (self.root / 'presets.jsonl').open('a', encoding='utf8') as f: f.write(json.dumps(entry, ensure_ascii=False) + '\n')
        return dict(rev=saved['rev'], changed=changed, reload_preview='row_ratio' in changed)

    def history(self):
        f = self.root / 'presets.jsonl'
        return [json.loads(l) for l in f.read_text(encoding='utf8').splitlines() if l.strip()] if f.exists() else []
