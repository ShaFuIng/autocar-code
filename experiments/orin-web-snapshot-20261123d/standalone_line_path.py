"""Independent experimental PID path. Derived from Claude line estimator; no K or fallback guard.
Known acute-bend/branch identity failures remain; quality is not a probability.
"""
import copy
import cv2
import numpy as np

REVISION = 'standalone-line-v1'
LOST_RIGHT_REVISION = 'lost-right-r2'
import json as _json
from pathlib import Path as _Path
def lost_right_config():
    c = dict(enabled=False, window_s=1.0, x_max=600.0, ymax_min=268.0)
    try: c.update(_json.loads(_Path(__file__).with_name('standalone_line.json').read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c
def relabel_lost_right(trs, last_r, t, c):
    """lost-right-r2: a lone L track reaching the lower image within window_s after a RIGHT track seen at bottom x <= x_max keeps R."""
    if not c.get('enabled') or last_r is None or not (0 <= t - last_r['t'] <= float(c['window_s'])) or last_r['x'] > float(c['x_max']): return None
    if any(tr['side'] == 'R' for tr in trs): return None
    lone = [tr for tr in trs if tr['side'] == 'L']
    if len(lone) != 1 or lone[0]['ymax'] < float(c['ymax_min']): return None
    lone[0]['side'] = 'R'; lone[0]['lost_right'] = dict(revision=LOST_RIGHT_REVISION, since_s=round(t - last_r['t'], 2), last_r_x=round(last_r['x'], 1), ymax=lone[0]['ymax'])
    return lone[0]['lost_right']

ROWS = list(range(140, 344, 4))          # 640x360 rows used for the estimate
from line_width_prior import WIDTH_FIT, prior_width  # Unchanged v1 geometry
MEMORY_S = 1.0     # width ratio observed within this time is reused (r1 value; superseded by WIDTH_MEMORY_S below)
# width-memory-r1 (user 2026-11-06, live 481ca099): with MEMORY_S 1.0 the car fell back to the prior ratio 1.0 for 4-7 s stretches
# where only one line was seen while the observed ratio was 0.76-0.98; the single-line centre was then ~7 % of the lane off and
# every 0.85 <-> 1.0 switch jumped it. The observed ratio is now kept WIDTH_MEMORY_S and the applied ratio moves toward its
# target at most WIDTH_RATE per second. width_memory.json next to this file overrides both. Experiment values.
WIDTH_MEMORY_REVISION = 'width-memory-r1'
WIDTH_MEMORY_S, WIDTH_RATE = 30.0, 0.3
try:
    import json as _json
    from pathlib import Path as _Path
    _wm = _json.loads((_Path(__file__).with_name('width_memory.json')).read_text(encoding='utf8'))
    WIDTH_MEMORY_S = float(_wm.get('memory_s', WIDTH_MEMORY_S)); WIDTH_RATE = float(_wm.get('rate_per_s', WIDTH_RATE))
except Exception:
    pass
KINK = 2.0         # |dx/dy| change over +/-3 rows that ends a track (branch)
LINE_RMS = 2.5     # px; a curved or noisy boundary is not extended
LINE_REACH = 80    # rows a fitted boundary may be extended toward the car


# pathfast2-r1: median of three in the path smoothing without the np.median call. For an odd count np.median returns the
# middle element of the partitioned values (mean of that single element = the element), and NaN if any value is NaN. The
# smoothed xs are finite (np.isfinite gate above) -> the middle value of the sorted three. Equal values have the same bits,
# except +0.0 / -0.0 (which one partition puts in the middle is an implementation detail), so a zero middle value, or any
# non-finite value, goes to np.median itself. MEDIAN3 False = always np.median.
MEDIAN3 = True


def _med3(a, b, c, xs, i):
    if a <= b:
        m = b if b <= c else (c if a <= c else a)
    else:
        m = a if a <= c else (c if b <= c else b)
    if m == 0.0 or not (a == a and b == b and c == c and abs(a) < 1e308 and abs(b) < 1e308 and abs(c) < 1e308):
        return np.median(xs[i - 1:i + 2])
    return m


def row_runs(mask, y):
    row = mask[max(0, y - 1):y + 2].any(axis=0).astype(np.int8)
    d = np.diff(np.r_[0, row, 0]); s = np.flatnonzero(d == 1); e = np.flatnonzero(d == -1)
    # Runs cut by the frame edge have a biased centre; they are not boundary positions.
    return [((a + b - 1) / 2, b - a) for a, b in zip(s, e) if 2 <= b - a <= 160 and a > 1 and b < row.size - 1]


def _runs_table(mask, rows=ROWS):
    """pathfast-r1: {y: row_runs(mask, y)} for every y in rows in one vectorised pass. Same runs in the same order, same element
    types (centre np.float64, width np.int64). Falls back to row_runs per row outside the plain case (2-D, 1 <= y, y + 2 <= height)."""
    r = np.asarray(rows)
    if mask.ndim != 2 or len(r) == 0 or r.min() < 1 or r.max() + 2 > mask.shape[0]:
        return {y: row_runs(mask, y) for y in rows}
    W = mask.shape[1]
    P = np.zeros((len(r), W + 2), np.int8)
    P[:, 1:-1] = np.logical_or(np.logical_or(mask[r - 1], mask[r]), mask[r + 1])
    d = np.diff(P, axis=1)
    si, sc = np.nonzero(d == 1); ec = np.nonzero(d == -1)[1]
    w = ec - sc
    keep = (w >= 2) & (w <= 160) & (sc > 1) & (ec < W - 1)
    cen = ((sc + ec - 1) / 2)[keep]; w = w[keep]
    n = np.bincount(si[keep], minlength=len(r)).tolist()
    out = {}; i = 0
    for y, k in zip(rows, n):
        out[y] = list(zip(cen[i:i + k], w[i:i + k])); i += k
    return out


def tracks(mask, table=None):
    if table is None: table = _runs_table(mask)   # pathfast-r1 (was row_runs per row)
    done, live = [], []
    for y in reversed(ROWS):
        runs = table[y]; used = set(); nxt = []
        for tr in live:
            (ly, lx), pred = tr['pts'][-1], tr['pts'][-1][1]
            if len(tr['pts']) >= 2:
                (py, px) = tr['pts'][-2]; pred = lx + (lx - px) * (ly - y) / max(1, py - ly)
            best = min(((abs(x - pred), i) for i, (x, w) in enumerate(runs) if i not in used), default=None)
            if best and best[0] <= 28 + 0.3 * runs[best[1]][1]:
                used.add(best[1]); tr['pts'].append((y, runs[best[1]][0])); nxt.append(tr)
            elif ly - y <= 12:
                nxt.append(tr)
            else:
                done.append(tr)
        for i, (x, w) in enumerate(runs):
            if i not in used: nxt.append(dict(pts=[(y, x)]))
        live = nxt
    out = []
    for tr in done + live:
        p = np.array(tr['pts'], float)
        if len(p) < 4: continue
        a, b = np.polyfit(p[:, 0], p[:, 1], 1)
        side = 'X' if abs(a) > 4 else ('L' if a < -0.15 else 'R' if a > 0.15 else '?')
        out.append(dict(pts={int(y): x for y, x in p}, slope=float(a), side=side, ymax=int(p[:, 0].max()), ymin=int(p[:, 0].min())))
    return out


def cut_at_kink(tr):
    ys = sorted(tr['pts'], reverse=True); xs = [tr['pts'][y] for y in ys]
    for i in range(3, len(ys) - 3):
        a = (xs[i] - xs[i - 3]) / (ys[i] - ys[i - 3]); b = (xs[i + 3] - xs[i]) / (ys[i + 3] - ys[i])
        if abs(a - b) > KINK:
            tr['pts'] = {y: tr['pts'][y] for y in ys[:i + 1]}; tr['ymin'] = ys[i]
            return tr
    return tr


def fit_line(tr):
    ys = np.array(sorted(tr['pts']), float)
    if len(ys) < 8 or ys[-1] - ys[0] < 40: return None
    xs = np.array([tr['pts'][int(y)] for y in ys]); a, b = np.polyfit(ys, xs, 1)
    if np.sqrt(np.mean((xs - (a * ys + b)) ** 2)) > LINE_RMS: return None
    return a, b, int(ys[0]), int(ys[-1])


class LineEstimate:
    """Recorded-mask line estimator; history selects boundaries and retains width ratio."""
    def __init__(self, continue_steep=False):
        self.continue_steep = continue_steep
        self.last = {'L': None, 'R': None}; self.ratio = None; self.ratio_t = None; self.t = None; self.k_used = None; self.k_t = None   # width-memory-r1
        self.last_r = None; self.lr_cfg = lost_right_config()   # lost-right-r2

    allow_offscreen = False
    width = staticmethod(prior_width)
    get_tracks = staticmethod(tracks)

    def _pair(self, trs):
        return self._pick(trs, 'L'), self._pick(trs, 'R')

    def _pick(self, trs, side):
        cand = [t for t in trs if t['side'] == side]
        if not cand: return None
        ref = self.last[side]
        if ref is not None:
            def dist(t):
                common = [abs(t['pts'][y] - ref[y]) for y in t['pts'] if y in ref]
                return np.median(common) if common else 1e9
            best = min(cand, key=dist)
            if dist(best) <= 30: return best
        inner = lambda t: -t['pts'][t['ymax']] if side == 'L' else t['pts'][t['ymax']]
        long = [t for t in cand if t['ymax'] - t['ymin'] >= 24] or cand
        return min(long, key=inner)

    def step(self, mask, t):
        previous_t = self.t
        if self.t is not None and (t <= self.t or t - self.t > 1): self.__init__(self.continue_steep)
        self.t = t
        trs = [cut_at_kink(x) for x in self.get_tracks(mask)]
        if self.continue_steep and previous_t is not None and 0 < t-previous_t <= .4:
            for tr in trs:
                if tr['side'] != 'X': continue
                # Only continue a side observed in the immediately previous frame.
                # Reuse the existing 24-row span / 30-pixel association limits;
                # slope alone still cannot admit a new steep boundary.
                side = 'L' if tr['slope'] < 0 else 'R'
                ref = self.last[side]
                common = sorted(y for y in tr['pts'] if ref and y in ref)
                if len(common) < 4 or common[-1]-common[0] < 24: continue
                distance = float(np.median([abs(tr['pts'][y]-ref[y]) for y in common]))
                if distance <= 30:
                    tr['side'] = side
                    tr['continued_steep'] = dict(side=side, slope=tr['slope'], overlap_rows=len(common), median_dx=distance)
        for tr in trs:
            if tr['side'] == '?':
                x = tr['pts'][tr['ymax']]; tr['side'] = 'L' if x < 250 else 'R' if x > 390 else '?'
        lr_info = relabel_lost_right(trs, self.last_r, t, self.lr_cfg)   # lost-right-r2
        L, R = self._pair(trs)
        if R: self.last_r = dict(t=t, x=float(R['pts'][R['ymax']]))   # bottom-most x of the right track (640 frame)
        if L and R:
            common = [y for y in L['pts'] if y in R['pts']]
            bad = [y for y in common if not 0.45 * self.width(y) <= R['pts'][y] - L['pts'][y] <= 1.7 * self.width(y)]
            if common and len(bad) > len(common) / 2:
                if (L['ymax'] - L['ymin']) >= (R['ymax'] - R['ymin']): R = None
                else: L = None
        ratios = [(R['pts'][y] - L['pts'][y]) / self.width(y) for y in L['pts'] if y in R['pts']] if L and R else []
        if len(ratios) >= 3: self.ratio = float(np.clip(np.median(ratios), .7, 1.4)); self.ratio_t = t
        remembered = self.ratio is not None and 0 <= t - self.ratio_t <= WIDTH_MEMORY_S   # width-memory-r1
        target = self.ratio if remembered else 1.0
        if self.k_used is None or self.k_t is None or t < self.k_t: self.k_used = target
        else:
            lim = WIDTH_RATE * max(0.0, min(0.5, t - self.k_t)); self.k_used += max(-lim, min(lim, target - self.k_used))
        self.k_t = t; k = float(self.k_used)
        fits = {s: f for s, f in (('L', fit_line(L) if L else None), ('R', fit_line(R) if R else None)) if f}
        def at(side, tr, y):
            f = fits.get(side)
            if f and f[2] - 8 <= y <= f[3] + LINE_REACH: return f[0] * y + f[1]
            return tr['pts'].get(y) if tr else None
        pts = []; sources = {}
        for y in ROWS:
            xl = at('L', L, y); xr = at('R', R, y); half = k * self.width(y) / 2
            if xl is not None and xr is not None: x, src = (xl + xr) / 2, 'double'
            elif xr is not None: x, src = xr - half, 'right+width'
            elif xl is not None: x, src = xl + half, 'left+width'
            else: continue
            if np.isfinite(x) and (self.allow_offscreen or 0 <= x < 640): pts.append((y, x)); sources[src] = sources.get(src, 0) + 1
        if len(pts) >= 3:
            xs = np.array([p[1] for p in pts]); sm = xs.copy()
            if MEDIAN3:   # pathfast2-r1: same values as np.median(xs[i - 1:i + 2]) (see _med3)
                xv = xs.tolist()
                for i in range(1, len(xs) - 1): sm[i] = _med3(xv[i - 1], xv[i], xv[i + 1], xs, i)
            else:
                for i in range(1, len(xs) - 1): sm[i] = np.median(xs[i - 1:i + 2])
            pts = [(y, float(x)) for (y, _), x in zip(pts, sm)]
        chains = []
        for y, x in pts:
            if chains and y - chains[-1][-1][0] <= 12 and abs(x - chains[-1][-1][1]) <= 40: chains[-1].append((y, x))
            else: chains.append([(y, x)])
        best = max(chains, key=len) if chains else []
        self.last = {'L': L['pts'] if L else None, 'R': R['pts'] if R else None}
        result = dict(path=[[x * 2, y * 2] for y, x in sorted(best)], sources=sources, line_fit=sorted(fits), width_ratio=k, width_source='recent_observation' if remembered else 'prior_or_unit_observation',
                      width_memory=WIDTH_MEMORY_REVISION, width_target=round(float(target), 3), width_observed_age_s=None if self.ratio_t is None else round(float(t - self.ratio_t), 2))   # width-memory-r1
        if self.continue_steep: result['steep_continuation'] = [tr['continued_steep'] for tr in (L,R) if tr and 'continued_steep' in tr]
        if lr_info is not None: result['lost_right'] = lr_info
        return result



class StandaloneLine:
    algorithm = 'standalone-line-v1'
    def __init__(self, row=.6, continue_steep=False):
        self.row=row;self.continue_steep=continue_steep
        self.set_row(row)
    def set_row(self,row):
        from warm_color_candidate import WarmColor
        self.row=row;self.color=WarmColor();self.estimate=LineEstimate(self.continue_steep);self.prepared=None
    def preprocess(self,image,captured):
        corrected,diag=self.color.step(cv2.resize(image,(640,360)),captured)
        self.prepared=(captured,diag)
        return corrected
    def step(self,image,mask,captured):
        if self.prepared is None or self.prepared[0]!=captured:
            raise ValueError('Same-frame preprocessing required')
        m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
        est=self.estimate.step(m,captured);p=est['path']
        diag=dict(path_revision=REVISION,capture_s=captured,mode='STANDALONE_LINE',
                  color_correction=self.prepared[1],line_estimate={k:v for k,v in est.items() if k!='path'},
                  reason='CURRENT_MASK_ESTIMATE' if p else 'NO_CURRENT_MASK_PATH',
                  no_k=True,no_optical_flow=True,metric_calibration=False,
                  junction_policy='No route commitment; follows selected visible boundaries')
        self.prepared=None
        return dict(path=p,valid=len(p)>=3,quality=0.,path_diagnostic=diag)
    def close(self):pass
