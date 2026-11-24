"""lap-pos-r1: camera-free lap-position estimator for the 10 Hz control loop (offline candidate, map-20261114, NOT deployed).

Estimator = trackpos-speed-20261114 candidate track_pos.py (track-pos-r1, a46ac257) renamed: the car already has an unrelated
web/track_pos.py (11-08, used by ops07_runtime.apply_seed / Runtime.observe, writes proposal['track_pos']), so this module is
lap_pos.py / lap_pos.json and logs to proposal['lap_pos']. Additions vs track-pos-r1: the C2e anchor record (self.c2e) used by the
position-anchored crossing zone (xp-anchor-r1 in live_pid), and status()['c2e'] for the log. Detection logic unchanged unless
lap_pos.json says otherwise (see the keys documented in the json note).

Inputs per frame (all already present in the frame; nothing is read here): t (s, monotonic), gz (deg/s, left +, the frame's
proposal.inputs.gz), pitch_rel (deg, proposal.imu_boost.pitch_rel_deg), enc_b (rear-wheel counts, proposal.inputs.enc_b).
Missing values are tolerated (gz / pitch held, odometer held).

Logic = the reference study's landmark detector (redesign-20261113/reference/common.py) made causal:
- yaw = trapezoid integral of gz; odometer D = max-accumulated enc_b * m_per_count.
- a landmark is a level crossing (heading vs the preceding straight's plateau, or pitch) that persists `hold` metres; when confirmed
  the crossing point (odometer, interpolated between frames) is the landmark position, so the position re-anchors exactly there:
  s = nominal(landmark) + (D - D_cross). The decision comes `hold` metres later (delay), the position does not lag.
- plateaus (the straight's heading) are medians over odometer windows that lie entirely in the past when they are used.
- every landmark has an acceptance window around its nominal lap position; a crossing before the window is ignored, passing the
  window's end counts a miss (confidence drops), passing it by `lost_after_m` sets lost.
- heading corridor: on each segment the heading relative to the segment's anchor is compared with the reference profile
  (median over 36 lap passes, track_pos.json); outside max(k_sd * sd + add, min) for `persist_m` -> lost.
- when lost: dead reckoning continues; the pitch landmarks (RAMP, DOWN) re-acquire the lap position (unique on this track).
Standard library only. No I/O except load_config(). Returns a dict for proposal['lap_pos'] (log only).
"""
import json, math
from pathlib import Path

REVISION = 'lap-pos-r1'
REVISION_R2 = 'lap-pos-r2'   # verify-20261114: lost_clear_n / corr_skip_m set in lap_pos.json (absent = r1 exactly)
REVISION_R3 = 'lap-pos-r3'   # next-20261115: c2e_fallback set in lap_pos.json (absent = r2 / r1 exactly)
CONFIG_FILE = Path(__file__).with_name('lap_pos.json')

# segment names in lap order (start position of each, by landmark)
SEGMENTS = [('C1s', 'curve1'), ('C1e', 'diagonal_v'), ('C2s', 'curve2'), ('C2e', 'circle_straight'), ('BEND', 'left_bend'),
            ('C3s', 'curve3'), ('RAMP', 'ramp'), ('TOP', 'top'), ('DASHs', 'dashed_turn'), ('DASHe', 'upper_straight'),
            ('DOWN', 'downhill'), ('LAND8', 'bottom'), ('C4s', 'curve4'), ('C4e', 'main_straight')]


def load_config(path=None):
    p = Path(path) if path else CONFIG_FILE
    return json.loads(p.read_text(encoding='utf8'))


def _median(xs):
    xs = sorted(xs); n = len(xs)
    if n == 0: return None
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


class _Cross(object):
    """Causal 'x crosses level (sign) and stays crossed for hold metres'. feed() returns the crossing odometer when confirmed."""

    def __init__(self, level, sign, hold):
        self.level = level; self.sign = sign; self.hold = hold; self.d0 = None; self.prev = None

    def feed(self, d, x):
        hit = None
        if x is not None:
            on = self.sign * (x - self.level) > 0
            if on and self.d0 is None:
                p = self.prev
                if p is not None and p[1] is not None and p[1] != x and self.sign * (p[1] - self.level) <= 0:
                    f = (self.level - p[1]) / (x - p[1]); self.d0 = p[0] + max(0., min(1., f)) * (d - p[0])
                else: self.d0 = d
            elif not on: self.d0 = None
            if self.d0 is not None and d - self.d0 >= self.hold: hit = self.d0
        self.prev = (d, x)
        return hit


class LapPos(object):
    def __init__(self, cfg=None):
        self.c = cfg if cfg is not None else load_config()
        self.nom = dict(self.c['nominal_m']); self.lap_m = float(self.c['lap_m'])
        self.mpc = float(self.c['m_per_count'])
        self.prof = self.c.get('profile') or {}
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        self.t = None; self.gz = None; self.yaw = 0.; self.pitch = None; self.enc0 = None; self.D = 0.
        self.hist = []                       # (D, yaw) of the last ~8 m, for plateau medians
        self.lap = 0; self.anchor = None     # (name, D_cross, s_nominal)
        self.s0 = float(self.c['start_s'])   # lap-0 start position (car placed by hand before curve 1)
        self.base = None; self.step = 'BASE0'; self.det = {}; self.found = {}
        self.misses = 0; self.lost = False; self.lost_reason = None; self.lost_at = None
        self.corr = None                     # (segment key, anchor yaw, anchor D) for the heading corridor
        self.out_d0 = None; self.dev = None; self.events = []; self.n = 0; self.gaps = 0; self.skips = 0
        self.inwin = 0                       # lap-pos-r2: consecutive in-window landmarks while lost
        self.c2e = None                      # last curve-2 end: dict(lap, d_cross, d_decide, n) (lap-pos-r1 addition, read by xp-anchor-r1)
        self._fb_arm = None                  # lap-pos-r3: odometer where curve 2 passed c2e_fallback.level_deg (None = not yet)

    # ------------------------------------------------------------------ helpers
    def s(self):
        if self.anchor is None: return self.s0 + self.D
        return self.anchor[2] + (self.D - self.anchor[1])

    def _plateau(self, a, b):
        return _median([y for d, y in self.hist if a <= d <= b])

    def _yaw_at(self, d):
        h = self.hist
        for i in range(len(h) - 1, 0, -1):
            if h[i - 1][0] <= d <= h[i][0]:
                (d0, y0), (d1, y1) = h[i - 1], h[i]
                return y0 if d1 <= d0 else y0 + (y1 - y0) * (d - d0) / (d1 - d0)
        return self.yaw

    def _arm(self, name, field, level, sign, hold):
        self.det[name] = (field, _Cross(level, sign, hold))

    def _anchor(self, name, dcross, reanchor=True):
        nom = self.nom[name]
        exp = self.s() - (self.D - dcross)            # estimated lap position of the crossing before re-anchoring
        if name == 'C1s':
            if self.lap > 0: exp -= self.lap_m
            self.lap += 1
        self.events.append(dict(name=name, lap=self.lap, d_cross=round(dcross, 3), d_decide=round(self.D, 3), s_err=round(exp - nom, 3),
                                anchor=reanchor))
        self.found[name] = dcross
        if name == 'C2e': self.c2e = dict(lap=self.lap, d_cross=float(dcross), d_decide=float(self.D), n=self.n)
        if reanchor: self.anchor = (name, dcross, nom); self.skips = 0
        ca = self.c['corridor_anchor']; ref = ca.get(name)
        if name in ca and ref is None: self.corr = None; self.out_d0 = None   # curve starts: no corridor inside curves
        if ref is not None: self.corr = (ref, self._yaw_at(dcross) if name in self.c['pitch_landmarks'] else self.det[name][1].level, dcross)
        if name in self.det: del self.det[name]
        n_ = self.c.get('lost_clear_n')
        if self.lost and n_ and name not in ('RAMP', 'DOWN'):   # lap-pos-r2: N consecutive landmarks inside their acceptance windows clear lost
            w = self.c['window_m']
            self.inwin = self.inwin + 1 if -float(w[0]) <= exp - nom <= float(w[1]) else 0
            if self.inwin >= int(n_):
                self.lost = False; self.lost_reason = None; self.out_d0 = None; self.inwin = 0
                self.events.append(dict(name='REACQUIRED', at=name, lap=self.lap, by='IN_WINDOW'))
        if self.lost and name in ('RAMP', 'DOWN'):   # re-acquire: restart the sequence at this pitch landmark
            self.lost = False; self.lost_reason = None; self.out_d0 = None
            self.found = {name: dcross}; self.det = {}; self.step = name
            if name == 'DOWN': self.found['DASHe'] = dcross - (self.nom['DOWN'] - self.nom['DASHe'])
            self.events.append(dict(name='REACQUIRED', at=name, lap=self.lap))

    def _set_lost(self, why):
        if not self.lost:
            self.lost = True; self.lost_reason = why; self.lost_at = self.D; self.inwin = 0; self.events.append(dict(name='LOST', why=why, lap=self.lap, d=round(self.D, 3)))

    # ------------------------------------------------------------------ the landmark sequence
    def _sequence(self):
        c = self.c; T = c['turn_deg']; L = float(c['level_deg']); H = float(c['hold_m']); D = self.D; st = self.step; f = self.found
        if st == 'BASE0' and D >= c['base0_m'][1]:
            self.base = self._plateau(*c['base0_m']); self._arm('C1s', 'yaw', self.base - L, -1, H); self.step = 'C1s'
        elif st == 'C1s' and 'C1s' in f:
            self._arm('C1e', 'yaw', self.base + T['C1'] + L, -1, H); self.step = 'C1e'
        elif st == 'C1e' and 'C1e' in f and D >= f['C1e'] + 3.:
            b = self._plateau(f['C1e'] + 1., f['C1e'] + 3.); self.base = b = b if b is not None else self.base; self._arm('C2s', 'yaw', b - L, -1, c['hold_start_m']); self.step = 'C2s'
        elif st == 'C2s' and 'C2s' in f:
            self._arm('C2e', 'yaw', self.base + T['C2'] + L, -1, H); self.step = 'C2e'
        elif st == 'C2e' and 'C2e' in f and D >= f['C2e'] + 6.5:
            b = self._plateau(f['C2e'] + 3.5, f['C2e'] + 6.5); self.base = b = b if b is not None else self.base; self._arm('C3s', 'yaw', b - L, -1, c['hold_start_m']); self.step = 'C3s'
        elif st == 'C3s' and 'C3s' in f:
            self._arm('C3e', 'yaw', self.base + T['C3'] + L, -1, H); self._arm('RAMP', 'pitch', c['ramp_deg'], +1, .2); self.step = 'RAMP'
        elif st == 'RAMP' and 'RAMP' in f and D >= f['RAMP'] + .5:
            self._arm('TOP', 'pitch', c['ramp_deg'], -1, .3); self.step = 'TOP'
        elif st == 'TOP' and 'TOP' in f and D >= f['TOP'] + .8:
            b = self._plateau(f['TOP'], f['TOP'] + .8); self.base = b = b if b is not None else self.base
            self._arm('DASHs', 'yaw', b + L, +1, H); self.step = 'DASHs'
        elif st == 'DASHs' and 'DASHs' in f:
            self._arm('DASHe', 'yaw', self.base + T['DASH'] - L, +1, H); self.step = 'DASHe'
        elif st == 'DASHe' and 'DASHe' in f:
            self._arm('DOWN', 'pitch', c['down_deg'], -1, .1); self.step = 'DOWN'
        elif st == 'DOWN' and 'DOWN' in f:
            b = self._plateau(f['DASHe'] + .5, f['DOWN']); self.base = b if b is not None else self.base
            self._arm('LAND8', 'pitch', c['down_deg'], +1, .2); self.step = 'LAND8'
        elif st == 'LAND8' and 'LAND8' in f:
            self._arm('C4s', 'yaw', self.base - L, -1, H); self.step = 'C4s'
        elif st == 'C4s' and 'C4s' in f:
            self._arm('C4e', 'yaw', self.base + T['C4'] + L, -1, H); self.step = 'C4e'
        elif st == 'C4e' and 'C4e' in f and D >= f['C4e'] + 3.:
            b = self._plateau(f['C4e'] + 1., f['C4e'] + 3.); self.base = b = b if b is not None else self.base
            f.clear()
            self._arm('C1s', 'yaw', b - L, -1, H); self.step = 'C1s'

    def _c2e_fallback(self):
        """lap-pos-r3 (next-20261115, 9972 lap 4: curve 2 ended at -194.3 deg vs the -195 level, C2e skipped, no crossing anchor).
        While the sequence waits for C2e and the primary level has not fired:
          QUIET   the turn passed level_deg (rel. to the base plateau) and over the last quiet_m the heading turned right by at most
                  quiet_deg -> C2e at D - quiet_m (start of the quiet stretch, near the end of the fast turn);
        Same acceptance window as the detectors (too early = ignored); not while lost. Decided at least quiet_m after the arm point,
        so a primary crossing that follows within quiet_m always wins."""
        fb = self.c.get('c2e_fallback')
        if self.step != 'C2e' or 'C2e' not in self.det or self.base is None or self.lost:
            self._fb_arm = None; return
        D = self.D; rel = self.yaw - self.base; dc = None; how = None
        if rel <= -float(fb.get('level_deg', 180.)) and self._fb_arm is None: self._fb_arm = D
        q = float(fb.get('quiet_m', 1.0))
        if self._fb_arm is not None and D - q >= self._fb_arm and self._yaw_at(D - q) - self.yaw <= float(fb.get('quiet_deg', 6.)):
            dc = D - q; how = 'QUIET'
        if dc is None: return
        if self.s() - (D - dc) < self.nom['C2e'] - self.c['window_m'][0]: return   # too early (same rule as the detectors)
        k = len(self.events)
        self._anchor('C2e', dc, reanchor='C2e' not in self.c['no_anchor'])
        self.events[k]['fallback'] = how; self._fb_arm = None
        if self.c2e is not None: self.c2e['fallback'] = how

    # ------------------------------------------------------------------ one frame
    def update(self, t, gz=None, pitch_rel=None, enc_b=None):
        c = self.c; self.n += 1
        if t is not None and self.t is not None and t - self.t > c['gap_s']:
            # frames without a gyro sample (hold / lost-line frames carry no gz): the integral over the gap is a guess, so the heading
            # corridor is suspended until the next anchor (its reference heading predates the gap)
            self.corr = None; self.out_d0 = None; self.gaps += 1
        if gz is not None and isinstance(gz, (int, float)) and math.isfinite(gz):
            if self.t is not None and self.gz is not None and t is not None and t > self.t: self.yaw += 0.5 * (gz + self.gz) * (t - self.t)
            self.gz = float(gz)
        elif self.t is not None and self.gz is not None and t is not None and t > self.t: self.yaw += self.gz * (t - self.t)
        if t is not None: self.t = t
        if pitch_rel is not None and isinstance(pitch_rel, (int, float)) and math.isfinite(pitch_rel): self.pitch = float(pitch_rel)
        if enc_b is not None and isinstance(enc_b, (int, float)):
            if self.enc0 is None: self.enc0 = enc_b
            self.D = max(self.D, (enc_b - self.enc0) * self.mpc)
        D = self.D
        self.hist.append((D, self.yaw))
        while self.hist and D - self.hist[0][0] > c['hist_m']: self.hist.pop(0)
        # landmark detectors
        for name in list(self.det):
            if name not in self.det: continue           # removed by a re-acquisition earlier in this frame
            field, cr = self.det[name]
            x = self.yaw if field == 'yaw' else self.pitch
            hit = cr.feed(D, x)
            if hit is None: continue
            s_hit = self.s() - (D - hit)
            if self.lap == 0 and name == 'C1s': s_hit = self.s0 + hit   # lap 0: position from the start
            nom = self.lap_m if (name == 'C1s' and self.lap > 0) else self.nom[name]; w = c['window_m']
            if not self.lost and s_hit < nom - w[0]:
                cr.d0 = None; continue                                  # too early: ignore (e.g. the V crossing)
            self._anchor(name, hit, reanchor=name not in c['no_anchor'])
        if c.get('c2e_fallback'): self._c2e_fallback()   # lap-pos-r3 (absent = r2)
        self._sequence()
        # lost: the landmark the sequence waits for is passed by lost_after_m without being seen
        s = self.s(); nxt = self.pending()
        if nxt is not None and not self.lost and nxt in self.det:
            late = s - (self.lap_m if (nxt == 'C1s' and self.lap > 0) else self.nom[nxt])
            if late > (c.get('skip_after_by') or {}).get(nxt, c['skip_after_m']):   # lap-pos-r1: optional per-landmark skip distance (absent = track-pos-r1)
                # not seen: take it as passed at its nominal position (no re-anchor) so the sequence goes on; two in a row = lost
                if nxt not in c['no_anchor']: self.skips += 1    # a skipped non-anchor (C4s) does not count towards lost
                dv = D - late
                self.events.append(dict(name='SKIPPED', at=nxt, lap=self.lap, d=round(D, 3)))
                if nxt == 'C1s': self.lap += 1; self.anchor = ('C1s*', dv, 0.)
                del self.det[nxt]; self.found[nxt] = dv
                if self.skips >= c['lost_skips']: self._set_lost('MISSED_' + nxt)
        # while lost: a ramp or a downhill anywhere re-anchors (unique pitch events on this track)
        if self.lost:
            if 'RAMP' not in self.det: self._arm('RAMP', 'pitch', c['ramp_deg'], +1, .2)
            if 'DOWN' not in self.det: self._arm('DOWN', 'pitch', c['down_deg'], -1, .1)
        # heading corridor
        self.dev = None
        if self.corr is not None and not self.lost:
            key, y0, d0 = self.corr; pr = self.prof.get(key)
            if pr is not None:
                x = D - d0; i = int(x / pr['step']); sk = (c.get('corr_skip_m') or {}).get(key)   # lap-pos-r2: corridor not checked for the first sk m
                if 0 <= i < len(pr['psi']) and not (sk is not None and x < float(sk)):
                    psi, sd = pr['psi'][i], pr['sd'][i]
                    tol = max(c['corr_k_sd'] * sd + c['corr_add_deg'], c['corr_min_deg'])
                    self.dev = (self.yaw - y0) - psi
                    if abs(self.dev) > tol:
                        if self.out_d0 is None: self.out_d0 = D
                        if D - self.out_d0 >= c['corr_persist_m']:
                            self._set_lost('HEADING'); self.events[-1].update(seg_ref=key, x=round(x, 2), dev=round(self.dev, 1), tol=round(tol, 1))
                    else: self.out_d0 = None
        return self.status()

    def pending(self):
        """The next landmark the sequence waits for (None before the start plateau is known)."""
        st = self.step
        if st not in self.nom: return None
        if st not in self.found: return st
        order = self.c['order']; i = order.index(st)
        return order[(i + 1) % len(order)]

    def segment(self, s=None):
        s = self.s() if s is None else s
        if self.lap == 0 and s < 0: return 'start_straight'
        x = s % self.lap_m if s >= 0 else s + self.lap_m
        name = 'main_straight'
        for key, seg in SEGMENTS:
            pos = self.nom[key] if key != 'BEND' else self.nom['C2e'] + self.c['bend_after_c2e_m']
            if x >= pos: name = seg
        return name

    def status(self):
        s = self.s(); nxt = self.pending()
        to_next = None
        if nxt is not None:
            to_next = (self.lap_m if (nxt == 'C1s' and self.lap > 0) else self.nom[nxt]) - s
        since = None if self.anchor is None else self.D - self.anchor[1]
        conf = 0. if self.lost else max(0., 1. - self.c['conf_per_m'] * (since if since is not None else self.D))
        return dict(revision=REVISION_R3 if self.c.get('c2e_fallback') else (REVISION_R2 if (self.c.get('lost_clear_n') or self.c.get('corr_skip_m')) else REVISION), lap=self.lap, s_m=round(s, 2), seg=self.segment(s), next=nxt,
                    to_next_m=None if to_next is None else round(to_next, 2), anchor=None if self.anchor is None else self.anchor[0],
                    since_anchor_m=None if since is None else round(since, 2), conf=round(conf, 2), lost=self.lost, lost_reason=self.lost_reason,
                    dev_deg=None if self.dev is None else round(self.dev, 1), odo_m=round(self.D, 3),
                    c2e_m=None if self.c2e is None else round(self.D - self.c2e['d_cross'], 2))


TrackPos = LapPos   # name used by the track-pos-r1 tests / tools
