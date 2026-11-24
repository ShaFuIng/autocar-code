"""Junction switch candidate (offline only): the general road and curves stay
with the on-car baseline path; the perspective corridor takes over only when
it has strong two-side evidence of a straight passage and the baseline points
somewhere else. This module does NOT detect curves or junctions as such; it
reacts to corridor support and disagreement, and says so in its output.

Independent of the corridor implementation: each step() gets the baseline
path, the corridor result for the same frame, and the control row.

States
  FOLLOW     target = baseline path (its own invalid / stop rules unchanged)
  TAKEN      target = corridor path with current image support
  PREDICT    target = corridor model path without current support (labelled)
  NO_TARGET  no target; after STOP_S the existing stale-path rule stops the car
  STOPPED    terminal for this run (the real car would have stopped)

Release to the baseline needs, for AGREE_FRAMES consecutive frames: corridor
with current support, baseline valid, the control row within AGREE_PX and the
direction-check rows above / below it within AGREE_DIR_PX (a branch that only
crosses at the sample row differs there), and the baseline inside the CURRENT
frame's corridor at every checked row (no boundaries from an earlier frame). Loss of two-side support is a
re-evaluation, not a release: TAKEN holds on single-side support, PREDICT is
limited to PREDICT_S, then NO_TARGET. A time gap or backward time resets.
All numbers are experiment values compared in switch_eval.py, not safety limits.
"""
import numpy as np

# Far comparison row: up to SPAN above the control row, limited to the range
# both paths cover. SPAN / MIN_SPAN reuse the preview-turn direction spans
# (nominal .15, minimum .06 of the image height); a fixed .45 row lies above
# the old-pose horizon, where the corridor has no path.
SPAN, MIN_SPAN = .15 * 359, .06 * 359


def top(path):
    return None if path is None or len(path) < 2 else float(np.min(np.asarray(path, float)[:, 1])) / 2


def x_at(path, y):
    """x (640 scale) on a 1280-scale path at row y; no extrapolation."""
    if path is None or len(path) < 2: return None
    q = np.asarray(path, float) / 2; q = q[np.argsort(q[:, 1])]
    if not q[0, 1] <= y <= q[-1, 1]: return None
    return float(np.interp(y, q[:, 1], q[:, 0]))


# pathfast2-r1: inside one measure() call the same paths are sampled at up to three rows; _XQ prepares each path once (the
# same asarray / sort as x_at) and samples it with the same np.interp. Same values as x_at. XQ_CACHE False = x_at per call.
XQ_CACHE = True


class _XQ:
    def __init__(self): self.c = {}

    def __call__(self, path, y):
        if path is None or len(path) < 2: return None
        e = self.c.get(id(path))
        if e is None or e[0] is not path:
            q = np.asarray(path, float) / 2; q = q[np.argsort(q[:, 1])]
            e = (path, q, q[:, 1], q[:, 0]); self.c[id(path)] = e
        q = e[1]
        if not q[0, 1] <= y <= q[-1, 1]: return None
        return float(np.interp(y, e[2], e[3]))


class JunctionSwitch:
    REVISION = 'junction-switch-r1'
    P = dict(N=20, D=80, AGREE_PX=20, AGREE_DIR_PX=30, AGREE_FRAMES=3,
             WEAK_S=1.0, PREDICT_S=.4, STOP_S=.4, GAP_S=.4)

    def __init__(self, **kw):
        self.p = dict(self.P, **kw); self.reset(None)

    def reset(self, t):
        self.state, self.since, self.last_strong, self.agree_n = 'FOLLOW', t, None, 0
        self.t, self.prev_target = None, None

    # -- measurements ----------------------------------------------------
    def measure(self, car_path, car_valid, cor, y):
        """Gap at the control row plus direction checks: a row up to SPAN
        above (if both paths reach it) and a row up to SPAN below. Two paths
        that only cross at the control row differ at one of them."""
        path = cor.get('path') or []
        x_at = _XQ() if XQ_CACHE else globals()['x_at']   # pathfast2-r1 (paths are not changed during measure)
        d = lambda a, b: None if a is None or b is None else a - b
        m = dict(y=y, car=x_at(car_path, y) if car_valid else None, cor=x_at(path, y))
        m['d_near'] = d(m['cor'], m['car'])
        diag = cor.get('diag') or {}
        m['status'], m['L'], m['R'] = cor.get('status'), diag.get('left_rows'), diag.get('right_rows')
        m['support'] = ('strong' if m['status'] == 'BOTH' and min(m['L'] or 0, m['R'] or 0) >= self.p['N'] else
                        'weak' if m['status'] in ('BOTH', 'SINGLE') else
                        'predicted' if m['status'] == 'COAST' else 'none')
        b = cor.get('boundaries') if m['support'] in ('strong', 'weak') else None
        checks = []
        if car_valid and len(car_path) > 1 and len(path) > 1:
            cy, py = np.asarray(car_path, float)[:, 1] / 2, np.asarray(path, float)[:, 1] / 2
            above = max(cy.min(), py.min(), y - SPAN); below = min(cy.max(), py.max(), y + SPAN)
            for yy in (above, below):
                if abs(yy - y) < MIN_SPAN: continue
                c, k = x_at(car_path, yy), x_at(path, yy)
                xl = x_at(b['left'], yy) if b else None; xr = x_at(b['right'], yy) if b else None
                checks.append(dict(y=round(yy, 1), car=c, cor=k, d=d(k, c), inside=None if None in (xl, xr, c) else bool(xl <= c <= xr)))
        m['checks'] = checks
        far = [c['d'] for c in checks if c['d'] is not None]
        m['d_dir'] = max(far, key=abs) if far else None
        m['dir_diff'] = d(m['d_dir'], m['d_near'])      # change of the gap away from the control row
        xl = x_at(b['left'], y) if b else None; xr = x_at(b['right'], y) if b else None
        m['car_inside_now'] = [None if None in (xl, xr, m['car']) else bool(xl <= m['car'] <= xr)] + [c['inside'] for c in checks]
        return m

    def _agrees(self, m):
        p = self.p
        return (m['support'] in ('strong', 'weak') and m['d_near'] is not None and abs(m['d_near']) <= p['AGREE_PX']
                and len(m['checks']) > 0 and all(c['d'] is not None and abs(c['d']) <= p['AGREE_DIR_PX'] for c in m['checks'])
                and all(v is True for v in m['car_inside_now']))

    def _takes(self, m):
        return m['support'] == 'strong' and m['d_near'] is not None and abs(m['d_near']) > self.p['D']

    # -- step --------------------------------------------------------------
    def step(self, t, car_path, car_valid, cor, y):
        p, events = self.p, []
        m = self.measure(car_path, car_valid, cor, y)
        if self.state == 'STOPPED':
            return self._out(t, m, events)
        if self.t is not None and not 0 < t - self.t <= p['GAP_S']:
            events.append(self._event(t, 'RESET', 'TIME_BACKWARD' if t <= self.t else 'TIME_GAP', m, 'FOLLOW'))
            self.reset(t)
        self.t = t
        s = self.state
        if m['support'] == 'strong': self.last_strong = t
        if s == 'FOLLOW':
            if self._takes(m):
                self._go(t, 'TAKEN', 'STRONG_CORRIDOR_DISAGREES', m, events)
            elif m['car'] is None:          # baseline invalid or not covering the row
                self._go(t, 'NO_TARGET', 'BASELINE_INVALID', m, events)
        elif s in ('TAKEN', 'PREDICT'):
            agree = self._agrees(m); self.agree_n = self.agree_n + 1 if agree else 0
            if m['status'] == 'OFF_CORRIDOR':
                self._go(t, 'NO_TARGET', 'VEHICLE_OUTSIDE_CORRIDOR', m, events)
            elif self.agree_n >= p['AGREE_FRAMES']:
                self._go(t, 'FOLLOW', 'BASELINE_REJOINED', m, events)
            elif m['support'] in ('strong', 'weak'):
                if self.last_strong is not None and t - self.last_strong > p['WEAK_S']:
                    self._go(t, 'NO_TARGET', 'NO_STRONG_SUPPORT', m, events)
                elif s == 'PREDICT':
                    self._go(t, 'TAKEN', 'SUPPORT_RETURNED', m, events)
            elif m['support'] == 'predicted' and s == 'TAKEN':
                self._go(t, 'PREDICT', 'SUPPORT_LOST', m, events)
            elif m['support'] == 'none' or (s == 'PREDICT' and t - self.since > p['PREDICT_S']):
                self._go(t, 'NO_TARGET', 'PREDICTION_LIMIT' if m['support'] == 'predicted' else 'CORRIDOR_LOST', m, events)
        elif s == 'NO_TARGET':
            if t - self.since > p['STOP_S'] and self.reason != 'BASELINE_INVALID':
                # A missing baseline keeps the baseline's own (unchanged) stop rule.
                self._go(t, 'STOPPED', 'STALE_STOP', m, events)
            elif self.reason == 'BASELINE_INVALID':
                # The corridor is not a fallback for a missing baseline.
                if m['car'] is not None:
                    self._go(t, 'TAKEN' if self._takes(m) else 'FOLLOW', 'STRONG_CORRIDOR_DISAGREES' if self._takes(m) else 'BASELINE_VALID_AGAIN', m, events)
            else:
                agree = self._agrees(m); self.agree_n = self.agree_n + 1 if agree else 0
                if self.agree_n >= p['AGREE_FRAMES']:
                    self._go(t, 'FOLLOW', 'BASELINE_REJOINED', m, events)
                elif m['support'] == 'strong' and m['status'] != 'OFF_CORRIDOR':
                    self._go(t, 'TAKEN', 'STRONG_CORRIDOR_RETURNED', m, events)
        return self._out(t, m, events, car_path, cor)

    def _go(self, t, state, reason, m, events):
        events.append(self._event(t, self.state, reason, m, state))
        if state == 'TAKEN' and self.state == 'FOLLOW': self.last_strong = t
        self.state, self.since, self.reason, self.agree_n = state, t, reason, 0

    def _event(self, t, before, reason, m, after):
        keys = ('d_near', 'd_dir', 'dir_diff', 'car', 'cor', 'checks', 'status', 'L', 'R', 'support', 'car_inside_now')
        return dict(t=t, before=before, after=after, reason=reason, **{k: m[k] for k in keys})

    def _out(self, t, m, events, car_path=None, cor=None):
        s = self.state
        src = {'FOLLOW': 'baseline', 'TAKEN': 'corridor', 'PREDICT': 'corridor_predicted'}.get(s)
        x = m['car'] if src == 'baseline' else m['cor'] if src else None
        path = (car_path if src == 'baseline' else (cor or {}).get('path') if src else None) or []
        if src == 'baseline' and x is None: src, path = None, []
        jump = None if x is None or self.prev_target is None else x - self.prev_target
        for e in events: e['target_jump'] = jump; e['target'] = x
        if x is not None: self.prev_target = x
        return dict(t=t, state=s, source=src, target=x, target_path=path, jump=jump, events=events, m=m)


# ---- on-car candidate (offline origin: handpush-newpose-20261031/fallback.py) ----
class FallbackSwitch(JunctionSwitch):
    """Baseline has no target at the control row while the corridor has strong
    two-side support (not OFF_CORRIDOR): use the corridor, unless the remaining
    baseline path disagrees with it by more than D at the lowest row it still
    covers (no switching onto another branch)."""
    REVISION = 'junction-switch-r1-fallback'

    def _fallback(self, m, car_path, cor):
        if m['car'] is not None or m['support'] != 'strong' or m['status'] == 'OFF_CORRIDOR': return False
        path = cor.get('path') or []
        if car_path and len(car_path) > 1 and len(path) > 1:
            yb = float(np.max(np.asarray(car_path, float)[:, 1])) / 2
            a, b = x_at(car_path, yb), x_at(path, yb)
            if a is not None and b is not None and abs(a - b) > self.p['D']: return False
        return True

    def step(self, t, car_path, car_valid, cor, y):
        if self.state in ('FOLLOW', 'NO_TARGET') and (self.state == 'FOLLOW' or self.reason == 'BASELINE_INVALID') and self.t is not None and 0 < t - self.t <= self.p['GAP_S']:
            m = self.measure(car_path, car_valid, cor, y)
            if self._fallback(m, car_path, cor):
                self.t = t; self.last_strong = t; events = []
                self._go(t, 'TAKEN', 'BASELINE_NO_PATH_CORRIDOR', m, events)
                return self._out(t, m, events, car_path, cor)
        return super().step(t, car_path, car_valid, cor, y)


class FallbackHandback(FallbackSwitch):
    """When the corridor has had no strong support for WEAK_S, re-evaluate
    instead of dropping the target. r2 (Codex review 2026-10-31): hand back only
    if the valid baseline is within D at the control row AND at the direction
    rows, and lies inside the CURRENT frame's corridor boundaries at every
    checked row; otherwise NO_TARGET (the existing stop rule follows)."""
    REVISION = 'junction-switch-r1-fallback-handback-r12'
    # r4: direction-aware take-over. 192bc52f junction 2: corridor r1.5 kept the
    # straight (correct) branch while the baseline turned into the left branch
    # with only 25-60 px at the control row (< D) but 70-99 px at the direction
    # rows for ~1.6 s; the 80 px take-over came at 195 px, too late. Take over
    # also when the corridor has strong support and |d_dir| > DIR_TAKE_PX for
    # DIR_TAKE_FRAMES consecutive frames. dir_take.py (corridor r1.5, 10
    # sources): 70 px / 5 frames takes 192bc52f at 49.88 s (26 px gap) and adds
    # no take-over on hand-push 853059f6, the straight runs or other laps;
    # 3 frames added a curve take-over (3ca87113 30.88 s). Experiment values.
    P = dict(FallbackSwitch.P, DIR_TAKE_PX=70, DIR_TAKE_FRAMES=5)

    # r5: live 94b6309c stopped at 5.72 s. The baseline flipped in one frame
    # from right boundary + width (34 points) to left boundary + width (14
    # points, the left line diverging at junction 1); its remnant ended above
    # the control row, 105 px from a strong two-side corridor, so the fallback
    # guard (remnant vs corridor > D -> no take-over) refused and PID stopped.
    # The same remnant was 94 px away from the previous frame's baseline at
    # that row. The guard now vetoes only a remnant continuous with the
    # previous frame's baseline (within D); a remnant that itself jumped more
    # than D is treated as a baseline glitch. With no previous baseline the
    # guard stays as before. D reuses the existing take-over distance.
    def reset(self, t):
        super().reset(t); self.dir_n = 0; self.prev_car = self.prev_car_t = None; self.agree_hist = []; self.back_n = 0

    # r6: live b6baecb7 stopped at 117.5 s at the corner before the ramp, and
    # hand-push 9a2c6570 has a 1.04 s gap at the same place (40.6 s): the
    # right road edge has no white line, the baseline (left line + width)
    # ends above the control row, and the corridor sees the left boundary on
    # 76-150 rows but the right one on 5-9 -> support 'weak' -> no fallback.
    # The fallback now also accepts weak support when one side has at least
    # WEAK_FALLBACK_ROWS rows, the corridor is BOTH/SINGLE and covers the
    # control row. weak_fallback.py: 60 rows removes the corner gap, changes
    # only the b6baecb7 stop frame and nothing in 20 other runs / 853059f6
    # (40 rows added a take at the PWM20 divergence frame de90b02c 19.03 s).
    # Once taken, the existing WEAK_S hold and hand-back test apply.
    P = dict(P, WEAK_FALLBACK_ROWS=60)

    # r9: live de4888e4 stopped at 30.75 s at the same corner: the car came in
    # at a large heading angle, the corridor saw the left boundary on 52 rows
    # (< 60) -> no fallback. The row count depends on how the car approaches
    # (heading / lateral offset change how many rows of a diagonal boundary are
    # in the image). Also accept the weak fallback by continuity: one side has
    # >= CONT_ROWS rows, the corridor target is at least CONT_EDGE_PX from the
    # image sides, and on the last CONT_FRAMES baseline-valid frames (recent,
    # within GAP_S + 0.1 s per frame) the corridor was within CONT_PX of the
    # baseline at the control row (de4888e4: 9-32 px). The edge margin keeps
    # de90b02c 19.03 s (PWM20, baseline and corridor both drifted to x 26-46)
    # unchanged. continuity_fallback.py: only de4888e4 changes in 45 sources.
    P = dict(P, CONT_ROWS=20, CONT_PX=40, CONT_FRAMES=3, CONT_EDGE_PX=80)

    # r10: live b3a817b4 stopped at 30.81 s in the same corner: the last three
    # baseline frames were -24 / -23 / +41 px from the corridor (r9 limit 40) and
    # the remnant on the stop frame was 9 px from it. In this corner the two
    # differ by 20-70 px on many runs (the corridor is a straight model, the road
    # bends into the ramp). The continuity limit is now D (80 px, the existing
    # take-over distance). An early hand-over while the baseline is still a
    # short remnant (junction_switch-r10-stub-rejected.py) was rejected: it
    # also fired at the ramp crest and added two stops (2302a9fa 43.25 s,
    # 0f47b0fd 63.65 s) in replay-r10-stub-rejected.json.
    P = dict(P, CONT_PX=80)

    # r11: live 91a23044 stopped at 33.06 s entering the ramp: the corner
    # fallback took the corridor at 31.03 s, the baseline was back and
    # continuous from 32.13 s (55-81 px right of the corridor: the road bends
    # into the ramp, the corridor is a straight model); at NO_STRONG_SUPPORT
    # (WEAK_S 2 s) the r8 hand-back needed |d_near| <= D 80 and got 81 -> stop.
    # Only at that moment (the alternative is NO_TARGET, i.e. a stop) a
    # fallback take now also hands back when the baseline has been valid and
    # continuous on the last BACK_FRAMES frames and is within BACK_D. BACK_D
    # 100 px: 91a23044 needs 81; at the ramp crest the baseline is 130-160 px
    # off (0e343cb4) and stays refused. An earlier hand-back as soon as the
    # baseline returned was rejected (junction_switch-r11-early-rejected.py,
    # replay-r11-unrestricted.json / -weak-rejected.json: it changed junction
    # 1 in 20 runs and made 6c2476a7 lose its target from 12.76 s).
    P = dict(P, BACK_FRAMES=3, BACK_D=100)

    # r12 (user 2026-11-03, live c933191a 32.99-33.09 s): crossing a junction the baseline jumped 374 -> 300 -> 254
    # px in two frames into the side branch while the strong corridor stayed at 368-375; the first frame was still
    # under the 80 px take-over (68 px) and the steering dipped for one frame. A baseline that moves more than
    # JUMP_PX from the previous frame's baseline at the control row, while the strong corridor is within JUMP_COR_PX
    # of that previous baseline, is taken over by the corridor at once (normal hand-back afterwards).
    P = dict(P, JUMP_PX=40, JUMP_COR_PX=25)
    # crossings-worker-r1 (crossings-20261114): a fallback take (baseline without a control-row target) is refused when the
    # corridor target is more than fb_cont_px from the previous valid baseline (<= GAP_S old) at the control row. None = off.
    fb_cont_px = None
    fb_refused = 0
    # curve3-handback-r1 (curve3-20261115): when a corridor take ends with VEHICLE_OUTSIDE_CORRIDOR, hand back to the baseline
    # if it covers the control row and is within oh_px of the previous valid baseline (<= GAP_S old) -- for every take
    # reason, not only BASELINE_NO_PATH_CORRIDOR (r7) -- and repeat that test on each NO_TARGET frame of that episode until
    # STOP_S. Without it the switch waits STOP_S, latches STOPPED, is unlatched by JunctionLine and follows the same
    # baseline anyway (curve 3: ~0.5 s JUNCTION_NO_TARGET with a continuous left+width baseline). oh_on False = unchanged.
    # Restrictions: oh_side -- the baseline lies on the same side of the visual centre (oh_centre, set by JunctionLine) as
    # the last emitted target and at least as far out (the straight corridor lags on the inside of a curve; a hand-back
    # never reverses the steering side); oh_hist_n -- the baseline covered the control row on the last oh_hist_n frames,
    # each step <= oh_hist_px. Experiment values (curve3-20261115 SUMMARY), not validated limits.
    oh_on = False
    oh_px = 80.0
    oh_n = 0
    oh_side = False
    oh_centre = None
    oh_hist_n = 0
    oh_hist_px = 120.0
    oh_hist = ()
    # corridor-guard-r1 (corridor-20261118, offline candidate): run-4a76 laps 2 / 3 and 8a43 lap 3 (ramp-foot corner) -- the
    # fallback corridor (straight two-line model, one visible outer line) rotated toward the upper left within 0.3-0.4 s while
    # the baseline (left line + width) was back, continuous and 100-200 px to the right; the take ended only at OFF_CORRIDOR,
    # handing back 153 / 193 px away. cg_* are set by JunctionLine from crossings_worker.json 'corridor_guard' (None = off,
    # unchanged): refusals of a fallback take (cg_off / cg_gz_max / cg_cal_hmax / cg_cal_age_s / cg_dir_px) and an early
    # hand-back of a fallback take to a continuous baseline that diverges from the corridor (cg_back). Experiment values.
    cg_on = False
    cg_off = False
    cg_gz_max = None
    cg_cal_hmax = None
    cg_cal_age_s = None
    cg_dir_px = None
    cg_dir_dy = 40.0
    cg_back = None
    cg_gz = None   # worker IMU gz this frame (JunctionLine; None = unknown -> no gz refusal)
    cg_cal = None  # (horizon_y, capture_t) of the corridor calibration after this frame's corridor step (JunctionLine)
    cg_refused = None
    cg_backs = 0
    # corridor-weak-r1 (corridor-weak-20261118, offline candidate): run-0f91 2030.7 / 2048.1 s (main straight / diagonal), the car
    # off-centre: a fallback take (one-frame baseline gap) on 'weak' support (one side 57-127 rows, other 5-14) put the corridor
    # 100-170 px right of a baseline that was valid again and built from both lines; the take held for WEAK_S 2 s, then
    # NO_STRONG_SUPPORT -> NO_TARGET 0.4 s -> unlatched -> baseline with a 0.4-0.6 image-error step. cw_* are set by JunctionLine
    # from crossings_worker.json 'corridor_weak' (None = off, unchanged). Experiment values, not validated limits.
    cw_on = False
    cw_px = 60.0
    cw_cont_px = 40.0
    cw_hold_s = 0.1
    cw_reasons = ('BASELINE_NO_PATH_CORRIDOR',)
    cw_two_min = None
    cw_skip_zones = ()
    cw_expire = False
    cw_expire_px = None
    cw_ramp_s = None
    cw_two = None    # fraction of baseline rows from both lines this frame (JunctionLine; None = unknown)
    cw_zone = None   # live_pid zone flag this frame (JunctionLine)
    cw_since = None
    cw_ramp = None   # (t0, offset_px at t0) of an active hand-back ramp
    cw_backs = 0
    cw_expires = 0
    cw_strong = None   # strong_take {two_min, lr_max} (None = off)
    cw_gz_max = None
    cw_gz = None     # worker IMU gz this frame (JunctionLine; None = unknown)
    cw_strong_refused = 0

    def _fallback_r8(self, m, car_path, cor):
        if m['car'] is not None or m['status'] == 'OFF_CORRIDOR': return False
        weak_ok = (m['support'] == 'weak' and m['status'] in ('BOTH', 'SINGLE') and m['cor'] is not None
                   and max(m['L'] or 0, m['R'] or 0) >= self.p['WEAK_FALLBACK_ROWS'])
        if m['support'] != 'strong' and not weak_ok: return False
        path = cor.get('path') or []
        if car_path and len(car_path) > 1 and len(path) > 1:
            yb = float(np.max(np.asarray(car_path, float)[:, 1])) / 2
            a, b = x_at(car_path, yb), x_at(path, yb)
            if a is not None and b is not None and abs(a - b) > self.p['D']:
                prev = getattr(self, 'prev_car', None); q = x_at(prev, yb) if prev else None
                if q is None or abs(a - q) <= self.p['D']: return False      # r5 remnant guard
        return True

    def _fallback(self, m, car_path, cor):
        ok = self._fallback_core(m, car_path, cor)
        if ok and self.fb_cont_px is not None:   # crossings-worker-r1
            prev = getattr(self, 'prev_car', None); q = x_at(prev, m['y']) if prev else None
            if q is not None and m['cor'] is not None and abs(m['cor'] - q) > self.fb_cont_px:
                self.fb_refused += 1; return False
        if ok and self.cg_on:   # corridor-guard-r1
            why = self._cg_refuse(m, cor)
            if why:
                self.cg_refused = dict(self.cg_refused or {}); self.cg_refused[why] = self.cg_refused.get(why, 0) + 1; return False
        return ok

    def _cg_refuse(self, m, cor):   # corridor-guard-r1: reason string when a fallback take is refused, else None
        if self.cg_off: return 'OFF'
        if self.cg_gz_max is not None and self.cg_gz is not None and abs(self.cg_gz) > self.cg_gz_max: return 'GZ'
        cal = self.cg_cal
        if self.cg_cal_hmax is not None and cal is not None and cal[0] > self.cg_cal_hmax: return 'CAL_H'
        if self.cg_cal_age_s is not None and cal is not None and getattr(self, 't_now', None) is not None and self.t_now - cal[1] > self.cg_cal_age_s: return 'CAL_AGE'
        if self.cg_dir_px is not None:
            prev = getattr(self, 'prev_car', None); y = m['y']; yu = y - self.cg_dir_dy
            q0 = x_at(prev, y) if prev else None; q1 = x_at(prev, yu) if prev else None
            c0 = m['cor']; c1 = x_at(cor.get('path') or [], yu)
            if None not in (q0, q1, c0, c1) and abs((c1 - c0) - (q1 - q0)) > self.cg_dir_px: return 'DIR'
        return None

    def _cw_back_ok(self, m):   # corridor-weak-r1: weak corridor, baseline back, continuous and diverging
        if m['support'] != 'weak' or m['car'] is None or m['cor'] is None: return False
        if self.cw_zone is not None and self.cw_zone in self.cw_skip_zones: return False
        if self.cw_two_min is not None and (self.cw_two is None or self.cw_two < self.cw_two_min): return False
        if self.cw_gz_max is not None and (self.cw_gz is None or abs(self.cw_gz) > self.cw_gz_max): return False
        prev = getattr(self, 'prev_car', None); q = x_at(prev, m['y']) if prev else None
        if q is None or abs(m['car'] - q) > self.cw_cont_px: return False
        return abs(m['car'] - m['cor']) > self.cw_px

    def _cw_take_gate(self, m, hit):   # corridor-weak-r1 strong_take (off -> hit unchanged)
        if hit and self.cw_on and self.cw_strong and m['car'] is not None and not (self.cw_zone is not None and self.cw_zone in self.cw_skip_zones):
            g = self.cw_strong
            if (self.cw_two is not None and self.cw_two >= float(g.get('two_min', .75))
                    and min(m['L'] or 0, m['R'] or 0) < float(g.get('lr_max', 30))):
                self.cw_strong_refused += 1; return False
        return hit

    def _cw_expire_ok(self, m):   # corridor-weak-r1: at NO_STRONG_SUPPORT, a valid baseline continuous with the previous one
        if m['car'] is None or not self._baseline_continuous(m): return False
        if self.cw_zone is not None and self.cw_zone in self.cw_skip_zones: return False
        return self.cw_expire_px is None or (m['d_near'] is not None and abs(m['d_near']) <= self.cw_expire_px)

    def _out(self, t, m, events, car_path=None, cor=None):   # corridor-weak-r1 ramp after a hand-back (off: unchanged)
        if self.cw_ramp is not None and self.state == 'FOLLOW' and car_path:
            t0, off = self.cw_ramp; w = 1. - (t - t0) / self.cw_ramp_s
            if w > 0 and m['car'] is not None:
                o = super()._out(t, m, events, [[float(p[0]) + 2. * off * w, float(p[1])] for p in car_path], cor)
                o['target'] = self.prev_target = m['car'] + off * w; o['cw_ramp_w'] = round(w, 3)
                return o
        self.cw_ramp = None
        return super()._out(t, m, events, car_path, cor)

    def _cg_back_ok(self, m):   # corridor-guard-r1: early hand-back of a fallback take (see cg_back)
        b = self.cg_back; prev = getattr(self, 'prev_car', None); q = x_at(prev, m['y']) if prev else None
        if m['car'] is None or q is None or m['cor'] is None: return False
        if abs(m['car'] - q) > float(b.get('cont_px', 40.)) or getattr(self, 'back_n', 0) < int(b.get('n', 2)): return False
        if b.get('gz_min') is not None:   # only while the car yaws toward the baseline side (straight model lagging in a curve)
            if self.cg_gz is None or abs(self.cg_gz) < float(b['gz_min']) or (m['car'] - m['cor']) * -self.cg_gz <= 0: return False
        return abs(m['car'] - m['cor']) > float(b.get('px', 60.))

    def _fallback_core(self, m, car_path, cor):
        if self._fallback_r8(m, car_path, cor): return True
        if m['car'] is not None or m['status'] not in ('BOTH', 'SINGLE') or m['cor'] is None: return False
        if max(m['L'] or 0, m['R'] or 0) < self.p['CONT_ROWS']: return False
        if not self.p['CONT_EDGE_PX'] <= m['cor'] <= 640 - self.p['CONT_EDGE_PX']: return False
        n = self.p['CONT_FRAMES']; t = getattr(self, 't_now', None)
        h = [d for s, d in getattr(self, 'agree_hist', []) if t is not None and 0 < t - s <= self.p['GAP_S'] + .1 * n]
        if len(h) < n or any(abs(d) > self.p['CONT_PX'] for d in h[-n:]): return False
        path = cor.get('path') or []
        if car_path and len(car_path) > 1 and len(path) > 1:
            yb = float(np.max(np.asarray(car_path, float)[:, 1])) / 2
            a, b = x_at(car_path, yb), x_at(path, yb)
            if a is not None and b is not None and abs(a - b) > self.p['D']: return False
        return True

    def _takes(self, m):
        hit = m['support'] == 'strong' and m['d_dir'] is not None and abs(m['d_dir']) > self.p['DIR_TAKE_PX']
        self.dir_n = getattr(self, 'dir_n', 0) + 1 if hit else 0
        return self._cw_take_gate(m, super()._takes(m) or self.dir_n >= self.p['DIR_TAKE_FRAMES'])   # corridor-weak-r1 gate

    def _handback_ok(self, m):
        D = self.p['D']
        return (m['car'] is not None and m['d_near'] is not None and abs(m['d_near']) <= D
                and len(m['checks']) > 0 and all(c['d'] is not None and abs(c['d']) <= D for c in m['checks'])
                and all(v is True for v in m['car_inside_now']))

    # r7: live 0005e824 stopped at 31.59 s in the corner before the ramp. r6 had
    # handed a missing baseline to the corridor (30.22 s, weak, left boundary
    # only); in the curve the straight-corridor fit rotated until the vehicle
    # was outside it (OFF_CORRIDOR) -> NO_TARGET -> stop, while the baseline was
    # valid again and continuous (x 402-494). When the corridor was only
    # covering for a missing baseline (take reason BASELINE_NO_PATH_CORRIDOR)
    # and the corridor puts the vehicle outside itself (VEHICLE_OUTSIDE_CORRIDOR),
    # hand back to the baseline if it covers the control row and is within D
    # of the previous valid baseline. Support loss (PREDICTION_LIMIT /
    # CORRIDOR_LOST / NO_STRONG_SUPPORT) is unchanged: at the ramp crest
    # (hand-push 9a2c6570 61.2 s, live 0e343cb4) the baseline jumps to the far
    # right while support fades, and handing back there steered toward the edge
    # in the first r7 draft. Junction take-overs are unchanged.
    # r8: live 50dcb260 stopped at 28.44 s in the ramp corner: a fallback corridor
    # (baseline briefly without a control-row point) reached NO_STRONG_SUPPORT;
    # the baseline was back, continuous and 17 px from the corridor at the
    # control row, but 105 px away at the upper direction row because the road
    # bends into the ramp while the corridor is a straight model -> the
    # junction hand-back test refused -> stop. For a fallback take (reason
    # BASELINE_NO_PATH_CORRIDOR) the hand-back now needs only the control row
    # (|d_near| <= D) and a baseline continuous with the previous valid one.
    # Junction take-overs keep the full test; the ramp crest (0e343cb4,
    # baseline 130-160 px off at the control row) is still refused.
    def _go(self, t, state, reason, m, events):
        if reason == 'NO_STRONG_SUPPORT' and (self._handback_ok(m) or (
                getattr(self, 'take_reason', None) == 'BASELINE_NO_PATH_CORRIDOR' and m['d_near'] is not None and self._baseline_continuous(m)
                and (abs(m['d_near']) <= self.p['D'] or (abs(m['d_near']) <= self.p['BACK_D'] and getattr(self, 'back_n', 0) >= self.p['BACK_FRAMES'])))):
            state, reason = 'FOLLOW', 'CORRIDOR_WEAK_BASELINE_WITHIN_D'
        elif (state == 'NO_TARGET' and reason == 'VEHICLE_OUTSIDE_CORRIDOR'
              and self.state in ('TAKEN', 'PREDICT') and getattr(self, 'take_reason', None) == 'BASELINE_NO_PATH_CORRIDOR'
              and self._baseline_continuous(m)):
            state, reason = 'FOLLOW', 'FALLBACK_CORRIDOR_FAILED_BASELINE_BACK'
        elif (self.oh_on and state == 'NO_TARGET' and reason == 'VEHICLE_OUTSIDE_CORRIDOR'   # curve3-handback-r1
              and self.state in ('TAKEN', 'PREDICT') and self._oh_ok(m)):
            state, reason = 'FOLLOW', 'OUTSIDE_CORRIDOR_BASELINE_BACK'; self.oh_n += 1
        elif (self.cw_on and self.cw_expire and state == 'NO_TARGET' and reason == 'NO_STRONG_SUPPORT'   # corridor-weak-r1
              and getattr(self, 'take_reason', None) in self.cw_reasons and self._cw_expire_ok(m)):
            state, reason = 'FOLLOW', 'CORRIDOR_WEAK_EXPIRED_BASELINE'; self.cw_expires += 1
            if self.cw_ramp_s and m['cor'] is not None and m['car'] is not None: self.cw_ramp = (t, m['cor'] - m['car'])
        if state == 'TAKEN' and self.state != 'TAKEN' and reason != 'SUPPORT_RETURNED': self.take_reason = reason
        super()._go(t, state, reason, m, events)

    def _oh_ok(self, m):   # curve3-handback-r1: continuity + optional side / history restrictions
        prev = getattr(self, 'prev_car', None); q = x_at(prev, m['y']) if prev else None
        if m['car'] is None or q is None or abs(m['car'] - q) > self.oh_px: return False
        if self.oh_side:
            c, tg = self.oh_centre, self.prev_target
            if c is None or tg is None or (m['car'] - c) * (tg - c) <= 0 or abs(m['car'] - c) < abs(tg - c): return False
        if self.oh_hist_n:
            h = list(self.oh_hist)[-self.oh_hist_n:]
            if len(h) < self.oh_hist_n or not 0 < self.t_now - h[-1][0] <= self.p['GAP_S']: return False
            if any(not 0 < b[0] - a[0] <= self.p['GAP_S'] or abs(b[1] - a[1]) > self.oh_hist_px for a, b in zip(h, h[1:])): return False
        return True

    def _baseline_continuous(self, m):
        prev = getattr(self, 'prev_car', None); q = x_at(prev, m['y']) if prev else None
        return m['car'] is not None and q is not None and abs(m['car'] - q) <= self.p['D']

    def step(self, t, car_path, car_valid, cor, y):
        # previous valid baseline path, only if it is at most GAP_S old
        pc = getattr(self, 'prev_car_t', None)
        self.prev_car = self.prev_car if pc is not None and 0 < t - pc <= self.p['GAP_S'] else None
        self.t_now = t
        out = self._step(t, car_path, car_valid, cor, y)
        m = out.get('m') or {}
        # r11: consecutive valid, continuous baseline frames (counted after this frame's decision)
        cont = car_valid and car_path and len(car_path) > 1 and self.prev_car is not None and m.get('car') is not None and x_at(self.prev_car, y) is not None and abs(m['car'] - x_at(self.prev_car, y)) <= self.p['D']
        self.back_n = getattr(self, 'back_n', 0) + 1 if cont else 0
        if m.get('car') is not None and m.get('cor') is not None and m.get('support') in ('strong', 'weak'):
            self.agree_hist = (getattr(self, 'agree_hist', []) + [(t, m['d_near'])])[-10:]
        elif m.get('car') is not None: self.agree_hist = []
        if car_valid and car_path and len(car_path) > 1: self.prev_car, self.prev_car_t = car_path, t
        if self.oh_on:   # curve3-handback-r1: recent baseline control-row targets (history restriction)
            self.oh_hist = (tuple(self.oh_hist) + ((t, m['car']),))[-10:] if car_valid and m.get('car') is not None else ()
        return out

    def _step(self, t, car_path, car_valid, cor, y):
        if self.cw_on:   # corridor-weak-r1: early hand-back of a weak-supported take (see cw_*)
            if (self.state == 'TAKEN' and getattr(self, 'take_reason', None) in self.cw_reasons
                    and self.t is not None and 0 < t - self.t <= self.p['GAP_S']):
                m = self.measure(car_path, car_valid, cor, y)
                if self._cw_back_ok(m):
                    if self.cw_since is None: self.cw_since = t
                    if t - self.cw_since >= self.cw_hold_s - 1e-9:
                        self.t = t; events = []; self.cw_backs += 1; self.cw_since = None
                        if self.cw_ramp_s: self.cw_ramp = (t, m['cor'] - m['car'])
                        self._go(t, 'FOLLOW', 'CORRIDOR_WEAK_BASELINE_DIVERGES', m, events)
                        return self._out(t, m, events, car_path, cor)
                else: self.cw_since = None
            else: self.cw_since = None
        if (self.cg_on and self.cg_back and self.state in ('TAKEN', 'PREDICT')   # corridor-guard-r1
                and getattr(self, 'take_reason', None) in self.cg_back.get('reasons', ('BASELINE_NO_PATH_CORRIDOR',))
                and self.t is not None and 0 < t - self.t <= self.p['GAP_S']):
            m = self.measure(car_path, car_valid, cor, y)
            if self._cg_back_ok(m):
                self.t = t; events = []; self.cg_backs += 1
                if m['support'] == 'strong': self.last_strong = t
                self._go(t, 'FOLLOW', 'FALLBACK_BASELINE_DIVERGES', m, events)
                return self._out(t, m, events, car_path, cor)
        if self.state == 'FOLLOW' and self.prev_car is not None and car_valid and self.t is not None and 0 < t - self.t <= self.p['GAP_S']:
            m = self.measure(car_path, car_valid, cor, y); q = x_at(self.prev_car, y)
            if (q is not None and m['car'] is not None and m['cor'] is not None and m['support'] == 'strong' and m['status'] != 'OFF_CORRIDOR'
                    and abs(m['car'] - q) > self.p['JUMP_PX'] and abs(m['cor'] - q) <= self.p['JUMP_COR_PX']):
                self.t = t; self.last_strong = t; events = []
                self._go(t, 'TAKEN', 'BASELINE_JUMP_CORRIDOR', m, events)
                return self._out(t, m, events, car_path, cor)
        if (self.oh_on and self.state == 'NO_TARGET' and getattr(self, 'reason', None) == 'VEHICLE_OUTSIDE_CORRIDOR'   # curve3-handback-r1
                and self.t is not None and 0 < t - self.t <= self.p['GAP_S'] and t - self.since <= self.p['STOP_S']):
            m = self.measure(car_path, car_valid, cor, y)
            if self._oh_ok(m):
                self.t = t; events = []; self.oh_n += 1
                self._go(t, 'FOLLOW', 'OUTSIDE_CORRIDOR_BASELINE_BACK', m, events)
                return self._out(t, m, events, car_path, cor)
        # r3: after NO_STRONG_SUPPORT the same hand-back test is repeated on each
        # frame until the existing STOP_S expires, instead of only on the single
        # expiry frame (corridor r1.5 synthetic case: 83 px on that frame, 75-77
        # px on the next ones). The test itself is unchanged.
        if (self.state == 'NO_TARGET' and getattr(self, 'reason', None) == 'NO_STRONG_SUPPORT' and self.t is not None
                and 0 < t - self.t <= self.p['GAP_S'] and t - self.since <= self.p['STOP_S']):
            m = self.measure(car_path, car_valid, cor, y)
            if self._handback_ok(m):
                self.t = t; events = []
                self._go(t, 'FOLLOW', 'CORRIDOR_WEAK_BASELINE_WITHIN_D', m, events)
                return self._out(t, m, events, car_path, cor)
        return super().step(t, car_path, car_valid, cor, y)
