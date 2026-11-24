"""Ramp integral release (i-ramp-r1, ramp-bias-20261119; offline candidate, NOT deployed).

Problem (runs aabb / d2c7 / 436d / 091c of 2026-11-19, 16 climbs): every right curve drives the camera PID integral to its
limit (+40 us = ki 40 x integral_limit 1.0). Curve 3 ends at the ramp foot, so the car starts every climb with I = +40 and it
bleeds only ~7 us/s; on the ramp (C3e + 1.0..1.8 m) the median is e -0.17 (car ~6-7 cm right), P -39 us, I +31 us, D ~0 ->
command only ~-12 us from centre, gz ~0: the integral holds the car on the right side until the crest.

This module only changes the camera PID's integral state (controller.i), never the steering value of the current frame (so the
layer owner / this frame's command are unchanged; the effect starts with the next PID step). Config web/i_ramp.json
(file absent, unreadable or enabled false -> load() returns None -> live_pid never calls it = car behaviour unchanged):

  pitch_on_deg  climb detected when dash_route pitch_rel_deg >= this ...
  hold_s        ... continuously for this long (spikes at the landing / bumps never arm it)
  pitch_off_deg the climb ends (state reset) when the pitch drops below this
  tau_s         while armed: i *= exp(-dt / tau_s)  (decay toward 0; tau_s <= 0 -> no decay)
  cap_us        while armed: |ki * i| <= cap_us (null -> no cap)
  sign          'positive' (only a right-turn memory is released; default) or 'both'

i-exit-r1 (exit-bias-20261119; offline candidate, NOT deployed): the same release after the right-curve exits. Every right curve
drives the integral to +40 us; on the straight after C1e / C2e / C4e it is still +24..+31 us 4.5 m later and holds the car ~0.15-0.2
(5-8 cm) right of the path. Optional keys (absent = exactly the i-ramp-r1 behaviour above, log records unchanged):
  ramp          false -> the climb release above is skipped (exit release only); absent / true -> as before
  exit          {"C1e": {...}, "C2e": {...}, "C4e": {...}}  per lap_pos anchor (lap_pos state of its last update, ~5 Hz):
                  enabled  false -> this anchor is ignored
                  start_m  release starts when since_anchor_m >= start_m ...
                  end_m    ... and stops after since_anchor_m > end_m (then the integral runs as before)
                  tau_s    inside the window: i *= exp(-dt / tau_s) while i > 0 (time constant, not distance: lap_pos steps 0.2-0.3 m)
                  cap_us   inside the window: ki * i <= cap_us (null -> no cap)
                Not active while lap_pos is lost / has no anchor / is disabled. Only a positive (right-turn) integral is touched.
                Log: proposal.i_ramp = {revision: 'i-exit-r1', exit: {...}} (or the ramp record + 'exit') while inside a window.

i-land-r1 (land-smooth-20261120; offline candidate, NOT deployed): the integral left by the upper section's left turns (-28..-36 us on the
landings without a seam blackout, 6 of 15 in f675/a5c0/709c/e81e; 0 when lost seam frames reset it) is released after the landing, where
curve 4 (right) starts. Optional key (absent = no change, log records unchanged):
  land          {"anchor": "LAND8", "start_m": 0.0, "end_m": 1.5, "tau_s": 0.3, "cap_us": null, "sign": "negative"}
                inside [start_m, end_m] after the anchor: i *= exp(-dt / tau_s) while the integral has the configured sign
                ('negative' = a left memory, default; 'positive'; 'both'); cap_us bounds |ki * i| of that sign. lap_pos lost / no anchor -> off.
                Log: proposal.i_ramp.land = {state, anchor, x_m, i_us_before, i_us} while inside the window (revision i-land-r1).
                A separate top-level key (not an 'exit' anchor): the i-exit-r1 module ignores it, so this file is inert with i_ramp 19a93210.

i-preload-r1 (c1preload 2026-11-21; offline candidate, NOT deployed): on the lap from the start the camera PID reaches curve 1 with
ki*i = -1..-10 us at the curve start (9 lap-1 passes after the 11-21 linkage repair), later laps with +16..+26 us (12 passes). The
missing ~28 us is ~0.13 of error: the early error peak is 0.80 instead of 0.72 and curve_ff (floor 0.55..0.85 -> centre+517 us) lifts
the command to 1900-1992 us (5/9 vs 2/12), the car over-rotates, the command falls back to centre mid-curve, saturates again and the
exit is right of the lane (run-69a8 e -1.00, run-bab2 -0.54). Optional key (absent = no change, log records unchanged):
  preload       {"target_us": 20, "e_on": 0.30, "n_on": 2, "min_s": 1.5, "max_s": 15, "rate_us_s": 150}
                Waits while lap_pos says lap 0 with no anchor yet (= the lap from the start, before the first landmark). When the PID's
                control error (controller.e) is >= e_on on n_on consecutive steps, at least min_s after the first step, the integral
                is raised to target_us (ki*i) at rate_us_s (null = in one step) and the preload is finished for the run. It only
                raises: an integral already at / above the target is left alone. Never again in the run: a lap_pos lap > 0 or an
                anchor before the trigger, lap_pos lost at the trigger, max_s without a trigger, or a replaced PID object (pause /
                resume rebuilds the controller) all switch it off. As the releases above it writes controller.i only; this frame's
                command is unchanged (the effect starts with the next PID step).
                Log: proposal.i_ramp.preload = {state RAMP / DONE / OFF, ...} on the frames where it acts or switches off (revision
                i-preload-r1 when no other i_ramp record exists on that frame).
"""
import json, math
from pathlib import Path

REVISION = 'i-ramp-r1'
EXIT_REVISION = 'i-exit-r1'   # exit-bias-20261119
EXIT_DEFAULTS = dict(enabled=True, start_m=0.6, end_m=3.0, tau_s=0.3, cap_us=None)
LAND_REVISION = 'i-land-r1'   # land-smooth-20261120
LAND_DEFAULTS = dict(enabled=True, anchor='LAND8', start_m=0.0, end_m=1.5, tau_s=0.3, cap_us=None, sign='negative')
PRE_REVISION = 'i-preload-r1'   # c1preload-20261121
PRE_DEFAULTS = dict(enabled=True, target_us=20., e_on=0.30, n_on=2, min_s=1.5, max_s=15., rate_us_s=150.)
CONFIG_FILE = Path(__file__).with_name('i_ramp.json')
DEFAULTS = dict(enabled=True, pitch_on_deg=8., hold_s=0.3, pitch_off_deg=4., tau_s=0.5, cap_us=10., sign='positive')


def load(path=None):
    try: raw = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
    except (OSError, ValueError): return None
    if not isinstance(raw, dict) or raw.get('enabled', True) is False: return None
    c = dict(DEFAULTS); c.update(raw)
    return IRamp(c)


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _exits(raw):
    """i-exit-r1: {anchor: config} of the enabled exits, or None (key absent / nothing enabled / malformed)."""
    if not isinstance(raw, dict): return None
    out = {}
    for an, v in raw.items():
        if not isinstance(v, dict) or v.get('enabled', True) is False: continue
        e = dict(EXIT_DEFAULTS); e.update(v)
        if not (_num(e['start_m']) and _num(e['end_m']) and _num(e['tau_s'])) or e['end_m'] <= e['start_m']: continue
        out[str(an)] = e
    return out or None


def _land_cfg(raw):
    """i-land-r1: the landing release config, or None (key absent / disabled / malformed)."""
    if not isinstance(raw, dict) or raw.get('enabled', True) is False: return None
    e = dict(LAND_DEFAULTS); e.update(raw)
    if not (_num(e['start_m']) and _num(e['end_m']) and _num(e['tau_s'])) or e['end_m'] <= e['start_m'] or e['sign'] not in ('negative', 'positive', 'both'): return None
    return e


def _pre_cfg(raw):
    """i-preload-r1: the lap-1 curve-1 preload config, or None (key absent / disabled / malformed)."""
    if not isinstance(raw, dict) or raw.get('enabled', True) is False: return None
    e = dict(PRE_DEFAULTS); e.update(raw)
    if not all(_num(e[k]) for k in ('target_us', 'e_on', 'n_on', 'min_s', 'max_s')) or e['target_us'] <= 0 or e['e_on'] <= 0 or e['n_on'] < 1: return None
    if e.get('rate_us_s') is not None and not (_num(e['rate_us_s']) and e['rate_us_s'] > 0): return None
    return e


class IRamp(object):
    def __init__(self, c):
        self.c = c; self.t_on = None; self.t_last = None; self.armed = False
        self.ramp_on = c.get('ramp', True) is not False   # i-exit-r1: absent -> the ramp release runs as before
        self.exits = _exits(c.get('exit'))               # i-exit-r1: absent -> None (no exit release, step() == i-ramp-r1)
        self.land = _land_cfg(c.get('land'))              # i-land-r1: absent -> None (no landing release)
        self.pre = _pre_cfg(c.get('preload'))            # i-preload-r1: absent -> None (step() == the body below, unchanged)
        self.wants_lap = self.exits is not None or self.land is not None or self.pre is not None
        self.p_state = 'WAIT'; self.p_ctrl = None; self.p_t0 = None; self.p_n = 0; self.p_t = None
        self.l_key = None; self.l_t = None; self.l_on = False
        self.x_key = None; self.x_t = None; self.x_on = False

    def step(self, now, pitch, ctrl, lap=None):
        """i-preload-r1 wrapper: without the preload key exactly _step_base (the previous step())."""
        r = self._step_base(now, pitch, ctrl, lap)
        if self.pre is None or self.p_state == 'OFF': return r
        pl = self._preload(now, ctrl, lap)
        if pl is None: return r
        return dict(revision=PRE_REVISION, preload=pl) if r is None else dict(r, preload=pl)

    def _preload(self, now, ctrl, lap):
        """i-preload-r1: raise the integral once at the entry of the first curve of the lap from the start."""
        c = self.pre
        if ctrl is not None and not _num(getattr(ctrl, 'i', None)) and getattr(ctrl, 'pid', None) is not None: ctrl = ctrl.pid
        if ctrl is None or not _num(getattr(ctrl, 'i', None)): return None
        if self.p_ctrl is None: self.p_ctrl = ctrl; self.p_t0 = now
        elif ctrl is not self.p_ctrl:   # the PID object was replaced (pause / resume): this is no longer the run's start
            self.p_state = 'OFF'; return dict(state='OFF', reason='CONTROLLER_REPLACED')
        ki = float((getattr(ctrl, 'p', None) or {}).get('ki') or 0.)
        if ki <= 0: self.p_state = 'OFF'; return dict(state='OFF', reason='NO_KI')
        e = getattr(ctrl, 'e', None)
        if self.p_state == 'WAIT':
            if not isinstance(lap, dict): self.p_n = 0; return None   # no lap_pos: cannot tell the first lap
            if lap.get('lap') != 0 or lap.get('anchor') is not None:
                self.p_state = 'OFF'; return dict(state='OFF', reason='NOT_BEFORE_FIRST_LANDMARK')
            if now - self.p_t0 > float(c['max_s']): self.p_state = 'OFF'; return dict(state='OFF', reason='TIMEOUT')
            if lap.get('lost') or not _num(e) or e < float(c['e_on']) or now - self.p_t0 < float(c['min_s']): self.p_n = 0; return None
            self.p_n += 1
            if self.p_n < int(c['n_on']): return None
            self.p_state = 'RAMP'; self.p_t = None
        tgt = float(c['target_us']) / ki; i0 = float(ctrl.i)
        if i0 >= tgt: self.p_state = 'OFF'; return dict(state='DONE', i_us=round(ki * i0, 2))
        rate = c.get('rate_us_s'); dt = None if self.p_t is None else now - self.p_t; self.p_t = now
        if rate is None: i1 = tgt
        else: i1 = min(tgt, i0 + float(rate) / ki * (dt if dt is not None and 0 < dt <= 0.1 else 1. / 30.))
        ctrl.i = i1
        if i1 >= tgt: self.p_state = 'OFF'
        return dict(state='DONE' if i1 >= tgt else 'RAMP', e=None if not _num(e) else round(float(e), 3), i_us_before=round(ki * i0, 2), i_us=round(ki * i1, 2))

    def _step_base(self, now, pitch, ctrl, lap=None):
        """now: monotonic s; pitch: dash_route pitch_rel_deg (None allowed); ctrl: the running PID controller (attributes i, p);
        lap (i-exit-r1, only passed when wants_lap): dict(lap, anchor, since_anchor_m, lost) of the last lap_pos update.
        Returns a small log dict, or None when nothing was evaluated."""
        r = self._ramp(now, pitch, ctrl) if self.ramp_on else None
        if self.land is not None:   # i-land-r1 (absent -> skipped: the original body below)
            ld = self._land(now, ctrl, lap)
            if ld is not None: r = dict(revision=LAND_REVISION, land=ld) if r is None else dict(r, land=ld)
        if self.exits is None: return r
        x = self._exit(now, ctrl, lap)
        if x is None: return r
        if r is None: return dict(revision=EXIT_REVISION, exit=x)
        r = dict(r); r['exit'] = x; return r

    def _land(self, now, ctrl, lap):
        """i-land-r1: decay the integral of the configured sign inside [start_m, end_m] after the landing anchor."""
        c = self.land
        if ctrl is not None and not _num(getattr(ctrl, 'i', None)) and getattr(ctrl, 'pid', None) is not None: ctrl = ctrl.pid
        off = None
        if self.l_on: off = dict(state='END', anchor=c['anchor'])
        if not isinstance(lap, dict) or ctrl is None or not _num(getattr(ctrl, 'i', None)) or lap.get('lost'): self.l_on = False; self.l_t = None; return off
        x = lap.get('since_anchor_m')
        if lap.get('anchor') != c['anchor'] or not _num(x) or x < float(c['start_m']) or x > float(c['end_m']): self.l_on = False; self.l_t = None; return off
        key = lap.get('lap')
        if key != self.l_key or not self.l_on: self.l_key = key; self.l_t = None
        self.l_on = True
        dt = None if self.l_t is None else now - self.l_t
        self.l_t = now
        ki = float((getattr(ctrl, 'p', None) or {}).get('ki') or 0.)
        i0 = float(ctrl.i); i1 = i0; sg = c['sign']
        if (i0 < 0 and sg in ('negative', 'both')) or (i0 > 0 and sg in ('positive', 'both')):
            tau = float(c['tau_s'])
            if tau > 0 and dt is not None and 0 < dt <= 0.5: i1 = i1 * math.exp(-dt / tau)
            cap = c.get('cap_us')
            if _num(cap) and ki > 0: lim = max(0., float(cap) / ki); i1 = max(-lim, min(lim, i1))
            ctrl.i = i1
        return dict(state='ON', anchor=c['anchor'], x_m=round(float(x), 2), i_us_before=round(ki * i0, 2), i_us=round(ki * i1, 2))

    def _x_off(self):
        if not self.x_on: return None
        self.x_on = False; self.x_t = None
        return dict(state='END', anchor=None if self.x_key is None else self.x_key[1])

    def _exit(self, now, ctrl, lap):
        """i-exit-r1: decay a positive integral inside [start_m, end_m] after an exit anchor."""
        if ctrl is not None and not _num(getattr(ctrl, 'i', None)) and getattr(ctrl, 'pid', None) is not None: ctrl = ctrl.pid
        if not isinstance(lap, dict) or ctrl is None or not _num(getattr(ctrl, 'i', None)) or lap.get('lost'): return self._x_off()
        an = lap.get('anchor'); c = self.exits.get(an) if an is not None else None; x = lap.get('since_anchor_m')
        if c is None or not _num(x) or x < float(c['start_m']) or x > float(c['end_m']): return self._x_off()
        key = (lap.get('lap'), an)
        if key != self.x_key or not self.x_on: self.x_key = key; self.x_t = None
        self.x_on = True
        dt = None if self.x_t is None else now - self.x_t
        self.x_t = now
        ki = float((getattr(ctrl, 'p', None) or {}).get('ki') or 0.)
        i0 = float(ctrl.i); i1 = i0
        if i0 > 0:
            tau = float(c['tau_s'])
            if tau > 0 and dt is not None and 0 < dt <= 0.5: i1 = i1 * math.exp(-dt / tau)
            cap = c.get('cap_us')
            if _num(cap) and ki > 0: i1 = min(i1, max(0., float(cap) / ki))
            ctrl.i = i1
        return dict(state='ON', anchor=an, x_m=round(float(x), 2), i_us_before=round(ki * i0, 2), i_us=round(ki * i1, 2))

    def _ramp(self, now, pitch, ctrl):
        """i-ramp-r1 climb release (unchanged; was step())."""
        c = self.c
        if ctrl is not None and not _num(getattr(ctrl, 'i', None)) and getattr(ctrl, 'pid', None) is not None: ctrl = ctrl.pid   # PreviewErrorPID / OffscreenPID wrap the PID as .pid
        if not _num(pitch) or ctrl is None or not _num(getattr(ctrl, 'i', None)):
            self.t_last = now; return None
        if pitch < float(c['pitch_off_deg']):
            was = self.armed; self.t_on = None; self.armed = False; self.t_last = now
            return dict(revision=REVISION, state='END') if was else None
        if pitch >= float(c['pitch_on_deg']):
            if self.t_on is None: self.t_on = now
        elif not self.armed:
            self.t_on = None
        if not self.armed and self.t_on is not None and now - self.t_on >= float(c['hold_s']): self.armed = True
        dt = None if self.t_last is None else now - self.t_last
        self.t_last = now
        if not self.armed: return None
        ki = float((getattr(ctrl, 'p', None) or {}).get('ki') or 0.)
        i0 = float(ctrl.i); i1 = i0
        if c.get('sign', 'positive') == 'positive' and i0 <= 0: return dict(revision=REVISION, state='ARMED', i_us=round(ki * i0, 2))
        tau = float(c.get('tau_s') or 0.)
        if tau > 0 and dt is not None and 0 < dt <= 0.5: i1 = i1 * math.exp(-dt / tau)
        cap = c.get('cap_us')
        if _num(cap) and ki > 0:
            lim = float(cap) / ki; i1 = max(-lim, min(lim, i1))
        ctrl.i = i1
        return dict(revision=REVISION, state='ARMED', i_us_before=round(ki * i0, 2), i_us=round(ki * i1, 2))
