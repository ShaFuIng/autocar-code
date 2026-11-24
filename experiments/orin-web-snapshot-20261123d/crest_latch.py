"""crest-latch-r1 (crest-latch-20261116, offline candidate): keep the crest two-line midpoint until the crest is passed.

Problem (run 85dc-20261116 lap 6, t 2764.36-2765.25): crest_fallback (two-line midpoint, crest_fallback.py) took one invalid frame,
then the normal path was valid again for 4 frames and live_pid used it; it was a single-line estimate whose side flipped every frame
(left+width / left+width / right+width / left+width, image error +0.50 / +0.49 / -0.56 / +0.53, servo 1806..1863, gz -39..-55) and
the car left the deck after the crest. On 11-16 about 20 of 49 crest passes went back to the normal path for single frames after a
crest take (small errors there, no harm).

Rule (only when web/crest_latch.json exists with enabled true; live_pid builds no object otherwise):
  start   on a frame where crest_fallback was used (info.used) while dash_route pitch_rel >= pitch_deg and (need_climbed) the
          dash_route window is CLIMBED (= pitched >= 8 deg for 2 s: the last ~1 s before the crest; the ramp foot after curve 3,
          where the normal path does real corrections, is excluded).
  latched every frame whose crest mode would be None (normal valid path) and not dash_route is sent to crest_fallback.try_path
          with mode 'latch' (the midpoint, no MAX_S: the latch has its own max_s).
          A latched 'latch' frame (normal valid path in) where the midpoint is not available (NO_TWO_LINES, ...) is made INVALID
          (no fallback to the single-line estimate): live_pid's existing lost-line rules then act (PID_HOLD = last valid command,
          i.e. the previous latched frame's command, for <= 0.4 s; then LOST_HEADING / LOST_TURN). Corridor-mode frames keep the
          existing behaviour (corridor path when the midpoint fails): replay showed that invalidating them mostly replaced small
          errors by a PID_HOLD frame (and its PWM restart) without a benefit.
  release pitch_rel < pitch_deg, pitch unknown / IMU stale, latched > max_s, route OPEN / DONE, or dash_route selected.
          After a release no new latch until the pitch has been below pitch_deg once (next climb).
Pitch = dash_route.update_route's pitch_rel_deg of this frame (the same boost._read() - flat_pitch_deg that crest_fallback uses):
no extra IMU read. Python 3 standard library only; never raises into live_pid (live_pid wraps the calls)."""
import json
from pathlib import Path

REVISION = 'crest-latch-r1'
CONFIG_FILE = Path(__file__).with_name('crest_latch.json')
DEFAULTS = dict(pitch_deg=8.0, max_s=1.5, need_climbed=True)   # max_s 1.5: 11-16 replay, latches on normal climbs end by PITCH_LOW after 0.64-1.0 s; only the two long climbs (1e69 L1 2.16 s, f70e L1 MAX_S) held longer, at PWM 25 (crest frames count as special for the speed policy)


def load_config(path=None):
    """dict of the effective parameters, or None = off (file missing / unreadable / not a dict / enabled not true / bad numbers)."""
    try:
        c = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
        if not isinstance(c, dict) or c.get('enabled') is not True: return None
        out = dict(DEFAULTS)
        for k in ('pitch_deg', 'max_s'):
            if c.get(k) is not None: out[k] = float(c[k])
        if 'need_climbed' in c: out['need_climbed'] = bool(c['need_climbed'])
        if not (out['max_s'] > 0.): return None
        return out
    except Exception:
        return None


class CrestLatch:
    def __init__(self, c):
        self.c = dict(c); self.t0 = None; self.done = False; self.last = None

    @property
    def latched(self):
        return self.t0 is not None

    def _pitch(self, dash_info):
        di = dash_info or {}
        if di.get('reason') in ('IMU_STALE', 'IMU_UNAVAILABLE'): return None
        p = di.get('pitch_rel_deg')
        return float(p) if isinstance(p, (int, float)) and not isinstance(p, bool) else None

    def mode(self, now, dash_info, selected_source, crest_mode):
        """before crest_fallback: releases the latch when due; returns the crest mode to use ('latch' replaces None while latched)."""
        p = self._pitch(dash_info); route = (dash_info or {}).get('route'); self.last = None
        if p is not None and p < self.c['pitch_deg']: self.done = False
        if self.t0 is not None:
            why = ('PITCH_UNKNOWN' if p is None else 'PITCH_LOW' if p < self.c['pitch_deg'] else 'MAX_S' if now - self.t0 > self.c['max_s']
                   else 'ROUTE_' + str(route) if route in ('OPEN', 'DONE') else 'DASH_ROUTE' if selected_source == 'dash_route' else None)
            if why is not None:
                self.last = dict(revision=REVISION, state='RELEASE', why=why, held_s=round(now - self.t0, 3)); self.t0 = None; self.done = True
                return crest_mode
            if crest_mode is None and selected_source != 'dash_route': return 'latch'
        return crest_mode

    def after(self, now, dash_info, crest_mode, info):
        """after crest_fallback.try_path (info = its info dict). Returns 'INVALID' when this latched frame must be made invalid, else None.
        Starts the latch on a used crest frame (see module doc)."""
        info = info or {}; used = bool(info.get('used'))
        if self.t0 is not None:
            held = round(now - self.t0, 3)
            if crest_mode == 'latch' and not used:
                self.last = dict(revision=REVISION, state='LATCHED', action='INVALID', mode=crest_mode, reason=info.get('reason'), held_s=held); return 'INVALID'
            self.last = dict(revision=REVISION, state='LATCHED', action='MIDPOINT' if used else 'NONE', mode=crest_mode, held_s=held); return None
        if used and not self.done:
            p = self._pitch(dash_info); route = (dash_info or {}).get('route')
            if p is not None and p >= self.c['pitch_deg'] and (route == 'CLIMBED' or not self.c['need_climbed']):
                self.t0 = now; self.last = dict(revision=REVISION, state='START', mode=crest_mode, pitch=round(p, 2))
        return None
