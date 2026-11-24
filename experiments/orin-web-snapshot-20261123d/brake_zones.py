"""Odometer brake zones (brake-zones-r1).

User 2026-11-06 (52116a8f): the big right curve 11 comes 25 m after the start and the camera preview sees it only ~0.5 s
ahead; at PWM 35 (100-120 cm/s) the car ran out. The rear wheel odometer (enc_b forward - reverse, 0.0222 cm/count) placed
the same landmarks within +-0.5 m over four laps (first right curve 9.4-9.9 m, big curve 24.9-25.3 m, climb start
46.4-47.2 m, slope end 59.2-59.9 m from the same start; big curve = first right curve + 15.4-15.5 m). After a cut to 15 the
car coasts from 100 to ~50 cm/s in about 2 s / 1.5 m.
This module keeps the distance driven since the run start and three anchors:
  start        the run start (0 m)
  first_curve  the first frame after first_curve_min_m where the speed schedule slows for a right curve
               (reason ERROR_LARGE / CURVE_AHEAD with a positive error) - set once per run
  slope_end    the frame where the pitch comes back above slope_end_level_deg after having been below
               slope_end_pitch_deg (set again on every slope)
Zones in brake_zones.json are (anchor, from_m, to_m, cap_pwm): while the distance from the anchor is inside the zone the
final PWM of a valid frame is capped at cap_pwm (and the schedule's ramp state is lowered to it, so it ramps back up at
up_rate after the zone instead of jumping). Nothing else changes; outside the zones the schedule runs as before.
Experiment values from today's laps, not a safety limit; a wrong start position shifts start-anchored zones only.
brake-zones-r2 (verify-20261114 offline candidate): with lap_zones true in brake_zones.json, a zone that has lap_anchor (a lap_pos
landmark: C1s C1e C2s C2e C3s C3e RAMP TOP DASHs DASHe DOWN LAND8 C4s C4e) / lap_from_m / lap_to_m is measured from the last lap_pos
crossing of that landmark (rear-wheel counts, the same odometer) while lap_pos is enabled, not lost and the crossing is at most
lap_max_since_m (default lap_to_m + lap_margin_m) behind; otherwise the zone uses its anchor / from_m / to_m as before (fallback).
lap_zones absent / false or no lap_anchor zone = brake-zones-r1 exactly (same code path, log and revision).
"""
import json
from pathlib import Path

REVISION = 'brake-zones-r1'
REVISION_LAP = 'brake-zones-r2'   # used only while lap zones are active
CONFIG_FILE = Path(__file__).with_name('brake_zones.json')
DEFAULTS = dict(enabled=False, cm_per_count=0.0222, first_curve_min_m=5., first_curve_err=0., slope_end_pitch_deg=-8., slope_end_level_deg=-5., zones=[])


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


class BrakeZones:
    def __init__(self, config=None):
        self.c = dict(config) if config is not None else load_config()
        self.zero = None; self.d_m = 0.; self.anchors = {'start': 0.}; self.steep = False; self.last_counts = None
        self.uses_lap = bool(self.c.get('lap_zones')) and any(isinstance(z, dict) and z.get('lap_anchor') for z in (self.c.get('zones') or []))   # r2

    def _odometer(self, vehicle):
        b = (vehicle or {}).get('enc_b') or {}
        try: counts = int(b['forward']) - int(b['reverse'])
        except (KeyError, TypeError, ValueError): return None
        if self.zero is None: self.zero = counts
        self.last_counts = counts
        self.d_m = (counts - self.zero) * float(self.c.get('cm_per_count') or 0.0222) / 100.
        return self.d_m

    def _lap_rel(self, lap, z):
        """r2: metres since the last lap_pos crossing of z['lap_anchor'] (None = not usable -> fallback anchor)."""
        try:
            if lap is None or lap is False or getattr(lap, 'lost', True) or getattr(lap, 'enc0', None) is None or self.last_counts is None: return None
            name = z['lap_anchor']; dc = None
            for e in reversed(lap.events):
                if e.get('name') == name and 'd_cross' in e: dc = float(e['d_cross']); break
            if dc is None: return None
            rel = (self.last_counts - lap.enc0) * float(lap.mpc) - dc
            mx = z.get('lap_max_since_m')
            mx = float(mx) if mx is not None else float(z['lap_to_m']) + float(self.c.get('lap_margin_m', 5.))
            return rel if -0.5 <= rel <= mx else None
        except Exception: return None

    def update(self, vehicle, speed_reason=None, image_error=None, far_error=None, pitch_rel=None, lap=None):
        """Call once per valid frame. Returns (cap_pwm or None, info)."""
        c = self.c
        if not c.get('enabled'): return None, None
        d = self._odometer(vehicle)
        if d is None: return None, dict(revision=REVISION, reason='NO_ODOMETER')
        if 'first_curve' not in self.anchors and d >= float(c.get('first_curve_min_m') or 0) and speed_reason in ('ERROR_LARGE', 'CURVE_AHEAD'):
            e = image_error if image_error is not None else far_error
            if e is not None and float(e) > float(c.get('first_curve_err') or 0): self.anchors['first_curve'] = d
        if pitch_rel is not None:
            if pitch_rel < float(c.get('slope_end_pitch_deg', -8.)): self.steep = True
            elif self.steep and pitch_rel > float(c.get('slope_end_level_deg', -5.)): self.steep = False; self.anchors['slope_end'] = d
        cap = None; zone = None
        if not self.uses_lap:
            for z in c.get('zones') or []:
                a = self.anchors.get(z.get('anchor'))
                if a is None: continue
                if c.get('fallback_max_pitch_deg') is not None and pitch_rel is not None and float(pitch_rel) > float(c['fallback_max_pitch_deg']): continue   # arc2-r1
                rel = d - a
                if float(z['from_m']) <= rel <= float(z['to_m']) and (cap is None or float(z['cap_pwm']) < cap): cap = float(z['cap_pwm']); zone = dict(z, rel_m=round(rel, 2))
            return cap, dict(revision=REVISION, d_m=round(d, 2), anchors={k: round(v, 2) for k, v in self.anchors.items()}, zone=zone, cap_pwm=cap)
        src = {}   # r2: per lap zone, 'lap' or 'fallback'
        for z in c.get('zones') or []:
            fr, to, rel, how = z.get('from_m'), z.get('to_m'), None, None
            if z.get('lap_anchor'):
                rel = self._lap_rel(lap, z)
                if rel is not None: fr, to, how = z['lap_from_m'], z['lap_to_m'], 'lap'
                else: how = 'fallback'
                src[z['lap_anchor']] = how if rel is None else '%s %.2f' % (how, rel)
            if rel is None:
                a = self.anchors.get(z.get('anchor'))
                if a is None or fr is None or to is None: continue
                rel = d - a
                fmp = c.get('fallback_max_pitch_deg')   # arc2-r1: an odometer-anchored zone does not cap while the car is pitched up (f70e lap 1: lap-1 fallback cap 21 on the ramp)
                if fmp is not None and pitch_rel is not None and float(pitch_rel) > float(fmp): continue
            if float(fr) <= rel <= float(to) and (cap is None or float(z['cap_pwm']) < cap): cap = float(z['cap_pwm']); zone = dict(z, rel_m=round(rel, 2), src=how or 'anchor')
        return cap, dict(revision=REVISION_LAP, d_m=round(d, 2), anchors={k: round(v, 2) for k, v in self.anchors.items()}, zone=zone, cap_pwm=cap, lap=src)
