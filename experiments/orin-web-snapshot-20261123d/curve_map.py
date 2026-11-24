"""Curve-angle map: a yaw-rate floor for the inner loop while a known curve is not finished (curve-map-r1).

User 2026-11-07 (live 095ccf5e): 'I have the PWM, a map and a fixed start - the car can know it is in a curve'. In the long
curve 11-12 after the downhill the inner loop cycled every ~3 s: hold while the far row was >= 0.3, the car moved inside,
the far row fell below 0.3, -120 pull, the rotation stopped while the curve went on, push +120 again. The inner loop only
knows the lateral error, not the yaw rate the curve needs.

The total angle of each flat right curve is very stable (9 runs, curves merged across gaps < 1.5 s, gz low-passed 0.3 s):
from the start 130-145 / 180-212 / 252-278 deg (curve 1 / 2 / the chain before the climb), after the downhill 138-155 deg.
Distance from the start is less useful (two start positions ~6 m apart tonight), so the map is indexed by the order of the
curves: map_start at the run start, map_after_down after the downhill (route DONE seen).

A curve starts when the low-passed yaw rate passes start_gz (-12) on a flat frame; it ends after end_gz (-5) has held for
gap_s (1.5 s); its turned angle is the gyro integral. While the curve is not finished (turned < expected - margin_deg) the
inner loop's yaw-rate target is not allowed above -floor_dps (the car keeps turning at least ~20 dps). Released early when
the car is well inside (near <= release_near) or the far row turns the other way (far <= release_far), and after max_s.
Experiment values from tonight's logs; closed loop not verified."""
import json, math
from pathlib import Path

REVISION = 'curve-map-r6'
CONFIG_FILE = Path(__file__).with_name('curve_map.json')
DEFAULTS = dict(enabled=False, start_gz=-12., end_gz=-5., gap_s=1.5, lp_tau_s=.3, min_yaw=30., map_start=[140., 195., 268.], map_after_down=[145., 140., 195., 268.],
                margin_deg=25., floor_dps=20., release_near=-.35, release_far=-.1, max_s=14.,
                lock=False, radius_m=1.2, lock_v_default=.55, trim_cap_us=100., lock_k_us_per_dps=6., lock_cap_us=150., lock_gain_dps_per_us=None)   # curve-map-r2: yaw-rate lock instead of the floor


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


class CurveMap:
    def seed(self, plan):
        from ops07_route import seed_curve_map
        return seed_curve_map(self, plan)

    def __init__(self, config=None):
        self.c = dict(DEFAULTS, **config) if config is not None else load_config()
        self.lp = None; self.t = None; self.seg = 'start'; self.idx = 0; self.cur = None; self.quiet = None; self.prev_route = None; self.lap = 0
        self.dist = 0.; self.odo_t = None; self.odo = None; self.odo_missed = 0; self.done_at = None   # r6: odometry distance at the last curve that turned its full mapped angle   # curve-map-r5: odometry distance (m) and clock, armed landmark, missed landmarks

    def lock_dps(self, speed):
        """r2: yaw-rate target (deg/s, negative = right) of a curve of radius_m at this speed; None when the lock is off."""
        if not self.c.get('lock'): return None
        v = float(speed) if speed is not None and math.isfinite(float(speed)) else float(self.c['lock_v_default'])
        return -v / float(self.c['radius_m']) * 180. / math.pi

    def _odo_c(self):
        o = self.c.get('odo'); return o if isinstance(o, dict) and o.get('enabled') else None

    def _arm(self, after, seg, idx):
        """r5: arm the landmark of this event: the next curve (next_idx) starts dist_m further on."""
        for lm in (self._odo_c() or {}).get('landmarks') or []:
            if lm.get('after') == after and lm.get('seg') == seg and (after == 'down' or lm.get('idx') == idx):
                self.odo = dict(at=self.dist + float(lm['dist_m']), next_idx=int(lm['next_idx']), lm=lm); return

    def step(self, now, gz, route, flat, near=None, far=None, speed=None):
        """Returns (floor_dps or None, info). floor_dps is negative (right curve): the yaw-rate target must not be above it."""
        if self._odo_c() is not None and self.c.get('enabled'):   # curve-map-r5: distance on its own clock (NO_GYRO frames still count)
            if self.odo_t is not None and isinstance(speed, (int, float)) and math.isfinite(float(speed)) and float(speed) >= 0.: self.dist += float(speed) * max(0., min(.5, now - self.odo_t))
            self.odo_t = now
        f, info = self._step(now, gz, route, flat, near, far, speed)
        if self._odo_c() is None or not self.c.get('enabled'): return f, info   # curve-map-r5: odo off -> the r3 result unchanged
        if info is None: info = dict(revision=REVISION, reason='IDLE', seg=self.seg, idx=self.idx)
        info.update(dist_m=round(self.dist, 2), odo_missed=self.odo_missed)
        if self.odo is not None: info.update(odo_next_idx=self.odo['next_idx'], odo_to_m=round(self.odo['at'] - self.dist, 2))
        return f, info

    def _step(self, now, gz, route, flat, near, far, speed):
        c = self.c
        if not c.get('enabled'): return None, None
        if route == 'DONE' and self.prev_route != 'DONE':   # r3: every downhill (each lap) restarts the after-downhill map
            self.seg = 'after_down'; self.idx = 0; self.cur = None; self.quiet = None; self.lap += 1
            self.odo = None; self._arm('down', 'after_down', None); self.done_at = None   # curve-map-r5: the downhill start is a landmark
        if route is not None: self.prev_route = route
        if gz is None or not math.isfinite(float(gz)): return None, dict(revision=REVISION, reason='NO_GYRO', seg=self.seg, idx=self.idx)
        dt = 0. if self.t is None else max(0., min(.5, now - self.t)); self.t = now
        o = self._odo_c()   # curve-map-r5: an armed landmark passed by late_m without a curve is missed
        if o is not None and self.odo is not None and self.cur is None and self.dist > self.odo['at'] + float(o.get('late_m', 1.2)): self.odo = None; self.odo_missed += 1
        self.lp = float(gz) if self.lp is None else self.lp + (1. - math.exp(-dt / max(1e-3, float(c['lp_tau_s'])))) * (float(gz) - self.lp)
        mp = c['map_start'] if self.seg == 'start' else c['map_after_down']
        if self.cur is None:
            od = self.odo if o is not None and flat and self.odo is not None and self.idx == self.odo['next_idx'] else None   # curve-map-r5: early start lead_m before the landmark
            gap = c.get('min_gap_after_done_m')   # curve-map-r6 (2026-11-10 fa44/bbdb/9515): right after a fully turned curve the S swing through the crossing
            if (gap is not None and flat and self.lp < float(c['start_gz']) and self.done_at is not None and self.dist - self.done_at < float(gap)):   # starts the next curve 0.2-3.8 m later; real next curves start >= 4.8 m later
                return None, dict(revision=REVISION, reason='GAP_HOLD', seg=self.seg, idx=self.idx, after_done_m=round(self.dist - self.done_at, 2))
            if flat and self.lp < float(c['start_gz']):
                self.cur = dict(t0=now, yaw=0., expected=float(mp[self.idx]) if self.idx < len(mp) else None, released=None); self.quiet = None
            elif od is not None and self.dist >= od['at'] - float(o.get('lead_m', .4)):
                self.cur = dict(t0=now, yaw=0., expected=float(mp[self.idx]) if self.idx < len(mp) else None, released=None, early=True, at=od['at']); self.quiet = None; self.odo = None
            else:
                return None, None
        cur = self.cur; cur['yaw'] += -float(gz) * dt
        if cur.get('early'):   # curve-map-r5: an early curve has no quiet end before the yaw starts; cancelled if not turned 15 deg by late_m
            if self.lp < float(c['start_gz']): cur['seen'] = True
            if cur['yaw'] < 15. and self.dist >= cur['at'] + float((o or {}).get('late_m', 1.2)):
                self.cur = None; self.quiet = None
                return None, dict(revision=REVISION, reason='EARLY_CANCEL', seg=self.seg, idx=self.idx, turned_deg=round(cur['yaw'], 1), odo_m=round(self.dist - cur['at'], 2))
        if self.lp < float(c['end_gz']) or (cur.get('early') and not cur.get('seen') and flat): self.quiet = None
        else:
            if self.quiet is None: self.quiet = now
            if now - self.quiet >= float(c['gap_s']) or not flat:
                info = dict(revision=REVISION, reason='CURVE_END', seg=self.seg, idx=self.idx, turned_deg=round(cur['yaw'], 1), expected_deg=cur['expected'])
                if cur['yaw'] > float(c['min_yaw']): self.idx += 1
                if cur['expected'] is not None and cur['yaw'] >= cur['expected'] - float(c['margin_deg']): self.done_at = self.dist   # r6
                self.cur = None; self.quiet = None
                if o is not None:   # curve-map-r5: a counted curve end arms its landmark (info idx = the curve that just ended)
                    if self.odo is not None and self.idx > self.odo['next_idx']: self.odo = None
                    if cur['yaw'] > float(c['min_yaw']): self._arm('curve_end', self.seg, info['idx'])
                return None, info
        info = dict(revision=REVISION, seg=self.seg, idx=self.idx, turned_deg=round(cur['yaw'], 1), expected_deg=cur['expected'], since_s=round(now - cur['t0'], 2))
        if cur.get('early'): info.update(early=True, odo_m=round(self.dist - cur['at'], 2))   # curve-map-r5
        if cur['expected'] is None: info.update(reason='NO_MAP'); return None, info
        if cur['released'] is None:
            if cur['yaw'] >= cur['expected'] - float(c['margin_deg']): cur['released'] = 'ANGLE_DONE'
            elif near is not None and math.isfinite(float(near)) and float(near) <= float(c['release_near']): cur['released'] = 'INSIDE'
            elif far is not None and math.isfinite(float(far)) and float(far) <= float(c['release_far']): cur['released'] = 'FAR_OTHER_WAY'
            elif now - cur['t0'] > float(c['max_s']): cur['released'] = 'MAX_TIME'
        if cur['released'] is not None: info.update(reason='RELEASED', released=cur['released']); return None, info
        info.update(reason='FLOOR', floor_dps=-float(c['floor_dps'])); return -float(c['floor_dps']), info
