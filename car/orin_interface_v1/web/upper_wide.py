"""upper-wide-r1 (upper-wide-20261115, offline candidate, NOT deployed): PWM side of the 'wide' dashed left turn on the upper section.
The road after the dashed turn is two lanes wide (dashed-line centre -> right solid line inner edge ~0.55-0.60 m where the car crosses,
road_geom.py), so the turn may end up to ~0.2-0.3 m past the dashed line and be taken faster. Called from speed_policy.py only; every
number is a key of upper_wide.json, read once per run (first call of the run). File missing / enabled false / unreadable -> get() returns
None and speed_policy behaves exactly as before (no extra clock / IMU / MCU-status read in either case: all inputs come from the
proposal and from objects live_pid already updated this frame).

 turn_pwm  (a)  PWM for the upper-section 'turning' frames (dashed TURN and the post-TURN frames with |heading-hold error| > 20 deg):
                raises speed_adapt's result (upper_pwm 18) and lifts the lane-lock / TURN caps to this value (the car holds its speed
                through the turn instead of dropping to ~0.45 m/s). Writes speed_adapt.cur (explicit hidden write, like brake zones) so the
                post-turn ramp starts from it.
 post_pwm  (d)  PWM target after the turn (speed_policy.post_turn frames) while upper_single is not finishing its rotation (stall fix
                active), |FOLLOW error| <= post_err_max (HOLD frames: no error, allowed) and |h_target - yaw| <= post_head_max_deg;
                ramp post_up_rate PWM/s from the frame PWM; lifts the post-turn cap to post_pwm.
 coast     (d)  speed-aware cap so that the car reaches the downhill (DOWN) at v_down: y = sum(d_odo * sin(yaw - pre)) from the first dashed
                TURN frame (along the dashed line; pre = heading_hold ref0 - 75; odometer = brake_zones.last_counts, 2.22e-4 m/count);
                y_rem = y_down_m - y; v_allow = sqrt(v_down^2 + 2 a max(0, y_rem - margin_m)); cap = pwm (coast) while v > v_allow, else
                the PWM whose level-ground speed is v_allow (vss table, upper-section fit). Speed = live_pid._cm_speed (last frame's wheel
                speed). Missing input (no TURN yet, no odometer, no speed, no heading reference) -> no cap; the crest brake zone of
                brake_zones.json stays as the fallback. Only on route OPEN frames with y >= start_y_m.
Sources: y_down 3.56 m median (sd 0.14, 31 passes; along.py) -> 3.45 default (earlier coasting); coasting -0.25 m/s^2 median on the
upper straight (upper-20261115 longit.py) -> 0.22 default; v before DOWN 0.55-0.62 m/s (brief) -> 0.60. Experiment values, not limits."""
import json, math
from pathlib import Path

REVISION = 'upper-wide-r1'
CONFIG_FILE = Path(__file__).with_name('upper_wide.json')
VSS = ((10., .05), (13., .30), (18., .50), (21., .64), (24., .75), (27., .85), (30., .93), (35., 1.03), (40., 1.11), (50., 1.22))
COAST_DEFAULTS = dict(y_down_m=3.45, v_down=.60, a=.22, margin_m=.30, pwm=10., m_per_count=2.22e-4, turn_deg=75., start_y_m=.8)


def load_config(path=None):
    try:
        c = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
        return c if isinstance(c, dict) else {}
    except Exception: return {}


def _f(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError): return None


def pwm_for(v):
    """level-ground PWM whose steady speed is v (inverse of the upper-section vss fit)."""
    if v <= VSS[0][1]: return VSS[0][0]
    for (a, va), (b, vb) in zip(VSS, VSS[1:]):
        if v <= vb: return a + (b - a) * (v - va) / (vb - va)
    return VSS[-1][0]


class UpperWide:
    def __init__(self, config, run_id=None):
        c = dict(config or {}); self.run_id = run_id; self.error = None
        try:
            self.turn_pwm = _f(c.get('turn_pwm')) if c.get('turn_pwm') is not None else None
            self.post_pwm = _f(c.get('post_pwm')) if c.get('post_pwm') is not None else None
            self.post_up_rate = float(c.get('post_up_rate', 30.)); self.post_err_max = float(c.get('post_err_max', .6)); self.post_head_max = float(c.get('post_head_max_deg', 12.))
            co = c.get('coast')
            self.coast = None
            if isinstance(co, dict) and co.get('enabled', True):
                self.coast = dict(COAST_DEFAULTS); self.coast.update({k: float(v) for k, v in co.items() if k in COAST_DEFAULTS})
            if (c.get('turn_pwm') is not None and self.turn_pwm is None) or (c.get('post_pwm') is not None and self.post_pwm is None): raise ValueError('pwm')
            self.need_aligned = bool(c.get('post_need_aligned', False))   # upper-edges-r1: post ramp only while upper_single's edge estimate says 'aligned'
            self.on = bool(c.get('enabled', False)) and (self.turn_pwm is not None or self.post_pwm is not None or self.coast is not None)
        except Exception as e:
            self.on = False; self.turn_pwm = self.post_pwm = self.coast = None; self.error = '%s:%s' % (type(e).__name__, e)
        self.y = None; self.c_last = None; self.post_cur = None; self.t_post = None

    def reset_along(self): self.y = None; self.c_last = None

    def along(self, route_open, turn_frame, yaw, ref0, counts):
        """y (m along the dashed line since the first TURN frame) or None. Call once per valid frame (any order of inputs None-safe)."""
        if not route_open: self.reset_along(); return None
        if counts is None: return self.y
        if self.y is None:
            if turn_frame: self.y = 0.; self.c_last = counts
            return self.y
        if yaw is not None and ref0 is not None:
            self.y += (counts - self.c_last) * self.coast['m_per_count'] * math.sin(math.radians(yaw - (ref0 - self.coast['turn_deg'])))
        self.c_last = counts
        return self.y

    def coast_cap(self, v):
        """(cap or None, info) for the current y and speed v (m/s)."""
        k = self.coast
        if k is None or self.y is None or self.y < k['start_y_m'] or v is None: return None, None
        y_rem = k['y_down_m'] - self.y
        va = math.sqrt(k['v_down'] ** 2 + 2. * k['a'] * max(0., y_rem - k['margin_m']))
        cap = k['pwm'] if v > va else max(k['pwm'], pwm_for(va))
        return round(cap, 2), dict(y=round(self.y, 3), y_rem=round(y_rem, 3), v=round(v, 3), v_allow=round(va, 3), cap=round(cap, 2))

    def post_target(self, now, pwm):
        """ramped post-turn PWM target (>= pwm when below post_pwm)."""
        base = float(pwm) if self.post_cur is None or self.t_post is None or now - self.t_post > .5 else max(float(pwm), self.post_cur)
        dt = 0. if self.t_post is None else max(0., min(.5, now - self.t_post))
        tgt = min(self.post_pwm, base + self.post_up_rate * dt) if base < self.post_pwm else base
        self.post_cur = tgt; self.t_post = now
        return tgt


def get(lp):
    """this run's UpperWide, or None when off / missing. Built at the first call of each run (config read once per run)."""
    try:
        rid = (getattr(lp, 'run', None) or {}).get('id')
        w = getattr(lp, '_upper_wide', None)
        if w is None or w.run_id != rid:
            w = UpperWide(load_config(), rid); lp._upper_wide = w
        return w if w.on else None
    except Exception: return None
