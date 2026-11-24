"""07 semantic entry plans. No device I/O; coordinates are estimates, not survey truth."""
import copy
import math

ENTRIES = {
    'flag': (0, 17.65), '12_to_01': (0, 17.65),
    '02_to_03_before': (1, None), '02_to_03_after': (1, 32.),
    '04_to_05_before': (2, None), '04_to_05_after': (2, 45.),
    '05_to_06': (2, 48.), 'ramp': (3, 58.4), 'crest': (3, 60.4),
}
PACKAGES = ('slow-steer-7fbf', 'best-today-46db', 'latest-2lap-97ed')


def entry_plan(package, location, phase='first', *, x_m=None, route_lap=0):
    if package not in PACKAGES or location not in ENTRIES or phase not in ('first', 'after_down'):
        raise ValueError('請選擇已提供的方案、起跑位置與下坡階段。')
    if type(route_lap) is not int or route_lap < 0:
        raise ValueError('路線圈次格式不符。')
    idx, s = ENTRIES[location]
    plan = dict(package=package, location=location, phase=phase,
                seg='start' if phase == 'first' else 'after_down',
                idx=idx + (phase == 'after_down'), route_lap=route_lap,
                dash_state='OPEN' if location == 'crest' else 'WAIT_CLIMB',
                s_m=s, source='estimated_entry', crossing={}, swing_end=None, landmark=None)
    if location.endswith('_before'):
        # x is measured from the preceding curve END, not from the camera.
        if type(x_m) not in (float, int) or not math.isfinite(x_m) or x_m < 0:
            raise ValueError('路口前起跑需要現場定點及距前一彎結束的實量 x；不能拿假里程代替。')
        first = location.startswith('02')
        window = (3., 4.6) if first else (.4, 2.2)
        if x_m > window[1]:
            raise ValueError('此定點已超過路口窗口，請改用路口後入口。')
        plan['s_m'] = (23.54 if first else 39.61) + x_m
        if package == 'latest-2lap-97ed' or (not first and package == 'best-today-46db'):
            plan['crossing'][1 if first else 0] = -float(x_m)
        if first:plan['landmark'] = dict(after='curve_end', idx=plan['idx']-1, x_m=float(x_m))
        elif package == 'latest-2lap-97ed':plan['swing_end'] = -float(x_m)
    return plan


def seed_curve_map(cm, plan):
    cm.__init__(copy.deepcopy(cm.c))
    cm.seg, cm.idx, cm.lap = plan['seg'], plan['idx'], plan['route_lap']
    cm.prev_route = plan['dash_state']
    lm = plan['landmark']
    if lm:
        cm._arm(lm['after'], cm.seg, lm['idx'])
        if cm.odo is None or cm.odo['next_idx'] != cm.idx:
            raise ValueError('方案缺少這個入口的里程地標。')
        cm.odo['at'] -= lm['x_m']  # Keep the original lm payload used by _step.


def seed_dash(dash, plan):
    dash.__init__(dash.boost, crest_start=plan['dash_state']=='OPEN',
                  left_lane=dash.left_lane, lane=dash.lane)
    dash.lap = plan['route_lap']


def seed_track(tp, plan):
    tp.reset()
    tp.d0, tp.s0 = 0., float(plan['s_m'])
    tp.route = plan['dash_state']
    tp.anchor = dict(name='USER_ENTRY_ESTIMATE', s=tp.s0, source=plan['source'])


class LapClock:
    """Flag crossings require DONE -> curve4 anchor; odometry jumps don't count."""
    def __init__(self, location='flag'):
        self.active_s=0.; self.wall_s=0.; self.lap_active=0.; self.lap_wall=0.
        self.laps=[]; self.partial=location!='flag'; self.last_s=None
        self.done=False; self.curve4=False; self.last_anchor=None

    def tick(self, dt, running):
        self.wall_s += dt; self.lap_wall += dt
        if running:self.active_s += dt; self.lap_active += dt

    def relocate(self):
        self.partial=True; self.last_s=None; self.done=False; self.curve4=False

    def observe(self, position):
        s=position.get('s_m'); a=position.get('anchor') or {}; name=a.get('name')
        if type(s) not in (int,float) or not math.isfinite(s):return None
        anchor=(name,a.get('t'))
        if anchor != self.last_anchor:
            if name=='DOWNHILL_START':self.done=True;self.curve4=False
            if name=='CURVE_START_curve4' and self.done:self.curve4=True
            self.last_anchor=anchor
        old=self.last_s;self.last_s=s
        if old is None:return None
        delta=(s-old)%66.8
        # Experimental discontinuity filter; not a safety/control gate.
        if delta>2.5:
            self.partial=True;return None
        if self.done and self.curve4 and old<17.65<=s:
            result=dict(active_s=self.lap_active,wall_s=self.lap_wall,
                        valid=not self.partial,source='estimated_flag_crossing')
            self.laps.append(result);self.lap_active=self.lap_wall=0.
            self.partial=False;self.done=self.curve4=False
            return result


class FrontCarGate:
    """Distance-provider contract. 3 m is the user's trial setting, not calibration."""
    def __init__(self, clear_m=3.):self.clear_m=clear_m
    def can_continue(self, reading, now, max_age_s):
        # All timestamps must come from the same Orin monotonic clock.
        if not isinstance(reading,dict):return False
        t=reading.get('captured_at');distance=reading.get('distance_m')
        if type(t) not in (int,float) or not math.isfinite(t) or not 0<=now-t<=max_age_s:return False
        if reading.get('clear') is True:return True  # explicit detector observation
        return type(distance) in (int,float) and math.isfinite(distance) and distance>=self.clear_m
