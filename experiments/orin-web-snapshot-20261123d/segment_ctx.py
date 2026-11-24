"""segment-ctx-r1 (integration B2, 2026-11-12): per-frame segment context + ordered layer table for live_pid.py.

Pure restructuring: no command changes (proven by the differential replay in integration-20261112/b23). Python 3 standard
library only; no config file (nothing here is tunable).

- Frame: the values the valid-frame layers share (formerly locals of LivePID._loop). ONE object per _loop call, reused across
  frames exactly like the locals were: a value a frame does not assign keeps the previous frame's value (live_pid read `ssg`
  through locals().get(); that stale-value behaviour is kept on purpose).
- Ctx: built once per running frame in _loop, right after the downhill windows, before the lost-path branch. The window values
  (dpr / down_tail / downhill / down_level / down_steep) are the ones _loop computed. Route, pitch, selected source and junction
  state are read from this frame's self.latest when asked (self.latest is not reassigned later in the frame and its dash_route /
  selected_source / path_diagnostic are not mutated), so a ctx field is the same value the replaced inline expression read.
  Predicates that call float() are methods evaluated at the call site, with the same short-circuit order, so they raise exactly
  where the inline expression raised (the callers' try blocks see the same exceptions).
- Layer / Group / run(): the ordered table. A Layer is (name, eligibility(obj, F) or None, fn(obj, F)). A Group runs its layers in
  one block: on_error None = no try (an exception propagates as before), else one try around the whole group (as the old
  `try: ... except Exception as e: proposal[...] = ERROR` around several steps). F.owner = name of the last layer that changed
  proposal['steer_us'] (log only).
"""
REVISION = 'segment-ctx-r1'


class Frame(object):
    """Shared values of the valid-frame layers (see module doc)."""
    pass


class Ctx(object):
    """Segment context of one running frame (see module doc)."""

    def __init__(self, latest, dpr, down_tail, downhill, down_level, down_steep):
        self.latest = latest; self.dpr = dpr; self.down_tail = down_tail; self.downhill = downhill
        self.down_level = down_level; self.down_steep = down_steep

    # ---- this frame's dash-route / selection / junction fields (read from self.latest when asked)
    @property
    def route(self):
        return (self.latest.get('dash_route') or {}).get('route')

    @property
    def pitch(self):   # dash_route.pitch_rel_deg (raw; NOT the flat-dip-guard value dpr, NOT imu_boost's pitch)
        return (self.latest.get('dash_route') or {}).get('pitch_rel_deg')

    @property
    def sel(self):
        return self.latest.get('selected_source')

    def jstate(self):   # junction switch state; may raise like the inline expression
        return ((self.latest.get('path_diagnostic') or {}).get('junction') or {}).get('state')

    # ---- pitch predicates (float(): evaluated at the call site)
    def level(self):   # |pitch| <= 3
        pt = self.pitch
        return pt is not None and abs(float(pt)) <= 3.

    def flat_wc(self):   # flat lap: route WAIT_CLIMB and |pitch| <= 3 (was written out 7 times)
        return self.route == 'WAIT_CLIMB' and self.level()

    def steep10(self):   # pitch <= -10 (yaw_inner's steep downhill)
        pt = self.pitch
        return pt is not None and float(pt) <= -10.

    # ---- proposal predicates (evaluated when called: the proposal changes during the frame)
    @staticmethod
    def proposal_only(p):
        return p.get('reason') == 'PROPOSAL_ONLY'

    @staticmethod
    def hh_active(p):
        return bool((p.get('heading_hold') or {}).get('active'))

    def normal_rest(self, p):   # no lane segment, no active heading hold, no dashed TURN, normal path source
        return 'lane_segment' not in p and not self.hh_active(p) and 'dash_turn' not in p and self.sel is None

    @staticmethod
    def xp_window(p):   # inside the crossing window this frame (active or released); not the fade, not an ERROR dict
        xp = p.get('crossing_pass')
        return isinstance(xp, dict) and 'dist_after_m' in xp

    @staticmethod
    def xp_any(p):   # any crossing_pass dict this frame: window, fade or ERROR (curve_ff's exclusion)
        return isinstance(p.get('crossing_pass'), dict)

    # ---- log label
    CURVES = {('start', 0): 'curve1', ('start', 1): 'curve2', ('start', 2): 'curve3', ('after_down', 0): 'curve4',
              ('after_down', 1): 'curve1', ('after_down', 2): 'curve2', ('after_down', 3): 'curve3'}

    def segment(self, p, xp_open=False):
        """compact segment name for the log (diagnostic only, nothing reads it): start / dash_turn / upper_pre / upper_post / ramp /
        downhill / bottom / after_down / crossing / curve1..4 / curve / climb / straight / other."""
        if (p.get('start_centre') or {}).get('done') is False: return 'start'
        r = self.route
        if r == 'OPEN':
            hh = p.get('heading_hold') or {}
            if 'dash_turn' in p or hh.get('reason') == 'TURN': return 'dash_turn'
            if hh.get('reason') in (None, 'NO_REFERENCE'): return 'upper_pre'
            return 'upper_post'
        if r == 'CLIMBED': return 'ramp'
        if self.downhill: return 'downhill'
        if self.down_tail: return 'bottom'
        if r == 'DONE': return 'after_down'
        if r == 'WAIT_CLIMB':
            if self.xp_window(p) or (p.get('valid') is False and xp_open): return 'crossing'
            cm = p.get('curve_map') or {}
            if cm.get('reason') in ('FLOOR', 'RELEASED'): return self.CURVES.get((cm.get('seg'), cm.get('idx')), 'curve')
            try:
                if self.pitch is not None and float(self.pitch) > 3.: return 'climb'
            except Exception: pass
            return 'straight'
        return 'other'


class Layer(object):
    __slots__ = ('name', 'elig', 'fn')

    def __init__(self, name, elig, fn):
        self.name = name; self.elig = elig; self.fn = fn


class Group(object):
    __slots__ = ('name', 'elig', 'layers', 'on_error')

    def __init__(self, name, elig, layers, on_error=None):
        self.name = name; self.elig = elig; self.layers = tuple(layers); self.on_error = on_error


def _steer(p):
    try: return p.get('steer_us')
    except Exception: return None


def changed(a, b):
    """True when b is a different steering value than a (log only; never raises)."""
    try: return not (a is b or a == b)
    except Exception: return False


def owner_after(cur, name, before, p):
    """owner bookkeeping for inline call sites: name if p['steer_us'] differs from before, else cur (never raises)."""
    return name if changed(before, _steer(p)) else cur


def run(table, obj, F):
    """Run an ordered layer table on frame F (F.p = proposal). Eligibility is evaluated right before each entry, so it sees what the
    entries before it wrote (as the inline `if` did)."""
    for it in table:
        if it.elig is not None and not it.elig(obj, F): continue
        if isinstance(it, Group):
            if it.on_error is None: run(it.layers, obj, F)
            else:
                try: run(it.layers, obj, F)
                except Exception as e: it.on_error(obj, F, e)
            continue
        b = _steer(F.p)
        try: it.fn(obj, F)
        finally:
            if changed(b, _steer(F.p)): F.owner = it.name


def names(table, prefix=''):
    """flat list of the table's entry names in run order ('group/layer'), for NOTES / tests."""
    out = []
    for it in table:
        if isinstance(it, Group): out += names(it.layers, prefix + it.name + '/')
        else: out.append(prefix + it.name)
    return out
