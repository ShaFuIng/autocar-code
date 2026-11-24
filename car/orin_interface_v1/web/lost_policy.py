"""lost-policy-r1 (integration B3c, 2026-11-12): the lost-line policy of live_pid.py as one ordered table.

Pure restructuring: same decisions, same order, same side effects as the inline branch it replaces (proven by the differential replay).

Executed here (a frame of a running PID whose path is invalid or stale; LivePID._loop asks decide() and then finish()):
  groups in order, first row that proposes a command wins; a group's precondition is tested first; a `catch` group swallows an
  exception of its prelude / rows and the decision goes on with the next group (as the old try around the three blind holds);
  without `catch` an exception propagates (as before: PID_MODEL_FAILURE).
    blind     (fresh frame, heading + hold objects)  UPPER_SINGLE_HOLD -> LANE_BLIND_HOLD -> HEADING_BLIND      [catch]
    pid_hold  (fresh frame)                          PID_HOLD (last valid command, <= HOLD_S 0.4 s)
    downhill  (fresh, down_tail, last valid <= downhill_hold_max_s)  DOWNHILL_HOLD
    lost      (fresh frame, heading + hold)          LOST_HEADING / LOST_TURN (|last - centre| >= lost_turn_us, not in the crossing window)
  nothing proposed -> stop the run (PID_INVALID_OR_STALE_PATH).
Each row then builds its logged proposal and runs its modifiers in order (hold_pwm, turn_pwm, down_straight GYRO, crossing_hold
gyro, bottom_map MAP_TURN, downhill clamps / gz limit); the names are in lost_policy.json "mods".

lost_policy.json (read once at import; missing / malformed -> DEFAULTS = today's table): "groups" (order of groups and rows), "mods"
(modifier order per row), and the three thresholds that were literals in live_pid: lost_turn_us 200, downhill_hold_max_s 3.0,
downhill_hold_left_us 150. Changing the order or the mods changes behaviour (that is what the table is for); only today's values are
proven equal to the old code.

CATALOGUE lists all 19 lost-line / bad-path behaviours of the stack (audit CONFLICTS 6) with where they are decided; the rows of
stage 'lost' are the ones executed by decide(); the others stay where they are (path stage / valid-frame pipeline) and are listed so
the whole policy is in one place. Python 3 standard library only."""
import json
from pathlib import Path

REVISION = 'lost-policy-r1'
CONFIG_FILE = Path(__file__).with_name('lost_policy.json')
DEFAULTS = dict(
    groups=[dict(name='blind', rows=['UPPER_SINGLE_HOLD', 'LANE_BLIND_HOLD', 'HEADING_BLIND']),
            dict(name='pid_hold', rows=['PID_HOLD']),
            dict(name='downhill', rows=['DOWNHILL_HOLD']),
            dict(name='lost', rows=['LOST_HEADING'])],
    mods=dict(UPPER_SINGLE_HOLD=['hold_pwm', 'turn_pwm', 'down_straight', 'crossing_hold'],
              LANE_BLIND_HOLD=['hold_pwm', 'turn_pwm', 'down_straight', 'crossing_hold'],
              HEADING_BLIND=['hold_pwm', 'turn_pwm', 'down_straight', 'crossing_hold'],
              PID_HOLD=['hold_pwm', 'downhill_clamp', 'turn_pwm', 'as_pid_hold', 'downhill_gz_if_downhill', 'down_straight', 'crossing_hold'],
              DOWNHILL_HOLD=['bottom_map', 'bottom_curve', 'downhill_gz_hold', 'down_straight'],
              LOST_HEADING=['hold_pwm', 'turn_pwm', 'crossing_hold']),
    lost_turn_us=200., downhill_hold_max_s=3., downhill_hold_left_us=150.)

# all lost-line / bad-path behaviours (segment, how long lost, last state -> behaviour); stage 'lost' rows are executed by decide()
CATALOGUE = (
    dict(name='UPPER_SINGLE_HOLD', stage='lost', segment='upper section after the dashed turn (upper_single armed)', lost='upper_single max', last='-', behaviour='gyro hold (upper_single HOLD)'),
    dict(name='LANE_BLIND_HOLD', stage='lost', segment='slope entry after a lane-segment command', lost='<= BLIND_HOLD_S', last='recent lane commands', behaviour='hold the lane mean'),
    dict(name='HEADING_BLIND', stage='lost', segment='route OPEN, not turning, heading reference set', lost='<= BLIND_MAX_S 6 s', last='-', behaviour='gyro hold (heading_hold)'),
    dict(name='PID_HOLD', stage='lost', segment='any', lost='<= HOLD_S 0.4 s', last='last valid command', behaviour='hold last (downhill: clamp c0 +-150, gz limit)'),
    dict(name='DOWNHILL_HOLD', stage='lost', segment='down_tail (4 s after pitch < -5)', lost='0.4 .. 3 s', last='median of 0.5 s sent', behaviour='hold last (clamped), bottom: map turn'),
    dict(name='LOST_HEADING', stage='lost', segment='any', lost='0.4 s .. LOST_MAX_S', last='yaw of the last valid frame', behaviour='gyro hold (heading_hold.lost)'),
    dict(name='LOST_TURN', stage='lost', segment='outside the crossing window', lost='0.4 s .. LOST_MAX_S', last='|last - c0| >= 200', behaviour='hold last command'),
    dict(name='STOP', stage='lost', segment='any', lost='nothing above applies', last='-', behaviour='stop (PID_INVALID_OR_STALE_PATH)'),
    dict(name='down_straight GYRO', stage='lost (modifier)', segment='pitch <= steep_deg', lost='any hold', last='-', behaviour='gyro hold replaces the held value'),
    dict(name='crossing_hold', stage='lost (modifier)', segment='crossing window (last valid frame inside)', lost='<= 1.5 s', last='-', behaviour='gyro hold replaces the held value'),
    dict(name='bottom_map MAP_TURN', stage='lost (modifier)', segment='level bottom, curve 4 started', lost='DOWNHILL_HOLD frames', last='-', behaviour='turn by gyro rate target'),
    dict(name='WARMUP held', stage='valid', segment='any', lost='controller warm-up after a hold', last='recent sent command', behaviour='hold last'),
    dict(name='dash_route HOLD_LAST', stage='path', segment='upper section, dashes lost', lost='<= 0.5 s', last='last dash fit', behaviour='keep the last dash line'),
    dict(name='crest_fallback', stage='path', segment='pitched up, path invalid', lost='-', last='-', behaviour='crest midpoint path'),
    dict(name='crossing jump hold', stage='valid', segment='crossing window', lost='>= jump_gap_s gap / one-frame step', last='last normal error', behaviour='no release (reduced camera stays)'),
    dict(name='jump_filter', stage='valid', segment='upper section after the turn', lost='<= 5 frames', last='controller snapshot', behaviour='error flip treated as bogus, hold last sent'),
    dict(name='error-jump-guard', stage='valid', segment='any (right commands)', lost='0.3 / 1 s', last='last sent > c0', behaviour='hold last sent'),
    dict(name='downhill_bottom', stage='valid', segment='level bottom', lost='<= 1.5 s', last='median 0.5 s sent', behaviour='hold median'),
    dict(name='bottom_left INVALID / downhill_nocorridor INVALID', stage='path', segment='bottom / downhill', lost='-', last='-', behaviour='force a lost frame'),
)


MOD_NAMES = ('hold_pwm', 'turn_pwm', 'down_straight', 'crossing_hold', 'bottom_map', 'bottom_curve', 'downhill_gz_hold', 'downhill_clamp',
             'as_pid_hold', 'downhill_gz_if_downhill')


def _valid(c):
    """same groups and rows as DEFAULTS (any order), known modifier names, float thresholds; else ValueError."""
    g = c.get('groups'); m = c.get('mods')
    names = [r for x in g for r in x['rows']]
    ok = (sorted(x['name'] for x in g) == sorted(x['name'] for x in DEFAULTS['groups'])
          and sorted(names) == sorted(r for x in DEFAULTS['groups'] for r in x['rows'])
          and all(isinstance(m.get(r), list) and all(k in MOD_NAMES for k in m[r]) for r in names)
          and all(isinstance(c[k], float) for k in ('lost_turn_us', 'downhill_hold_max_s', 'downhill_hold_left_us')))
    if not ok: raise ValueError('lost_policy.json does not match the known rows / modifiers')
    return c


def load_config():
    """DEFAULTS, replaced as a whole by a valid lost_policy.json (never raises: live_pid imports this module)."""
    try:
        f = json.loads(CONFIG_FILE.read_text(encoding='utf8'))
        c = dict(groups=[dict(name=str(x['name']), rows=[str(r) for r in x['rows']]) for x in f['groups']],
                 mods={str(k): list(v) for k, v in f['mods'].items()},
                 **{k: float(f[k]) for k in ('lost_turn_us', 'downhill_hold_max_s', 'downhill_hold_left_us')})
        return _valid(c)
    except Exception:
        return json.loads(json.dumps(DEFAULTS))


CONFIG = load_config()


def param(name):
    return CONFIG[name]


class Row(object):
    __slots__ = ('name', 'propose', 'make', 'mods', 'flag', 'owner')

    def __init__(self, name, propose, make, mods, flag, owner):
        self.name = name; self.propose = propose; self.make = make; self.mods = tuple(mods); self.flag = flag; self.owner = owner


class LGroup(object):
    __slots__ = ('name', 'pre', 'prelude', 'catch', 'rows')

    def __init__(self, name, pre, prelude, catch, rows):
        self.name = name; self.pre = pre; self.prelude = prelude; self.catch = catch; self.rows = tuple(rows)


def build(groups_code, rows_code, mods_code, config=None):
    """the executable table from the config: groups_code {name: (pre, prelude, catch)}, rows_code {name: (propose, make, flag, owner)},
    mods_code {name: (owner label, fn)} (all supplied by live_pid)."""
    c = CONFIG if config is None else config
    out = []
    for g in c['groups']:
        pre, prelude, catch = groups_code[g['name']]
        rows = []
        for r in g['rows']:
            propose, make, flag, owner = rows_code[r]
            rows.append(Row(r, propose, make, [(m,) + tuple(mods_code[m]) for m in c['mods'][r]], flag, owner))
        out.append(LGroup(g['name'], pre, prelude, catch, rows))
    return tuple(out)


def decide(table, obj, F):
    """(row, proposed value) of the first row that proposes, or (None, None) -> stop."""
    for g in table:
        if g.pre is not None and not g.pre(obj, F): continue
        if g.catch:
            try:
                if g.prelude is not None: g.prelude(obj, F)
                for r in g.rows:
                    v = r.propose(obj, F)
                    if v is not None: return r, v
            except Exception: pass
        else:
            if g.prelude is not None: g.prelude(obj, F)
            for r in g.rows:
                v = r.propose(obj, F)
                if v is not None: return r, v
    return None, None


def finish(row, v, obj, F, owner_after):
    """the logged proposal of the chosen row after its modifiers (in order); F.owner = last modifier that changed steer_us (log only)."""
    p = row.make(obj, F, v)
    F.owner = row.owner if row.owner is not None else p['reason'].lower()
    for name, label, fn in row.mods:
        b = p.get('steer_us'); r = fn(obj, F, p)
        if r is not None: p = r
        F.owner = owner_after(F.owner, label, b, p)
    return p
