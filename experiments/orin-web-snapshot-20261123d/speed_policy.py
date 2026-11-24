"""speed-policy-r1 (integration B3d, 2026-11-12): the PWM decisions of live_pid.py gathered in one object (module), today's order.
speed-guard-r2 hooks (2026-11-13 offline candidate, NOT deployed): guard() after the valid-frame caps, after hold_pwm() and on the
DOWNHILL_HOLD PWM (speed_guard.py / speed_guard.json, read once per run; all switches off or the module missing = no effect, nothing
logged). r2 adds the log-only overspeed watch (it never changes the PWM; it gets F.downhill, and the DOWNHILL_HOLD row as downhill).
REVISION is unchanged on purpose so that logs with the guard off stay identical to speed-policy-r1.

Pure restructuring (proven by the differential replay): same steps, same order, same calls, same exceptions. Per valid frame:
  1. boost(lp, F)        imu_boost.step (pitch boost / pwm ramp)                              [at its old place: before the steering
                         layers, because gain_schedule reads proposal['imu_boost'] and the IMU file read must stay in that order]
  2. adapt_inputs(lp, F) speed_adapt inputs: special (dash / crest / heading hold / dashed TURN), imu_boost pitch through flat_dip_guard,
                         crest-marks reset                                                     [speed group, before upper_path]
     (upper_path steering runs here, between 2 and 3, inside the same try as before - it reads nothing of the speed policy)
  3. adapt(lp, F)        speed_adapt.step (curve / weave / upper pre / post / crest / downhill / uphill sub-policies)
  4. caps(lp, F)         upper-section cap (lane-lock, pre / post turn caps, dashed TURN uses TURN_PWM) -> brake zones -> turn-pwm cap
Lost-path frames: hold_pwm(lp, F, p) (speed_adapt.hold_pwm), downhill_hold_pwm(lp, F) (DOWNHILL_HOLD's PWM), then caps via upper_cap.
HIDDEN WRITES made explicit (kept, not removed):
  - brake(): when a zone caps the PWM below speed_adapt's current value it writes speed_adapt.cur (write_speed_cur);
  - hold_pwm(): speed_adapt.hold_pwm / safe_pwm write speed_adapt.cur themselves (inside speed_adapt.py);
  - adapt(): remember_post_turn() stores this frame's post-turn state for the NEXT frame's jump filter (lp._last_post_turn); it is
    evaluated inside the speed_adapt call's argument list, after pre_turn(), as before;
  - gain_schedule (steering) reads speed_adapt.cur of the previous frame (not changed here).
Config: speed_policy.json (read once at import): turn_pwm 18 (turn-pwm-r4) and turn_pwm_gate 20 deg (were live_pid literals).
The other PWM numbers stay in their own files (speed_adapt.json, brake_zones.json, imu_boost.json, lane_segment.LOCK_PWM).
Python 3 standard library + the web/ modules it calls."""
import json, time
import pause_clock   # pause-clock-r1 (pause-resume-20261120): control timeline (offset 0 = Orin monotonic)
from pathlib import Path
from lane_segment import LOCK_PWM as LANE_LOCK_PWM
from imu_boost import REVISION as IMU_BOOST_REVISION
import brake_zones
try:import speed_guard   # speed-guard-r2 candidate: ramp cap + downhill over-speed guard + log-only overspeed watch (speed_guard.py / speed_guard.json)
except Exception:speed_guard=None
try:import lap_speed   # lap-speed-r1 (lap65-20261115 candidate): position-based PWM floor from lap_pos (lap_speed.json; absent / off = no effect)
except Exception:lap_speed=None
try:import upper_wide   # upper-wide-r1 (upper-wide-20261115 offline candidate): wide dashed turn PWM side (upper_wide.py / upper_wide.json, read once per run; missing = off)
except Exception:upper_wide=None

REVISION = 'speed-policy-r1'
TURN_PWM_REVISION = 'turn-pwm-r4'
CONFIG_FILE = Path(__file__).with_name('speed_policy.json')
DEFAULTS = dict(turn_pwm=18., turn_pwm_gate=20.)


def load_config():
    c = dict(DEFAULTS)
    try:
        f = json.loads(CONFIG_FILE.read_text(encoding='utf8'))
        n = {k: float(f[k]) for k in DEFAULTS if k in f}
        c.update(n)
    except Exception: pass
    return c


CONFIG = load_config()


def param(k):
    return CONFIG[k]


TURN_PWM = param('turn_pwm'); TURN_PWM_GATE = param('turn_pwm_gate')


# ---- frame steps
def boost(lp, F):
    proposal = F.p
    try: proposal['pwm'], proposal['imu_boost'] = lp.boost.step(proposal['pwm'], lp.run['imu_boost']['run_pwm_ceiling'])
    except Exception as e: proposal['imu_boost'] = dict(revision=IMU_BOOST_REVISION, reason='BOOST_ERROR', error=type(e).__name__, applied=False)
    lp.boost_status = proposal['imu_boost']


def adapt_inputs(lp, F):
    proposal = F.p; ctx = F.ctx; dr = F.dr
    jst = ctx.jstate()   # value unused (as before); kept because its evaluation can raise -> SPEED_ERROR
    # junction-normal-r1 (user 2026-11-03): junction take-over no longer forces LOW speed; error / steering rules still apply
    F.special = (ctx.sel in ('dash_route', 'crest_midpoint') or ctx.hh_active(proposal) or 'dash_turn' in proposal)
    if F.special and ctx.sel == 'crest_midpoint' and crest_not_special(lp):   # throttle-free-r1: the crest midpoint is a normal path for the PWM
        if not (ctx.hh_active(proposal) or 'dash_turn' in proposal): F.special = False; proposal['speed_special'] = 'CREST_NOT_SPECIAL'
    pr = (proposal.get('imu_boost') or {}).get('pitch_rel_deg')   # NOTE: imu_boost's pitch (not dash_route's)
    if dr.get('route') != 'OPEN': lp.cm_seen = False   # upper-lateral-r3: reset off the upper section (every valid frame; not a PWM item, kept at its place)
    if getattr(lp, 'dipg', None) is not None:
        pr = lp.dipg.clamp(pr)
        if lp.dipg.active or (pr is not None and pr <= -1.9 and lp.dipg.guarding): proposal['flat_dip_guard'] = dict(lp.dipg.info or {}, revision='flat-dip-guard-r1', speed_pitch_rel_deg=round(float(pr), 2))
    F.pr = pr


def crest_not_special(lp):
    """throttle-free-r1: speed_adapt.json crest_not_special (read once per run with the rest of speed_adapt.json); absent -> False."""
    try: return bool(getattr(lp, 'speed', None) is not None and lp.speed.c.get('crest_not_special'))
    except Exception: return False


def adapt(lp, F):
    proposal = F.p; dr = F.dr
    proposal['pwm'], proposal['speed_adapt'] = lp.speed.step(pause_clock.monotonic(), proposal['pwm'], proposal.get('image_error'), proposal['steer_us'], lp.run['parameters']['center_us'], F.special, F.pr, route=dr.get('route'), far=(proposal.get('turn_preview') or {}).get('far_projected_error'), wheel=lp._wheel_cps(), pre_turn=pre_turn(lp, proposal), post_turn=remember_post_turn(lp, proposal), weave_slow=bool(getattr(F, 'ssg', None)) and ((proposal.get('error') or 0.) <= 0. or (lambda b_: bool(b_) and max(g_ for _, g_ in b_) >= 10. and min(g_ for _, g_ in b_) <= -10.)(getattr(lp, '_ss_gz', None) or [])), **speed_ctx(lp, proposal))
    w_adapt(lp, F)   # upper-wide-r1: TURN / post-turn PWM floors (upper_wide.json off or missing -> returns at once)


CURVE_KEYS = {('start', 0): 'c1', ('start', 1): 'c2', ('start', 2): 'c3', ('after_down', 0): 'c4', ('after_down', 1): 'c1', ('after_down', 2): 'c2', ('after_down', 3): 'c3'}   # = segment_ctx.CURVES


def speed_ctx(lp, p):
    """speed-plan-r1 (2026-11-14 offline candidate): extra speed_adapt inputs (curve key from curve_map, crossing window flag), ONLY when
    speed_adapt.json sets curve_pwm_by / xing_cap_pwm / derr_confirm_n; otherwise {} -> exactly today's call. Never raises."""
    try:
        c = lp.speed.c
        if c.get('curve_pwm_by') is None and c.get('xing_cap_pwm') is None and c.get('derr_confirm_n') is None: return {}
        cm = p.get('curve_map') or {}; xp = p.get('crossing_pass')
        key = CURVE_KEYS.get((cm.get('seg'), cm.get('idx'))) if cm.get('reason') in ('FLOOR', 'RELEASED') else None
        return dict(curve_key=key, xp=isinstance(xp, dict) and 'dist_after_m' in xp)
    except Exception: return {}


def caps(lp, F):
    """lane-lock / upper-section cap -> brake zones -> turn-pwm cap (each with its old eligibility, in this order)."""
    p = F.p
    lap_profile(lp, F, p)   # lap-speed-r1: raise towards the lap-position PWM first; every cap below still applies (off -> returns at once)
    if ((p.get('lane_segment') or {}).get('reason') == 'OK' or F.ctx.route == 'OPEN') and p.get('pwm'): lane_cap(lp, p)
    if getattr(lp, 'bzones', None) is not None and p.get('pwm') and p.get('reason') != 'WARMUP': brake(lp, p)
    upper_cap(lp, p)
    w_coast(lp, F, p)   # upper-wide-r1: speed-aware coast cap before the downhill (off -> returns at once)
    guard(lp, F, p, True)   # speed-guard-r2: last PWM decision of a valid frame (switches off -> returns at once)


def lap_profile(lp, F, p):
    """lap-speed-r1 (lap65-20261115 offline candidate): position-based PWM floor (lap_speed.py / lap_speed.json, read once per run).
    Only raises, only on eligible flat-lap frames inside a lap_pos zone; off / module missing -> returns at once (nothing logged)."""
    g = lap_speed.get(lp) if lap_speed is not None else None
    if g is None: return
    try:
        log = g.apply(lp, F, p, getattr(getattr(lp, 'speed', None), 't', None))
        if log is not None: p['lap_speed'] = log
    except Exception as e: p['lap_speed'] = dict(revision=getattr(lap_speed, 'REVISION', 'lap-speed-r1'), reason='ERROR', error=type(e).__name__)


# ---- upper-turn-r1 (upper-20261115, offline candidate): optional speed_adapt.json keys (SpeedAdapt reads the file once per run)
#  upper_pwm_turn : replaces TURN_PWM (speed_policy.json, read at import) in lane_cap / upper_cap for the dashed TURN
#  upper_pwm_align: cap for the frames after the dashed TURN that still turn by the heading error (|err| > TURN_PWM_GATE, no dash_turn),
#                   which today fall back to lane_segment.LOCK_PWM 13 (11-14: 7 of 29 passes, ALIGN at PWM 13 for 1.2-1.6 s).
#  Absent -> TURN_PWM / None -> today's caps exactly. speed_adapt's own target still applies (upper_pwm for these frames).
def _sa_key(lp, k):
    try:
        v = lp.speed.c.get(k) if getattr(lp, 'speed', None) is not None else None
        return None if v is None else float(v)
    except Exception: return None


def turn_pwm(lp):
    v = _sa_key(lp, 'upper_pwm_turn'); b = TURN_PWM if v is None else v
    w = _wide(lp); return b if w is None or w.turn_pwm is None else max(b, w.turn_pwm)   # upper-wide-r1: turn_pwm lifts the TURN cap


def align_pwm(lp, p, dr):
    try:
        v = _sa_key(lp, 'upper_pwm_align')
        if v is None or 'dash_turn' in p or bool(dr.get('used') and dr.get('mode') == 'TURN'): return None
        hh = p.get('heading_hold') or {}; err = hh.get('err_deg')
        if hh.get('reason') in (None, 'NO_REFERENCE', 'TURN', 'DONE') or err is None or abs(float(err)) <= TURN_PWM_GATE: return None
        return v
    except Exception: return None


# ---- caps
def lane_cap(lp, proposal):   # lane-segment-r5 dash lock (r8: whole upper section): slower on the upper section
    cap_u = pre_turn_cap(lp) if pre_turn(lp, proposal) else (post_turn_cap(lp) if post_turn(lp, proposal) else LANE_LOCK_PWM)   # upper-pre-turn-r1; upper-post-turn-r1
    if 'dash_turn' in proposal: cap_u = max(cap_u, turn_pwm(lp))   # upper-turn-r1: TURN_PWM unless upper_pwm_turn is set
    al_ = align_pwm(lp, proposal, (lp.latest or {}).get('dash_route') or {})   # upper-turn-r1: None unless upper_pwm_align is set
    if al_ is not None: cap_u = max(cap_u, al_)
    wt_ = w_turning_pwm(lp, proposal)   # upper-wide-r1: post-TURN turning frames (|hh err| > gate) keep turn_pwm instead of LANE_LOCK_PWM 13
    if wt_ is not None: cap_u = max(cap_u, wt_)
    # dash-turn-pwm-r1 (2026-11-10): the dashed turn uses TURN_PWM, the rest of the upper section keeps LANE_LOCK_PWM
    proposal['pwm'] = min(float(proposal['pwm']), cap_u); proposal['lane_lock_pwm'] = cap_u


def write_speed_cur(lp, v):
    """HIDDEN WRITE made explicit (brake-zones-r1): speed_adapt's internal speed follows a brake-zone cap below it."""
    if getattr(lp, 'speed', None) is not None and getattr(lp.speed, 'cur', None) is not None and lp.speed.cur > v: lp.speed.cur = v


def brake(lp, proposal):   # brake-zones-r1 (brake_zones.py, user 2026-11-06 52116a8f): odometer zones cap the PWM before the curves the camera sees too late
    try:
        sa_ = proposal.get('speed_adapt') or {}; tp_ = proposal.get('turn_preview') or {}
        bcap, binfo = lp.bzones.update((lp.manual.state() or {}).get('vehicle'), sa_.get('reason'), proposal.get('image_error'), tp_.get('far_projected_error'), (proposal.get('imu_boost') or {}).get('pitch_rel_deg'), **({'lap': getattr(lp, '_lpos', None)} if getattr(lp.bzones, 'uses_lap', False) else {}))   # brake-zones-r2: lap_pos only when a lap zone is configured
        if binfo is not None: proposal['brake_zones'] = binfo
        if bcap is not None and float(proposal['pwm']) > bcap:
            proposal['brake_zones']['pwm_before'] = round(float(proposal['pwm']), 1); proposal['pwm'] = bcap
            write_speed_cur(lp, bcap)
    except Exception as e: proposal['brake_zones'] = dict(revision=brake_zones.REVISION, reason='ERROR', error=type(e).__name__)


def turning(p, dr, err):   # turn-pwm-r1 / upper-post-turn-r1: the one "turning" test of the upper-section PWM caps
    return bool(dr.get('used') and dr.get('mode') == 'TURN') or 'dash_turn' in p or (err is not None and abs(float(err)) > TURN_PWM_GATE)


def pre_turn(lp, p):   # upper-pre-turn-r1: upper section before the dashed turn (no heading reference yet, dash line not used)
    try:
        dr = (lp.latest or {}).get('dash_route') or {}; hh = p.get('heading_hold') or {}
        return bool(dr.get('route') == 'OPEN' and not dr.get('used') and hh.get('reason') == 'NO_REFERENCE' and 'dash_turn' not in p and (p.get('lane_segment') or {}).get('reason') != 'OK')
    except Exception: return False


def pre_turn_cap(lp):   # upper-pre-turn-r1: cap on pre-turn frames, never below LANE_LOCK_PWM
    try:
        v = lp.speed.c.get('upper_pwm_pre_turn') if getattr(lp, 'speed', None) is not None else None
        return LANE_LOCK_PWM if v is None else max(float(LANE_LOCK_PWM), float(v))
    except Exception: return LANE_LOCK_PWM


def post_turn(lp, p):   # upper-post-turn-r1: upper section after the dashed turn started (heading reference set) and not turning now
    try:
        dr = (lp.latest or {}).get('dash_route') or {}; hh = p.get('heading_hold') or {}; err = hh.get('err_deg')
        if dr.get('route') != 'OPEN' or pre_turn(lp, p) or hh.get('reason') in (None, 'NO_REFERENCE'): return False
        return not turning(p, dr, err)
    except Exception: return False


def remember_post_turn(lp, p):
    """HIDDEN WRITE made explicit (jump-filter-r2): this frame's post_turn, remembered for the next frame's jump filter."""
    v = post_turn(lp, p); lp._last_post_turn = (pause_clock.monotonic(), bool(v)); return v


def post_turn_cap(lp):   # upper-post-turn-r1: never below LANE_LOCK_PWM
    try:
        v = lp.speed.c.get('upper_pwm_post_turn') if getattr(lp, 'speed', None) is not None else None
        b = LANE_LOCK_PWM if v is None else max(float(LANE_LOCK_PWM), float(v))
        w = _wide(lp); return b if w is None or w.post_pwm is None else max(b, w.post_pwm)   # upper-wide-r1: post_pwm lifts the post-turn cap
    except Exception: return LANE_LOCK_PWM


def upper_cap(lp, p):   # turn-pwm-r1: upper section (route OPEN) at most LANE_LOCK_PWM; dashed TURN until within TURN_PWM_GATE at most TURN_PWM
    try:
        dr = (lp.latest or {}).get('dash_route') or {}
        if dr.get('route') != 'OPEN' or not p.get('pwm'): return
        hh = p.get('heading_hold') or {}; err = hh.get('err_deg')
        turning_ = turning(p, dr, err)
        al_ = align_pwm(lp, p, dr) if turning_ else None   # upper-turn-r1
        cap = (al_ if al_ is not None else turn_pwm(lp)) if turning_ else (pre_turn_cap(lp) if pre_turn(lp, p) else (post_turn_cap(lp) if post_turn(lp, p) else LANE_LOCK_PWM))   # upper-pre-turn-r1; upper-post-turn-r1
        if turning_ and al_ is not None and w_turning_pwm(lp, p) is not None: cap = max(cap, w_turning_pwm(lp, p))   # upper-wide-r1
        if float(p['pwm']) > cap: p['turn_pwm'] = dict(revision=TURN_PWM_REVISION, cap=cap, turning=turning_, pwm_before=round(float(p['pwm']), 1)); p['pwm'] = cap
    except Exception as e: p['turn_pwm'] = dict(revision=TURN_PWM_REVISION, reason='ERROR', error=type(e).__name__)


# ---- lost-path frames
def hold_pwm(lp, F, p):
    """speed-adapt-r1 / r7: base PWM without a valid path; downhill PWM on the downhill (speed_adapt.hold_pwm writes speed_adapt.cur)."""
    if getattr(lp, 'speed', None) is not None: p['pwm'] = lp.speed.hold_pwm(p['pwm'], (F.result.get('dash_route') or {}).get('pitch_rel_deg'), F.downhill and (F.result.get('dash_route') or {}).get('route') != 'OPEN')
    guard(lp, F, p, False)   # speed-guard-r2 (lost rows with the hold_pwm modifier)


def downhill_hold_pwm(lp, F):
    """bottom-hold-pwm-r1: DOWNHILL_HOLD's PWM: downhill PWM only while the steep window is on; the level tail keeps at least min_pwm."""
    v = (float(lp.speed.c.get('downhill_pwm') or lp.hold.last['pwm']) if F.downhill else max(float(lp.hold.last['pwm'] or 0), float(lp.speed.c.get('min_pwm') or 15.))) if getattr(lp, 'speed', None) is not None else lp.hold.last['pwm']
    if speed_guard is None or speed_guard.get(lp) is None: return v   # speed-guard-r2: off -> today's value
    q = dict(pwm=v); guard(lp, F, q, False, 'DOWNHILL_HOLD'); return q['pwm']   # DOWNHILL_HOLD row (no proposal to log into: the watch keeps its records for the next frame)


# ---- speed-guard-r2 (2026-11-13 offline candidate; r1 + the log-only overspeed watch)
def guard(lp, F, p, valid, row=None):
    """ramp cap + downhill over-speed guard on p['pwm'] + log-only overspeed watch (speed_guard.py). Off / module missing -> returns at once
    (speed_guard.json is read once per run by speed_guard.get). A lowered PWM also lowers speed_adapt.cur (write_speed_cur, the brake-zone
    hidden write) so the schedule ramps up from there. row 'DOWNHILL_HOLD': p is not logged by live_pid (the watch keeps its records)."""
    g = speed_guard.get(lp) if speed_guard is not None else None
    if g is None: return
    try:
        if valid: dr = getattr(F, 'dr', None) or {}; pr = getattr(F, 'pr', None)
        else: dr = (getattr(F, 'result', None) or {}).get('dash_route') or {}; pr = dr.get('pitch_rel_deg')
        route = dr.get('route') if isinstance(dr, dict) else None
        v = (lp.manual.state() or {}).get('vehicle') if getattr(lp, 'manual', None) is not None else None
        before = p.get('pwm')
        dl = row == 'DOWNHILL_HOLD'
        new, log = g.apply(pause_clock.monotonic(), before, pr, route, v, bool(getattr(F, 'down_tail', False)), dl or bool(getattr(F, 'downhill', False)), not dl)
        if log is not None: p['speed_guard'] = log
        if new is not None and before is not None and float(new) < float(before):
            p['pwm'] = new; write_speed_cur(lp, float(new))
    except Exception as e: p['speed_guard'] = dict(revision=getattr(speed_guard, 'REVISION', 'speed-guard-r2'), reason='ERROR', error=type(e).__name__)


# ---- upper-wide-r1 (upper-wide-20261115 offline candidate): wide dashed turn, PWM side. upper_wide.json read once per run (upper_wide.get);
# missing / enabled false -> every function below returns at once and nothing is logged (no clock / IMU / MCU-status read added).
def _wide(lp):
    try: return upper_wide.get(lp) if upper_wide is not None else None
    except Exception: return None


def w_turning_pwm(lp, p):
    """turn_pwm on the upper-section turning frames (dashed TURN, or |heading-hold error| > TURN_PWM_GATE after it), else None."""
    w = _wide(lp)
    if w is None or w.turn_pwm is None: return None
    try:
        dr = (lp.latest or {}).get('dash_route') or {}; hh = p.get('heading_hold') or {}
        if dr.get('route') != 'OPEN' or (hh.get('reason') in (None, 'NO_REFERENCE') and 'dash_turn' not in p): return None
        return w.turn_pwm if turning(p, dr, hh.get('err_deg')) else None
    except Exception: return None


def w_adapt(lp, F):
    """after speed_adapt.step: raise the PWM to turn_pwm on turning frames, or ramp it to post_pwm on gated post-turn frames.
    Raising writes speed_adapt.cur (explicit hidden write) so the schedule continues from the raised value."""
    w = _wide(lp)
    if w is None: return
    p = F.p
    try:
        if not p.get('pwm'): return
        info = None; tp = w_turning_pwm(lp, p)
        if tp is not None:
            if float(p['pwm']) < tp: info = dict(revision=upper_wide.REVISION, kind='TURN', pwm_before=round(float(p['pwm']), 2), pwm=tp); p['pwm'] = tp
        elif w.post_pwm is not None and post_turn(lp, p):
            us = p.get('upper_single') or {}; st = us.get('state'); e = us.get('e'); h = us.get('h_target_deg'); y = us.get('yaw_deg')
            ok = (us.get('active') and st in ('FOLLOW', 'HOLD') and (us.get('stall') or {}).get('state') != 'ON'
                  and (st == 'HOLD' or (e is not None and abs(float(e)) <= w.post_err_max)) and h is not None and y is not None and abs(float(h) - float(y)) <= w.post_head_max
                  and (w.coast is None or (w.y is not None and getattr(lp, '_cm_speed', None) is not None))
                  and (not getattr(w, 'need_aligned', False) or bool((us.get('edge') or {}).get('aligned'))))   # upper-edges-r1: post_need_aligned   # with a coast profile: only while it can brake (along position + speed known)
            if ok:
                tg = w.post_target(float(F.now), p['pwm'])
                if tg > float(p['pwm']): info = dict(revision=upper_wide.REVISION, kind='POST', pwm_before=round(float(p['pwm']), 2), pwm=round(tg, 2)); p['pwm'] = tg
            else: w.post_cur = None
        if info is not None:
            p['upper_wide'] = info
            if getattr(lp, 'speed', None) is not None: lp.speed.cur = float(p['pwm'])
    except Exception as e: p['upper_wide'] = dict(revision=getattr(upper_wide, 'REVISION', 'upper-wide-r1'), reason='ERROR', error=type(e).__name__)


def w_coast(lp, F, p):
    """after the upper-section caps: along-line position update + speed-aware coast cap (lowers only; writes speed_adapt.cur like a
    brake zone)."""
    w = _wide(lp)
    if w is None or w.coast is None: return
    try:
        dr = (lp.latest or {}).get('dash_route') or {}; hh = p.get('heading_hold') or {}; bz = getattr(lp, 'bzones', None)
        y = w.along(dr.get('route') == 'OPEN', 'dash_turn' in p, hh.get('yaw_deg'), hh.get('ref0_deg'), getattr(bz, 'last_counts', None) if bz is not None else None)
        if y is None or not p.get('pwm'): return
        cap, info = w.coast_cap(getattr(lp, '_cm_speed', None))
        if info is None: return
        info['revision'] = upper_wide.REVISION
        if cap is not None and float(p['pwm']) > cap:
            info['pwm_before'] = round(float(p['pwm']), 2); p['pwm'] = cap; write_speed_cur(lp, cap)
        q = p.get('upper_wide'); p['upper_wide'] = dict(q, coast=info) if isinstance(q, dict) else dict(revision=upper_wide.REVISION, coast=info)
    except Exception as e: p['upper_wide'] = dict(revision=getattr(upper_wide, 'REVISION', 'upper-wide-r1'), reason='ERROR', error=type(e).__name__)
