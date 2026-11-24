"""pause-resume-r1 (pause-resume-20261120): in-place pause / resume of a running live PID run, run timing (laps), and the mask reader.

Not active until POST /api/pid/pause is called: LivePID only calls hold_frame / loop_top when a pause state exists (self._pz) or the
pause_clock offset is not 0; lap_hook / on_start / on_end / remaining_s / deadline_expired give today's values without a pause.

Pause (POST /api/pid/pause {"run_id"}): always accepted for a running run of this run_id (any segment). Under LivePID.lock:
  output latch at manual_drive (centre, PWM 0, at once), manual_drive's own deadline suspended, state 'paused'. Every following frame
  is a held frame (hold_frame): the worker result is published as latest (mask / path / preview keep updating), centre + PWM 0 is sent,
  no segment module / route state / PID is stepped. While the rear encoder still moves (coast), the odometer + gyro + pitch of that
  frame go into lap_pos (only lap_pos), until the encoder is still for still_frames frames.
Resume (POST /api/pid/resume {"run_id"}): accepted only if evaluate() finds no reason (codes below). The shift is applied by the control
  thread at the top of its next frame (loop_top): pause_clock removes D = (capture time of the last held frame) - (capture time of the
  last frame whose data entered the control: the last controlled frame or the last coast frame) from the control timeline, latest is set
  back to the last controlled frame, and (reset_on_resume, default on) the PID controller, hold memory, play / command compensation,
  wheel-speed meters, PWM slew and the yaw inner loop restart; manual_drive's latch is released and its deadline restored.
Codes: STALE_RUN, NOT_PAUSED, RUN_ENDED, ON_SLOPE, UPPER_DECK, FAULT, ESTOP, MOVED_ACROSS_SEGMENT (contract) +
  STILL_MOVING, NO_FRESH_PATH, UNSUPPORTED_MODE (additions; see INTERFACE.md).
Config: pause_resume.json next to this file, read at each pause (absent / broken = DEFAULTS). Python 3 standard library."""
import base64, json, struct
from pathlib import Path
import pause_clock as _pclk

REVISION = 'pause-resume-r1'
CONFIG_FILE = Path(__file__).with_name('pause_resume.json')
DEFAULTS = dict(slope_deg=4.0, coast_max_m=0.5, coast_max_yaw_deg=8.0, still_frames=5, reset_on_resume=True, m_per_count=2.22e-4,
                allow_segment_cross=False,   # pause-resume-r1c (2026-11-21, run f0938b9d): true = coasting over a lap_pos landmark / into the next segment on the
                                             # flat no longer blocks the resume; the segment the car stands in now must not be a slope / upper-deck one, lap_pos not lost
                upper_post_turn_resume=False)   # pause-resume-r1b (2026-11-21, user): true = a pause on the upper straight AFTER the dashed turn may be resumed
                                                # (lap_pos segment upper_straight, not lost, DashRoute.turned). Before the turn / in the turn / ramp / downhill: as before.
UPPER_ROUTES = ('CLIMBED', 'OPEN')
SLOPE_SEGS = ('ramp', 'downhill', 'bottom')
UPPER_SEGS = ('top', 'dashed_turn', 'upper_straight')
TEXT = dict(
    STALE_RUN='不是目前這一輪的指令（輪次識別碼不符），請重新讀取狀態。',
    NOT_PAUSED='這一輪沒有在暫停，不需要繼續。',
    RUN_ENDED='這一輪已結束，請回旗子起點重新開始。',
    ON_SLOPE='車在坡道、下坡或落地段，不能原地繼續；請停止本輪，回旗子起點重新開始。',
    UPPER_DECK='車在上層（爬坡後到下坡前），不能原地繼續；請停止本輪，回旗子起點重新開始。',
    FAULT='車端故障或控制板／I²C 通訊異常，不能繼續；請停止本輪並排除故障。',
    ESTOP='實體急停中，不能繼續；請停止本輪並排除急停。',
    MOVED_ACROSS_SEGMENT='暫停時車滑過路段或地標（或被移動），位置與控制狀態可能不一致；請停止本輪，回旗子起點重新開始。',
    STILL_MOVING='車還在滑行或剛暫停，請等車完全停住再按繼續。',
    NO_FRESH_PATH='目前沒有連續有效的新路徑（畫面看不到線或影像過舊），請確認車在跑道上、鏡頭正常。',
    UNSUPPORTED_MODE='這個執行模式不支援暫停（只支援 orin_timed／browser_lease）。',
)


class PauseError(ValueError):
    def __init__(self, code, **extra):
        ValueError.__init__(self, TEXT.get(code, code)); self.code = code; self.extra = extra


def load_config(path=None):
    c = dict(DEFAULTS)
    try:
        d = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
        if isinstance(d, dict): c.update({k: d[k] for k in DEFAULTS if k in d})
    except (OSError, ValueError): pass
    return c


def _real():
    return _pclk._time.monotonic()


def _running(lp):
    return bool(lp.run and lp.run.get('state') == 'running')


def _continuous(run):
    return bool(run and run.get('execution_mode') == 'orin_continuous')


def _enc(v):
    try:
        b = (v or {}).get('enc_b') or {}
        return int(b['forward']) - int(b['reverse'])
    except (KeyError, TypeError, ValueError): return None


def _pitch_rel(lp):
    try:
        b = lp.boost; _, p = b._read(); return float(p) - float(b.c['flat_pitch_deg'])
    except Exception: return None


def _lpos(lp):
    tp = getattr(lp, '_lpos', None)
    return tp if tp and getattr(lp, '_lpos_run', None) == (lp.run or {}).get('id') else None


def _log(lp, rec):
    try:
        lp.log.write(json.dumps(dict(rec, revision=REVISION), ensure_ascii=False) + '\n'); lp.log.flush()
    except Exception: pass


def _cur(lp):
    pz = getattr(lp, '_pz', None)
    return pz.get('cur') if pz else None


# ---------------------------------------------------------------------------------------------- run start / end / timing
def on_start(lp):
    """LivePID.start, after self.run is built (under lock): no pause state; run timing starts."""
    lp._pz = None
    t0 = lp.start_t   # set by start() just before (no extra clock read: the replay read-order trace stays the same)
    lp._pt = dict(run_id=lp.run['id'], start_v=t0, start_real=_pclk.to_real(t0), marks=[], last_lap=None, end_v=None, end_real=None)


def on_end(lp, reason):
    """LivePID.end, after manual.request_stop (under lock): an open pause episode is closed in the log; timing frozen."""
    try:
        pt = getattr(lp, '_pt', None)
        if pt and pt.get('run_id') == lp.run.get('id') and pt.get('end_v') is None:
            c = _cur(lp); pt['end_v'] = c['since_v'] if c else _pclk.monotonic(); pt['end_real'] = _real()
        pz = getattr(lp, '_pz', None)
        if pz and pz.get('cur'):
            c = pz['cur']; pz['state'] = 'ended'
            _log(lp, dict(kind='pause_end', why='RUN_ENDED', stop_reason=reason, episode=pz['n'], paused_real_s=round(_real() - c['since_real'], 3),
                          coast_m=c.get('coast_m'), held_frames=c.get('held', 0)))
            pz['paused_real_total'] += _real() - c['since_real']; pz['cur'] = None; pz['apply_pending'] = None
    except Exception: pass


def lap_hook(lp, proposal):
    """after LivePID._lap_pos on a controlled frame: a lap_pos lap step (C1s anchor or skipped C1s) marks a lap boundary (log-free)."""
    try:
        pt = getattr(lp, '_pt', None); tp = _lpos(lp)
        if not pt or tp is None or pt.get('run_id') != lp.run.get('id'): return
        if pt['last_lap'] is None: pt['last_lap'] = tp.lap
        if tp.lap > pt['last_lap']:
            t = (proposal.get('inputs') or {}).get('t_send')   # this frame's logged control time (no extra clock read)
            if not isinstance(t, (int, float)): return
            pt['last_lap'] = tp.lap
            pt['marks'].append(dict(lap_pos_lap=tp.lap, anchor=None if tp.anchor is None else tp.anchor[0],
                                    active_s=t - pt['start_v'], total_s=_pclk.to_real(t) - pt['start_real']))
    except Exception: pass


def timing(lp):
    pt = getattr(lp, '_pt', None)
    if not pt or not lp.run or pt.get('run_id') != lp.run.get('id'): return None
    c = _cur(lp); pz = getattr(lp, '_pz', None)
    now_v = pt['end_v'] if pt['end_v'] is not None else (c['since_v'] if c else _pclk.monotonic())
    now_r = pt['end_real'] if pt['end_real'] is not None else _real()
    m = pt['marks']; laps = []
    for i in range(1, len(m)):
        laps.append(dict(lap=i, active_s=round(m[i]['active_s'] - m[i - 1]['active_s'], 2), total_s=round(m[i]['total_s'] - m[i - 1]['total_s'], 2)))
    return dict(revision=REVISION, active_s=round(now_v - pt['start_v'], 2), total_s=round(now_r - pt['start_real'], 2),
                paused_s=round((pz['paused_real_total'] if pz else 0.) + ((_real() - c['since_real']) if c else 0.), 2),
                compensated_s=round(pz['comp_total'] if pz else 0., 3), pauses=pz['n'] if pz else 0,
                laps_completed=len(laps), laps=laps,
                current_lap_active_s=None if not m else round(now_v - pt['start_v'] - m[-1]['active_s'], 2),
                current_lap_total_s=None if not m else round(now_r - pt['start_real'] - m[-1]['total_s'], 2),
                lap_basis='lap_pos 圈數加一（彎1起點 C1s 地標判定，或漏判時依標稱位置補）＝一個圈界；相鄰兩個圈界之間＝一圈。起跑到第一個圈界、最後一個圈界到停止都不算完整圈。active_s 不含暫停；total_s 含暫停。',
                ended=pt['end_v'] is not None)


def remaining_s(lp):
    """status remaining_s: 0 = no running run (as before); None = no deadline (orin_continuous, or orin_timed with seconds null);
    while a pause episode is open the value stays at the pause moment."""
    run = lp.run
    if not run or run.get('state') != 'running': return 0
    if _continuous(run) or run.get('deadline') is None: return None
    c = _cur(lp); now = c['since_v'] if c else _pclk.monotonic()
    return max(0, run['deadline'] - now)


def deadline_expired(lp, now):
    """check_safety: orin_continuous as ops07 (deadline must be null); no deadline -> never; open pause episode -> not expiring."""
    run = lp.run
    if _continuous(run):
        if run.get('deadline') is not None: raise ValueError('07 deadline must be null')
        return False
    if run.get('deadline') is None or _cur(lp) is not None: return False
    return now >= run['deadline']


# ---------------------------------------------------------------------------------------------- evaluation
def evaluate(lp, c, d=None, pitch=None):
    """(code or None) why a resume is not allowed now. d = manual.state() (read here when None); pitch = IMU pitch_rel."""
    cfg = c['cfg']
    if d is None:
        try: d = lp.manual.state() or {}
        except Exception: d = {}
    v = d.get('vehicle') or {}
    if (v.get('pins') or {}).get('D11') == 0 or v.get('reason') == 'PHYSICAL_ESTOP': return 'ESTOP'
    cur_gen = (getattr(lp.manual, 'current', None) or {}).get('generation')
    if (d.get('uart_error') or d.get('uart_age_s', 99) > 1 or v.get('pca_ok') is False or v.get('steer_output_enabled') is False
            or (v.get('state') is not None and v.get('state') != 'MANUAL') or (cur_gen is not None and v.get('generation') is not None and v.get('generation') != cur_gen)
            or not lp.manual.busy()):
        return 'FAULT'
    if pitch is None: pitch = _pitch_rel(lp)
    sd = float(cfg['slope_deg'])
    if (pitch is not None and abs(pitch) >= sd) or (c.get('pitch0') is not None and abs(c['pitch0']) >= sd) or c.get('down_tail0') or c.get('bottom0'):
        return 'ON_SLOPE'
    if c.get('seg0') in SLOPE_SEGS and not c.get('lost0'): return 'ON_SLOPE'
    if c.get('route0') in UPPER_ROUTES or (c.get('seg0') in UPPER_SEGS and not c.get('lost0')):
        if not (cfg.get('upper_post_turn_resume') is True and c.get('seg0') == 'upper_straight' and not c.get('lost0') and c.get('turned0') is True): return 'UPPER_DECK'   # r1b
    tp = _lpos(lp)
    if tp is not None and c.get('ev0') is not None:
        if (len(tp.events) != c['ev0'] or tp.segment() != c['seg0'] or tp.pending() != c['next0'] or tp.lap != c['lap0']):
            if cfg.get('allow_segment_cross') is not True or tp.lost: return 'MOVED_ACROSS_SEGMENT'
            now_seg = tp.segment()   # r1c: where the car stands after the coast
            if now_seg in SLOPE_SEGS: return 'ON_SLOPE'
            if now_seg in UPPER_SEGS: return 'UPPER_DECK'
    if (c.get('coast_m') or 0.) > float(cfg['coast_max_m']) or abs(c.get('coast_yaw') or 0.) > float(cfg['coast_max_yaw_deg']): return 'MOVED_ACROSS_SEGMENT'
    if c.get('held', 0) == 0 or c.get('still', 0) < int(cfg['still_frames']): return 'STILL_MOVING'
    lt = lp.latest or {}
    if not (lt.get('valid') is True and lp.consecutive >= 3 and _pclk.monotonic() - lt.get('capture_at', -1e18) <= lp.run['parameters']['max_gap']):
        return 'NO_FRESH_PATH'
    return None


def _pause_obj(lp):
    pz = getattr(lp, '_pz', None); c = pz.get('cur') if pz else None
    if not lp.run: return dict(state='idle', since_s=None, can_resume=False, reason=None, reason_text=None, coast_m=None)
    if not _running(lp):
        return dict(state='ended', since_s=None, can_resume=False, reason='RUN_ENDED', reason_text=TEXT['RUN_ENDED'], coast_m=None,
                    pauses=pz['n'] if pz else 0)
    if c is None:
        return dict(state='running', since_s=None, can_resume=False, reason='NOT_PAUSED', reason_text=TEXT['NOT_PAUSED'], coast_m=None,
                    pauses=pz['n'] if pz else 0, resume_pending=False)
    code = c.get('code', 'STILL_MOVING')
    return dict(state='paused' if pz['state'] == 'paused' else 'running', since_s=round(_real() - c['since_real'], 2),
                can_resume=pz['state'] == 'paused' and code is None, reason=code, reason_text=TEXT.get(code) if code else None,
                coast_m=None if c.get('coast_m') is None else round(c['coast_m'], 3), coast_yaw_deg=None if c.get('coast_yaw') is None else round(c['coast_yaw'], 1),
                moving=c.get('still', 0) < int(c['cfg']['still_frames']), held_frames=c.get('held', 0), episode=pz['n'],
                resume_pending=pz.get('apply_pending') is not None)


def status_pause(lp):
    try: return _pause_obj(lp)
    except Exception as e: return dict(state='unknown', reason='STATUS_ERROR', error=type(e).__name__)


# ---------------------------------------------------------------------------------------------- API (web thread)
def _check_run(lp, p):
    run = lp.run; rid = (p or {}).get('run_id')
    if not run: raise PauseError('RUN_ENDED')
    if not isinstance(rid, str) or rid != run.get('id'): raise PauseError('STALE_RUN', current_run_id=run.get('id'))
    if run.get('state') != 'running': raise PauseError('RUN_ENDED')
    if _continuous(run): raise PauseError('UNSUPPORTED_MODE')
    return run


def pause(lp, p):
    with lp.lock:
        run = _check_run(lp, p)
        pz = getattr(lp, '_pz', None)
        if pz is None or pz.get('run_id') != run['id']:
            pz = lp._pz = dict(run_id=run['id'], state='running', n=0, cur=None, apply_pending=None, comp_total=0., paused_real_total=0.)
        if pz['state'] == 'paused':
            return dict(ok=True, run_id=run['id'], state='paused', duplicate=True, pause=_pause_obj(lp))
        if pz.get('cur') is not None and pz.get('apply_pending') is not None:   # resume accepted but not applied yet: stay in the same episode
            pz['apply_pending'] = None; pz['state'] = 'paused'
            _log(lp, dict(kind='pause_again', wall=_pclk._time.time(), episode=pz['n']))
            return dict(ok=True, run_id=run['id'], state='paused', pause=_pause_obj(lp))
        cfg = load_config(); now_v = _pclk.monotonic(); lt = lp.latest or {}
        tp = _lpos(lp); iv = getattr(lp, '_in_v', None)
        c = dict(cfg=cfg, since_real=_real(), since_v=now_v, ctrl_latest=lp.latest, anchor_cap_v=lt.get('capture_at'), last_held_cap_v=None,
                 held=0, still=0, enc0=_enc(iv), enc_last=_enc(iv), coast_m=0., coast_yaw=0., coast_frames=0,
                 pitch0=_pitch_rel(lp), route0=(lt.get('dash_route') or {}).get('route'),
                 turned0=bool(getattr(getattr(lp, 'dash', None), 'turned', False)),   # r1b: the dashed turn of this pass was already started
                 down_tail0=bool(getattr(lp, 'down_until', None) is not None and now_v <= lp.down_until),
                 bottom0=bool(getattr(lp, 'bown', None) is not None and getattr(lp.bown, 'active', False)),   # bottom_owner scope: landing -> curve-4 FLOOR / t_max
                 odo0=tp.D if tp else None, yaw0=tp.yaw if tp else None, seg0=tp.segment() if tp else None, next0=tp.pending() if tp else None,
                 ev0=len(tp.events) if tp else None, lap0=tp.lap if tp else None, lost0=tp.lost if tp else None, code='STILL_MOVING',
                 remaining_v=None if run.get('deadline') is None else run['deadline'] - now_v, manual_remaining=None)
        pz['n'] += 1; pz['cur'] = c; pz['state'] = 'paused'
        try: lp.manual.pid_pause_hold(run['id'], True)   # output latch: centre, PWM 0 now (the next frame is a held frame anyway)
        except Exception as e: c['latch_error'] = type(e).__name__
        try: c['manual_remaining'] = lp.manual.pause_deadline(run['id'])
        except Exception as e: c['deadline_error'] = type(e).__name__
        _log(lp, dict(kind='pause', wall=_pclk._time.time(), run_id=run['id'], episode=pz['n'], t_v=round(now_v, 4), pause_offset_s=round(_pclk.offset(), 6),
                      last_frame_seq=lt.get('frame_seq'), last_capture_at=lt.get('capture_at'), route=c['route0'], turned=c['turned0'], pitch_rel=c['pitch0'], down_tail=c['down_tail0'],
                      bottom_scope=c['bottom0'], lap_pos=dict(seg=c['seg0'], next=c['next0'], odo_m=c['odo0'], lap=c['lap0'], lost=c['lost0']),
                      remaining_s=c['remaining_v'], latch_error=c.get('latch_error'), deadline_error=c.get('deadline_error')))
        return dict(ok=True, run_id=run['id'], state='paused', pause=_pause_obj(lp))


def resume(lp, p):
    with lp.lock:
        run = _check_run(lp, p)
        pz = getattr(lp, '_pz', None)
        if pz is None or pz.get('run_id') != run['id'] or pz['n'] == 0: raise PauseError('NOT_PAUSED')
        if pz['state'] != 'paused':   # already resumed (applied or pending): the same answer, no second action
            return dict(ok=True, run_id=run['id'], state='running', duplicate=True, pause=_pause_obj(lp))
        c = pz['cur']; code = evaluate(lp, c); c['code'] = code
        if code is not None:
            _log(lp, dict(kind='resume_rejected', wall=_pclk._time.time(), episode=pz['n'], code=code))
            raise PauseError(code, pause=_pause_obj(lp))
        pz['state'] = 'running'; pz['apply_pending'] = dict(at_real=_real(), at_v=_pclk.monotonic())
        _log(lp, dict(kind='resume_accepted', wall=_pclk._time.time(), episode=pz['n']))
        return dict(ok=True, run_id=run['id'], state='running', pause=_pause_obj(lp))


# ---------------------------------------------------------------------------------------------- control thread
def hold_frame(lp, result, captured, frame, seq, preview):
    """inside LivePID._loop under lp.lock, right after the worker result checks. True = this frame was a held frame (the loop continues)."""
    pz = getattr(lp, '_pz', None)
    if pz is None or pz.get('cur') is None or not _running(lp) or pz.get('run_id') != lp.run['id']: return False
    c = pz['cur']; run = lp.run; now = _pclk.monotonic()
    valid = result.get('valid') is True and bool(result.get('path')) and now - captured <= run['parameters']['max_gap']
    lp.latest = dict(result, capture_at=captured, frame_seq=frame, valid=valid, consumed_at=now, control_age_s=now - captured, paused=True)
    if _pclk.offset(): lp.latest['pause_offset_s'] = round(_pclk.offset(), 6)
    lp.consecutive = lp.consecutive + 1 if valid else 0; lp.state = 'ready' if valid else 'no_reliable_path'
    if preview is not None: lp.preview = preview; lp.preview_seq = seq
    c['held'] += 1; c['last_held_cap_v'] = captured
    try: d = lp.manual.state() or {}
    except Exception: d = {}
    enc = _enc(d.get('vehicle'))
    if enc is not None:
        if c['enc_last'] is None: c['enc_last'] = enc; c['enc0'] = enc if c['enc0'] is None else c['enc0']
        if enc != c['enc_last']:   # coast (or the car is moved): odometer + gyro + pitch of this frame into lap_pos only
            c['enc_last'] = enc; c['still'] = 0; c['coast_frames'] += 1; c['anchor_cap_v'] = captured
            tp = _lpos(lp)
            if tp is not None:
                gz = None
                try: gz = round(float(lp.heading._gz(now)), 2)
                except Exception: gz = None
                try: tp.update(now, gz, _pitch_rel(lp), enc)
                except Exception as e: c['lap_pos_error'] = type(e).__name__
                if c.get('yaw0') is not None: c['coast_yaw'] = tp.yaw - c['yaw0']
        else: c['still'] += 1
        if c['enc0'] is not None: c['coast_m'] = abs(enc - c['enc0']) * float(c['cfg']['m_per_count'])
    if pz['state'] == 'paused': c['code'] = evaluate(lp, c, d)
    p = dict(owner=lp.owner, run_id=run['id'], seq=seq, target_us=run['parameters']['center_us'], pwm=0)
    try: lp.manual.pid_input(p)
    except ValueError:
        info = lp._manual_ended()
        if info is None: raise
        run['manual_session'] = info; _log(lp, dict(kind='manual_session_end', wall=_pclk._time.time(), **info))
        lp.end(info['stop_reason']); return True
    _log(lp, dict(kind='pause_hold', seq=seq, frame_seq=frame, capture_at=captured, valid=valid, enc=enc, still=c['still'], code=c.get('code')))
    return True


def loop_top(lp, previous_capture, pend):
    """top of each LivePID._loop iteration while a pause state exists or the clock offset is not 0. Applies an accepted resume (shift +
    restore + resets) or, with no running run, gives the removed time back (offset 0 again). Returns (previous_capture, pend)."""
    with lp.lock:
        pz = getattr(lp, '_pz', None)
        if pz is not None and pz.get('apply_pending') is not None and pz.get('cur') is not None and _running(lp):
            c = pz['cur']; ap = pz['apply_pending']; run = lp.run
            d = 0.
            if c['last_held_cap_v'] is not None and c['anchor_cap_v'] is not None: d = max(0., c['last_held_cap_v'] - c['anchor_cap_v'])
            if d > 0.:
                _pclk.shift(d)
                if previous_capture is not None: previous_capture = previous_capture - d
                if pend is not None: pend = (pend[0], pend[1], pend[2] - d, pend[3])
            if c['ctrl_latest'] is not None:   # times of the last controlled frame are already on the new timeline
                lt = dict(c['ctrl_latest']); a = c.get('anchor_cap_v')   # pause-resume-r1a (2026-11-21, run 47abbf59): coast frames stay on the timeline, so the
                if isinstance(a, (int, float)) and isinstance(lt.get('capture_at'), (int, float)) and a > lt['capture_at']: lt['capture_at'] = a   # restored frame is as old as the coast lasted (0.54 s > max_gap) -> check_safety ended the run with PID_VISION_STALE before the next frame; date it at the last coast frame
                lp.latest = lt
            resets = apply_resets(lp) if c['cfg'].get('reset_on_resume', True) else []
            try: lp.manual.pid_pause_hold(run['id'], False)
            except Exception: pass
            try: lp.manual.resume_deadline(run['id'], c['manual_remaining'])
            except Exception: pass
            paused_real = _real() - c['since_real']
            pz['comp_total'] += d; pz['paused_real_total'] += paused_real
            _log(lp, dict(kind='resume', wall=_pclk._time.time(), episode=pz['n'], compensated_s=round(d, 6), paused_real_s=round(paused_real, 3),
                          accepted_to_applied_s=round(_real() - ap['at_real'], 3), pause_offset_s=round(_pclk.offset(), 6), held_frames=c['held'],
                          coast_m=round(c['coast_m'], 3), coast_yaw_deg=round(c['coast_yaw'], 2), coast_frames=c['coast_frames'], resets=resets,
                          restored_frame_seq=(c['ctrl_latest'] or {}).get('frame_seq')))
            pz['cur'] = None; pz['apply_pending'] = None; pz['state'] = 'running'
        elif not _running(lp) and _pclk.offset() and (pz is None or pz.get('cur') is None):
            d = _pclk.offset(); _pclk.shift(-d)
            if previous_capture is not None: previous_capture = previous_capture + d
            if pend is not None: pend = (pend[0], pend[1], pend[2] + d, pend[3])
            lt = lp.latest
            if lt:
                lt = dict(lt); lt.pop('pause_offset_s', None)
                for k in ('capture_at', 'consumed_at'):
                    if isinstance(lt.get(k), (int, float)): lt[k] = lt[k] + d
                lp.latest = lt
    return previous_capture, pend


def apply_resets(lp):
    """reset_on_resume: restart what carries the pre-pause motion (not position / route state)."""
    out = []
    def step(name, fn):
        try: fn(); out.append(name)
        except Exception as e: out.append(name + ':ERROR:' + type(e).__name__)
    step('controller', lambda: setattr(lp, 'controller', type(lp.controller)(lp.run['parameters'])))   # PID integral / derivative / backlash state
    def _hold():
        from pid_hold import PidHold
        lp.hold = PidHold()
    step('pid_hold', _hold)   # no pre-pause command replayed by WARMUP / PID_HOLD
    def _pcomp():
        if getattr(lp, 'pcomp', None) is not None:
            import play_comp, output_stage
            lp.pcomp = play_comp.PlayComp(output_stage.play_comp_config())
    step('play_comp', _pcomp)   # steering-play direction
    def _ccomp():
        if getattr(lp, 'ccomp', None) is not None:
            import cmd_comp
            lp.ccomp = cmd_comp.CmdComp()
    step('cmd_comp', _ccomp)   # in-flight steering sum
    def _speed():
        from downhill_speed import SpeedMeter
        lp.dspeed = SpeedMeter(); lp.wspeed = SpeedMeter(); lp._cm_speed = None
    step('speed_meters', _speed)
    def _boost():
        lp.boost.out = None; lp.boost.out_t = None
    step('imu_boost_slew', _boost)
    def _yin():
        if getattr(lp, 'yin', None) is not None: lp.yin.reset()
    step('yaw_inner', _yin)
    def _ce():
        ce = getattr(lp, 'centre_est', None)
        if ce is not None: ce.cmd = []; ce.gz = []
    step('centre_estimator', _ce)
    return out


# ---------------------------------------------------------------------------------------------- mask reader (web thread, no lock)
def _png_size(b64):
    try:
        h = base64.b64decode(b64[:44])
        if h[:8] == b'\x89PNG\r\n\x1a\n' and h[12:16] == b'IHDR': return struct.unpack('>II', h[16:24])
    except Exception: pass
    return None, None


def mask_view(lp):
    """latest control frame's mask + path, or None (no frame / no mask). Reads one dict reference, no lock, no re-encoding."""
    lt = lp.latest
    if not lt: return None
    png = lt.get('mask_png')
    if not isinstance(png, str) or not png: return None
    run = lp.run; running = bool(run and run.get('state') == 'running')
    pz = getattr(lp, '_pz', None); paused = bool(running and pz and pz.get('cur') is not None)
    cap = lt.get('capture_at'); w, h = _png_size(png)
    held = bool(lt.get('paused'))
    return dict(run_id=run.get('id') if run else None, run_state=run.get('state') if run else None, seq=lt.get('seq'), frame_seq=lt.get('frame_seq'),
                age_s=None if not isinstance(cap, (int, float)) else round(_pclk.monotonic() - cap, 3), valid=bool(lt.get('valid')),
                mask=dict(png_base64=png, width=w, height=h),
                path=dict(points=list(lt.get('path') or []), width=1280, height=720,
                          source=('worker_paused' if held else 'control') if running else 'preview'),
                path_algorithm=lt.get('path_algorithm'), paused=paused)
