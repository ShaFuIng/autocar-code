"""output-stage-r1 (integration B3a, 2026-11-12): the one output stage of live_pid.py, moved out of LivePID._pid_send / _ps_out.

Pure restructuring of play-single-r1 (fix33): same steps, same order, same calls, same exceptions. Per command:
  1. ops07 centre hold (orin_continuous + holding): target = centre, PWM 0, no wheel PWM, never compensated;
  2. record the PRE-compensation offset in cmd_comp's in-flight sum (cmd_comp.sent);
  3. play compensation (play_comp.PlayComp.apply on the target, which also clamps it to min_us..max_us) when the caller passes the
     logged proposal `pc` (hold-type frames and normal frames; None = send as is: ops07 hold, start-centring, WARMUP at the centre,
     external callers); the proposal keeps its pre-compensation steer_us and gets the play_comp info;
  4. send (manual.pid_input); a refused command after this run's manual session ended stops the run (manual-end-r1).
Rate limits: none in this stage. The ones that bind sit outside live_pid and are unchanged: manual_drive's steer rate (1500 us/s
from the PID document's steer_rate) and the MCU slew (2000 us/s); the controller's own limiter acts on its own output upstream.

Config: output_stage.json (next to this file, read once per run in LivePID.start through play_comp_config()):
  play_comp_config  the json file the play-compensation values are read from (default "play_comp.json", i.e. unchanged: the user
                    keeps tuning play_comp.json; its content goes to PlayComp exactly as play_comp.load_config() built it).
Python 3 standard library + play_comp (already in web/)."""
import json, time
import pause_clock   # pause-clock-r1 (pause-resume-20261120)
from pathlib import Path
import play_comp
try: import output_unify   # output-unify-r1 (output-unify-20261117); missing module -> unchanged output stage
except Exception: output_unify = None

REVISION = 'output-stage-r1'   # + output-unify-r1 hook (unify_start / _unify_pre / _unify_comp)
CONFIG_FILE = Path(__file__).with_name('output_stage.json')
DEFAULTS = dict(play_comp_config='play_comp.json')


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


def play_comp_config(cfg=None):
    """the PlayComp config dict, built exactly like play_comp.load_config() (DEFAULTS, then the file; unreadable file -> DEFAULTS)
    but from the file output_stage.json names."""
    cfg = load_config() if cfg is None else cfg
    c = dict(play_comp.DEFAULTS)
    try: c.update(json.loads(Path(__file__).with_name(str(cfg.get('play_comp_config') or 'play_comp.json')).read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


def compensate(lp, p, pc, single):
    """step 3 (was LivePID._ps_out): the one steering-play compensation, applied to the dispatched target only. pc = the logged
    proposal of this command (gets the play_comp info; its steer_us stays the pre-compensation value). Error -> sent uncompensated."""
    try:
        pcm = getattr(lp, 'pcomp', None)
        if pcm is None or not pcm.c.get('enabled'): return p
        rp = lp.run['parameters']; out, info = pcm.apply(p['target_us'], rp['center_us'], rp['min_us'], rp['max_us'])
        if info is None: return p
        pc['play_comp'] = dict(info, single=single, stage='output')
        return dict(p, target_us=out)
    except Exception as e:
        pc['play_comp'] = dict(revision=play_comp.REVISION, single=single, stage='output', reason='ERROR', error=type(e).__name__); return p


def unify_start(lp):
    """output-unify-r1: once per run in LivePID.start, right after _ps_init. Sets lp.ounify (None = this stage exactly as before:
    no file, unreadable file, enabled false, no play-comp object, or any error)."""
    lp.ounify = None
    if output_unify is None: return
    try:
        c = output_unify.load_config()
        if c is None or not c.get('enabled'): return
        pcm = getattr(lp, 'pcomp', None)
        if pcm is None: return
        lp.ounify = output_unify.OutputUnify(c, pcm.c)
    except Exception: lp.ounify = None


def _unify_pre(lp, ou, p, pc):
    """output-unify-r1 stage 1 (before cmd_comp.sent): the time-based treatment of the dispatched command. Returns (p, log dict)."""
    try:
        byp = output_unify.bypass_reason(pc, tuple(ou.c.get('bypass_reasons') or ()))
        t = (getattr(lp, 'latest', None) or {}).get('capture_at')
        pre = float(p['target_us']); s = ou.stage1(t, pre, byp is not None)
        info = dict(revision=output_unify.REVISION, method=ou.method, pre_us=round(pre, 1), smoothed_us=round(s, 1), bypass=byp,
                    dt_s=None if getattr(ou, '_dt', None) is None else round(ou._dt, 4))
        return dict(p, target_us=s), info
    except Exception as e:
        return p, dict(revision=output_unify.REVISION, reason='ERROR', error=type(e).__name__)


def _unify_comp(lp, ou, p, pc, single, info):
    """output-unify-r1 stage 2: the play compensation decided by output_unify (replaces PlayComp.apply on this path)."""
    try:
        rp = lp.run['parameters']; byp = info.get('bypass') is not None
        out, ci = ou.stage2(p['target_us'], rp['center_us'], rp['min_us'], rp['max_us'], byp)
        pc['play_comp'] = dict(revision=play_comp.REVISION, single=single, stage='unify', raw_us=round(float(p['target_us']), 1),
                               offset_us=ci['offset_us'], direction=ci['direction'], reversal=ci['reversal'], reversals=ou.reversals, steer_us=round(out, 1))
        info.update(sent_us=round(out, 1), offset_us=ci['offset_us'], reversal=ci['reversal'])
        if 'basis_us' in ci: info['basis_us'] = ci['basis_us']
        if 'wheel_model_us' in ci: info['wheel_model_us'] = ci['wheel_model_us']
        pc['output_unify'] = info
        return dict(p, target_us=out)
    except Exception as e:
        pc['output_unify'] = dict(info, reason='ERROR', error=type(e).__name__); return p


def send(lp, p, pc, ops07, single):
    """steps 1-4 (was LivePID._pid_send). lp = the LivePID; ops07 = its ops07_runtime module (continuous()); single = live_pid's PS_REVISION (logged in play_comp.single). Returns True when sent,
    False when the run was stopped because the manual session ended; other pid_input errors propagate (as before)."""
    if ops07.continuous(lp.run) and lp.ops07.holding:
        p = dict(p, target_us=lp.run['parameters']['center_us'], pwm=0)
        p.pop('pwm_right', None); p.pop('pwm_left', None); pc = None   # play-single-r1: the ops07 centre hold is never compensated
    ou = getattr(lp, 'ounify', None); oui = None   # output-unify-r1: None -> the path below is unchanged
    if ou is not None: p, oui = _unify_pre(lp, ou, p, pc)
    try:   # cmd-comp-r1: the steering actually sent feeds the in-flight command sum
        if getattr(lp, 'ccomp', None) is not None and lp.run: lp.ccomp.sent(pause_clock.monotonic(), float(p['target_us']) - float(lp.run['parameters']['center_us']))
    except Exception: pass
    if pc is not None:
        if ou is not None and oui is not None and 'reason' not in oui: p = _unify_comp(lp, ou, p, pc, single, oui)   # output-unify-r1
        else: p = compensate(lp, p, pc, single)   # play-single-r1: after cmd_comp.sent (pre-compensation), right before pid_input
    try: lp.manual.pid_input(p); return True
    except ValueError:
        info = lp._manual_ended()
        if info is None: raise
    with lp.lock:
        if lp.run and lp.run['state'] == 'running':
            lp.run['manual_session'] = info
            try: lp.log.write(json.dumps({'kind': 'manual_session_end', 'wall': time.time(), **info}) + '\n')
            except Exception: pass
    lp.end(info['stop_reason']); return False
