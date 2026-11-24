"""output-unify-r1 (output-unify-20261117, offline candidate): one time-based treatment of the steering command of every segment,
right before the steering-play compensation, in output_stage.send.

Problem (inventory-20261117, measured): at 30 frames/s the per-frame jitter of the pre-compensation command (median |delta| 5-29 us)
keeps re-triggering play_comp's direction detector (threshold 15-30 us on the raw per-frame value). Each trigger moves the sent
command by 2 x 55 us, i.e. 53-86 % of the sent command's per-frame jitter is play_comp flipping. The detector was tuned at 10 frames/s.

Methods (web/output_unify.json "method"; model comparison in output-unify-20261117/sim):
  smooth     (B, recommended: smooth_tau_s 0.05, threshold_us 10) see below.
  dirfilter  (A)  the DIRECTION of the play compensation is decided on a first-order low-pass (time constant tau_s,
             seconds, independent of the frame rate) of the pre-compensation command; the command itself is sent unfiltered:
             sent = pre + offset(direction of the filtered command). No phase lag on the command; only the moment the compensation
             changes side moves (by about tau_s on a real reversal).
  smooth     (B) the command itself is low-passed (smooth_tau_s) and optionally rate-limited (rate_us_s); the direction is decided on
             the smoothed command; sent = smoothed + offset. Adds lag to every steering change (bypass rows excepted).
  wheel      (C) a wheel-position model with backlash (play_model_us full width): the compensation side is the side the wanted
             position lies on relative to the modelled wheel, with a dead band db_us; inside the dead band the side is kept.
Bypass (immediate commands, never filtered; the filter state is reset to the command so leaving the bypass does not jump):
  no proposal (ops07 centre hold, start centring, WARMUP at the centre: these are not compensated at all, as before),
  hold-type rows of the lost policy (valid False: UPPER_SINGLE_HOLD, LANE_BLIND_HOLD, HEADING_BLIND, PID_HOLD, DOWNHILL_HOLD,
  LOST_HEADING, LOST_TURN and their gyro/profile modifiers), WARMUP (held), the dashed-row TURN (proposal.dash_turn present),
  the heading-hold TURN (heading_hold.reason TURN), start centring not done, and the reasons listed in bypass_reasons.
  On a bypass row the direction is decided on the raw command exactly as play_comp does.
Frame time: the capture time of the frame being sent (LivePID.latest.capture_at); gaps > dt_max_s reset the filter.
Config: web/output_unify.json, read once per run start (LivePID.start -> output_stage.unify_start). Missing file, unreadable file,
enabled false, or play_comp disabled for the method that only acts on the compensation -> no object (output_stage path unchanged).
Python 3 standard library + play_comp."""
import json, math
from pathlib import Path
import play_comp

REVISION = 'output-unify-r1'
CONFIG_FILE = Path(__file__).with_name('output_unify.json')
DEFAULTS = dict(enabled=False, method='smooth', tau_s=0.05, threshold_us=10., smooth_tau_s=0.05, rate_us_s=None,
                play_model_us=110., db_us=None, dt_max_s=0.3, bypass_reasons=[], log=True)
HOLD_ROWS = ('UPPER_SINGLE_HOLD', 'LANE_BLIND_HOLD', 'HEADING_BLIND', 'PID_HOLD', 'DOWNHILL_HOLD', 'LOST_HEADING', 'LOST_TURN')


def load_config(path=None):
    c = dict(DEFAULTS)
    try: c.update(json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8')))
    except (OSError, ValueError): return None
    return c


def bypass_reason(pc, extra=()):
    """why this command must not be filtered (None = filter). pc = the logged proposal (None for uncompensated commands)."""
    if pc is None: return 'NO_PROPOSAL'
    r = pc.get('reason')
    if pc.get('valid') is False: return 'HOLD_ROW'
    if r in HOLD_ROWS: return 'HOLD_ROW'
    if r == 'WARMUP' or pc.get('warmup_held'): return 'WARMUP'
    if isinstance(pc.get('dash_turn'), dict): return 'DASH_TURN'
    hh = pc.get('heading_hold')
    if isinstance(hh, dict) and hh.get('reason') == 'TURN': return 'HEADING_TURN'
    sc = pc.get('start_centre')
    if isinstance(sc, dict) and sc.get('done') is False: return 'START_CENTRE'
    if r in extra: return 'REASON'
    return None


class OutputUnify:
    def __init__(self, cfg, pc_cfg):
        self.c = dict(DEFAULTS); self.c.update(cfg or {})
        self.pcfg = dict(play_comp.DEFAULTS); self.pcfg.update(pc_cfg or {})
        th = self.c.get('threshold_us')
        if th is not None: self.pcfg['threshold_us'] = float(th)
        self.method = str(self.c.get('method') or 'dirfilter')
        if self.method not in ('dirfilter', 'smooth', 'wheel'): raise ValueError('method')
        self.comp_on = bool(self.pcfg.get('enabled'))
        self.det = play_comp.PlayComp(dict(self.pcfg, enabled=True))   # direction detector (same state machine as play_comp)
        self.f = None; self.s = None; self.t = None; self.lim_t = None
        self.wm = None; self.side = 0; self.reversals = 0; self.last_sent = None

    def _alpha(self, dt, tau):
        if tau is None or tau <= 0: return 1.
        return 1. - math.exp(-dt / tau)

    def stage1(self, t, pre, bypass):
        """before cmd_comp.sent: returns the command that is dispatched (before compensation). Only 'smooth' changes it."""
        pre = float(pre)
        dt = None if (self.t is None or t is None) else float(t) - self.t
        gap = dt is None or dt <= 0 or dt > float(self.c.get('dt_max_s') or 0.3)
        if t is not None: self.t = float(t)
        if bypass or gap:
            self.f = pre; self.s = pre; self._dt = dt; self._gap = gap; return pre
        self._dt = dt; self._gap = False
        self.f = self.f + self._alpha(dt, float(self.c.get('tau_s') or 0.)) * (pre - self.f)
        if self.method != 'smooth': return pre
        s = self.s + self._alpha(dt, float(self.c.get('smooth_tau_s') or 0.)) * (pre - self.s)
        r = self.c.get('rate_us_s')
        if r: mx = float(r) * dt; s = max(self.s - mx, min(self.s + mx, s))
        self.s = s
        return s

    def stage2(self, cmd, center, mn, mx, bypass):
        """the play compensation on the dispatched command. Returns (sent, info)."""
        cmd = float(cmd)
        if not self.comp_on: return max(float(mn), min(float(mx), cmd)), dict(offset_us=0., direction=0, reversal=False)
        if self.method == 'wheel':
            h2 = float(self.c.get('play_model_us') or 110.) / 2.
            db = float(self.c['db_us']) if self.c.get('db_us') is not None else float(self.pcfg.get('threshold_us', 15.))
            if self.wm is None: self.wm = float(center)
            if self.last_sent is not None:   # modelled wheel follows the last sent command through the backlash
                if self.last_sent > self.wm + h2: self.wm = self.last_sent - h2
                elif self.last_sent < self.wm - h2: self.wm = self.last_sent + h2
            d0 = 0. if bypass else db
            if cmd > self.wm + d0: new = 1
            elif cmd < self.wm - d0: new = -1
            else: new = self.side
            rev = new != self.side and self.side != 0
            if new != self.side: self.reversals += rev
            self.side = new
            off = float(self.pcfg.get('right_us', 0.)) if new > 0 else (-float(self.pcfg.get('left_us', 0.)) if new < 0 else 0.)
            out = max(float(mn), min(float(mx), cmd + off)); self.last_sent = out
            return out, dict(offset_us=round(off, 1), direction=new, reversal=bool(rev), wheel_model_us=round(self.wm, 1))
        basis = cmd if bypass else (self.f if self.method == 'dirfilter' else cmd)   # smooth: cmd is already the smoothed command
        _, info = self.det.apply(basis, center, mn, mx)
        off = float(info['offset_us']); out = max(float(mn), min(float(mx), cmd + off)); self.last_sent = out
        self.reversals = self.det.reversals
        return out, dict(offset_us=round(off, 1), direction=info['direction'], reversal=bool(info['reversal']), basis_us=round(basis, 1))
