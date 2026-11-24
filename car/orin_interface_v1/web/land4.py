"""land4-r1 (field-20261122/landing4, offline candidate, NOT deployed): downhill -> landing -> curve-4 entry steered on the ABSOLUTE
heading of the upper straight. web/land4.json absent / enabled false / no enabled stage = live_pid builds nothing and never calls this
module (frame-identical to the car, proof in landing4/proof).

Why (23 landings 11-21 / 11-22, tools/tab*.py): the heading at the landing instant spreads -7..-17 deg from the upper-straight heading,
and the existing landing band (arc2 land) measures its turn from that instant; the slope chain is the camera PID, whose single-frame
error jumps at the slope mouth reach the servo (5 of 24 passes, one failure). The upper-straight heading itself is stable (heading at
the slope mouth 0.6 deg +- 1.3 from the mean of the last 1.2 s of the gyro-held upper straight).

One law for two stages (each stage has its own switch and gains):
  psi   = IMU yaw - ref - glitch                 (deg, left +; ref = mean IMU yaw of the last ref_s of the upper straight)
  u     = metres since the landing detection (live_pid down_level_t0); on the slope u = distance since the mouth - slope_len_m (<= u_slope_max)
          distance = rear encoder counts, extrapolated with the wheel speed between the 5 Hz MCU status updates
  th    = theta(u + x_shift) + clip(-k_cam * (e_f - e_ref(u)), +-cam_lim)     camera = limited shift of the heading target
          e_f = median-of-3 then low-passed near error of valid frames (single-frame jumps do not pass); no camera term before cam_from_m
  cmd   = centre + off_us + min(kappa(u + v*lead_s), kappa_max) / g(v) + clip(k_psi * (psi - th), +-psi_lim) + clip(k_gz * (gz + kappa(u)*v), +-gz_lim)
          limited to centre - lim_left_us .. centre + lim_right_us and to max_step_us per frame from the previous command.
          g = the stage's 'gain' (deg/m per us of pre-compensation command, tools/ident.py: landing zone 0.186 at 0.20-0.25 s delay, lower slope 0.14).
  slope stage : from the first downhill frame (pitch < -5, live_pid down_tail) to the landing; owns every frame, valid or lost
                (the jump guard / down_straight / camera chain result is not used; their states keep running).
  land stage  : from the landing to x_end, then blended back into the chain over hand_m (the chain = bottom_owner cap, curve-4 floor /
                band, arc2 landing band; with step 3 the curve-4 band starts at 1.2 m).
  A stage that is disabled is only tracked (reference, distance, logging); its command is the chain's.
Glitch: the MPU6050 gyro clips at +-250 dps on the landing impact (46c9 lap 1: one 251 dps sample = +5 deg in the logger's yaw);
a frame-to-frame yaw change that differs by more than spike_deg from (mean gyro reading of the two frames) x dt (frames <= spike_dt_max_s
apart; clipped readings not used) is replaced by that prediction and the difference is kept as an offset for this pass; never on two
consecutive frames, total limited to glitch_max_deg.
No reference (too few upper-straight samples, spread > ref_spread_deg) or IMU unreadable -> no action on this pass / frame (chain value).
Conventions: servo us larger = right; yaw / gz left positive. Standard library only; no clock / IMU / file access except load_config()."""
import json, math
from pathlib import Path

REVISION = 'land4-r1'
CONFIG_FILE = Path(__file__).with_name('land4.json')
M_PER_COUNT = 0.000222

DEFAULTS = dict(
    enabled=False, ref_s=1.2, ref_min_n=8, ref_spread_deg=8.0, ref_max_age_s=0.6, slope_len_m=2.20, u_slope_max=-0.05, x_shift_m=0.0,
    spike_deg=3.0, spike_dt_max_s=0.08, sat_dps=240.0, glitch_max_deg=12.0, spike_on=True, enc_age_s=0.0, enc_ext_max_s=0.35, v_default=1.0, lead_s=0.25, kappa_max=48.0, off_us=0.0,
    gain_table=[[0.80, 0.192], [0.93, 0.167], [1.06, 0.159], [1.22, 0.150], [1.37, 0.130]],
    theta=[[-1.7, 0.0], [0.0, -11.0], [3.0, -79.0]], kappa=[[-1.7, 0.0], [3.0, 0.0]], e_ref=[[-1.6, 0.0], [3.0, 0.0]],
    cam_tau_s=0.25, cam_hold_s=0.5, cam_gap_s=0.15,
    slope=dict(enabled=False, gain=0.14, k_psi=10.0, psi_lim=120.0, k_gz=1.5, gz_lim=60.0, k_cam=15.0, cam_lim=5.0, cam_from_m=0.4, lim_right_us=200.0, lim_left_us=150.0,
               max_step_us=40.0, out_frames=8),
    land=dict(enabled=False, gain=0.186, k_psi=10.0, psi_lim=120.0, k_gz=1.5, gz_lim=60.0, k_cam=12.0, cam_lim=8.0, lim_right_us=260.0, lim_left_us=120.0,
              max_step_us=60.0, x_end=1.2, hand_m=0.3, t_max_s=3.0))


def load_config(path=None):
    """land4.json -> merged config, or None (absent / unreadable / enabled false / no stage enabled)."""
    try: raw = json.loads(Path(path or CONFIG_FILE).read_text(encoding='utf8'))
    except (OSError, ValueError): return None
    return merge(raw)


def merge(raw):
    if not isinstance(raw, dict) or raw.get('enabled') is not True: return None
    c = dict(DEFAULTS); c.update({k: v for k, v in raw.items() if k not in ('slope', 'land')})
    for st in ('slope', 'land'):
        s = dict(DEFAULTS[st]); r = raw.get(st)
        if isinstance(r, dict): s.update(r)
        s['enabled'] = isinstance(r, dict) and r.get('enabled') is True
        c[st] = s
    if not (c['slope']['enabled'] or c['land']['enabled'] or raw.get('track_only') is True): return None
    return c


def _num(x): return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x))


def interp(t, x):
    if x <= t[0][0]: return float(t[0][1])
    if x >= t[-1][0]: return float(t[-1][1])
    for (x0, y0), (x1, y1) in zip(t, t[1:]):
        if x0 <= x <= x1: return float(y0) + (float(y1) - float(y0)) * (x - x0) / max(1e-9, x1 - x0)
    return float(t[-1][1])


def _clip(x, a): return max(-a, min(a, x))


class Odo:
    """rear-encoder distance between the 5 Hz MCU status updates: counts of the last status + wheel speed * time since it was first seen."""

    def __init__(self, age_s=0.0, ext_max_s=0.35, v_default=1.0):
        self.enc = None; self.ms = None; self.t_seen = None; self.v = None; self.age = float(age_s); self.ext = float(ext_max_s); self.vd = float(v_default)

    def feed(self, now, enc, ms):
        if not _num(enc): return
        if self.enc is None or (ms != self.ms if _num(ms) else enc != self.enc):
            if self.enc is not None and _num(ms) and _num(self.ms) and 0 < ms - self.ms <= 1000:
                v = (enc - self.enc) * M_PER_COUNT / ((ms - self.ms) / 1000.)
                if 0. <= v <= 3.: self.v = v
            elif self.enc is not None and self.t_seen is not None and now - self.t_seen > 0.05 and not _num(ms):
                v = (enc - self.enc) * M_PER_COUNT / (now - self.t_seen)
                if 0. <= v <= 3.: self.v = v
            self.enc = float(enc); self.ms = ms; self.t_seen = float(now)

    def speed(self, hint=None):
        if self.v is not None: return self.v
        return float(hint) if _num(hint) and float(hint) > 0.2 else self.vd

    def dist(self, now, hint=None):
        if self.enc is None: return None
        dt = max(0., min(self.ext, float(now) - self.t_seen + self.age))
        return self.enc * M_PER_COUNT + self.speed(hint) * dt


class Land4:
    def __init__(self, cfg):
        self.c = cfg; self.odo = Odo(cfg['enc_age_s'], cfg['enc_ext_max_s'], cfg['v_default'])
        self.buf = []          # (t, yaw) of upper-straight frames
        self.state = 'IDLE'; self._reset()

    @classmethod
    def from_file(cls, path=None):
        c = load_config(path)
        return cls(c) if c else None

    def _reset(self):
        self.ref = None; self.ref_n = 0; self.d_mouth = None; self.d_land = None; self.t_land = None; self.glitch = 0.; self.glitches = 0
        self.yprev = None; self.gprev = None; self.flagged = False; self.rate_ok = 0.; self.last = None; self.ef = None; self.e3 = []; self.e_t = None; self.fade = 0; self.why = None; self.t_prev = None

    def wants_imu(self, upper, down_tail):
        """live_pid reads the IMU for this module only on frames where this is True (upper straight / slope / landing)."""
        return bool(upper or down_tail or self.state in ('SLOPE', 'LAND'))

    # ---- helpers
    def _land_ok(self, now, v_hint):
        """land4-r2 (2026-11-22, run-54cb lap 3): the pitch crossed DOWN_STEEP twice at the slope mouth, live_pid set down_level_t0 for one
        frame, the stage switched SLOPE -> LAND 1 frame after the mouth and the landing turn was driven on the slope. With land_min_m /
        land_min_s in land4.json the switch needs that distance since the mouth OR that time on the slope (the time alone covers a dead
        odometer). Both keys absent = r1: the first level flag switches."""
        c = self.c; m = c.get('land_min_m'); s = c.get('land_min_s')
        if not _num(m) and not _num(s): return True
        if _num(s) and now - float(getattr(self, 't_slope', now)) >= float(s): return True
        if _num(m) and self.d_mouth is not None:
            d = self.odo.dist(now, v_hint)
            if d is not None and d - self.d_mouth >= float(m): return True
        return False

    def _ref_take(self, now):
        c = self.c
        ys = [y for t, y in self.buf if 0. <= now - t <= float(c['ref_s'])]
        fresh = bool(self.buf) and now - self.buf[-1][0] <= float(c['ref_max_age_s'])
        if len(ys) < int(c['ref_min_n']) or not fresh: self.why = 'NO_REF_SAMPLES'; return
        if max(ys) - min(ys) > float(c['ref_spread_deg']): self.why = 'REF_SPREAD'; return
        self.ref = sum(ys) / len(ys); self.ref_n = len(ys)

    def _despike(self, now, yaw, gz=None):
        """yaw with the impact glitches removed (see module doc). Prediction of the frame-to-frame change = mean of this and the previous
        gyro reading x dt (a clipped reading, |gz| >= sat_dps, is not used; without a usable reading: the previous frame's rate).
        Never two frames in a row, and the total removed per pass is limited to glitch_max_deg."""
        c = self.c
        gok = gz if (_num(gz) and abs(gz) < float(c['sat_dps'])) else None
        if self.yprev is not None and c.get('spike_on'):
            dt = now - self.yprev[0]
            if 0.005 < dt <= float(c['spike_dt_max_s']):
                raw = yaw - self.yprev[1]
                gp = self.gprev
                rate = (0.5 * (gok + gp) if (gok is not None and gp is not None) else (gok if gok is not None else (gp if gp is not None else self.rate_ok)))
                exc = raw - rate * dt
                if abs(exc) > float(c['spike_deg']) and not self.flagged and abs(self.glitch + exc) <= float(c['glitch_max_deg']):
                    self.glitch += exc; self.glitches += 1; self.flagged = True
                else:
                    if not self.flagged: self.rate_ok = raw / dt   # the frame right after a removed glitch does not set the fallback rate
                    self.flagged = False
            else: self.rate_ok = 0.; self.flagged = False
        self.yprev = (now, yaw); self.gprev = gok
        return yaw - self.glitch

    def _cam(self, now, valid, near, u, P, use):
        c = self.c
        if valid and _num(near):
            if self.e_t is not None and now - self.e_t > float(c['cam_gap_s']): self.e3 = []   # after a gap the median restarts (3 fresh frames before the camera moves the target again)
            self.e3.append(float(near)); self.e3 = self.e3[-3:]
            med = sorted(self.e3)[len(self.e3) // 2] if len(self.e3) >= 3 else None
            if med is not None:
                d = med - interp(c['e_ref'], u)
                dt = 0.035 if self.e_t is None else max(0.005, min(0.06, now - self.e_t))
                a = min(1., dt / max(1e-3, float(c['cam_tau_s'])))
                self.ef = d if self.ef is None else self.ef + a * (d - self.ef)
            self.e_t = now
        elif self.e_t is not None and now - self.e_t > float(c['cam_hold_s']): self.ef = None; self.e3 = []
        if not use or self.ef is None: return 0.
        return _clip(-float(P['k_cam']) * self.ef, float(P['cam_lim']))

    def _law(self, P, u, psi, gz, v, dth, c0):
        c = self.c
        g = float(P['gain']) if _num(P.get('gain')) and float(P['gain']) > 0. else interp(c['gain_table'], v)   # stage gain (deg/m per us, measured in that zone) or the lap_ff speed table
        th = interp(c['theta'], u + float(c['x_shift_m'])) + dth
        kf = min(float(c['kappa_max']), max(0., interp(c['kappa'], u + float(c['x_shift_m']) + v * float(c['lead_s']))))
        kn = max(0., interp(c['kappa'], u + float(c['x_shift_m'])))
        ff = kf / g
        hp = _clip(float(P['k_psi']) * (psi - th), float(P['psi_lim']))
        gt = _clip(float(P['k_gz']) * (gz + kn * v), float(P['gz_lim'])) if _num(gz) else 0.
        raw = float(c0) + float(c['off_us']) + ff + hp + gt
        cmd = max(float(c0) - float(P['lim_left_us']), min(float(c0) + float(P['lim_right_us']), raw))
        return cmd, dict(th=round(th, 2), psi=round(psi, 2), ff_us=round(ff, 1), head_us=round(hp, 1), gz_us=round(gt, 1), raw_us=round(raw, 1))

    def _slew(self, cmd, md, start):
        if self.last is None: self.last = float(start)
        out = self.last + _clip(cmd - self.last, float(md)); self.last = out
        return out

    # ---- one frame (valid, WARMUP or lost)
    def step(self, now, valid, near, yaw, gz, enc, ms, v_hint, upper, down_tail, down_level, steer, c0, lo, hi, last_sent=None):
        """-> (steer_us, info or None). steer = the chain's command of this frame (valid: after every other layer; lost: the lost policy's).
        yaw / gz None = IMU not read or unreadable on this frame. down_level = live_pid down_level_t0 (None until the landing)."""
        c = self.c; st = float(steer); now = float(now)
        self.odo.feed(now, enc, ms)
        if upper and _num(yaw) and self.state == 'IDLE':
            self.buf.append((now, float(yaw))); self.buf = [p for p in self.buf if now - p[0] <= float(c['ref_s']) + 1.0]
        # ---- state machine
        if self.state == 'IDLE':
            if not down_tail: return st, None
            self._reset(); self.state = 'SLOPE'; self._ref_take(now); self.t_slope = now; self.early = 0
            self.d_mouth = self.odo.dist(now, v_hint)
            if self.d_mouth is None: self.why = self.why or 'NO_ODOMETER'
            self.last = float(last_sent) if _num(last_sent) else float(c0)
        if self.state == 'DONE':
            if not down_tail: self.state = 'IDLE'; self.buf = []
            return st, None
        if not down_tail:   # the downhill window closed before the stage ended
            self.state = 'IDLE'; self.buf = []
            return st, dict(revision=REVISION, state='END', reason='WINDOW_CLOSED')
        if self.state == 'SLOPE' and down_level is not None and not self._land_ok(now, v_hint):
            down_level = None; self.early = getattr(self, 'early', 0) + 1   # land4-r2: still at the slope mouth, the level flag is a pitch bounce
        if self.state == 'SLOPE' and down_level is not None:
            self.state = 'LAND'; self.t_land = now; self.d_land = self.odo.dist(now, v_hint)
            if not c['slope']['enabled'] or self.why: self.last = None   # the land stage starts from the chain's command
        if self.why:   # no reference / odometer on this pass: nothing changes
            if self.state == 'LAND' and now - self.t_land > float(c['land']['t_max_s']): self.state = 'DONE'
            return st, dict(revision=REVISION, state='OFF', reason=self.why)
        d = self.odo.dist(now, v_hint); v = self.odo.speed(v_hint)
        psi = None
        if _num(yaw): psi = self._despike(now, float(yaw), gz) - self.ref
        dtf = None if self.t_prev is None else now - self.t_prev
        self.t_prev = now
        if self.state == 'SLOPE':
            P = c['slope']; dm = (d - self.d_mouth) if d is not None else 0.
            u = min(float(c['u_slope_max']), dm - float(c['slope_len_m']))
            dth = self._cam(now, valid, near, u, P, dm >= float(P['cam_from_m']))
            info = dict(revision=REVISION, state='SLOPE', u_m=round(u, 3), d_mouth_m=round(dm, 3), v=round(v, 3), ref=round(self.ref, 2), ref_n=self.ref_n, cam_deg=round(dth, 2),
                        e_f=None if self.ef is None else round(self.ef, 3), glitch_deg=round(self.glitch, 2), chain_us=round(st, 1))
            if psi is None:
                info.update(reason='NO_IMU', steer_us=round(st, 1), applied=False); self.last = st if P['enabled'] else None
                return st, info
            cmd, li = self._law(P, u, psi, gz, v, dth, c0); info.update(li)
            if not P['enabled']:
                info.update(shadow_us=round(cmd, 1), steer_us=round(st, 1), applied=False); return st, info
            out = max(float(lo), min(float(hi), self._slew(cmd, _step(P['max_step_us'], dtf), st)))
            info.update(steer_us=round(out, 1), applied=out != st)
            return out, info
        # ---- LAND
        P = c['land']; x = (d - self.d_land) if (d is not None and self.d_land is not None) else None
        if x is None: x = v * (now - self.t_land)
        x_end = float(P['x_end']); hand = max(1e-3, float(P['hand_m']))
        if x > x_end + hand or now - self.t_land > float(P['t_max_s']):
            self.state = 'DONE'
            return st, dict(revision=REVISION, state='END', reason='HANDED_OVER' if x > x_end + hand else 'T_MAX', x_m=round(x, 3), glitches=self.glitches, glitch_deg=round(self.glitch, 2))
        dth = self._cam(now, valid, near, x, P, True)
        info = dict(revision=REVISION, state='LAND', x_m=round(x, 3), v=round(v, 3), ref=round(self.ref, 2), cam_deg=round(dth, 2), e_f=None if self.ef is None else round(self.ef, 3),
                    glitch_deg=round(self.glitch, 2), chain_us=round(st, 1))
        if psi is None:
            info.update(reason='NO_IMU', steer_us=round(st, 1), applied=False); self.last = st if P['enabled'] else None
            return st, info
        cmd, li = self._law(P, x, psi, gz, v, dth, c0); info.update(li)
        if not P['enabled']:
            if c['slope']['enabled'] and self.last is not None and self.fade < int(c['slope']['out_frames']):   # slope stage alone: rate-limited return to the chain
                self.fade += 1; out = max(float(lo), min(float(hi), self._slew(st, _step(c['slope']['max_step_us'], dtf), st)))
                if out == st: self.last = None
                info.update(shadow_us=round(cmd, 1), steer_us=round(out, 1), applied=out != st, fade=self.fade); return out, info
            self.last = None
            info.update(shadow_us=round(cmd, 1), steer_us=round(st, 1), applied=False); return st, info
        w = 1. if x <= x_end else max(0., 1. - (x - x_end) / hand)
        tgt = st + w * (cmd - st)
        out = max(float(lo), min(float(hi), self._slew(tgt, _step(P['max_step_us'], dtf), st)))
        info.update(w=round(w, 3), steer_us=round(out, 1), applied=out != st)
        return out, info


def _step(max_step_us, dt):
    """per-frame step limit scaled with the frame interval (nominal 0.035 s; lost / late frames may move more, capped at 3 frames)."""
    if dt is None or dt <= 0.: return float(max_step_us)
    return float(max_step_us) * max(1., min(3., dt / 0.035))
