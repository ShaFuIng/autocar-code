"""Inner yaw-rate loop (yaw-inner-r1).

User 2026-11-07 (.515 / .517 analysis, 8 new-linkage runs): the lateral loop is error -> steering -> yaw rate -> heading ->
lateral position (two integrations) with about 0.75 s of delay, of which 0.5-0.6 s is command -> yaw (servo travel and yaw
build-up, measured on the jack: command -> servo start about 0.15 s). Such a loop is marginal at any kp/kd: the straight S
(period 2.3-3.3 s, amplitude 0.35-0.6) did not change with kp x0.67, kd 60..200, speed 15..22 or play compensation. The
pooled command -> yaw curve is linear: gz = -0.073 deg/s per us around the centre (zero at 1387 us).
[single-centre-r1 note: 1387 was the 11-07 old-linkage measurement; the code never uses it - the zero is the run's center_us.]

This module closes an inner loop on the gyro yaw rate (IMU latest.json, 50 Hz, ~0.03 s old): the outer command u (steering
offset from the centre after the PID and gain schedule) is read as a yaw-rate target r = -gain_dps_per_us * u; the term
k_us_per_dps * (r - gz) / (-gain) ... written out: term = k * (gain * u + gz), i.e. while the car has not yet responded the
command is pushed further, and a residual rotation at the centre (gz -25 deg/s at a centre crossing, the overshoot) is opposed
at once instead of 0.5 s later through the camera. In a steady curve gz ~ -gain * u and the term is ~0; the curve_ff floor is
applied after this module and still wins. The term is limited to +-cap_us; gz is low-passed over gz_tau_s. Inner loop gain
k * gain: 8 x 0.073 = 0.58 (gyro_damp's yaw mode was K 18 -> 1.3, capped at 90 us -> bang-bang; this is the proportional,
target-following version of that idea). Only on normal flat-path frames; other frames reset the filter. Values from
yaw_inner.json, read at each run start. Experiment values; closed loop not verified.
"""
import json, math
from pathlib import Path
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

REVISION = 'yaw-inner-r17'
CONFIG_FILE = Path(__file__).with_name('yaw_inner.json')
DEFAULTS = dict(enabled=False, k_us_per_dps=8., gain_dps_per_us=.073, cap_us=150., gz_tau_s=.1, max_u_us=400.,
                speed_gain=False, v_lo=.35, gain_lo=.047, v_hi=.55, gain_hi=.07,
                curve_hold=False, bend_min=10., u_min=60.,
                bend_entry=None, settle_pitch=None, settle_s=1., push_cap=None, u_straight=200.,
                climb_gain=None, climb_pitch=5.,
                crest_hold_pitch=None, crest_delay_s=2., crest_hold_s=1., cam_frac=.3, cam_cap_us=100., downhill_gain=None, down_pitch=-10., flat_pitch=3.,   # r6
                cam_rate_us_s=None, curve_cap_us=None, u_curve=200.,   # r7: crest camera rate limit, curve term cap
                pv_k=None, pv_cap=150., pv_bend=20., pv_dmin=.3, pv_gz=10., pv_near=.4, down_push=True,   # r8: entry preview, downhill no push
                heading_hold=False, hh_band=.25, hh_k_pos_dps_per_unit=8., hh_ref_rate_cap_dps=6., hh_k_h_per_s=1.5, hh_target_cap_dps=15., hh_trim_cap_us=60.,
                hh_bend_max=20., hh_bend_frames=3, hh_gz_max=10., hh_cap_us=120.,   # yaw-inner-r12: straight heading hold
                hold_far=None, curve_pull_cap=None, weave_unhold_dps=None, weave_unhold_s=1.5, map_done_unhold_margin_deg=None, term_rate_us_s=None, oppose_cap_us=None, oppose_err=.1, pv_rate_us_s=None)   # r17: rate limit on the entry preview (also its fade-out)   # r16: cap on a term that opposes the camera direction   # r15: rate limit on the inner-loop term (None = off, r14 behaviour)   # r13: opposing-term cap while a mapped curve is unfinished   # r9: no opposition while the far row says the curve goes on   # r5: inner loop on the climb with its own gain   # r4: entry hold, crest settle, straight push cap   # r3: no opposition inside a bent curve   # r2: command -> yaw gain by wheel speed (m/s)


def gain_for(c, speed):
    """r2: dps per us of steering at this wheel speed; the fixed r1 gain when speed scaling is off or the speed is unknown."""
    if not c.get('speed_gain') or speed is None: return float(c['gain_dps_per_us'])
    v, vl, vh, gl, gh = float(speed), float(c['v_lo']), float(c['v_hi']), float(c['gain_lo']), float(c['gain_hi'])
    if v <= vl: return gl
    if v >= vh: return gh
    return gl + (gh - gl) * (v - vl) / (vh - vl)


def load_config():
    c = dict(DEFAULTS)
    try: c.update(json.loads(CONFIG_FILE.read_text(encoding='utf8')))
    except (OSError, ValueError): pass
    return c


class YawInner:
    def __init__(self, config=None):
        self.c = dict(config) if config is not None else load_config(); self.lp = None; self.t = None; self.steep_t = None; self.crest_t = None; self.climb_t0 = None; self.ucam = None; self.ucam_t = None; self.hh = None; self.hh_bad = 0; self.tp = 0.; self.tp_t = None; self.pvs = 0.

    def reset(self): self.lp = None; self.t = None; self.hh = None; self.hh_bad = 0; self.tp = 0.   # r15: an off frame is a zero term; the next term ramps from 0   # yaw-inner-r12: the heading hold restarts at psi = psi_ref = 0

    def step(self, now, gz, steer, center, lo, hi, eligible=True, speed=None, bend=None, pitch=None, far=None, near=None, floor=None, in_curve=False, lock=None, lock_cfg=None, map_curve=False, no_hold=False):
        c = self.c
        if not c.get('enabled'): return steer, None
        climbing = c.get('climb_gain') is not None and pitch is not None and math.isfinite(float(pitch)) and float(pitch) >= float(c['climb_pitch'])   # r5
        p_ = float(pitch) if pitch is not None and math.isfinite(float(pitch)) else None
        down = c.get('downhill_gain') is not None and p_ is not None and p_ <= float(c['down_pitch'])   # r6
        if c.get('crest_hold_pitch') is not None and p_ is not None and p_ >= float(c['crest_hold_pitch']):   # r6: climb timing for the crest hold
            if self.crest_t is None or now - self.crest_t > float(c['crest_hold_s']): self.climb_t0 = now
            self.crest_t = now
        crest = (c.get('crest_hold_pitch') is not None and self.crest_t is not None and self.climb_t0 is not None and 0. <= now - self.crest_t < float(c['crest_hold_s'])
                 and now - self.climb_t0 >= float(c['crest_delay_s']) and (p_ is None or p_ > -5.))
        if eligible and crest: return self._crest(now, gz, steer, center, lo, hi, speed)
        self.ucam = None   # r7: the rate-limited camera command starts again at the next crest window
        if c.get('settle_pitch') is not None and pitch is not None and math.isfinite(float(pitch)) and float(pitch) >= float(c['settle_pitch']): self.steep_t = now   # r4: on the climb
        if eligible and not climbing and pitch is not None and math.isfinite(float(pitch)) and float(pitch) >= float(c['climb_pitch']): self.reset(); return steer, dict(revision=REVISION, reason='CLIMB_OFF')
        if eligible and p_ is not None and not climbing and not down and abs(p_) > float(c['flat_pitch']): self.reset(); return steer, dict(revision=REVISION, reason='TRANSITION', pitch=round(p_, 1))   # r6
        if eligible and not climbing and self.steep_t is not None and 0. <= now - self.steep_t < float(c['settle_s']): self.reset(); return steer, dict(revision=REVISION, reason='CREST_SETTLE', since_steep_s=round(now - self.steep_t, 2))
        if not eligible: self.reset(); return steer, dict(revision=REVISION, reason='NOT_ELIGIBLE')
        if gz is None or not math.isfinite(float(gz)): self.reset(); return steer, dict(revision=REVISION, reason='NO_GYRO')
        dt = 0. if self.t is None else max(0., min(.5, now - self.t)); self.t = now
        self.lp = float(gz) if self.lp is None else self.lp + (1. - math.exp(-dt / max(1e-3, float(c['gz_tau_s'])))) * (float(gz) - self.lp)
        if lock is not None and math.isfinite(float(lock)) and not climbing and not down: return self._lock(gz, steer, center, lo, hi, speed, float(lock), lock_cfg or {})   # yaw-inner-r12: mapped curve, the gyro is the master
        pv = 0.
        if (c.get('pv_k') is not None and not down and far is not None and near is not None and bend is not None and math.isfinite(float(far)) and math.isfinite(float(near))
                and math.isfinite(float(bend))):   # r8: entry preview from the far row
            d = float(far) - float(near); sd = 1. if d > 0 else -1.
            if abs(float(bend)) >= float(c['pv_bend']) and float(bend) * d < 0 and abs(d) >= float(c['pv_dmin']) and sd * (-self.lp) < float(c['pv_gz']) and abs(float(near)) <= float(c['pv_near']):
                pv = max(-float(c['pv_cap']), min(float(c['pv_cap']), float(c['pv_k']) * d))
        pv_req = pv
        if c.get('pv_rate_us_s') is not None:   # r17 (2026-11-10 4e52: preview 0 -> 121 -> 150 us within 0.2 s before a curve, then 150 -> 0 in one frame at FLOOR)
            mxp = float(c['pv_rate_us_s']) * (dt if dt > 0. else .1); pv = self.pvs + max(-mxp, min(mxp, pv - self.pvs))
        self.pvs = pv
        steer = float(steer) + pv
        u = max(-float(c['max_u_us']), min(float(c['max_u_us']), float(steer) - float(center)))   # the target saturates like the car's yaw rate does
        g = float(c['climb_gain']) if climbing else (float(c['downhill_gain']) if down else gain_for(c, speed)); target = -g * u
        if c.get('heading_hold'):   # yaw-inner-r12: straight heading hold, the camera only moves the heading reference outside hh_band
            hh = self._hold(gz, dt, steer - pv, center, lo, hi, pv, speed, bend, near, floor, climbing, down)
            if hh is not None: return hh
        fl = False
        if floor is not None and math.isfinite(float(floor)) and not down and not climbing:   # r10: curve map - keep at least this turn rate
            if float(floor) < 0 and target > float(floor): target = float(floor); fl = True
            elif float(floor) > 0 and target < float(floor): target = float(floor); fl = True
        cap = float(c['cap_us']); term = max(-cap, min(cap, float(c['k_us_per_dps']) * (self.lp - target)))   # = k * (gain*u + gz)
        hold = False
        if no_hold: pass   # r14 weave mode: a swing or a finished mapped curve keeps the full gyro damping (no curve / far hold)
        elif (c.get('curve_hold') and bend is not None and math.isfinite(float(bend)) and abs(u) > float(c['u_min']) and u * self.lp < 0
                and abs(float(bend)) >= float(c['bend_min']) and float(bend) * self.lp > 0 and term * self.lp > 0):
            hold = True; term = 0.   # r3: turning the commanded way inside a bent curve -> do not oppose (push stays allowed)
        elif (c.get('curve_hold') and c.get('bend_entry') is not None and bend is not None and math.isfinite(float(bend)) and abs(u) <= float(c['u_min'])
                and abs(float(bend)) >= float(c['bend_entry']) and float(bend) * self.lp > 0 and term * self.lp > 0):
            hold = True; term = 0.   # r4: curve entry - the command is still near the centre but the road and the car already turn
        far_hold = False
        if (in_curve and not hold and not no_hold and c.get('hold_far') is not None and far is not None and math.isfinite(float(far)) and float(far) * (-self.lp) > 0
                and abs(float(far)) >= float(c['hold_far']) and term * self.lp > 0):
            far_hold = True; hold = True; term = 0.   # r9: the far row still turns the way the car rotates -> the curve goes on
        capped = False
        if c.get('push_cap') is not None and abs(u) < float(c['u_straight']) and term * u > 0 and term * self.lp <= 0 and abs(term) > float(c['push_cap']):
            term = math.copysign(float(c['push_cap']), term); capped = True   # r4: straight push cap
        if down and not c.get('down_push', True) and term * u > 0 and term * self.lp <= 0: term = 0.   # r8: on the slope oppose a rotation, never push
        pcap = False
        if c.get('curve_pull_cap') is not None and map_curve and term * self.lp > 0 and abs(term) > float(c['curve_pull_cap']):
            term = math.copysign(float(c['curve_pull_cap']), term); pcap = True   # r13: bd78 95-108 s, -120 pulls turned the car left inside a right curve
        ccap = False
        if c.get('curve_cap_us') is not None and abs(u) >= float(c['u_curve']) and abs(term) > float(c['curve_cap_us']):
            term = math.copysign(float(c['curve_cap_us']), term); ccap = True   # r7: curve term cap
        ocap = False
        if (c.get('oppose_cap_us') is not None and near is not None and math.isfinite(float(near)) and abs(float(near)) >= float(c['oppose_err'])
                and term * float(near) < 0 and abs(term) > float(c['oppose_cap_us'])):   # r16 (2026-11-10 c87f 10.0-10.6 s: camera e -0.25..-0.03 wanted left, the term added +57..+76 us right and turned the car into the tunnel)
            term = math.copysign(float(c['oppose_cap_us']), term); ocap = True   # image error > 0 = target right = larger us; a term of the other sign opposes the camera
        term_req = term; rl = False
        if c.get('term_rate_us_s') is not None:   # r15 (2026-11-09, a069/9515/3fb4: the term flips +-120 us between frames at hold toggles and curve ends, ~1/3 of the >=150 us steering jumps)
            mx = float(c['term_rate_us_s']) * (dt if dt > 0. else .1); d_ = term - self.tp
            if abs(d_) > mx: term = self.tp + math.copysign(mx, d_); rl = True
        self.tp = term
        out = max(float(lo), min(float(hi), float(steer) + term))
        return out, dict(revision=REVISION, reason='OK', preview_req_us=round(pv_req, 1), term_req_us=round(term_req, 1), term_rate_limited=rl, oppose_capped=ocap, no_hold=bool(no_hold), curve_capped=ccap, pull_capped=pcap, preview_us=round(pv, 1), far_hold=far_hold, curve_floor=fl, gz=round(float(gz), 2), gz_lp=round(self.lp, 2), target_dps=round(target, 2), u_us=round(u, 1), gain=round(g, 4), speed_mps=None if speed is None else round(float(speed), 3), term_us=round(term, 1), curve_hold=hold, push_capped=capped, climbing=climbing, downhill=down, bend_deg=None if bend is None else round(float(bend), 1), pid_steer_us=round(float(steer), 1), steer_us=round(out, 1))

    def _lock(self, gz, steer, center, lo, hi, speed, lock, lc):
        """r12 curve yaw-rate lock: feed-forward steering for the map's yaw rate + a capped camera trim + a gyro term on the rate error."""
        c = self.c; self.hh = None; g = float(lc['lock_gain_dps_per_us']) if lc.get('lock_gain_dps_per_us') else gain_for(c, speed); mu = float(c['max_u_us'])   # 1ca9 FLOOR frames: -27.7 dps from +240 us = 0.115, not the straight 0.07
        ff = max(-mu, min(mu, -lock / g))   # lock < 0 (right) -> +us; limited to the measured linear range like the r11 target
        tc = float(lc.get('trim_cap_us', 100.)); ut = max(-tc, min(tc, float(steer) - float(center)))
        lcap = float(lc.get('lock_cap_us', 150.)); term = max(-lcap, min(lcap, float(lc.get('lock_k_us_per_dps', 6.)) * (self.lp - lock)))
        self.tp = term   # r15: other modes hand their term to the limiter
        out = max(float(lo), min(float(hi), float(center) + ff + ut + term))
        return out, dict(revision=REVISION, reason='OK', mode='CURVE_LOCK', lock_dps=round(lock, 2), ff_us=round(ff, 1), trim_us=round(ut, 1), term_us=round(term, 1), gz=round(float(gz), 2),
                         gz_lp=round(self.lp, 2), gain=round(g, 4), speed_mps=None if speed is None else round(float(speed), 3), pid_steer_us=round(float(steer), 1), steer_us=round(out, 1))

    def _hold(self, gz, dt, steer, center, lo, hi, pv, speed, bend, near, floor, climbing, down):
        """r12 straight heading hold: yaw-rate target from the gyro heading error psi_ref - psi (deg, + = left); None = not a straight frame (state reset)."""
        c = self.c
        bad = bend is None or not math.isfinite(float(bend)) or abs(float(bend)) > float(c['hh_bend_max'])
        if (floor is not None and math.isfinite(float(floor))) or climbing or down or (self.hh is None and (bad or abs(self.lp) > float(c['hh_gz_max']))): self.hh = None; self.hh_bad = 0; return None
        self.hh_bad = self.hh_bad + _RI.w() if bad else 0   # bend P50 6 / P90 26 deg on recorded straights: one bent frame must not end the hold
        if self.hh_bad >= _RI.thr(int(c.get('hh_bend_frames', 3))): self.hh = None; self.hh_bad = 0; return None
        if self.hh is None: self.hh = dict(psi=0., ref=0.); dt = 0.
        h = self.hh; h['psi'] += float(gz) * dt; b = float(c['hh_band'])
        n = float(near) if near is not None and math.isfinite(float(near)) else None; inb = n is None or abs(n) <= b
        if not inb:   # near > 0 = lane centre right of the car -> the reference turns right (psi < 0)
            rc = float(c['hh_ref_rate_cap_dps']); h['ref'] += max(-rc, min(rc, -float(c['hh_k_pos_dps_per_unit']) * (n - math.copysign(b, n)))) * dt
        tc = float(c['hh_target_cap_dps']); t = max(-tc, min(tc, float(c['hh_k_h_per_s']) * (h['ref'] - h['psi'])))
        cap = float(c['hh_cap_us']); term = max(-cap, min(cap, float(c['k_us_per_dps']) * (self.lp - t)))
        tr = float(c['hh_trim_cap_us']); ut = max(-tr, min(tr, float(steer) - float(center)))
        self.tp = term   # r15: other modes hand their term to the limiter
        out = max(float(lo), min(float(hi), float(center) + ut + pv + term))
        return out, dict(revision=REVISION, reason='OK', mode='HEADING_HOLD', psi=round(h['psi'], 2), psi_ref=round(h['ref'], 2), target_dps=round(t, 2), trim_us=round(ut, 1), term_us=round(term, 1),
                         in_band=inb, preview_us=round(pv, 1), gz=round(float(gz), 2), gz_lp=round(self.lp, 2), speed_mps=None if speed is None else round(float(speed), 3),
                         bend_deg=round(float(bend), 1), pid_steer_us=round(float(steer), 1), steer_us=round(out, 1))

    def _crest(self, now, gz, steer, center, lo, hi, speed):
        """r6 crest hold: a reduced, limited camera command; the inner loop holds the yaw rate it asks for."""
        c = self.c
        if gz is None or not math.isfinite(float(gz)): return steer, dict(revision=REVISION, reason='CREST_NO_GYRO')
        dt = 0. if self.t is None else max(0., min(.5, now - self.t)); self.t = now
        self.lp = float(gz) if self.lp is None else self.lp + (1. - math.exp(-dt / max(1e-3, float(c['gz_tau_s'])))) * (float(gz) - self.lp)
        u = max(-float(c['max_u_us']), min(float(c['max_u_us']), float(steer) - float(center)))
        cc = float(c['cam_cap_us']); u_cam = max(-cc, min(cc, float(c['cam_frac']) * u))
        if c.get('cam_rate_us_s') is not None:   # r7: follow the camera at a limited rate instead of a small cap
            if self.ucam is None: self.ucam = u_cam
            else: step = float(c['cam_rate_us_s']) * max(0., min(.5, now - (self.ucam_t if self.ucam_t is not None else now))); self.ucam += max(-step, min(step, u_cam - self.ucam))
            self.ucam_t = now; u_cam = self.ucam
        g = float(c['climb_gain']) if c.get('climb_gain') is not None else gain_for(c, speed); target = -g * u_cam
        cap = float(c['cap_us']); term = max(-cap, min(cap, float(c['k_us_per_dps']) * (self.lp - target)))
        self.tp = term   # r15: other modes hand their term to the limiter
        out = max(float(lo), min(float(hi), float(center) + u_cam + term))
        return out, dict(revision=REVISION, reason='CREST_HOLD', gz=round(float(gz), 2), gz_lp=round(self.lp, 2), u_us=round(u, 1), u_cam_us=round(u_cam, 1),
                         target_dps=round(target, 2), term_us=round(term, 1), pid_steer_us=round(float(steer), 1), steer_us=round(out, 1), since_climb_s=round(now - self.climb_t0, 2))
