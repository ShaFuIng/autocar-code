"""IMU heading hold from the dashed-line turn to the downhill (heading-hold-r1).

User 2026-11-01: after the left turn at the dashed line the car must be
straight before the next (down) ramp. Live cd6f0c63: the turn completed
~90 deg (gyro z +40 in TURN, +50 while following the dashes), then the
dashed line was lost and the normal path in the two-lane section turned the
car 36 deg right before the downhill.

The reference heading is the yaw at the start of the dashed-row TURN plus
TURN_DEG (the dashed lanes run at a right angle to the approach; cd6f0c63
integrated +90.3 deg). While the route window is open and the TURN is over:
 - following the dashed line: the PID steering gets a small heading term
   (K_FOLLOW us/deg, at most LIM_FOLLOW);
 - otherwise (dashes not tracked): steering = centre - K_HOLD * heading error
   (at most LIM_HOLD from centre), i.e. the visual path no longer steers.
It ends when the route window closes (downhill <= -5 deg or 60 s). Yaw comes
from imu-logger-r4 (integrated gyro z, left positive). Stale or missing IMU
data -> PID steering unchanged. Gains and limits are experiment values; a
steering bias leaves a steady heading offset of bias / K_HOLD.
"""
import pause_clock   # pause-clock-r1 (pause-resume-20261120)
import json, math
from pathlib import Path
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

REVISION = 'heading-hold-r20'
TURN_DEG, K_FOLLOW, LIM_FOLLOW, K_HOLD, LIM_HOLD = 90., 4., 100., 12., 300.
# r2 (user 2026-11-01 accepted relaxing the stop rule here): live 29b9fd2b / 8f2e7fbc stopped in the
# two-lane section while the heading hold was steering, because the visual path was invalid. After
# the turn, while the window is open and the dashes are not tracked, an invalid visual path no longer
# ends the run: blind() gives the heading-only steering for up to BLIND_MAX_S without a valid frame.
# IMU loss, window end (downhill / 60 s), emergency stop and faults still stop. Experiment value.
BLIND_MAX_S = 6.
# r3: live 69f00708 turned slowly (user; the area is wide, overshooting is fine): after TURN the
# heading-only steering was capped at 300 us (1280) while TURN used 1132 (448 us), ~17 deg/s. The
# heading-only limit is now the same as the TURN floor, and while the heading error is above ALIGN_DEG
# the dashed line is not followed yet (heading-only first, FOLLOW bias once roughly aligned).
LIM_HOLD, ALIGN_DEG = 448., 30.
# r13 (user 2026-11-10 bbdb / 62fa: the dashed turn now overshoots; blind hold at 942 us (448) for 1.0-1.7 s, heading
# overshoot -12..-19 deg then a +-15 deg swing): 448 -> 360 (between the r3 300 'too slow' and 448). Experiment value.
LIM_HOLD = 360.
# r15 (user 2026-11-10 5262: the car weaves across the dashed line every 3-4 s, e -0.75 / +0.77; the camera PID moved the
# command 1028 -> 1714 while the heading term was only +-100): while following the dashes the camera part is scaled by
# CAM_FOLLOW around the centre and the heading term (towards the measured dash direction) is stronger. Experiment values.
CAM_FOLLOW, K_FOLLOW, LIM_FOLLOW = .6, 6., 150.
# r16 (user 2026-11-10 16:0x: the upper turn feels too much): reference = pre-turn mean + 80 deg (was 90). Experiment value.
TURN_DEG = 75.   # r20 (user 11-10 17:3x): 85 -> 75, the steering cannot unwind at the target so it overshoots ~10-15 deg (981f peaks 95/100)
# r4: live 64ecef58 drove to the right edge of the two-lane area (user paused it). The TURN started at
# the right end of the post-crest S swing (yaw -693, road about -685), so 'TURN-start yaw + 90' was ~8 deg
# right; heading-only held it while the valid normal path moved steadily left (image error -0.13 -> -1.0).
# 76a4d787 needed a reference ~10 deg the other way. Now:
#  - the reference is the MEAN yaw of normal-path frames in the REF_WIN_S before the TURN (+90); with
#    fewer than REF_MIN_N such frames the TURN-start yaw is used as before;
#  - heading-only frames with a valid normal path (roughly aligned, |heading error| <= ALIGN_DEG) move
#    the reference toward the path: ref -= K_REF * image_error * dt (image error < 0 = path left = more
#    yaw), at most REF_CAP_DEG from the starting reference. Dashed-line FOLLOW / blind frames unchanged.
# Experiment values.
REF_WIN_S, REF_MIN_N, K_REF, REF_CAP_DEG, REF_DT_MAX = 3., 5, 5., 15., .2
# r5 (user 2026-11-02: relax the stop protection, data even if the car goes wrong): live d14f0103
# stopped at the top of the downhill: the route window closed at pitch -5 (heading blind ends there),
# the path was invalid for 0.55 s > the 0.4 s hold -> PID_INVALID_OR_STALE_PATH. Anywhere in the run,
# once the 0.4 s hold has expired, lost() keeps the yaw of the last valid frame (straight on) for up to
# LOST_MAX_S more without a valid path: steering = centre - K_HOLD * yaw error, at most LIM_LOST from
# centre. Stale camera frames, IMU loss, emergency stop and faults still stop. Experiment values.
LOST_MAX_S, LIM_LOST = 3., 200.
# r6 (user 2026-11-02, live fe533e15): in the two-lane section follow the dashed centre line, and when the
# dashes end keep going straight. fe533e15 alternated between the dashes (FOLLOW) and the normal path
# (HEADING_VISUAL): image error +0.8 / -0.9, heading -20..+7 deg, the visual correction at its 15 deg cap.
# Now: while the dashes are followed (FOLLOW, roughly aligned) the reference moves toward the current yaw
# with time constant TAU_DASH (ref = direction of the dashed line), at most DASH_CAP_DEG from the start
# reference; without dashes the heading-only steering keeps that reference (no visual correction,
# K_VIS_ON False). Experiment values.
TAU_DASH, DASH_CAP_DEG, K_VIS_ON = 1., 30., False
# r7 (user 2026-11-02, live c1f62feb): after the sharp dash TURN the dash following swung +-12 deg
# (period ~3.5 s) and the dashes ended with the car 10 deg left; heading-only at 12 us/deg gave ~1620 us,
# only ~1.4 deg/s (small steering offsets turn the car weakly: steady 1 s windows 1601 us -1.4, 1673 us
# -2.2 deg/s), error -10 -> -6 deg in 4.8 s while the car moved to the left edge. Now:
#  - heading-only / blind / lost gain K_HOLD 12 -> 25 us/deg;
#  - heading-only adds an integral K_I_HOLD us per deg*s (at most LIM_I_HOLD), only while roughly aligned;
#  - dash following adds yaw-rate damping: steering + K_D_DASH * gyro z (left positive), at most LIM_D_DASH.
# Experiment values.
K_HOLD = 25.
K_I_HOLD, LIM_I_HOLD, K_D_DASH, LIM_D_DASH = 4., 120., 5., 120.
# r8 (user 2026-11-02, live 3917715e): a big steering command in the two-lane section is followed by a big
# swing. With lane_cap set (crest option, LANE_CAP_US) the steering stays within centre +- lane_cap once
# roughly aligned (|heading error| <= ALIGN_DEG), in step() and blind(); the TURN and the alignment after it
# keep the full range. Experiment value.
LANE_CAP_US = 250.
# r9 (live 2155b25b): after the TURN the heading-only alignment held the full 448 us (1052) for 2.6 s
# (heading error 93 -> 33 deg) and arced the car toward the left lane, then overshot 12 deg left. With
# lane_cap set (crest option) the alignment steering is limited to ALIGN_LIM_US (wider arc, the car stays
# right), and the lane target takes over from ALIGN_DEG_LANE instead of ALIGN_DEG. Experiment values.
ALIGN_LIM_US, ALIGN_DEG_LANE = 300., 40.
# r10 (live d61509a9): while following the dashes the r6 reference tracked the car's yaw, and the car had
# turned 15 deg left of the dashed line (yaw -933 -> -918) -> after the dashes the heading hold kept that wrong
# direction and the car drifted to the left edge. The dashed-line direction is now measured from the image:
# the fitted line crosses the horizon row HORIZON_Y at x_h; a line parallel to the car meets it at VP_X, so
# dash direction = yaw - atan((x_h - VP_X) / F_PX) (left positive). d61509a9 / 2155b25b / c1f62feb FOLLOW
# frames: yaw spread 15-30 deg, this estimate IQR 1-5 deg. VP_X from the straight-line vanishing point of
# 2026-11-02 (330-333), F_PX from the 66 deg horizontal FOV estimate (not calibrated).
HORIZON_Y, VP_X, F_PX = 60., 333., 493.
# r11 (live 05c8ec88): the 250 us lane cap held the correction at 1750 us while the car rotated 30 deg past
# the reference (this car turns only 2-4 deg/s at 1700-1750, ~15 deg/s from 1800) -> crest option cap 450 us.
# expected_rel() gives dash-route the expected dashed-line direction; a measured dash direction more than
# DIR_GATE_DEG from the reference does not move the reference.
LANE_CAP_US, DIR_GATE_DEG, DIR_GATE_WIDE_DEG = 450., 10., 25.
# The gate is centred on the median of the last accepted dash directions (>= 3), else on the reference with
# the wide gate (the start reference can be ~10 deg off: 05c8ec88 15.9-16.2 s real dashes 10-13 deg off).


class HeadingHold:
    def __init__(self, boost):
        self.boost = boost; self.ref = None; self.done = False; self.hist = []; self.ref0 = None; self.ref_t = None; self.track = None; self.i_us = 0.; self.lane_cap = None; self.dd_hist = []

    def _yaw(self, now):
        d = json.loads(Path(self.boost.c['latest_path']).read_text(encoding='utf8'))
        t, yaw = pause_clock.imu_t(float(d['t'])), float(d['yaw_deg'])   # pause-clock-r1
        if not (math.isfinite(t) and math.isfinite(yaw)): raise ValueError('non-finite yaw')
        if not -.05 <= now - t <= self.boost.c['max_age_s']: raise ValueError('stale yaw')
        return yaw

    def on_turn_start(self, now):
        if self.ref is None and not self.done:
            h = [y for t, y in self.hist if 0 <= now - t <= REF_WIN_S]
            try: base = sum(h) / len(h) if len(h) >= REF_MIN_N else self._yaw(now)
            except Exception: return
            self.ref = self.ref0 = base + TURN_DEG; self.ref_src = 'PRE_TURN_MEAN' if len(h) >= REF_MIN_N else 'TURN_START'; self.ref_n = len(h)

    def _dir_gate(self):
        if _RI.ON:   # rate-indep-r1: >= 3 and the last 5 entries by weight (= today at nominal frame intervals)
            ww = _RI.sync(self.dd_hist, getattr(self, 'dd_w', None))
            if sum(ww) >= _RI.thr(3): t_ = _RI.tail_by_weight(self.dd_hist, ww, 5); return sorted(t_)[len(t_) // 2], DIR_GATE_DEG
            return self.ref, DIR_GATE_WIDE_DEG
        if len(self.dd_hist) >= 3: return sorted(self.dd_hist[-5:])[len(self.dd_hist[-5:]) // 2], DIR_GATE_DEG
        return self.ref, DIR_GATE_WIDE_DEG

    def expected_rel(self, now):
        """(yaw - expected dash direction, gate deg) for the dash-route direction gate, or None."""
        if self.ref is None or self.done: return None
        try: centre, gate = self._dir_gate(); return self._yaw(now) - centre, gate
        except Exception: return None

    def _gz(self, now):
        d = json.loads(Path(self.boost.c['latest_path']).read_text(encoding='utf8'))
        t, g = pause_clock.imu_t(float(d['t'])), float(d['gyro_dps'][2])   # pause-clock-r1
        if not (math.isfinite(t) and math.isfinite(g)) or not -.05 <= now - t <= self.boost.c['max_age_s']: raise ValueError('stale gyro')
        return g

    def blind(self, now, route_open, turning, last_valid_t, center, lo, hi):
        """Heading-only steering for a frame without a valid path, or None (-> normal stop rules)."""
        if self.ref is None or self.done or not route_open or turning or last_valid_t is None or not 0 <= now - last_valid_t <= BLIND_MAX_S: return None
        try: yaw = self._yaw(now)
        except Exception: return None
        err = self.ref - yaw; out = max(lo, min(hi, center - max(-LIM_HOLD, min(LIM_HOLD, K_HOLD * err))))
        if self.lane_cap and abs(err) <= ALIGN_DEG: out = max(center - self.lane_cap, min(center + self.lane_cap, out))
        return out, dict(revision=REVISION, active=True, reason='HEADING_BLIND', ref_deg=round(self.ref, 2), yaw_deg=round(yaw, 2), err_deg=round(err, 2), steer_us=round(out, 1), blind_s=round(now - last_valid_t, 3))

    def lost(self, now, last_valid_t, center, lo, hi):
        """Hold the yaw of the last valid frame without a valid path (after the 0.4 s hold), or None."""
        if self.track is None or last_valid_t is None or not 0 <= now - last_valid_t <= LOST_MAX_S: return None
        try: yaw = self._yaw(now)
        except Exception: return None
        err = self.track[1] - yaw; out = max(lo, min(hi, center - max(-LIM_LOST, min(LIM_LOST, K_HOLD * err))))
        return out, dict(revision=REVISION, active=True, reason='LOST_HEADING', ref_deg=round(self.track[1], 2), yaw_deg=round(yaw, 2),
                         err_deg=round(err, 2), steer_us=round(out, 1), lost_s=round(now - last_valid_t, 3))

    def step(self, now, route_open, dash, steer, center, lo, hi, normal=False, vis_err=None):
        """normal: a valid normal-path frame (no special source); vis_err: its image error."""
        info = dict(revision=REVISION, active=False)
        try: self.track = (now, self._yaw(now))   # r5: yaw of the last valid frame for lost()
        except Exception: pass
        if self.ref is None and not self.done and normal:
            try: self.hist.append((now, self._yaw(now))); self.hist = [p for p in self.hist if now - p[0] <= REF_WIN_S]
            except Exception: pass
        if self.ref is not None and not route_open: self.ref, self.done = None, True
        if self.ref is None: info.update(reason='DONE' if self.done else 'NO_REFERENCE'); return steer, info
        mode = dash.get('mode') if dash.get('used') else None
        if mode == 'TURN': info.update(reason='TURN'); return steer, info
        try: yaw = self._yaw(now)
        except Exception as e: info.update(reason='IMU_UNAVAILABLE', error=str(e)[:60]); return steer, info
        err = self.ref - yaw
        if mode == 'FOLLOW' and abs(err) <= (ALIGN_DEG_LANE if self.lane_cap else ALIGN_DEG):
            out = float(center) + CAM_FOLLOW * (float(steer) - float(center)) - max(-LIM_FOLLOW, min(LIM_FOLLOW, K_FOLLOW * err)); kind = 'FOLLOW_BIAS'   # r15
            try: damp = max(-LIM_D_DASH, min(LIM_D_DASH, K_D_DASH * self._gz(now)))   # r7: turning left (gz > 0) -> steer right
            except Exception: damp = 0.
            out += damp; info['damp_us'] = round(damp, 1); self.i_us = 0.
            if self.ref_t is not None:   # r6: the reference follows the dashed-line direction
                a = min(1., max(0., min(REF_DT_MAX, now - self.ref_t)) / TAU_DASH)
                goal = self.ref   # r12 (live 43340794): frames without a dash line fit (HOLD_LAST) no longer pull the reference to the car's yaw
                if dash.get('line_a') is not None and dash.get('line_b') is not None:   # r10: measured dash direction
                    goal = yaw - math.degrees(math.atan((dash['line_a'] * HORIZON_Y + dash['line_b'] - VP_X) / F_PX)); info['dash_dir_deg'] = round(goal, 2)
                    centre, gate = self._dir_gate()
                    if abs(goal - centre) > gate: goal = self.ref; info['dash_dir_rejected'] = True   # r11
                    elif _RI.ON: self.dd_hist, self.dd_w = _RI.trim(self.dd_hist + [goal], _RI.sync(self.dd_hist, getattr(self, 'dd_w', None)) + [_RI.w()], 7)   # rate-indep-r1
                    else: self.dd_hist = (self.dd_hist + [goal])[-7:]
                self.ref = max(self.ref0 - DASH_CAP_DEG, min(self.ref0 + DASH_CAP_DEG, self.ref + a * (goal - self.ref)))
        else:
            kind = 'HEADING_ONLY'
            if K_VIS_ON and vis_err is not None and mode is None and abs(err) <= ALIGN_DEG and self.ref_t is not None:
                dt = max(0., min(REF_DT_MAX, now - self.ref_t))
                self.ref = max(self.ref0 - REF_CAP_DEG, min(self.ref0 + REF_CAP_DEG, self.ref - K_REF * float(vis_err) * dt))
                err = self.ref - yaw; kind = 'HEADING_VISUAL'
            if self.ref_t is not None and abs(err) <= ALIGN_DEG:   # r7: integral of the heading error
                self.i_us = max(-LIM_I_HOLD, min(LIM_I_HOLD, self.i_us + K_I_HOLD * err * max(0., min(REF_DT_MAX, now - self.ref_t))))
            lim = ALIGN_LIM_US if self.lane_cap and abs(err) > ALIGN_DEG else LIM_HOLD   # r9: softer alignment in the crest option
            out = center - max(-lim, min(lim, K_HOLD * err + self.i_us)); info['i_us'] = round(self.i_us, 1)
        self.ref_t = now
        if self.lane_cap and abs(err) <= ALIGN_DEG:   # r8
            capped = max(center - self.lane_cap, min(center + self.lane_cap, out))
            if capped != out: info['lane_capped_us'] = round(out - capped, 1)
            out = capped
        out = max(lo, min(hi, out))
        info.update(active=True, reason=kind, ref_deg=round(self.ref, 2), ref0_deg=round(self.ref0, 2), ref_src=getattr(self, 'ref_src', None),
                    yaw_deg=round(yaw, 2), err_deg=round(err, 2), pid_steer_us=round(steer, 1), steer_us=round(out, 1))
        return out, info
