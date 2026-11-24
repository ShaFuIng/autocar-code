"""Two-lane segment controller: dashed-line extension with side-line fallback (lane-segment-r1).

User 2026-11-03 (live 43340794): after the dashed-row TURN the car must go straight along the dashed line;
its extension leads to the centre of the downhill entrance. Without dashes, use the left / right solid line.
In 43340794 the gyro-only straight ignored the image (the normal path pointed right) and left the track.

Per frame, from the dash-route info of the same camera image (640 x 360, lines x = a*y + b):
 - dashed line: the dash-route fit (FOLLOW with line_a / line_b);
 - otherwise the dashed line is estimated from a side line plus the offset learnt while both were seen:
   x_dash(y) = x_side(y) + k_side * (y - HORIZON_Y)   (perspective: the gap grows linearly below the horizon);
   both sides seen -> mean of the two estimates; k_side is a low-pass of the measured ratio (K_ALPHA).
 - heading term: rel = atan((x_dash(HORIZON_Y) - VP_X) / F_PX) [deg]; the dashed / side lines are parallel, so a
   side line gives the same direction. rel > 0: line vanishes right of the car axis -> car points left -> steer right.
 - lateral term: lat = (x_dash(row) - AXIS) / 320 at the control row (target = on the dashed line).
 - no line at all: rel from the gyro, rel = yaw - dash_dir (dash_dir = yaw - rel, learnt while a line was seen);
   lat keeps its last value and decays with TAU_LAT.
 steering = centre + K_H * rel + K_L * lat + K_D * gz      (gz gyro yaw rate, left positive)
 K_H 10 us/deg, K_L 150 us, K_D 10 us/(deg/s): the most stable set in the closed-loop sim of 2026-11-02
 (offline-review-20261102/loop_sim.py); limit +-LIM_US from centre. Experiment values.
"""
import json, math
from pathlib import Path
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

REVISION = 'lane-segment-r13'
HORIZON_Y, VP_X, F_PX, AXIS = 60., 333., 493., 338.
K_H, K_L, K_D, LIM_US, TAU_LAT, K_ALPHA, K_RANGE = 10., 150., 10., 450., 1.5, .3, (.05, 2.5)
# r13 (user 2026-11-07, live ee70dc45 89-92 s: lat 0.40-0.71 gave only 60-105 us, the car stayed left of the dashes before the
# downhill): K_L from lane_segment.json {'k_l': ...}, read at import (a control-service restart applies it); key absent = 150.
try: K_L = float(json.loads(Path(__file__).with_name('lane_segment.json').read_text(encoding='utf8')).get('k_l', K_L))
except (OSError, ValueError, TypeError, AttributeError): pass
# Default dash-to-side gap ratio until learnt: c1f62feb 61.0 s, dashes ~425 px right of the left line at row 300
# (half-gap 0.85 per row below the horizon in dash-route r7) -> 1.7; right side assumed symmetric.
K_DEFAULT = {'left': 1.7, 'right': -1.7}
# A dash fit whose lateral position disagrees with the side-line estimate by more than DASH_VS_SIDE (edge pieces
# taken as dashes, 43340794 144.5 s) is dropped; with both sides the estimate closer to the last lateral value is
# used; lat is low-passed with LAT_A.
DASH_VS_SIDE, LAT_A = .4, .5
# r2 (live 3fd37b76, 2026-11-03): right after the dashed turn the 'right side' fit was a steep cross line (a -4.4) and one
# frame taught k_right -0.40 (default -1.7); afterwards the dash estimate sat next to the right line and the car stayed in
# the right lane past the downhill entrance. Parallel lines meet at the horizon: a side line is used for learning only
# when it meets the dash fit within VP_TOL px at HORIZON_Y, and a learnt k is used only after K_MIN_N such samples.
VP_TOL, K_MIN_N = 150., 3
# r3 (live a591a0ec 109.36-109.55 s): at the crest edge the floor track below gave side lines whose direction was +27 /
# -34 deg while the gyro said the car was within a few degrees of the dash direction; the steering jumped 1662 -> 955.
# A line estimate whose direction differs from the gyro direction (yaw - dash_dir) by more than REL_GATE is not used
# (the gyro direction is used instead). Experiment value.
REL_GATE = 15.
# r4 (live b80b77e1 10.6-11.2 s, crest start): at the hand-over from the dashed TURN the car still yawed left at 22-27 deg/s
# and K_D * gz gave +220 .. +285 us (right), more than the heading term (-50 .. -200): the steering went to 1382-1583, the
# turn was not finished and the car left on the right. Same pattern in 3fd37b76 / 65c2b828. The gyro term is limited to
# +-D_CAP_US. Experiment value.
D_CAP_US = 100.
# r5 dash lock (user 2026-11-03): after the dashed turn the car must go back onto the dashed line and follow it straight to
# the downhill whatever the turn left it at. With dash_only the side lines are not used at all: the dashed line when seen,
# otherwise the gyro holding the last measured dash direction (lateral value decays). live_pid starts the lane segment right
# after the TURN (no 40 deg alignment gate) and caps the PWM at LOCK_PWM while it steers.
LOCK_PWM = 13.   # r7 (user): PWM 15 while the dashes are followed; r8 (user): 13 on the whole upper section (route OPEN)
# r7 (5 runs, 6f6e7f09 / 037eb84e / b80b77e1 / 3fd37b76 / 65c2b828): the dashed TURN covers only 13-32 deg of the 82-92 deg
# turn; with the immediate hand-over the lane segment steered mildly and the rest of the turn took seconds. live_pid keeps
# the heading-hold turn until the heading error is within LOCK_GATE_DEG, then hands over to the lane segment.
LOCK_GATE_DEG = 20.
# r6 (live 037eb84e): with dash_only the dashes left the view at 11.8 s, the gyro kept the direction and the car stayed in
# the right lane until it left the track; the right line (seen until 19 s) would have put the dashes 0.45-0.8 to the left.
# live_pid uses the side-line estimate again (dash first, gyro last); immediate hand-over, LOCK_PWM and D_CAP_US are kept.
# r9 (user 2026-11-04, live 9eedee0a / 0ef89769): without a line the gyro mode held the direction of the LAST dash fit (one
# frame; 9eedee0a -69.4, 3.5 deg right of the downhill entrance the heading-hold reference -65.9 pointed at, user-confirmed at
# 21.8 s) with K_H 10 us/deg and no integral: 1320-1400 us while the car still yawed right 2-5 deg/s, 9 deg right at 24.85 s;
# heading hold asked 1413 -> 1182. In 0ef89769 14-18 s heading hold asked 1555-1685 (right, image agreed) and the lane segment
# gave 1330-1480. Now, when live_pid passes the heading-hold reference (ref), a frame without a usable line uses the
# heading-hold steering (source 'heading'). Side lines keep the r8 REL_GATE: a tested 8 deg gate against the reference dropped
# the right line that the successful runs 65076008 / bb013bf3 followed at 22-25.5 s (car 10-19 deg left of the reference, image
# target 0.4-0.7 left) and heading hold then asked 1550-1750 us (right). Heading hold has no lateral term, so it only fills
# frames without a usable line. Without ref the r8 behaviour is unchanged. Short gaps (dash blocks missing for a frame or two, within HANDOVER_S of the last
# line) keep the r8 gyro mode: alternating with the heading-hold command (FOLLOW_BIAS = PID on the dash path) would swing
# 150-250 us per frame (9eedee0a 15.9-17.4 s replay). The heading-hold command (heading_us) gets the same capped yaw-rate term
# as the lane steering (HEADING_ONLY has none; 9eedee0a 14.98 s: 1037 us while still turning left at 18 deg/s). Experiment values.
HANDOVER_S = .5
# r10 (user 2026-11-04, live d86fc9cc / 4459e292 / e55523e7 / c67e20c3): the downhill went smoothly only when the car reached the
# slope still pointing along the dashed line's direction (heading-hold reference). The three runs that held that direction up to
# the slope (75c85728, cfafa5c5, e55523e7) arrived within -1.3 .. +2.6 deg and none saturated on the slope; the runs that followed
# a side line in the last seconds arrived 10-14 deg off and saturated. In 4459e292 23.4-26.7 s and c67e20c3 23.4-27.0 s the car
# was within 0-5 deg of the reference while the 'right line' pointed -11 .. -22 deg (not parallel to the dashes) and pulled the
# car left. REL_GATE could not catch it because dash_dir is re-learnt from that same line. With the reference, a side line is
# used only when its direction agrees with yaw - ref within REF_GATE + REF_SCALE * |yaw - ref|; otherwise the frame has no
# usable line (heading-hold steering after HANDOVER_S, as in r9). The widening term: right after the dashed turn the measured
# line direction was 0.5-0.9 of yaw - ref (c67e20c3 13.7-16.9 s, still turning 8-22 deg/s); a fixed 5 deg would drop correct
# lines there. Dash fits are not gated. Experiment values.
REF_GATE, REF_SCALE = 5., .5
# r11 (user 2026-11-04, live 844a4274 / 89956cda and the 7 runs of raw-20261104e): in the two-lane straight the lane segment
# steered on the heading alone for 4.4-9.1 s of the 7-12 s in all 9 runs, the side lines being dropped by the direction gates,
# so nothing corrected the lateral position. 844a4274 18.9-22.3 s: the dropped 'right line' was the real right edge (video), its
# direction -4 .. -14 deg against yaw - ref -3.7 .. +4.8; the car drifted across it and onto the right rail (27-28 s). The lines
# dropped at the slope edge (floor circuit in view: 0fb475b7, 9c02d7da, a628f4f8, 2a62a163, 89956cda 24-28 s) pointed
# -15 .. -33 deg. A frame whose side line was dropped by a direction gate now keeps that line for the LATERAL term only, when
# its own direction is within LAT_REL_MAX of the car axis and the car is level (|pitch| <= LAT_PITCH_DEG): steering =
# heading-hold steering + yaw-rate damping + K_L * lat, the lateral part capped at +-LAT_CAP_US. The line's direction is not
# used: the heading stays on the reference (r10). LAT_PITCH_DEG: the mid-straight frames above (844a4274 19.9-21.5 s, fbf68cc9
# 16.7-19.5 s, 03a5e46a 16.3-22.2 s) had pitch 0.2-1.1 deg; in the last 1-3 s before the slope the platform tilts (65076008,
# bb013bf3, c67e20c3, d86fc9cc, 14d6d55c: 1.2-2.9 deg) and the side line there pulled the car off in r10's runs, so those
# frames keep the heading-hold steering. Thin margin (1.1 / 1.15); experiment values (lane-lat-only-20261104/replay.py).
LAT_REL_MAX, LAT_PITCH_DEG, LAT_CAP_US = 10., 1., 100.
# r12 steering play (user 2026-11-04, raw-20261104h: after the dashed-row turn the car did not line up on the straight dashes).
# After the turn (906 us, hard left) FOLLOW swung through the dash direction (rel 0 at gz +9..15 deg/s) to rel +7..12 deg,
# lat +0.5..0.7; the dash command then was only 1541-1582 us (centre 1390 + 150..190) and the car yawed right at only
# 1..4 deg/s, so the dashes slid out on the right. The record-only centre estimator (centre_estimator.py, 2026-11-04 21:09,
# ~100 samples per side) put the straight point at 1487 us when the steering arrives from the left and 1348 us from the
# right (play ~140 us): after a left turn 1560 us is only ~75 us right of straight. On the non-heading frames of the
# FOLLOW (dash, gyro, side line) the command now gets the estimator's offset of the side the steering arrives from (left value - centre while the
# command rises, right value - centre while it falls); the side flips when the command moves PLAY_TH_US past its last
# extreme. Offsets are read once per run (set_play), need PLAY_MIN_N samples per side and |offset| <= PLAY_MAX_US, else no
# compensation. Heading frames, the two-lane section and every other mode are unchanged. Experiment values (dash-play-20261104).
PLAY_MIN_N, PLAY_MAX_US, PLAY_TH_US, PLAY_GAP_S = 30, 150., 30., 1.


def _x(line, y): return line['a'] * y + line['b']


class LaneSegment:
    def __init__(self, dash_only=False):
        self.dash_only = dash_only
        self.k = {'left': None, 'right': None}; self.kn = {'left': 0, 'right': 0}; self.dash_dir = None; self.lat = 0.; self.t = None; self.t_line = None
        self.play = None; self.p_dir = None; self.p_ext = None; self.p_t = None   # r12

    def set_play(self, est, center):
        """r12: est = CentreEstimator.est ({'left': {value, n}, 'right': {value, n}}); returns the offsets used or None."""
        try:
            l, r = est['left'], est['right']
            if l.get('value') is None or r.get('value') is None or l.get('n', 0) < PLAY_MIN_N or r.get('n', 0) < PLAY_MIN_N: return None
            lo, ro = float(l['value']) - float(center), float(r['value']) - float(center)
            if max(abs(lo), abs(ro)) > PLAY_MAX_US or lo <= ro: return None
            self.play = dict(left=lo, right=ro); return dict(self.play)
        except (KeyError, TypeError, ValueError, AttributeError): return None

    def _play(self, now, desired, prev_us):
        """r12: command for the wanted steering `desired`, given the last sent command prev_us. Returns (cmd, info) ."""
        if self.play is None or prev_us is None: return desired, None
        prev_us = float(prev_us)
        if self.p_t is None or now - self.p_t > PLAY_GAP_S: self.p_dir = None; self.p_ext = prev_us
        self.p_t = now
        if self.p_dir == 1: self.p_ext = max(self.p_ext, prev_us)
        elif self.p_dir == -1: self.p_ext = min(self.p_ext, prev_us)
        if self.p_dir is None:
            if desired > prev_us + PLAY_TH_US: self.p_dir, self.p_ext = 1, prev_us
            elif desired < prev_us - PLAY_TH_US: self.p_dir, self.p_ext = -1, prev_us
        if self.p_dir is None: return desired, dict(dir=None, off_us=0.)
        cmd = desired + (self.play['left'] if self.p_dir == 1 else self.play['right'])
        if self.p_dir == 1 and cmd < self.p_ext - PLAY_TH_US: cmd = desired + self.play['right']; self.p_dir, self.p_ext = -1, cmd
        elif self.p_dir == -1 and cmd > self.p_ext + PLAY_TH_US: cmd = desired + self.play['left']; self.p_dir, self.p_ext = 1, cmd
        return cmd, dict(dir=self.p_dir, off_us=round(cmd - desired, 1), ext_us=round(self.p_ext, 1))

    def step(self, now, dash, yaw, gz, center, lo, hi, row_y, ref=None, heading_us=None, prev_us=None):
        """ref / heading_us: heading-hold reference yaw and steering of this frame (r9), or None. prev_us: last sent command (r12)."""
        info = dict(revision=REVISION)
        sides = {} if self.dash_only else ((dash or {}).get('sides') or {})
        dline = dict(a=dash['line_a'], b=dash['line_b']) if dash and dash.get('line_a') is not None and dash.get('line_b') is not None else None
        dt = 0. if self.t is None else max(0., min(.3, now - self.t)); self.t = now
        def side_est(prefer):
            kk = {s: (self.k[s] if self.k.get(s) is not None and self.kn[s] >= _RI.thr(K_MIN_N) else K_DEFAULT[s]) for s in sides if s in K_DEFAULT}
            cand = {s: dict(a=sides[s]['a'] + v, b=sides[s]['b'] - v * HORIZON_Y) for s, v in kk.items()}
            if not cand: return None, None
            s = min(cand, key=lambda q: abs((_x(cand[q], row_y) - AXIS) / 320. - prefer)) if len(cand) > 1 else next(iter(cand))
            return cand[s], 'side_' + s + ('' if self.k.get(s) is not None and self.kn[s] >= _RI.thr(K_MIN_N) else '_default')
        se, ssrc = side_est(self.lat); dropped = None   # r11: (line, direction) of a side line dropped by a direction gate
        if (dline is not None and se is not None and self.t_line is not None and abs((_x(dline, row_y) - _x(se, row_y)) / 320.) > DASH_VS_SIDE
                and abs((_x(dline, row_y) - AXIS) / 320. - self.lat) > DASH_VS_SIDE):   # jumps away from the last value AND the side line
            dline = None; info['dash_dropped'] = 'DISAGREES_WITH_SIDE'
        if dline is not None:   # learn the side offsets while the dashes are seen
            for s, L in sides.items():
                if s not in K_DEFAULT or abs(_x(dline, HORIZON_Y) - _x(L, HORIZON_Y)) > VP_TOL: continue   # r2: not parallel to the dashes
                kk = (_x(dline, row_y) - _x(L, row_y)) / max(1., row_y - HORIZON_Y)
                if K_RANGE[0] <= abs(kk) <= K_RANGE[1] and ((s == 'left' and kk > 0) or (s == 'right' and kk < 0)):
                    self.k[s] = kk if self.k[s] is None else self.k[s] + _RI.alpha(K_ALPHA) * (kk - self.k[s]); self.kn[s] += _RI.w()
            src = 'dash'
        else:
            dline, src = se, ssrc
        if dline is not None and src != 'dash' and self.dash_dir is not None and yaw is not None:
            rel_l = math.degrees(math.atan((_x(dline, HORIZON_Y) - VP_X) / F_PX)); g_rel = yaw - self.dash_dir
            if abs(rel_l - g_rel) > REL_GATE: info['line_dropped'] = dict(source=src, rel_deg=round(rel_l, 1), gyro_rel_deg=round(g_rel, 1)); dropped = (dline, rel_l); dline = None
        if dline is not None and src != 'dash' and ref is not None and yaw is not None:   # r10: side line must agree with the dash direction
            rel_l = math.degrees(math.atan((_x(dline, HORIZON_Y) - VP_X) / F_PX)); r_rel = yaw - ref; tol = REF_GATE + REF_SCALE * abs(r_rel)
            if abs(rel_l - r_rel) > tol: info['line_dropped'] = dict(source=src, rel_deg=round(rel_l, 1), ref_rel_deg=round(r_rel, 1), tol_deg=round(tol, 1), gate='REF'); dropped = (dline, rel_l); dline = None
        if dline is not None:
            self.t_line = now
            rel = math.degrees(math.atan((_x(dline, HORIZON_Y) - VP_X) / F_PX))
            if yaw is not None: self.dash_dir = yaw - rel
            lat = max(-1., min(1., (_x(dline, row_y) - AXIS) / 320.)); lat = self.lat + LAT_A * (lat - self.lat) if self.t is not None and dt > 0 else lat; self.lat = lat
        elif ref is not None and heading_us is not None and (self.t_line is None or now - self.t_line > HANDOVER_S):   # r9: no usable line -> heading-hold steering
            damp = max(-D_CAP_US, min(D_CAP_US, K_D * (gz or 0.))); lat_us = 0.
            pitch = (dash or {}).get('pitch_rel_deg')
            if dropped is not None and abs(dropped[1]) <= LAT_REL_MAX and pitch is not None and abs(float(pitch)) <= LAT_PITCH_DEG:   # r11: lateral only
                m = max(-1., min(1., (_x(dropped[0], row_y) - AXIS) / 320.)); self.lat = self.lat + LAT_A * (m - self.lat) if dt > 0 else m
                lat_us = max(-LAT_CAP_US, min(LAT_CAP_US, K_L * self.lat)); info['lat_only'] = dict(source=info['line_dropped']['source'], lat_meas=round(m, 3), lat_us=round(lat_us, 1))
            else: self.lat *= math.exp(-dt / TAU_LAT)
            out = max(lo, min(hi, float(heading_us) + damp + lat_us))
            info.update(reason='OK', source='heading_lat' if lat_us else 'heading', lat=round(self.lat, 3), ref_deg=round(ref, 2), heading_us=round(float(heading_us), 1), damp_us=round(damp, 1),
                        dash_rel_deg=None if self.dash_dir is None or yaw is None else round(yaw - self.dash_dir, 2), steer_us=round(out, 1))
            return out, info
        elif self.dash_dir is not None and yaw is not None:
            rel = yaw - self.dash_dir; src = 'gyro'; self.lat *= math.exp(-dt / TAU_LAT); lat = self.lat
        else:
            info.update(reason='NO_REFERENCE'); return None, info
        dev = max(-LIM_US, min(LIM_US, K_H * rel + K_L * lat + max(-D_CAP_US, min(D_CAP_US, K_D * (gz or 0.)))))
        want = center + dev; play = None
        if (dash or {}).get('mode') == 'FOLLOW': want, play = self._play(now, want, prev_us)   # r12: every non-heading FOLLOW frame (dash / gyro / side), no frame-to-frame jumps
        out = max(lo, min(hi, want))
        if play is not None: info['play'] = dict(play, desired_us=round(center + dev, 1))
        info.update(reason='OK', source=src, rel_deg=round(rel, 2), lat=round(lat, 3), gz=None if gz is None else round(gz, 2),
                    k_left=None if self.k['left'] is None else round(self.k['left'], 3), k_right=None if self.k['right'] is None else round(self.k['right'], 3), k_n=dict(self.kn),
                    steer_us=round(out, 1))
        return out, info
