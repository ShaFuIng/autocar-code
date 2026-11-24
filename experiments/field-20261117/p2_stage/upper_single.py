"""upper-edges-r1 hooks (upper-edges-20261115, r7 keys; REVISION string unchanged so OFF logs stay identical): the two solid edge
lines of the upper deck (a funnel toward the downhill) via upper_edges.UpperEdges (worker points -> car pose on the deck):
 - edge_source (false = off): build the estimator (needs web/upper_edges.py + camera_ground_calibration.json); live_pid then passes
   edge_det / y_rel_m / odo_m and asks the worker for the points; info["edge"] is logged.
 - edge_follow (false = off): a frame without a dash (or mask) error uses the estimator pose (measured, or dead-reckoned <= dr_max_m
   after the last fix, deck direction established) as a virtual dashed line -> the same FOLLOW law and gains (src "edge").
 - edge_dir_gate_deg (null = off): dash-direction observations farther than this from the edge deck direction (starts at the target h, refined by two-sided fits) are rejected (EDGE).
 - edge_hold_target (false = off): HOLD aims at the edge deck direction (EDGE_DIR) when established, before DASH_DIR / TARGET.
 - edge_est (dict, optional): UpperEdges keys (upper_edges.EST_DEFAULTS). info["edge"]["aligned"] is the post-turn "aligned" flag.
upper-single-r4 (v2/upper, 2026-11-13 review): two more switches, each OFF = r2 outputs:
 - hold_k_g (null = off) / hold_lim_g_us: HOLD adds the FOLLOW rate damping clamp(hold_k_g * gz, +-hold_lim_g_us) (gz left + ->
   steer right); e98f / dc85 entered HOLD while still yawing at 18-33 deg/s and HOLD had no gz term.
 - hold_dir_fit (false = off): live_pid builds the MaskDash angle helper (homography only, no find()) also when hold_dir_fit is
   true, so the dash-fit angle fit_ang (hold_dir) exists with mask_dash false; find() still needs mask_dash (wants_mask()).
upper-single-r3 (v2/upper, 2026-11-13 e98f / dc85: hand-over at dyaw -12 / -19 deg while still turning at +27..+31 deg/s, ALIGN
 ended after one frame, HOLD memorised the momentary yaw and overshot). Three switches, each OFF = r2 outputs:
 - align_exit_gz_dps (null = off): ALIGN may end on the angle only when |gz| <= this value (align_max_s timeout unchanged).
 - hold_fallback ("last_yaw" = off | "target"): HOLD without a dash direction holds the target h instead of the last camera yaw.
 - handover_dyaw_deg (null = off): with pre_armed (route OPEN, heading reference set, not done; set by live_pid as the attribute
   pre_armed or passed as the keyword) and yaw - h >= this value the module arms itself (ALIGN) although the dashed TURN has not
   ended; latched until the route window closes (reset()).
upper-single-r2 (fix31, consolidation-20261111/upper/side_lines): r1 + a second dash source and a dash-direction HOLD.
 - FOLLOW source priority: dash-route fit (JPEG) -> mask dashed line (mask_dash.py, ground-space classifier; passed in as the same
   image-line form x = a*y + b, so e and k_p_us are unchanged) -> (normal_fallback) -> HOLD. Both through the same jump rejection.
 - HOLD target (hold_dir true): median of the dashed-line directions D = yaw - ang over the accepted dash observations (fit or mask)
   within hold_dir_window_s before the latest one (>= hold_dir_min_n of them); ang = ground direction of the observed line relative to
   the car axis (deg, + right), yaw = gyro yaw (left +), so D is in the gyro yaw frame. Else the yaw of the last accepted frame (r1),
   else h. mask_dash false and hold_dir false = r1 (fix30) outputs.
upper-single-r1 (user 2026-11-11 5235: after the dashed turn the sent steering swung 956-1770 us, 13 reversals in 8.7 s, while
heading hold, lane segment, upper-lateral, solid takeover and the blind holds took turns; consolidation-20261111/upper/DESIGN.md).
One owner of the steering from the end of the dashed TURN until the route window leaves OPEN; the car drives ON the dashed line.
 ALIGN : gyro only. target h = pre-turn mean heading + align_target_deg (null: the heading-hold reference);
         steer = c - clamp(k_align * (h - yaw - align_lead_s * gz), +-lim_align); -> FOLLOW when |that error| <= align_exit_deg
         or after align_max_s (or at once without IMU).
 FOLLOW: camera only, ONE lateral reference = the dashed line. e = dash-route line fit at the control row (same definition as the
         PID image error: (x - 319.75 - visual centre) / 319.75, > 0 = line right of the car = car left of it -> steer right);
         normal_fallback true: without a fit the normal-path error + a learnt offset. A candidate more than cam_jump from the last
         accepted error within cam_jump_s is rejected. steer = c + clamp(k_p * e + k_de * de/dt, +-lim_follow) + clamp(k_g * gz, +-lim_g)
         (gz left positive: turning left -> steer right, the damping opposes the rotation; k_g 0 = off).
 HOLD  : no accepted camera error (valid or invalid frame): steer = c - clamp(k_hold * (psi - yaw), +-lim_hold), psi = yaw of the
         last accepted camera frame (target h before any). -> EXPIRED (returns None = existing rules) hold_max_s after the last
         accepted camera frame (or after ALIGN ended), or without IMU; an accepted camera error returns to FOLLOW.
 All commands: slew limit slew_us_s from the previous output (or the last sent command), clamp to min..max.
c = run centre (center_us from the run parameters) + centre_offset_us. Every number is a key of upper_single.json; the file is read
when the object is built (run start). enabled false / missing / unreadable file -> step() returns (None, None) always.
Experiment values (sources: DESIGN.md); not a verified safety limit. Pure module: no car I/O, the caller passes all inputs."""
import json, math
from pathlib import Path
import rate_indep as _RI   # rate-indep-r1 (pipeline2-20261116): ON False -> every _RI.* below returns the original value

REVISION = 'upper-single-r4'
DEFAULTS = dict(align_target_deg=87., k_align_us_per_deg=25., lim_align_us=360., align_lead_s=.5, align_exit_deg=10., align_max_s=3.,
                k_p_us=375., k_de_us_s=90., de_tau_s=.2, lim_follow_us=300., k_g=5., lim_g_us=100.,
                normal_fallback=False, off_init=.68, off_alpha=.3, off_gate=.3, cam_jump=.5, cam_jump_s=.6,
                k_hold_us_per_deg=25., lim_hold_us=200., hold_max_s=6., slew_us_s=900., centre_offset_us=0.,
                mask_dash=True, hold_dir=True, hold_dir_window_s=1.5, hold_dir_min_n=3.,
                mask_row0=100., mask_y_min_cm=30., mask_y_max_cm=140., mask_dash_l_min_cm=8., mask_dash_l_max_cm=25.,
                mask_dash_w_max_cm=10., mask_dash_col_cm=6., mask_dir_gate_deg=15., mask_max_pitch_deg=2., mask_max_px=800.,
                mask_space_min_cm=15., mask_space_max_cm=60.)
R3_OFF = dict(align_exit_gz_dps=None, hold_fallback='last_yaw', handover_dyaw_deg=None, hold_k_g=None, hold_lim_g_us=100., hold_dir_fit=False,
              hold_dir_gate_deg=None, obs_max_gz_dps=None, follow_head_cap_deg=None, k_head_cap_us_per_deg=25.,
              stall_k_us_per_deg=None, stall_lim_us=200., stall_exit_deg=3., stall_max_s=2.5, stall_rate_k=2., stall_rate_max_dps=40., stall_de_scale=0.,
              conv_rate=None, conv_max=.8, conv_head_k=1.25,
              edge_source=False, edge_follow=False, edge_hold_target=False, edge_dir_gate_deg=None)   # r7 keys (upper-edges-20261115) at OFF   # r6 keys (upper-wide-20261115) at OFF   # r5 keys (upper-20261115) at OFF   # r3 / r4 keys at their OFF values
DEFAULTS.update(R3_OFF)
BOOLS = ('normal_fallback', 'mask_dash', 'hold_dir', 'hold_dir_fit', 'edge_source', 'edge_follow', 'edge_hold_target')   # r4: hold_dir_fit
NULLABLE = ('align_target_deg', 'align_exit_gz_dps', 'handover_dyaw_deg', 'hold_k_g', 'hold_dir_gate_deg', 'obs_max_gz_dps', 'follow_head_cap_deg', 'stall_k_us_per_deg', 'conv_rate', 'edge_dir_gate_deg')   # r5; r6
CHOICES = dict(hold_fallback=('last_yaw', 'target'))   # r3: first = OFF (also for null)


def load_config(path=None):
    try:
        c = json.loads(Path(path or Path(__file__).with_name('upper_single.json')).read_text(encoding='utf8'))
        return c if isinstance(c, dict) else {}
    except Exception: return {}


def _f(x):
    try:
        v = float(x)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError): return None


def _clamp(x, lim): return max(-lim, min(lim, x))


class UpperSingle:
    def __init__(self, config=None):
        raw = load_config() if config is None else dict(config)
        self.c = dict(DEFAULTS); self.config_error = None
        try:
            for k in DEFAULTS:
                if k not in raw: continue
                if k in CHOICES:   # r3
                    v = CHOICES[k][0] if raw[k] is None else raw[k]
                    if v not in CHOICES[k]: raise ValueError(k)
                    self.c[k] = v
                elif k in BOOLS: self.c[k] = bool(raw[k])
                elif raw[k] is None and k in NULLABLE: self.c[k] = None
                else:
                    v = _f(raw[k])
                    if v is None: raise ValueError(k)
                    self.c[k] = v
            self.enabled = bool(raw.get('enabled', False))
        except Exception as e:
            self.enabled = False; self.config_error = '%s:%s' % (type(e).__name__, e)
        self.ue = None
        if self.enabled and self.c['edge_source']:   # r7: deck-edge estimator (any failure -> edges off, everything else unchanged)
            try:
                import upper_edges as _ue
                self._uem = _ue; self.ue_H, self.ue_Hinv = _ue.load_h(); self.ue = _ue.UpperEdges(raw.get('edge_est') if isinstance(raw.get('edge_est'), dict) else None)
            except Exception as e: self.ue = None; self.config_error = 'edge:%s' % type(e).__name__
        self.reset()

    def reset(self):
        self.state = None; self.t_state = None; self.prev = None; self.t_prev = None
        self.e_last = None; self.t_last = None; self.ef = 0.; self.off = self.c['off_init']
        self.psi = None; self.t_cam = None; self.t_follow0 = None; self.n_cam = 0; self.n_dash = 0; self.n_mask = 0; self.first = True
        self.obs = []; self.h_last = None   # r2: (t, dash direction D) of accepted dash observations; last target h
        self.early = False   # r3: armed by the hand-over trigger (latched until reset)
        self.fin = False; self.t_fin0 = None; self.e_ref = None; self.t_conv = None; self.conv_init = False   # r6: stall fix (finishing the rotation) / convergence ramp
        self.edge = None
        if getattr(self, 'ue', None) is not None: self.ue.reset()   # r7

    def _go(self, state, now):
        if state != self.state: self.state = state; self.t_state = now

    def _slew(self, now, raw, lo, hi, last_sent):
        p, tp = self.prev, self.t_prev
        if p is None and last_sent is not None and _f(last_sent[0]) is not None and _f(last_sent[1]) is not None: p, tp = float(last_sent[0]), float(last_sent[1])
        out = raw; lim = None
        if p is not None and tp is not None:
            lim = self.c['slew_us_s'] * max(0., now - tp); out = max(p - lim, min(p + lim, raw))
        out = max(float(lo), min(float(hi), out))
        self.prev, self.t_prev = out, now
        return out, (None if lim is None else round(raw - max(p - lim, min(p + lim, raw)), 1))

    def _obs_reject(self, D, gz, pre_deg, ref_deg):
        """r5 (upper-20261115): reject a dash-direction observation D (gyro yaw frame) taken while rotating faster than obs_max_gz_dps,
        or more than hold_dir_gate_deg from the target h. 11-14 d077: observations taken during ALIGN (gz 20-38 deg/s) gave D = 71-76 deg
        (h 87); HOLD then held 71.6 deg and the car stayed 13-16 deg short. Other passes: median D 84.6-90.5 deg. Returns None or a dict."""
        c = self.c
        if c['obs_max_gz_dps'] is not None and gz is not None and abs(gz) > c['obs_max_gz_dps']: return dict(why='GZ', D=round(D, 2), gz=round(gz, 1))
        if c['hold_dir_gate_deg'] is not None:
            h = _f(ref_deg if c['align_target_deg'] is None else (None if pre_deg is None else float(pre_deg) + c['align_target_deg']))
            if h is not None and abs(D - h) > c['hold_dir_gate_deg']: return dict(why='GATE', D=round(D, 2), h=round(h, 2))
        eg = self.edge if c['edge_dir_gate_deg'] is not None else None   # r7: edge deck direction gate
        if eg is not None and eg.get('D') is not None and abs(D - float(eg['D'])) > c['edge_dir_gate_deg']: return dict(why='EDGE', D=round(D, 2), D_edge=round(float(eg['D']), 2))
        return None

    def _edge(self, now, yaw, gz, pre_deg, ref_deg, dash_line, edge_det, y_rel_m, odo_m, info):
        """r7: one UpperEdges step (dash fit of this frame = lateral fix); stores self.edge and a compact info['edge']."""
        c = self.c
        h = _f(ref_deg if c['align_target_deg'] is None else (None if pre_deg is None else float(pre_deg) + c['align_target_deg']))
        Dp = h if h is not None else self.dir_median()   # pre-turn heading + 87 (dash observations taken while rotating can be ~10 deg off: d077, 4148 L3, 2485 L2)
        xd = None
        if dash_line is not None:
            try: xd = self._uem.dash_x_cm(self.ue_H, float(dash_line[0]), float(dash_line[1]))
            except Exception: xd = None
        try: o = self.ue.step(edge_det, yaw, y_rel_m, Dp, odo_m, xd, gz)
        except Exception as e: o = None; info['edge'] = dict(error=type(e).__name__)
        self.edge = o
        if o is not None: info['edge'] = {k: o.get(k) for k in ('ok', 'use', 'why', 'phi', 'x', 'D', 'D_ok', 'y', 'xref', 'dr_m', 'nL', 'nR', 'rms', 'aligned')}
        elif 'edge' not in info: info['edge'] = dict(why='NO_INPUT', det=edge_det is not None)
        return o

    def _fin_start(self, now):
        """r6: ALIGN -> FOLLOW: arm the stall fix (finish the rotation to the dash direction) when stall_k_us_per_deg is set."""
        if self.c['stall_k_us_per_deg'] is not None: self.fin = True; self.t_fin0 = now

    def _r6(self, now, e, yaw, gz, h, damp, info):
        """r6 (upper-wide-20261115). Stall fix: after ALIGN the FOLLOW gyro damping k_g*gz (and the derivative term, which during the
        rotation mostly measures the rotation) stopped the last ~10-20 deg of the turn (11-14 9972 L1: 70 -> 77 deg in 0.9 s at 1-9 deg/s,
        FOLLOW start -> within 3 deg 1.37 s median over 29 passes). While the heading is short of the dash direction tg (median of the
        accepted observations, else h) by more than stall_exit_deg (and at most stall_max_s): damping on (gz - gz_des) with
        gz_des = clamp(stall_rate_k * err, 0, stall_rate_max_dps), derivative x stall_de_scale, plus -min(stall_lim_us, stall_k * err)
        toward the dash direction. Convergence ramp: at the first FOLLOW frame the lateral part of e (e - conv_head_k * tan(yaw - tg))
        becomes a reference e_ref (|.| <= conv_max) that decays at conv_rate per second after the stall phase; FOLLOW uses e - e_ref.
        Returns (k_de, e_used, extra steering, damping)."""
        c = self.c; kde = c['k_de_us_s']; eu = e; st = 0.
        tg = self.dir_median()
        if tg is None: tg = h
        if c['conv_rate'] is not None and not self.conv_init and yaw is not None and tg is not None:
            self.conv_init = True; el = e - c['conv_head_k'] * math.tan(math.radians(max(-60., min(60., yaw - tg))))
            self.e_ref = max(-c['conv_max'], min(c['conv_max'], el)); self.t_conv = now
        if self.fin:
            if yaw is None or tg is None or tg - yaw <= c['stall_exit_deg'] or self.t_fin0 is None or now - self.t_fin0 > c['stall_max_s']:
                self.fin = False; info['stall'] = dict(state='END', err_deg=None if (yaw is None or tg is None) else round(tg - yaw, 2), s=None if self.t_fin0 is None else round(now - self.t_fin0, 3))
            else:
                herr = tg - yaw; gzd = min(c['stall_rate_max_dps'], max(0., c['stall_rate_k'] * herr))
                if gz is not None: damp = _clamp(c['k_g'] * (gz - gzd), c['lim_g_us'])
                kde = kde * c['stall_de_scale']; st = -min(c['stall_lim_us'], max(0., c['stall_k_us_per_deg'] * herr))
                info['stall'] = dict(state='ON', err_deg=round(herr, 2), gz_des=round(gzd, 1), term_us=round(st, 1))
        if self.e_ref is not None:
            if not self.fin and self.t_conv is not None:
                stp = c['conv_rate'] * max(0., now - self.t_conv)
                self.e_ref = 0. if abs(self.e_ref) <= stp else self.e_ref - stp * (1. if self.e_ref > 0 else -1.)
            self.t_conv = now; eu = e - self.e_ref; info['conv'] = dict(e_ref=round(self.e_ref, 3))
            if self.e_ref == 0.: self.e_ref = None
        return kde, eu, st, damp

    def dir_median(self):
        """r2: median dashed-line direction (gyro yaw frame, deg) of the accepted observations within hold_dir_window_s before the
        latest one, or None with fewer than hold_dir_min_n."""
        if not self.obs: return None
        tl = self.obs[-1][0]; W = sorted(o[1] for o in self.obs if tl - o[0] <= self.c['hold_dir_window_s'])
        if (len(W) if not _RI.ON else sum((o[2] if len(o) > 2 else 1.) for o in self.obs if tl - o[0] <= self.c['hold_dir_window_s'])) < _RI.thr(self.c['hold_dir_min_n']): return None   # rate-indep-r1: weighted count
        n = len(W); return W[n // 2] if n % 2 else .5 * (W[n // 2 - 1] + W[n // 2])

    def expect_deg(self, yaw):
        """r2: expected ground angle of the dashed line relative to the car axis (deg, + right) for the mask direction gate:
        yaw - (recent dash direction, else the last target h); None when unknown (no gate)."""
        yaw = _f(yaw); D = self.dir_median()
        if D is None: D = self.h_last
        return None if yaw is None or D is None else yaw - D

    def wants_mask(self):
        """r2: the caller computes the mask line only when it can be used (enabled, mask_dash, inside the window after ALIGN)."""
        return bool(self.enabled and self.c['mask_dash'] and self.state in ('FOLLOW', 'HOLD', 'EXPIRED'))

    def _handover(self, yaw, pre_deg, ref_deg):
        """r3: dict(by, dyaw_deg) when the module arms itself before the dashed TURN has ended, else None. Latched (self.early)."""
        c = self.c; yaw = _f(yaw)
        h = _f(ref_deg if c['align_target_deg'] is None else (None if pre_deg is None else float(pre_deg) + c['align_target_deg']))
        dy = None if yaw is None or h is None else yaw - h
        if self.early: return dict(by='LATCHED', dyaw_deg=None if dy is None else round(dy, 2))
        if dy is not None and dy >= c['handover_dyaw_deg']:
            self.early = True; return dict(by='DYAW', dyaw_deg=round(dy, 2))
        return None

    def step(self, now, route_open, armed, valid, yaw, gz, center, lo, hi, dash_line=None, e_normal=None, row_ratio=.6, vc_px=0.,
             last_sent=None, pre_deg=None, ref_deg=None, mask_line=None, fit_ang=None, mask_ang=None, pre_armed=None, edge_det=None, y_rel_m=None, odo_m=None):
        """One frame. route_open: dash route OPEN; armed: heading reference set, not turning (window started); valid: the frame has a
        valid path; yaw / gz: deg, deg/s left positive (None = IMU missing); dash_line: (a, b) of the dash-route fit x = a*y + b
        (640x360) or None; e_normal: normal camera path error at the control row or None; last_sent: (steer_us, t) of the last sent
        command; pre_deg: pre-turn mean heading; ref_deg: heading-hold reference. r2: mask_line (a, b) of the mask dashed line in the
        same image form, fit_ang / mask_ang their ground angles (deg, + right) or None. Returns (steer_us or None, info or None)."""
        if not self.enabled: return None, None
        if not route_open:
            if self.state is not None: self.reset()
            return None, None
        ho = None
        if not armed and self.c['handover_dyaw_deg'] is not None:   # r3 hand-over trigger
            if pre_armed is None: pre_armed = getattr(self, 'pre_armed', None)
            if pre_armed:
                ho = self._handover(yaw, pre_deg, ref_deg)
                if ho is not None: armed = True
        if not armed:
            return None, (dict(revision=REVISION, active=False, reason='NOT_ARMED', state=self.state) if self.state is not None else None)
        c = self.c; c0 = float(center) + c['centre_offset_us']; yaw = _f(yaw); gz = _f(gz)
        if self.state is None: self._go('ALIGN', now)
        info = dict(revision=REVISION, active=True)
        if ho is not None: info['handover'] = ho
        if self.first: info['cfg'] = {k: v for k, v in c.items() if k not in R3_OFF or v != R3_OFF[k] or (k == 'hold_lim_g_us' and c['hold_k_g'] is not None) or (k == 'k_head_cap_us_per_deg' and c['follow_head_cap_deg'] is not None)}; self.first = False   # r3/r4 keys logged only when ON
        # camera candidate (dash fit only by default)
        e_dash = None
        if valid and dash_line is not None and _f(dash_line[0]) is not None and _f(dash_line[1]) is not None:
            x = float(dash_line[0]) * (float(row_ratio) * 719. / 2.) + float(dash_line[1]); e_dash = max(-1., min(1., (x - 319.75 - float(vc_px)) / 319.75))
        e_mask = None
        if valid and c['mask_dash'] and mask_line is not None and _f(mask_line[0]) is not None and _f(mask_line[1]) is not None:
            x = float(mask_line[0]) * (float(row_ratio) * 719. / 2.) + float(mask_line[1]); e_mask = max(-1., min(1., (x - 319.75 - float(vc_px)) / 319.75))
        e_norm = _f(e_normal) if valid else None
        if e_dash is not None and e_norm is not None and abs((e_dash - e_norm) - self.off) <= c['off_gate']: self.off += _RI.alpha(c['off_alpha']) * ((e_dash - e_norm) - self.off)
        eo = self._edge(now, yaw, gz, pre_deg, ref_deg, dash_line if e_dash is not None else None, edge_det, y_rel_m, odo_m, info) if self.ue is not None else None   # r7
        e, src = None, None
        ang = None
        if e_dash is not None: e, src, ang = e_dash, 'dash', _f(fit_ang)
        elif e_mask is not None: e, src, ang = e_mask, 'mask', _f(mask_ang)
        elif c['edge_follow'] and eo is not None and eo.get('use') and eo.get('D_ok') and self.state != 'ALIGN' and self._uem.virtual_line(self.ue_Hinv, eo) is not None:   # r7: virtual dashed line from the deck pose
            va_, vb_ = self._uem.virtual_line(self.ue_Hinv, eo); x = va_ * (float(row_ratio) * 719. / 2.) + vb_; e, src = max(-1., min(1., (x - 319.75 - float(vc_px)) / 319.75)), 'edge'
        elif c['normal_fallback'] and e_norm is not None: e, src = e_norm + self.off, 'normal+off'
        if e is not None and self.e_last is not None and now - self.t_last <= c['cam_jump_s'] and abs(e - self.e_last) > c['cam_jump']:
            info['rejected'] = dict(src=src, e=round(e, 3), e_last=round(self.e_last, 3)); e, src = None, 'rejected_jump'
        if e is not None and src in ('dash', 'mask') and ang is not None and yaw is not None:   # r2: dash direction observation
            rj_ = self._obs_reject(yaw - ang, gz, pre_deg, ref_deg)   # r5: None = accepted (r5 keys off -> always None)
            if rj_ is None: self.obs.append((now, yaw - ang) if not _RI.ON else (now, yaw - ang, _RI.w())); self.obs = [o for o in self.obs if now - o[0] <= 10.]
            else: info['obs_rejected'] = rj_
        info.update(e_mask=None if e_mask is None else round(e_mask, 3), e_dash=None if e_dash is None else round(e_dash, 3), e_normal=None if e_norm is None else round(e_norm, 3), off=round(self.off, 3), e_src=src,
                    yaw_deg=None if yaw is None else round(yaw, 2), gz_dps=None if gz is None else round(gz, 2))
        h = (ref_deg if c['align_target_deg'] is None else (None if pre_deg is None else float(pre_deg) + c['align_target_deg']))
        h = _f(h); info['h_target_deg'] = None if h is None else round(h, 2); self.h_last = h
        raw = None
        if self.state == 'ALIGN':
            if yaw is None or h is None:
                info['align_skipped'] = 'NO_IMU' if yaw is None else 'NO_TARGET'; self._go('FOLLOW', now); self.t_follow0 = now; self._fin_start(now)
            else:
                err = h - (yaw + c['align_lead_s'] * (gz or 0.))
                raw = c0 - _clamp(c['k_align_us_per_deg'] * err, c['lim_align_us']); info.update(align_err_deg=round(err, 2))
                if e is not None: self.e_last, self.t_last, self.ef = e, now, 0.   # camera tracked for the jump gate / derivative start
                gzx = c['align_exit_gz_dps']; slow = gzx is None or abs(gz or 0.) <= gzx   # r3: no angle exit while still rotating
                if (abs(err) <= c['align_exit_deg'] and slow) or now - self.t_state >= c['align_max_s']:
                    info['align_exit'] = 'ANGLE' if (abs(err) <= c['align_exit_deg'] and slow) else 'TIMEOUT'; self._go('FOLLOW', now); self.t_follow0 = now; self._fin_start(now)
                elif abs(err) <= c['align_exit_deg']: info['align_wait_gz'] = round(gz or 0., 2)
                info['state'] = 'ALIGN'
        if raw is None:
            if e is not None:
                if self.e_last is not None and 0. < now - self.t_last <= c['cam_jump_s']:
                    de = (e - self.e_last) / (now - self.t_last); a = 1. - math.exp(-(now - self.t_last) / c['de_tau_s']); self.ef += a * (de - self.ef)
                else: self.ef = 0.
                self.e_last, self.t_last, self.t_cam = e, now, now; self.n_cam += 1; self.n_dash += src == 'dash'; self.n_mask += src == 'mask'
                if yaw is not None: self.psi = yaw
                damp = _clamp(c['k_g'] * gz, c['lim_g_us']) if gz is not None else 0.
                if self.fin or self.e_ref is not None or (c['conv_rate'] is not None and not self.conv_init):   # r6 (upper-wide-20261115); keys off -> never true
                    kde_, eu_, st_, damp = self._r6(now, e, yaw, gz, h, damp, info)
                    raw = c0 + _clamp(c['k_p_us'] * eu_ + kde_ * self.ef, c['lim_follow_us']) + damp
                    if st_: raw += st_
                else: raw = c0 + _clamp(c['k_p_us'] * e + c['k_de_us_s'] * self.ef, c['lim_follow_us']) + damp
                if c['follow_head_cap_deg'] is not None and yaw is not None:   # r5: heading guard around the dash direction (median of the observations, else h)
                    tg_ = self.dir_median()
                    if tg_ is None: tg_ = h
                    if tg_ is not None:
                        ex_ = yaw - tg_; ov_ = max(0., abs(ex_) - c['follow_head_cap_deg']) * (1. if ex_ > 0 else -1.)
                        if ov_: hc_ = c['k_head_cap_us_per_deg'] * ov_; raw += hc_; info['head_cap_us'] = round(hc_, 1)
                self._go('FOLLOW', now); info.update(state='FOLLOW', e=round(e, 3), de_dt=round(self.ef, 3), damp_us=round(damp, 1))
            else:
                t_ref = self.t_cam if self.t_cam is not None else (self.t_follow0 if self.t_follow0 is not None else now)
                if now - t_ref > c['hold_max_s'] or yaw is None:
                    self._go('EXPIRED', now); self.prev = None; self.t_prev = None
                    info.update(state='EXPIRED', active=False, reason='NO_IMU' if yaw is None else 'HOLD_TIMEOUT', since_cam_s=round(now - t_ref, 3))
                    return None, info
                if self.psi is None: self.psi = h if h is not None else yaw
                eg_ = self.edge if c['edge_hold_target'] else None   # r7: edge deck direction first
                tgt, how = (float(eg_['D']), 'EDGE_DIR') if (eg_ is not None and eg_.get('D_ok') and eg_.get('D') is not None) else ((self.dir_median(), 'DASH_DIR') if c['hold_dir'] else (None, None))
                if tgt is None and c['hold_fallback'] == 'target' and h is not None: tgt, how = h, 'TARGET'   # r3
                if tgt is None: tgt, how = self.psi, 'LAST_YAW'
                raw = c0 - _clamp(c['k_hold_us_per_deg'] * (tgt - yaw), c['lim_hold_us'])
                if c['hold_k_g'] is not None and gz is not None:   # r4: FOLLOW's rate damping in HOLD (yawing left -> steer right)
                    hdamp = _clamp(c['hold_k_g'] * gz, c['hold_lim_g_us']); raw += hdamp; info['hold_damp_us'] = round(hdamp, 1)
                self._go('HOLD', now); info.update(state='HOLD', psi_hold_deg=round(tgt, 2), hold_target=how, since_cam_s=round(now - t_ref, 3))
        out, sl = self._slew(now, raw, lo, hi, last_sent)
        info.update(raw_us=round(raw, 1), slew_limited_us=sl, steer_us=round(out, 1), t_state_s=round(now - self.t_state, 3), cam_frames=self.n_cam, dash_frames=self.n_dash, mask_frames=self.n_mask)
        return out, info
