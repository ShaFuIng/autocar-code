"""camera-pair-v2 baseline + perspective corridor + junction switch, per frame.

Path option camera-pair-v2-junction (experimental PID). The baseline is the
unchanged OffscreenLine (camera-pair-v2 / offscreen-center-r2); the corridor
is junction-corridor-r1.3; the switch is junction-switch-r1-fallback-handback-r2
(see junction_switch.py). The selected path goes to the unchanged OffscreenPID.

Validity is decided from the selected source, never inherited:
  baseline   -> the baseline's own validity
  corridor   -> the corridor path reaches the control row (current support)
  predicted  -> NOT valid for control (shown and logged only), so the existing
                0.4 s no-valid-path stop counts from the last supported frame
  none       -> not valid
This does not detect curves or junctions; it reacts to corridor support and
disagreement. No actuator access; only path computation.
"""
import cv2
import numpy as np
from offscreen_center import OffscreenLine
from junction_corridor import CorridorTracker, REVISION as CORRIDOR_REVISION, COARSE_SKIP as JC_SKIP
from junction_switch import FallbackHandback, x_at
try:
    import corridor_parallel as CPAR   # pathfast2-r1: corridor in parallel with the baseline (pathfast.json; absent = off)
except Exception:
    CPAR = None

REVISION = 'camera-pair-v2-junction-r5'
try:
    import crossings_worker as CW   # crossings-worker-r1 (missing module -> all candidate rules off)
except Exception:
    CW = None
# r4: weak (one-side) support may hold the corridor 2 s instead of 1 s. Live run
# 2fef254f stopped 1 s after the island tip while the corridor had only the
# island edge and the baseline was unusable; hand-push 853059f6 shows ~1.8 s of
# weak support there. weak-hold.json: same earlier-footage scores, both
# 853059f6 junctions then release normally; cost: f0e2cc54 (wrong-geometry run)
# stops 1 s later. Experiment value, not a safety limit.
LIVE_SWITCH = dict(WEAK_S=2.0)


def _path(points):
    """1280x720 points ordered by increasing y, inside 0 <= y < 720.
    OffscreenPID's continuous_paths rejects the whole path if y does not
    increase (r1: corridor emitted bottom-up, run a9721c2a) or if any point is
    outside the image rows (r5: after a calibration with the horizon above the
    frame the corridor path started at y=-48, preview after the 2026-10-31
    reboot). Rows outside the image carry no evidence; x stays unclipped."""
    return sorted(([float(x), float(y)] for x, y in (points or []) if 0 <= y < 720), key=lambda q: q[1])


class JunctionLine(OffscreenLine):
    algorithm = 'camera-pair-v2-junction'

    def set_row(self, row):
        super().set_row(row)
        # ground-identity-r1 (2026-11-09): ground_identity.json enabled -> single-line identity by ground geometry;
        # disabled / missing / any load error -> the original OffscreenEstimate (unchanged behaviour)
        self.ground_identity = None
        # crossings-worker-r1: crossings_worker.json read here, at each model load (all keys off = unchanged behaviour)
        try: self.cw = CW.load_config() if CW else None
        except Exception: self.cw = None
        try:
            import ground_identity
            gc = ground_identity.load_config()
            if gc.get('enabled'): self.estimate = ground_identity.GroundIdentityEstimate().configure(gc); self.ground_identity = ground_identity.REVISION
            if self.cw and self.cw.get('amb_one_side') and hasattr(self.estimate, 'amb_one_side'): self.estimate.amb_one_side = True   # crossings-worker-r1
            if self.cw and self.cw.get('slope_width') is not None and hasattr(self.estimate, 'slope_width'): self.estimate.slope_width = dict(self.cw['slope_width'])   # slope-width-r1
        except Exception as e: self.ground_identity = 'LOAD_ERROR:' + type(e).__name__
        self._reset_junction()
        # The worker calls estimate.reset_route() on a new route epoch
        # (preview -> PID run); reset corridor and switch at the same time.
        reset_baseline = self.estimate.reset_route

        def reset_route():
            reset_baseline(); self._reset_junction()
        self.estimate.reset_route = reset_route

    def _reset_junction(self):
        self.corridor = CorridorTracker(); self.switch = FallbackHandback(**LIVE_SWITCH)
        cw = getattr(self, 'cw', None)   # crossings-worker-r1: corridor calibration gate / fallback continuity (None = off)
        if cw:
            if cw.get('corridor_cal_h_max') is not None: self.corridor.cal_h_max = float(cw['corridor_cal_h_max'])
            if cw.get('fallback_cont_px') is not None: self.switch.fb_cont_px = float(cw['fallback_cont_px'])
            if cw.get('outside_handback'):   # curve3-handback-r1
                self.switch.oh_on = True
                if cw.get('outside_handback_px') is not None: self.switch.oh_px = float(cw['outside_handback_px'])
                self.switch.oh_side = bool(cw.get('outside_handback_side'))
                if cw.get('outside_handback_hist_n'): self.switch.oh_hist_n = int(cw['outside_handback_hist_n'])
                if cw.get('outside_handback_hist_px') is not None: self.switch.oh_hist_px = float(cw['outside_handback_hist_px'])
            g = cw.get('corridor_guard')   # corridor-guard-r1 (corridor-20261118); None / {} = off
            if g:
                sw = self.switch; sw.cg_on = True; sw.cg_refused = {}; sw.cg_backs = 0
                sw.cg_off = bool(g.get('off'))
                if g.get('gz_max') is not None: sw.cg_gz_max = float(g['gz_max'])
                if g.get('cal_hmax') is not None: sw.cg_cal_hmax = float(g['cal_hmax'])
                if g.get('cal_age_s') is not None: sw.cg_cal_age_s = float(g['cal_age_s'])
                if g.get('dir_px') is not None: sw.cg_dir_px = float(g['dir_px'])
                if g.get('dir_dy') is not None: sw.cg_dir_dy = float(g['dir_dy'])
                if g.get('back'): sw.cg_back = dict(g['back'])
            cg = cw.get('corridor_cal_gate')   # corridor-cal-gate-r1 (corridor-weak-20261118); None / {} = off
            if cg:
                co = self.corridor; co.gate_rejects = {}
                if cg.get('hmax') is not None: co.gate_hmax = float(cg['hmax'])
                if cg.get('ws_max') is not None: co.gate_ws_max = float(cg['ws_max'])
                if cg.get('wide_drop_pitch') is not None: co.gate_wide_drop_pitch = float(cg['wide_drop_pitch'])
                if cg.get('pitch') is not None: co.gate_pitch = float(cg['pitch'])
            w = cw.get('corridor_weak')   # corridor-weak-r1 (corridor-weak-20261118); None / {} = off
            if w:
                sw = self.switch; sw.cw_on = True; sw.cw_backs = sw.cw_expires = 0; sw.cw_since = sw.cw_ramp = None
                if w.get('px') is not None: sw.cw_px = float(w['px'])
                if w.get('cont_px') is not None: sw.cw_cont_px = float(w['cont_px'])
                if w.get('hold_s') is not None: sw.cw_hold_s = float(w['hold_s'])
                if w.get('reasons') is not None: sw.cw_reasons = tuple(w['reasons'])
                if w.get('two_min') is not None: sw.cw_two_min = float(w['two_min'])
                if w.get('skip_zones') is not None: sw.cw_skip_zones = tuple(w['skip_zones'])
                sw.cw_expire = bool(w.get('expire'))
                if w.get('expire_px') is not None: sw.cw_expire_px = float(w['expire_px'])
                if w.get('ramp_s'): sw.cw_ramp_s = float(w['ramp_s'])
                if w.get('strong_take'): sw.cw_strong = dict(w['strong_take']); sw.cw_strong_refused = 0
                if w.get('gz_max') is not None: sw.cw_gz_max = float(w['gz_max'])
        if CPAR is not None: self.corridor = CPAR.wrap(self.corridor)   # pathfast2-r1 (mode off = the tracker itself)

    def step(self, image, mask, captured):
        if self.ground_identity == 'ground-identity-r1': self.estimate.read_imu()
        if JC_SKIP: self.corridor.zone_hint = getattr(self.estimate, 'cs_zone', None)   # pathfast2 C (off by default)
        if getattr(self.corridor, 'gate_rejects', None) is not None:   # corridor-cal-gate-r1: worker IMU pitch for the calibration gate
            self.corridor.gate_pitch_now = getattr(self.estimate, 'pitch', None) if getattr(self.estimate, 'imu_note', None) == 'OK' else None
        if getattr(self.corridor, 'parallel', False):   # pathfast2-r1: same inputs, the corridor runs while the baseline runs
            m = cv2.resize(mask.astype('uint8'), (640, 360), interpolation=cv2.INTER_NEAREST) > 0
            job = self.corridor.submit(m, captured)
            try: base = super().step(image, mask, captured)
            except BaseException:
                job.abandon(); raise   # the sequential code never reaches the corridor in this case: undo its step
            cor = job.result()
        else:
            base = super().step(image, mask, captured)
            m = cv2.resize(mask.astype('uint8'), (640, 360), interpolation=cv2.INTER_NEAREST) > 0
            cor = self.corridor.step(m, captured)
        y = self.row * 719 / 2
        car_path = base['path'] if base['valid'] else []
        # Offline scoring latches STOPPED ("the car would have stopped"). On the
        # car the stop itself belongs to LivePID's no-valid-path rule; a latched
        # switch kept the preview without any target after the vehicle was moved
        # back (r2, 2026-10-31). Re-evaluate from FOLLOW instead.
        unlatched = self.switch.state == 'STOPPED'
        if unlatched: self.switch.reset(None)
        if self.switch.oh_on: self.switch.oh_centre = getattr(self, 'center_px', None)   # curve3-handback-r1 (visual centre, 640 px)
        if self.switch.cw_on:   # corridor-weak-r1: baseline two-line fraction and the live_pid zone flag of this frame
            srcs = ((base.get('path_diagnostic') or {}).get('line_estimate') or {}).get('sources') or {}
            ns = sum(srcs.values()) if isinstance(srcs, dict) else 0
            self.switch.cw_two = (srcs.get('double', 0) / ns) if ns else None
            self.switch.cw_zone = getattr(self.estimate, 'cs_zone', None)
            mo = getattr(self.estimate, 'motion', None) if getattr(self.estimate, 'imu_note', None) == 'OK' else None
            self.switch.cw_gz = None if mo is None else float(mo[1])
        if self.switch.cg_on:   # corridor-guard-r1: worker IMU gz and the corridor calibration for the fallback guards
            mo = getattr(self.estimate, 'motion', None) if getattr(self.estimate, 'imu_note', None) == 'OK' else None
            self.switch.cg_gz = None if mo is None else float(mo[1])
            en = getattr(self.corridor, 'entrance', None)
            self.switch.cg_cal = None if not en else (float(en['horizon_y']), float(en['capture_t']))
        o = self.switch.step(captured, car_path, bool(base['valid']), cor, y)
        source = o['source']
        path = _path(o['target_path']) if source else []
        if source == 'baseline': valid = bool(base['valid'])
        elif source == 'corridor': valid = x_at(path, y) is not None
        else: valid = False
        mm = o['m']; sw = self.switch
        diag = base['path_diagnostic']
        diag['ground_identity_revision'] = self.ground_identity
        diag.update(path_revision=self.algorithm, controller_revision='offscreen-center-r2', junction=dict(
            revision=REVISION, switch_revision=sw.REVISION, corridor_revision=CORRIDOR_REVISION, parameters=dict(sw.p),
            state=o['state'], source=source, selected_valid=valid, prediction_controls=False,
            events=[dict(t=e['t'], before=e['before'], after=e['after'], reason=e['reason'],
                         d_near=e['d_near'], d_dir=e['d_dir'], target_jump=e['target_jump']) for e in o['events']],
            reason=getattr(sw, 'reason', None), since=sw.since,
            baseline_valid=bool(base['valid']), baseline_path=_path(base['path']), corridor_path=_path(cor.get('path')),
            corridor_status=cor.get('status'), left_rows=mm['L'], right_rows=mm['R'], support=mm['support'],
            last_strong_support_s=sw.last_strong, support_age_s=None if sw.last_strong is None else captured - sw.last_strong,
            control_row_y_640=y, baseline_x=mm['car'], corridor_x=mm['cor'], d_near=mm['d_near'], d_dir=mm['d_dir'],
            inside_current_corridor=mm['car_inside_now'], target_jump=o['jump'], stop_unlatched=unlatched))
        if getattr(self, 'cw', None) and CW.any_on(self.cw):   # crossings-worker-r1 log (only with a rule on)
            diag['junction']['crossings_worker'] = dict(revision=CW.REVISION, cal_rejects=getattr(self.corridor, 'cal_rejects', 0),
                                                        fb_refused=getattr(self.switch, 'fb_refused', 0), one_side=getattr(self.estimate, 'one_side_n', 0))
            if self.cw.get('outside_handback'):   # curve3-handback-r1 log (only with the rule on)
                sw = self.switch; diag['junction']['crossings_worker']['curve3'] = dict(revision=CW.CURVE3_REVISION, handbacks=sw.oh_n, px=sw.oh_px, side=sw.oh_side, hist_n=sw.oh_hist_n, hist_px=sw.oh_hist_px)
            if getattr(self.corridor, 'gate_rejects', None) is not None:   # corridor-cal-gate-r1 log (only with the gate on)
                co = self.corridor; en = co.entrance or {}
                diag['junction']['crossings_worker']['corridor_cal_gate'] = dict(revision=CW.CAL_GATE_REVISION, rejects=dict(co.gate_rejects),
                                                                                  cal_h=None if not en else round(float(en['horizon_y']), 1), cal_ws=None if not en else round(float(en['width_slope']), 3),
                                                                                  cal_default=bool(en.get('default')) if en else None)
            if self.switch.cw_on:   # corridor-weak-r1 log (only with the rule on)
                sw = self.switch; diag['junction']['crossings_worker']['corridor_weak'] = dict(revision=CW.WEAK_REVISION, backs=sw.cw_backs, expires=sw.cw_expires, strong_refused=sw.cw_strong_refused,
                                                                                              two=None if sw.cw_two is None else round(sw.cw_two, 2), zone=sw.cw_zone, gz=sw.cw_gz,
                                                                                              ramp_w=o.get('cw_ramp_w'))
            if self.switch.cg_on:   # corridor-guard-r1 log (only with the guard on)
                sw = self.switch; diag['junction']['crossings_worker']['corridor_guard'] = dict(revision=CW.GUARD_REVISION, refused=dict(sw.cg_refused or {}), backs=sw.cg_backs, gz=sw.cg_gz,
                                                                                                cal_h=None if sw.cg_cal is None else round(sw.cg_cal[0], 1), cal_age_s=None if sw.cg_cal is None else round(captured - sw.cg_cal[1], 2))
        if source != 'baseline':
            diag['road_state'] = dict(diag.get('road_state') or {}, source='junction_corridor' if valid else 'invalid',
                                      predicted=source == 'corridor_predicted',
                                      reason='JUNCTION_CORRIDOR' if valid else ('JUNCTION_PREDICTION_NOT_CONTROL' if source == 'corridor_predicted' else 'JUNCTION_NO_TARGET'))
        return dict(base, path=path if valid or source == 'corridor_predicted' else [], valid=valid, path_diagnostic=diag)
