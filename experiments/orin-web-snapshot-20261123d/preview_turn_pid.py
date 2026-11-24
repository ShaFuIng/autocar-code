"""Image-space preview correction, no actuator access or metric calibration.

The path is unchanged. Perspective-normalized lateral offsets distinguish
heading from the convergence of parallel boundaries in the image. This is an
experimental approximation using the existing width prior, not wheel angle.
"""
import math
from k_adaptive_pid import KAdaptivePID, continuous_paths
from visual_pid import PID
from line_width_prior import prior_width

POLICY = dict(revision='preview-turn-r2', far_row=.45, min_row_span=.06, direction_row_span=.15,
              gain_us=400., max_us=90., tau_s=.15)


class PreviewPositionPID(PID):
    correction_us = 0.

    def feedforward(self):
        return self.correction_us


class PreviewTurnPID(KAdaptivePID):
    def __init__(self, parameters):
        super().__init__(parameters)
        if parameters.get('curve_outward_px_640', 0):
            raise ValueError('提前轉向版本外移須設0')
        self.pid = PreviewPositionPID(parameters)
        self.policy = dict(POLICY)
        self.turn_t = None
        self.turn_us = 0.

    def step_path(self, t, points, envelope=None):
        cfg = self.policy
        dt = t-self.turn_t if self.turn_t is not None else 0.
        if not 0 < dt <= self.pid.p['max_gap']:
            self.turn_us = 0.
        choices = []
        for chain in continuous_paths(points):
            lo = max(self.requested, chain[0][1]/719+.001)
            hi = min(.95, chain[-1][1]/719-.001)
            if lo <= hi:
                choices.append((lo, hi, chain))
        detail = dict(policy=dict(cfg), reason='NO_POSITION_PATH',
                      near_point=None, far_point=None, direction_delta=None,
                      raw_us=0., correction_us=0., metric_calibration=False)
        supported = False
        if choices:
            lo, hi, chain = min(choices, key=lambda c:(c[0], -c[1]))
            sample_dt = max(0, t-self.t) if self.t is not None else 0.
            near = max(lo, min(hi, self.row+max(-.2*sample_dt, min(.2*sample_dt, lo-self.row))))
            far = max(cfg['far_row'], chain[0][1]/719+.001)
            detail['position_sample_row'] = near
            # Direction uses the current chain independently of the PID sample.
            # Never bridge gaps, extend the path, or retain a past direction.
            near = max(near, min(hi, far+cfg['direction_row_span']))
            detail['reason'] = 'SHORT_CONTINUOUS_PATH'
            if near-far >= cfg['min_row_span']:
                offset = self.pid.p.get('visual_center_offset_px_640', 0.)
                # Keep unclipped coordinates for the preview geometry. path_error
                # intentionally clips position errors; all these points are in frame.
                def x_at(row):
                    y = row*719
                    for (x0,y0),(x1,y1) in zip(chain,chain[1:]):
                        if y0 <= y <= y1:
                            return (x0+(x1-x0)*(y-y0)/(y1-y0))/2
                xn, xf = x_at(near), x_at(far)
                center = 319.75+offset
                delta = ((xf-center)/prior_width(far*719/2)
                         -(xn-center)/prior_width(near*719/2))
                raw = max(-cfg['max_us'], min(cfg['max_us'], cfg['gain_us']*delta))
                supported = math.isfinite(raw)
                if supported:
                    alpha = dt/(cfg['tau_s']+dt) if 0 < dt <= self.pid.p['max_gap'] else 0.
                    self.turn_us += alpha*(raw-self.turn_us)
                    detail.update(reason='SUPPORTED', near_point=[xn,near*719/2],
                                  far_point=[xf,far*719/2], direction_delta=delta, raw_us=raw)
        if not supported:
            self.turn_us = 0.  # No old direction on missing/disconnected evidence.
        self.pid.correction_us = self.turn_us
        old_i = self.pid.i
        result = super().step_path(t, points, envelope)
        if not result['valid']:
            self.turn_us = 0.
            self.pid.correction_us = 0.
        elif result.get('limited'):
            # Additional command saturation must not wind up position integration.
            self.pid.i = old_i
        detail['correction_us'] = self.turn_us
        detail['position_desired_us'] = (result['desired_us']-self.turn_us
                                         if result.get('desired_us') is not None else None)
        result['turn_preview'] = detail
        result['sampling_version'] = cfg['revision']
        self.turn_t = t
        return result
