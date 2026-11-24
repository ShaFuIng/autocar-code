"""Experimental two-point image target, not a measured wheel-angle controller.

Current continuous path only; no full K, optical flow or actuator feedback.
The near/far normalized target enters both P and D, with no separate turn bias.
"""
import math
from visual_pid import PID, path_error
from k_adaptive_pid import continuous_paths
from line_width_prior import prior_width
from road_state_path import RoadStateLine, IdentityEstimate


class PreviewErrorLine(RoadStateLine):
    algorithm = 'preview-error-v1'
    def set_row(self, row):
        super().set_row(row)
        # Keep the deployed v1.2 estimator; width-relaxation experiment rejected.

    def step(self, image, mask, captured):
        result = super().step(image, mask, captured)
        result['path_diagnostic']['path_revision'] = self.algorithm
        return result


class NearDerivativePID(PID):
    near_error=None
    previous_near=None
    def step(self,t,error,imu=None):
        # P/I use the lookahead target; D damps observed near-position change.
        # Differentiating the far target amplified fork/short-track jitter.
        if error is not None and self.e is not None and self.previous_near is not None and self.near_error is not None:
            self.e=error-(self.near_error-self.previous_near)
        result=super().step(t,error,imu)
        self.previous_near=self.near_error if error is not None else None
        return result


class PreviewErrorPID:
    width = staticmethod(prior_width)
    path_error = staticmethod(path_error)
    continuous_paths = staticmethod(continuous_paths)
    fixed_row = False
    def __init__(self, parameters):
        if parameters.get('curve_outward_px_640',0) or parameters.get('direction_enabled',0):
            raise ValueError('Preview error requires outward=0 and legacy direction=0')
        self.pid = NearDerivativePID(parameters)
        self.requested = parameters['row_ratio']
        self.row = self.requested
        self.t = None
        self.previous = []
        self.previous_rows = None
        self.direction=0.
        self.policy = dict(revision='preview-error-r1', far_row=.45, far_weight=.35,
            min_span=.06, direction_tau_s=.15, derivative_source="near_position", feedforward=False, metric_calibrated=False)

    def errors(self, chain, near, far):
        offset = self.pid.p.get('visual_center_offset_px_640',0.)
        en = self.path_error(chain, near, visual_center_offset_px_640=offset)
        ef = self.path_error(chain, far, visual_center_offset_px_640=offset) if far is not None else None
        if en is None:
            return None, None, None
        # Perspective normalization keeps a parallel, laterally displaced road
        # from becoming a false heading term purely due to convergence.
        projected = ef*self.width(near*719/2)/self.width(far*719/2) if ef is not None else en
        target = en+self.policy['far_weight']*(projected-en) if ef is not None else en
        return en, projected, max(-1.,min(1.,target))

    def step_path(self, t, points, envelope=None):
        choices=[]
        for chain in self.continuous_paths(points):
            lo=max(self.requested,chain[0][1]/719+.001)
            hi=min(.95,chain[-1][1]/719-.001)
            if self.fixed_row:
                if chain[0][1]/719 <= self.requested <= chain[-1][1]/719:
                    choices.append((self.requested,self.requested,chain))
            elif lo<=hi: choices.append((lo,hi,chain))
        bounds=None; chain=[]; far=None; en=projected=target=None; rebase=False; clamped=False
        old=self.row
        if choices:
            lo,hi,chain=min(choices,key=lambda c:(c[0],-c[1]));bounds=[lo,hi]
            dt=max(0.,t-self.t) if self.t is not None else 0.
            self.row=max(lo,min(hi,old+max(-.2*dt,min(.2*dt,lo-old))))
            clamped=abs(self.row-old)>.2*dt+1e-6
            candidate=max(self.policy['far_row'],chain[0][1]/719+.001)
            if self.row-candidate>=self.policy['min_span']:far=candidate
            en,projected,target=self.errors(chain,self.row,far)
            rows=(self.row,far)
            if self.previous_rows!=rows and self.pid.e is not None:
                previous,_,_=self.errors(self.previous,self.row,far)
                self.pid.previous_near=en if previous is None else previous
                if previous is None:self.pid.d=0.
                rebase=True
            self.previous_rows=rows
        dt=t-self.t if self.t is not None else 0.
        raw_direction=(projected-en)*self.policy['far_weight'] if far is not None else 0.
        if not 0<dt<=self.pid.p['max_gap'] or far is None:
            self.direction=0.
        else:
            self.direction+=dt/(self.policy['direction_tau_s']+dt)*(raw_direction-self.direction)
        target=max(-1.,min(1.,en+self.direction)) if en is not None else None
        self.pid.near_error=en
        result=self.pid.step(t,target)
        detail=dict(policy=dict(self.policy),reason='TWO_POINT' if far is not None else 'NEAR_ONLY' if bounds else 'NO_POSITION_PATH',
            near_error=en,far_projected_error=projected,control_error=target,
            near_row=self.row,far_row=far,raw_us=0.,correction_us=0.,metric_calibrated=False)
        result.update(image_error=en,turn_preview=detail,sampling_version='preview-error-r1',
            visual_center_version=1,visual_center_offset_px_640=self.pid.p.get('visual_center_offset_px_640',0.),
            requested_row_ratio=self.requested,sample_row_ratio=self.row,sample_bounds=bounds,
            sample_reason='NO_SUPPORTED_ROW' if bounds is None else 'ADAPTED_NEAR' if self.row>self.requested+.001 else 'REQUESTED',
            derivative_rebased=rebase,sample_clamped=clamped)
        self.t=t;self.previous=chain
        return result
