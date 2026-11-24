"""Experimental preview release using current position and derivative demand.
No actuator access; travel, slew and freshness limits stay in the original PID.
"""
from preview_turn_pid import PreviewTurnPID, PreviewPositionPID

class ReleasePositionPID(PreviewPositionPID):
    release_scale = 1.
    release_reason = "UNCHANGED"
    def step(self, t, error, imu=None):
        self.release_scale = 1.
        self.release_reason = 'UNCHANGED'
        dt = t-self.t if self.t is not None else None
        if error is not None and dt is not None and 0 < dt <= self.p['max_gap']:
            derivative = self.d+dt/(self.p['derivative_tau']+dt)*((error-self.e)/dt-self.d)
            p = self.p['kp']*error
            d = self.p['kd']*derivative
            ff = self.correction_us
            # Only release while the present error is moving against preview.
            # Continuous at derivative sign changes, even after position crosses
            # zero. No absolute error threshold, timer or stale-error latch.
            if ff*d < 0:
                self.release_scale = max(0., 1.-abs(d)/abs(p)) if p != 0 else 0.
                self.release_reason = 'CURRENT_PD_RETURN' if self.release_scale < 1 else 'UNCHANGED'
        return super().step(t,error,imu)

    def feedforward(self):
        return self.correction_us*self.release_scale

class ReleasePreviewPID(PreviewTurnPID):
    def __init__(self, parameters):
        super().__init__(parameters)
        self.pid = ReleasePositionPID(parameters)
        self.policy.update(revision='preview-exit-release-r1', max_us=150.,
                           release_policy='current-pd-return')

    def step_path(self,t,points,envelope=None):
        result=super().step_path(t,points,envelope)
        detail=result['turn_preview']
        requested=detail['correction_us']
        applied=requested*self.pid.release_scale if result['valid'] else 0.
        detail.update(requested_correction_us=requested, correction_us=applied,
                      release_scale=self.pid.release_scale, release_reason=self.pid.release_reason,
                      position_desired_us=result['desired_us']-applied if result.get('desired_us') is not None else None)
        return result
