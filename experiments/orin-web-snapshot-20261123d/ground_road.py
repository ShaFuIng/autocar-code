"""Offline provisional ground geometry. No devices, no learned labels.

Six manually placed points, revision 7, have unconfirmed endpoint flags and
approximately +/-3 cm distance uncertainty. Ground coordinates are estimates
relative to the front tyre edge, NOT a calibrated axle or wheel-angle model.
"""
import math
import json
from pathlib import Path
import cv2
import numpy as np
from visual_pid import PID
from line_width_prior import prior_width
from k_adaptive_pid import KAdaptivePID, continuous_paths


class BearingErrorPID(PID):
    scale = 1.
    def step(self,t,error,imu=None):
        # A single bearing error replaces position+curvature double counting.
        converted=None if error is None else self.scale*math.atan(error/self.scale)
        if getattr(self,'rebase_image_error',False) and self.e is not None:
            self.e=self.scale*math.atan(self.e/self.scale)
        result=super().step(t,converted,imu)
        result['image_error']=error
        return result


class SharedRoadPID(KAdaptivePID):
    """Single target bearing; keeps existing observed-row support and zero.

For a planar pinhole camera X/Z is proportional to x-cx. The measured
homography supplies the local proportionality; it does not certify accuracy.
No polynomial extrapolation to the unobserved vehicle position is performed.
"""
    def __init__(self,parameters,geometry=None):
        super().__init__(parameters)
        if geometry is None:
            geometry=GroundGeometry(json.loads(Path(__file__).with_name('ground_road_calibration.json').read_text(encoding='utf8')))
        if parameters.get('curve_outward_px_640',0):
            raise ValueError('Shared road candidate requires outward=0')
        self.pid=BearingErrorPID(parameters)
        center=319.75+parameters.get('visual_center_offset_px_640',0.)
        # The same saved image zero is retained, rather than adopting the old
        # calibration photograph's lateral origin as today's steering zero.
        ref=geometry.project([[center,216],[center+1,216]])
        self.pid.scale=float(ref[0,1]/(ref[1,0]-ref[0,0])/319.75)
        if not .1<self.pid.scale<10:raise ValueError('Invalid provisional bearing scale')
        self.geometry=geometry
        self.policy=dict(revision='shared-road-bearing-r1',feedforward=False,
            bearing_scale=self.pid.scale,metric_calibrated=False)

    def step_path(self,t,points,envelope=None):
        choices=[]
        for chain in continuous_paths(points):
            lo=max(self.requested,chain[0][1]/719+.001);hi=min(.95,chain[-1][1]/719-.001)
            if lo<=hi:choices.append((lo,hi))
        self.pid.rebase_image_error=False
        if choices:
            lo,hi=min(choices,key=lambda c:(c[0],-c[1]));dt=max(0,t-self.t) if self.t is not None else 0
            row=max(lo,min(hi,self.row+max(-.2*dt,min(.2*dt,lo-self.row))))
            self.pid.rebase_image_error=abs(row-self.row)>1e-8
        result=super().step_path(t,points,envelope)
        detail=dict(policy=self.policy,reason='SUPPORTED' if result['valid'] else result['reason'],
            raw_us=0.,correction_us=0.,metric_calibrated=False)
        # Report local geometry independently; do not add it to the command.
        if len(self.previous)>=5:
            p=np.asarray(self.previous)/2
            g=self.geometry.project(p)
            axis=self.geometry.project(np.column_stack([np.full(len(p),319.75+self.pid.p.get('visual_center_offset_px_640',0)),p[:,1]]))
            g[:,0]-=axis[:,0]
            keep=np.isfinite(g).all(axis=1)&(g[:,1]>.25)&(g[:,1]<1.5)
            g=g[keep]
            if len(g)>=5 and np.ptp(g[:,1])>.15:
                z=float(self.geometry.project([[319.75,self.row*719/2]])[0,1])
                a=np.polyfit(g[:,1]-z,g[:,0],2)
                detail.update(local_offset_m=float(a[2]),local_heading_rad=float(math.atan(a[1])),
                    local_curvature_per_m=float(2*a[0]/(1+a[1]**2)**1.5),reference_z_m=z,
                    state_reference='at visible sample, not vehicle-position measurement')
        result.update(turn_preview=detail,sampling_version='shared-road-bearing-r1')
        return result


class GroundGeometry:
    def __init__(self, document):
        lines = document['camera-ground-measure']['data']['lines']
        pixels, ground = [], []
        for line in lines:
            for side, x in [('left', -.23), ('right', .23)]:
                pixels.append(line[side]); ground.append([x, line['cm']/100])
        self.h, _ = cv2.findHomography(np.array(pixels), np.array(ground), 0)
        self.inv = np.linalg.inv(self.h)
        recovered = self.project(pixels)
        self.calibration_residual_m = float(np.max(np.linalg.norm(recovered-ground, axis=1)))

    def project(self, points, inverse=False):
        p = np.asarray(points, dtype=float).reshape(-1, 2)
        q = np.column_stack([p, np.ones(len(p))]) @ (self.inv if inverse else self.h).T
        with np.errstate(divide='ignore', invalid='ignore'):
            return q[:, :2]/q[:, 2:]

    def estimate(self, boundaries):
        curves, diagnostics = [], {}
        for side in ['L', 'R']:
            pts = boundaries.get(side)
            if not pts or len(pts) < 6:
                continue
            p = self.project([[x, y] for y, x in sorted(pts.items())])
            p = p[np.isfinite(p).all(axis=1) & (p[:, 1] >= .25) & (p[:, 1] <= 1.5)]
            if len(p) < 6 or np.ptp(p[:, 1]) < .15:
                continue
            p = p[np.argsort(p[:, 1])]
            # X(Z) is only a local visible boundary, never extrapolated to Z=0.
            a = np.polyfit(p[:, 1], p[:, 0], 2)
            z = np.linspace(p[0, 1], p[-1, 1], 80)
            x = np.polyval(a, z); slope = np.polyval(np.polyder(a), z)
            sign = 1 if side == 'L' else -1
            norm = np.sqrt(1+slope*slope)
            center = np.column_stack([x+sign*.23/norm, z-sign*.23*slope/norm])
            # A folded offset is not a valid single-valued local road model.
            if np.any(np.diff(center[:, 1]) <= 0):
                diagnostics[side] = {'reason': 'FOLDED_NORMAL_OFFSET'}
                continue
            curves.append(center)
            diagnostics[side] = dict(reason='OBSERVED_BOUNDARY_NORMAL',
                fit_rms_m=float(np.sqrt(np.mean((np.polyval(a, p[:, 1])-p[:, 0])**2))),
                observed_z=[float(p[0, 1]), float(p[-1, 1])])
        if not curves:
            return [], dict(reason='NO_GROUND_SUPPORT', boundaries=diagnostics)
        # Fuse only shared observed support. Do not concatenate different roads.
        if len(curves) == 2:
            lo=max(c[0, 1] for c in curves); hi=min(c[-1, 1] for c in curves)
            if hi <= lo:
                return [], dict(reason='NO_SHARED_SUPPORT', boundaries=diagnostics)
            z=np.linspace(lo, hi, 80)
            xx=[np.interp(z,c[:,1],c[:,0]) for c in curves]
            mismatch=float(np.median(np.abs(xx[0]-xx[1])))
            if mismatch>.23:
                return [], dict(reason='BOUNDARIES_DISAGREE', disagreement_m=mismatch, boundaries=diagnostics)
            center=np.column_stack([np.mean(xx,axis=0),z])
        else:
            center=curves[0]
        pixels=self.project(center, inverse=True)
        good=np.isfinite(pixels).all(axis=1)&(pixels[:,0]>=0)&(pixels[:,0]<640)&(pixels[:,1]>=0)&(pixels[:,1]<360)
        pixels=pixels[good]
        pixels=pixels[np.argsort(pixels[:,1])]
        if len(pixels)<3:
            return [],dict(reason='CENTER_OUTSIDE_IMAGE',boundaries=diagnostics)
        ys=np.arange(math.ceil(pixels[0,1]/4)*4, pixels[-1,1]+.01,4)
        path=np.column_stack([np.interp(ys,pixels[:,1],pixels[:,0]),ys])*2
        return path.tolist(),dict(reason='GROUND_NORMAL_CENTER',boundaries=diagnostics,
            ground_center=center.tolist(), metric_calibrated=False, width_source='user_measured_46cm',
            calibration_source='camera-ground-measure revision 7; provisional')


class GroundAimPID:
    """One geometrical aim error, no separately added curvature command.

Keeps PID gains, freshness, travel and slew limits. The error units change;
unchanged gains do NOT imply unchanged closed-loop behaviour.
"""
    def __init__(self, parameters, geometry):
        if parameters.get('curve_outward_px_640',0) or parameters.get('direction_enabled',0):
            raise ValueError('Ground aim requires outward=0 and direction_enabled=0')
        self.pid=PID(parameters); self.geometry=geometry
        self.policy=dict(revision='ground-aim-r1',lookahead_m=.50,feedforward=False,metric_calibrated=False)

    def step_path(self,t,points,envelope=None):
        g=self.geometry.project(np.asarray(points,dtype=float).reshape(-1,2)/2)
        g=g[np.isfinite(g).all(axis=1)&(g[:,1]>.15)&(g[:,1]<2)]
        detail=dict(policy=self.policy,reason='NO_OBSERVED_LOOKAHEAD',metric_calibrated=False)
        error=None
        if len(g)>=3:
            g=g[np.argsort(g[:,1])]; z=.5
            if g[0,1]<=z<=g[-1,1]:
                x=float(np.interp(z,g[:,1],g[:,0]))
                angle=math.atan2(x,z)
                # Match local small-angle image error scaling at 50cm, not a
                # claimed conversion from desired wheel angle to servo PWM.
                px=self.geometry.project([[0,z],[.01,z]],inverse=True)
                scale=float((px[1,0]-px[0,0])/.01*z/319.75)
                error=max(-1.,min(1.,angle*scale))
                local=g[np.abs(g[:,1]-z)<=.2]
                detail.update(reason='SUPPORTED',aim_x_m=x,aim_z_m=z,aim_angle_rad=angle,
                    error_scale=scale, correction_us=0., raw_us=0.)
                if len(local)>=5 and np.ptp(local[:,1])>.1:
                    c=np.polyfit(local[:,1]-z,local[:,0],2)
                    detail.update(local_offset_m=float(c[2]),local_heading_rad=float(math.atan(c[1])),
                        local_curvature_per_m=float(2*c[0]/(1+c[1]**2)**1.5),
                        state_reference='at lookahead, NOT measured vehicle lateral offset')
        result=self.pid.step(t,error)
        result.update(turn_preview=detail,sampling_version='ground-aim-r1',sample_reason=detail['reason'],
            sample_row_ratio=None,requested_row_ratio=self.pid.p['row_ratio'])
        return result
