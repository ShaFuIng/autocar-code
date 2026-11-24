"""Visual PID candidate. File replay only; deliberately has no actuator adapter."""
import math

DEFAULTS = dict(kp=120., ki=0., kd=12., left_gain=1., right_gain=1.,
                center_us=1300., min_us=940., max_us=1900., steer_rate=300.,
                pwm=15., row_ratio=.75, derivative_tau=.15, integral_limit=.3,
                max_gap=.4)

VISION_DEFAULTS = dict(visual_center_offset_px_640=0.,curve_outward_px_640=0.)

DIRECTION_DEFAULTS = dict(direction_enabled=0, direction_far_row=.70, direction_weight=.5)

# Two-point preview of the camera-pair / offscreen-center controller (OffscreenPID):
# control error = near + weight * (projected far - near). Defaults are the values
# that were fixed in code before 2026-10-31; other controllers ignore them.
PREVIEW_DEFAULTS = dict(preview_far_weight=.35, preview_far_row=.45)

BACKLASH_DEFAULTS = dict(backlash_enabled=0, backlash_left_us=0., backlash_right_us=0.,
                         backlash_threshold_us=10., backlash_rate=300.,
                         backlash_initial_enabled=0, backlash_initial_left_us=0.,
                         backlash_initial_right_us=0., backlash_initial_threshold_us=10.,
                         backlash_initial_rate=300.)

def validate(p):
    if not isinstance(p, dict) or not set(DEFAULTS) <= set(p) or set(p) - set(DEFAULTS) - set(BACKLASH_DEFAULTS) - set(DIRECTION_DEFAULTS) - set(VISION_DEFAULTS) - set(PREVIEW_DEFAULTS):
        raise ValueError('PID參數欄位不完整或不支援')
    limits = dict(kp=(0,600),ki=(0,200),kd=(0,200),left_gain=(.1,3),right_gain=(.1,3),
                  center_us=(500,2500),min_us=(500,2500),max_us=(500,2500),
                  steer_rate=(50,1500),pwm=(0,25),row_ratio=(.1,.95),
                  derivative_tau=(.01,1),integral_limit=(0,2),max_gap=(.05,1))
    for k,(lo,hi) in limits.items():
        if type(p[k]) not in (int,float) or not math.isfinite(p[k]) or not lo<=p[k]<=hi:
            raise ValueError(f'{k} 須介於 {lo}～{hi}')
    if not p['min_us'] < p['center_us'] < p['max_us']:
        raise ValueError('左限位 < 中心 < 右限位')
    extras = dict(BACKLASH_DEFAULTS, **{k:p[k] for k in BACKLASH_DEFAULTS if k in p})
    for k,(lo,hi) in dict(backlash_enabled=(0,1),backlash_left_us=(0,200),backlash_right_us=(0,200),backlash_threshold_us=(1,100),backlash_rate=(50,800),backlash_initial_enabled=(0,1),backlash_initial_left_us=(0,200),backlash_initial_right_us=(0,200),backlash_initial_threshold_us=(1,100),backlash_initial_rate=(50,800)).items():
        if type(extras[k]) not in (int,float) or not math.isfinite(extras[k]) or not lo<=extras[k]<=hi:
            raise ValueError('Invalid backlash parameter: '+k)
    if any(extras[k] not in (0,1) for k in ('backlash_enabled','backlash_initial_enabled')):raise ValueError('Backlash enable must be 0 or 1')
    direction=dict(DIRECTION_DEFAULTS, **{k:p[k] for k in DIRECTION_DEFAULTS if k in p})
    for k,(lo,hi) in dict(direction_enabled=(0,1),direction_far_row=(.1,.94),direction_weight=(0,2)).items():
        v=direction[k]
        if type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError('Invalid direction parameter: '+k)
    if direction['direction_enabled'] not in (0,1):raise ValueError('Direction enable must be 0 or 1')
    preview=dict(PREVIEW_DEFAULTS, **{k:p[k] for k in PREVIEW_DEFAULTS if k in p})
    for k,(lo,hi) in dict(preview_far_weight=(0,1),preview_far_row=(.1,.9)).items():
        v=preview[k]
        if type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError('Invalid preview parameter: '+k)
    if preview['preview_far_row']>=p['row_ratio']:raise ValueError('preview_far_row 須小於前視列 row_ratio')
    if direction['direction_enabled'] and direction['direction_far_row']>p['row_ratio']-.02:raise ValueError('遠列必須比近列至少小0.02')
    offset=p.get('visual_center_offset_px_640',0.)
    if type(offset) not in (int,float) or not math.isfinite(offset) or not -319<=offset<=319:
        raise ValueError('視覺中心偏移須為有限數值，範圍-319～319（640寬像素）')
    outward=p.get('curve_outward_px_640',0.)
    if type(outward) not in (int,float) or not math.isfinite(outward) or not 0<=outward<=320:
        raise ValueError('彎道外移量須為0～320的有限數值（640寬）')
    return dict(p)

class _PID:
    def __init__(self,p):
        self.p=validate(p);self.reset()

    def reset(self):
        self.t=None;self.e=None;self.i=0.;self.d=0.;self.last=self.p['center_us']

    def step_path(self,t,points):
        features=path_features(points,self.p)
        result=self.step(t,features['control_error'])
        result.update(features)
        return result

    def feedforward(self):
        """Optional command contribution; zero for all existing controllers."""
        return 0.

    def step(self,t,error,imu=None):
        # Reserved synchronized input contract; deliberately not part of control.
        # imu: {sample_t: seconds on same clock, yaw_rate_dps: left positive,
        #       valid: bool}. Missing/invalid IMU never substitutes a vision error.
        if imu is not None:
            if not isinstance(imu,dict) or type(imu.get('valid')) is not bool:
                raise ValueError('IMU接口須提供valid、sample_t與yaw_rate_dps')
            for key in ('sample_t','yaw_rate_dps'):
                if type(imu.get(key)) not in (int,float) or not math.isfinite(imu[key]):
                    raise ValueError('無效IMU數值')
        p=self.p
        if type(t) not in (int,float) or not math.isfinite(t):
            raise ValueError('無效時間')
        dt=None if self.t is None else t-self.t
        reason=None
        if error is None:reason='NO_PATH_AT_LOOKAHEAD'
        elif type(error) not in (int,float) or not math.isfinite(error) or abs(error)>1:reason='INVALID_ERROR'
        elif dt is not None and (dt<=0 or dt>p['max_gap']):reason='INVALID_OR_STALE_TIME'
        if reason:
            self.reset()
            return dict(t=t,valid=False,reason=reason,pwm=0,steer_us=None,hardware_output=False)
        if dt is None:
            self.t=t;self.e=error
            return dict(t=t,error=error,valid=False,reason='WARMUP',pwm=0,steer_us=None,hardware_output=False)
        d=self.d+dt/(p['derivative_tau']+dt)*((error-self.e)/dt-self.d)
        candidate=max(-p['integral_limit'],min(p['integral_limit'],self.i+error*dt))
        def output(integral):
            terms=(p['kp']*error,p['ki']*integral,p['kd']*d)
            u=sum(terms);gain=p['right_gain'] if u>=0 else p['left_gain']
            target=p['center_us']+u*gain+self.feedforward()
            bounded=max(p['min_us'],min(p['max_us'],target))
            limited=max(self.last-p['steer_rate']*dt,min(self.last+p['steer_rate']*dt,bounded))
            return terms,target,limited
        terms,target,limited=output(candidate)
        # Conditional integration handles both travel and slew saturation.
        if (target-limited)*error<=0:
            self.i=candidate
        terms,target,limited=output(self.i)
        self.t=t;self.e=error;self.d=d;self.last=limited
        return dict(t=t,error=error,valid=True,reason='PROPOSAL_ONLY',p=terms[0],i=terms[1],d=terms[2],
                    desired_us=target,steer_us=limited,limited=abs(target-limited)>.001,
                    pwm=p['pwm'],hardware_output=False)

class LegacyPID(_PID):
    """Bounded, direction-memory compensation; no actual wheel-angle estimate."""
    compensation_version = 2

    def compensation(self, raw, result, cfg):
        return self.bl_offset, {}

    def reset(self):
        super().reset()
        self.bl_direction=0;self.bl_extreme=None;self.bl_offset=0.
        self.bl_last=self.p['center_us'];self.bl_time=None;self.bl_reversals=0;self.bl_source='none';self.bl_initial_count=0

    def step(self,t,error,imu=None):
        previous_i=self.i;previous_t=self.t
        result=super().step(t,error,imu)
        cfg=dict(BACKLASH_DEFAULTS, **{k:self.p[k] for k in BACKLASH_DEFAULTS if k in self.p})
        raw=result.get('steer_us')
        result.update(raw_steer_us=raw,backlash_version=self.compensation_version,backlash_offset_us=0.,
                      backlash_direction=0,backlash_state='disabled',backlash_reversals=0,
                      backlash_source='none',backlash_initial_count=0)
        initial=bool(cfg['backlash_initial_enabled'] and (cfg['backlash_initial_left_us'] or cfg['backlash_initial_right_us']))
        reversal=bool(cfg['backlash_enabled'] and (cfg['backlash_left_us'] or cfg['backlash_right_us']))
        if not (initial or reversal):return result
        if not result['valid']:
            result['backlash_state']='reset_no_valid_path'
            return result
        dt=t-self.bl_time if self.bl_time is not None else t-(self.t or t)
        # First valid sample starts from center, with no guessed preload direction.
        if self.bl_time is None:dt=t-previous_t if previous_t is not None else 0.
        self.bl_time=t
        if self.bl_extreme is None:self.bl_extreme=self.p['center_us']
        threshold=cfg['backlash_initial_threshold_us'] if initial and self.bl_direction==0 else cfg['backlash_threshold_us']
        change=raw-self.bl_extreme;switched=False;first=False
        if self.bl_direction==0:
            if abs(change)>=threshold:
                self.bl_direction=1 if change>0 else -1;self.bl_extreme=raw
                if initial:
                    first=True;self.bl_initial_count=1;self.bl_source='initial'
                    self.bl_offset=cfg['backlash_initial_right_us'] if self.bl_direction>0 else -cfg['backlash_initial_left_us']
        elif self.bl_direction>0:
            if raw>self.bl_extreme:self.bl_extreme=raw
            elif change<=-threshold:
                self.bl_direction=-1;switched=True
        else:
            if raw<self.bl_extreme:self.bl_extreme=raw
            elif change>=threshold:
                self.bl_direction=1;switched=True
        if switched:
            self.bl_extreme=raw;self.bl_reversals+=1
            # Absolute branch offset: never add the offset again each frame/reversal.
            self.bl_source='reversal' if reversal else 'none'
            self.bl_offset=(cfg['backlash_right_us'] if self.bl_direction>0 else -cfg['backlash_left_us']) if reversal else 0.
        effective_offset, compensation_detail = self.compensation(raw, result, cfg)
        target=raw+effective_offset
        bounded=max(self.p['min_us'],min(self.p['max_us'],target))
        rate=min(self.p['steer_rate'],cfg['backlash_initial_rate'] if self.bl_source=='initial' else cfg['backlash_rate'])
        output=max(self.bl_last-rate*dt,min(self.bl_last+rate*dt,bounded))
        limited=abs(target-output)>.001
        if limited and (target-output)*error>0:self.i=previous_i
        self.bl_last=output
        result.update(steer_us=output,compensated_target_us=target,
                      backlash_offset_us=effective_offset,backlash_direction=self.bl_direction,
                      backlash_source=self.bl_source,backlash_initial_count=self.bl_initial_count,
                      backlash_state='initial_takeup' if first else 'reversal' if switched else ('taking_up_command' if limited else ('initial_tracking' if self.bl_source=='initial' else 'branch_tracking' if self.bl_reversals else 'initial_direction')),
                      backlash_reversals=self.bl_reversals,limited=result.get('limited',False) or limited)
        result.update(compensation_detail)
        return result


class PID(LegacyPID):
    """Center-conditioned branch compensation; not measured mechanical backlash.

    Keep the legacy reversal detector, gains, limits and output slew. Fade the
    requested offset within the existing reversal threshold, but only if both
    the unsupplemented command AND position P term approach center. This avoids
    mistaking a large P term cancelled by D for a centered vehicle.
    """
    compensation_version = 3

    def compensation(self, raw, result, cfg):
        p_term = result['p']
        position_us = p_term * (self.p['right_gain'] if p_term >= 0 else self.p['left_gain'])
        demand = max(abs(raw-self.p['center_us']), abs(position_us))
        band = cfg['backlash_threshold_us']
        x = min(1., demand/band)
        scale = x*x*(3.-2.*x)
        offset = self.bl_offset*scale
        return offset, dict(backlash_nominal_offset_us=self.bl_offset,
                            backlash_release_scale=scale,
                            backlash_center_demand_us=demand,
                            backlash_center_band_us=band,
                            backlash_release_state='released' if scale == 0 else 'center_fade' if scale < 1 else 'full',
                            backlash_release_policy='center-smoothstep-v1')


def path_error(points,row_ratio,width=1280,height=720,visual_center_offset_px_640=0.):
    """Interpolate a recorded image-space center path; never extrapolate missing rows."""
    clean=[]
    for point in points:
        if not isinstance(point,(list,tuple)) or len(point)!=2:return None
        x,y=point
        if any(type(v) not in (int,float) or not math.isfinite(v) for v in (x,y)):return None
        if not (0<=x<width and 0<=y<height):return None
        clean.append((y,x))
    clean.sort();row=row_ratio*(height-1)
    for (y0,x0),(y1,x1) in zip(clean,clean[1:]):
        if y0<=row<=y1 and y1>y0:
            x=x0+(x1-x0)*(row-y0)/(y1-y0)
            error=(x-(width-1)/2-visual_center_offset_px_640*width/640)/((width-1)/2)
            return max(-1.,min(1.,error)) # Retain valid edge paths after shifting the zero point.
    return None

def path_features(points,p):
    """Image-space anticipation only; no ground angle or wheel position claim."""
    cfg=dict(DIRECTION_DEFAULTS, **{k:p[k] for k in DIRECTION_DEFAULTS if k in p})
    offset=p.get('visual_center_offset_px_640',0.)
    near=path_error(points,p['row_ratio'],visual_center_offset_px_640=offset)
    far=path_error(points,cfg['direction_far_row'],visual_center_offset_px_640=offset) if cfg['direction_enabled'] else None
    delta=far-near if near is not None and far is not None else None
    effective=near
    if cfg['direction_enabled']:
        effective=None if delta is None else max(-1.,min(1.,near+cfg['direction_weight']*delta))
    return dict(near_error=near,far_error=far,direction_delta=delta,control_error=effective,
                direction_enabled=cfg['direction_enabled'],ground_geometry_applied=False,visual_center_offset_px_640=offset,visual_center_version=1)


def replay(result,model,layer,parameters):
    p=validate(parameters)
    if model not in result['models'] or layer not in ('A','B','C'):
        raise ValueError('請選擇有效模型與路徑')
    controller=PID(p);rows=[]
    for frame in result['models'][model]:
        path=frame.get(layer,{}).get('path',[])
        row=controller.step_path(frame['capture_s'],path)
        row['frame']=frame['frame'];rows.append(row)
    return dict(rows=rows,hardware_output=False,parameters=p,model=model,layer=layer,
                analysis_id=result['id'],source=result['source'],
                valid_frames=sum(r['valid'] for r in rows),total_frames=len(rows),
                interpretation='固定原影片的命令重算，不是閉迴路行駛模擬；不能推論調參後的偏移或圈速。',
                imu_used=False)
