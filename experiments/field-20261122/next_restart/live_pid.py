import ops07_runtime as ops07_support
"""Finite live vision PID; model isolation, explicit browser lease, no direct UART."""
import base64,json,math,os,queue,re,subprocess,threading,time,uuid
import pause_clock as _pclk   # pause-clock-r1 (pause-resume-20261120): control timeline = Orin monotonic - removed pause time (0 until a resume)
time=_pclk.TIME   # pause-clock-r1: time.monotonic() in this file reads the control timeline; time.time / sleep / ... are the real module
import pause_resume   # pause-resume-r1 (pause-resume-20261120): pause / resume / run timing / mask reader
from pathlib import Path
import rate_indep   # rate-indep-r1 (pipeline2-20261116): rate_indep.json at each run start; absent = frame counts as today
try:import single_centre   # single-centre-r1 (single-centre-20261117): web/single_centre.json at each run start; absent / enabled not true = unchanged
except Exception:single_centre=None
import upper_path   # upper-path-r1
from feedback_observation import vision_context
from visual_pid import PID,validate,path_error,path_features
from k_adaptive_pid import KAdaptivePID
from preview_turn_pid import PreviewTurnPID
from straight_preview_pid import StraightPreviewPID
from geometry_preview_pid import GeometryPreviewPID
from ground_road import SharedRoadPID
from preview_error import PreviewErrorPID
from camera_pair import CameraPairPID
from offscreen_center import OffscreenPID, path_error as offscreen_path_error
from outward_path import validate_amount,overlay
from pid_paths import PATHS,selected,ROAD_ALGORITHMS,ROAD_FILES
from imu_boost import ImuBoost, REVISION as IMU_BOOST_REVISION
from crest_fallback import CrestFallback, REVISION as CREST_REVISION
from dash_route import DashRoute, REVISION as DASH_REVISION, turn_steer, TURN_MAX_DEG
from pid_hold import PidHold, REVISION as HOLD_REVISION, HOLD_S
from heading_hold import HeadingHold, REVISION as HEADING_REVISION
from centre_estimator import CentreEstimator, REVISION as CENTRE_REVISION
from downhill_bottom import BottomJump, REVISION as BOTTOM_REVISION, LEVEL_PITCH as BOTTOM_LEVEL_PITCH
from lane_segment import LaneSegment, REVISION as LANE_REVISION, LOCK_PWM as LANE_LOCK_PWM, LOCK_GATE_DEG as LANE_LOCK_GATE
from downhill_entry import DownhillEntry, REVISION as ENTRY_REVISION, narrow as downhill_narrow
from corner_width import widen as corner_widen, REVISION as CORNER_REVISION
from downhill_speed import SpeedMeter, REVISION as DSPEED_REVISION, ASSIST_US as DOWN_ASSIST_US, HOLD_RIGHT_US as DOWN_HOLD_RIGHT_US, GZ_MAX as DOWN_GZ_MAX, K_GZ as DOWN_K_GZ, GZ_CAP_US as DOWN_GZ_CAP, HOLD_US as DOWN_HOLD_US, gz_term as down_gz_term, ENTRY_DELAY_S as DOWN_GZ_DELAY
from speed_adapt import SpeedAdapt, REVISION as SPEED_REVISION
import downhill_pair
from crest_marks import CrestMarks, REVISION as CREST_MARKS_REVISION, decode as crest_marks_decode
import bottom_left
from flat_dip_guard import FlatDipGuard   # flat-dip-guard-r1
import right_entry   # right-entry-r1
import curve_ff   # curve-ff-r1
import gain_schedule   # gain-schedule-r4
import i_ramp   # i-ramp-r1 (ramp-bias-20261119): web/i_ramp.json absent -> not used
import yaw_inner   # yaw-inner-r1
import curve_map   # curve-map-r1
try:import arc_lat   # arc-lat-r1 (arc-lat-20261119 offline candidate): lateral feedback in the curves from the measured distance to the lines (arc_lat.py / arc_lat.json; absent = off)
except Exception:arc_lat=None
try:import curve_trim   # curve-trim-r1 (curves-20261120 offline candidate): distance-scheduled outward trim in curves 1 / 3 (curve_trim.py / curve_trim.json; absent = off)
except Exception:curve_trim=None
try:import lap_ff   # lap-ff-r1 (lap-ff-20261115 offline candidate): position-armed, far-preview-triggered steering floor into curves 1/2/3 (lap_ff.py / lap_ff.json; absent = off)
except Exception:lap_ff=None
import cmd_comp   # cmd-comp-r1
import straight_slew   # straight-slew-r1
import speed_policy   # speed-policy-r1 (integration B3d): the PWM decisions in one object (speed_policy.py / speed_policy.json)
import lost_policy   # lost-policy-r1 (integration B3c): the lost-line policy table (lost_policy.py / lost_policy.json)
import segment_ctx   # segment-ctx-r1 (integration B2): per-frame segment context + ordered layer table
try:import crest_latch   # crest-latch-r1 (crest-latch-20261116 offline candidate; 85dc lap 6: after one crest-midpoint frame the flipping single-line estimate was used again at the crest): keep the crest midpoint until the crest is passed (crest_latch.py / crest_latch.json; absent = off)
except Exception:crest_latch=None
try:import stuck_stop   # stuck-stop-r1 (2026-11-14 offline candidate; 4879: against a wall at PWM 21-28 for ~5 s, nothing stopped it): commanded PWM but both wheels stopped -> end the run (stuck_stop.py / stuck_stop.json)
except Exception:stuck_stop=None
START_CENTRE_REVISION,START_CENTRE_MARGIN_S,START_CENTRE_MAX_S='start-centre-r1',.3,1.5   # start-centre-r1 (user 2026-11-06, live 00f4c1ca): the servo sat at 1779 from the previous run and the car drove off while it swung; PWM 0 for |previous command - centre| / steer_rate + margin (max 1.5 s)
import play_comp   # play-comp-r1
import gyro_hold   # gyro-hold-r1 (integration B3b): the one gyro heading-hold primitive (gyro_hold.py / gyro_hold.json)
import output_stage   # output-stage-r1 (integration B3a): the one output stage (output_stage.py / output_stage.json)
try:import bottom_owner   # bottom-owner-r2 (integration v2, 2026-11-13: 5 of ~20 slope-bottom passes failed): one steering owner from the landing to curve 4's FLOOR (bottom_owner.py / bottom_owner.json)
except Exception:bottom_owner=None
try:import bottom_guide   # bottom-guide-r1 (bottom-20261114): profile hold + curl guard inside the bottom-owner scope (bottom_guide.py / bottom_guide.json)
except Exception:bottom_guide=None
import xp_corridor   # xp-corridor-r2 (integration v2, 2026-11-13 ffc2 / b44b / ed96: after curve 2 the car followed the V's left arm): heading corridor (xp_corridor.py / xp_corridor.json)
PS_REVISION='play-single-r1'   # play-single-r1 (user 2026-11-12 676e / 1b6c: play_comp and lane r12 play stacked and triggered each other after the dashed turn)
import brake_zones   # brake-zones-r1
import solid_takeover   # solid-takeover-r1
UPPER_PRE_TURN_REVISION='upper-pre-turn-r1'   # upper-pre-turn-r1: before the dashed turn the upper cap is speed_adapt.json upper_pwm_pre_turn
LOST_TURN_REVISION='lost-turn-r1';LOST_TURN_US=lost_policy.param('lost_turn_us')   # lost-policy-r1: 200 from lost_policy.json   # lost-turn-r1: last valid command this far from centre = keep it when the path is lost (flat)
import upper_lateral
try:   # upper-single-r1 (user 2026-11-11 5235: after the dashed turn the steering swung 956-1770 us between 6 modules): one steering owner from the dashed-turn end to the downhill (upper_single.py / upper_single.json)
    import upper_single
    US_REVISION=upper_single.REVISION
except Exception:upper_single=None;US_REVISION='upper-single-r1'
try:   # upper-single-r2: mask dashed line (mask_dash.py); a missing / broken module = no mask source
    import mask_dash
except Exception:mask_dash=None
from mirror_identity import BoundaryIdentity, boundary_x as mid_boundary_x, AXIS as MID_AXIS, REVISION as MID_REVISION
# b1-dead-layers-r1 (integration B1, 2026-11-12): removed the layers that were off by code constant or config and changed no command: downhill-coast, downhill-approach (record-only), heading-term, straight-slew step, gyro-damp step, preview-lock, bottom-commit, curve-hold, lock-slow, diff-assist, auto-centre (integration-20261112/b01/b1/NOTES.md)
# cleanup-r1 (cleanup-20261117): removed exit-damp-r1 (exit_damp.json enabled false), slope-pre-right-r1 (bottom_map.json pre_right_us 0) and
# xp1-hold-r1 (crossings_ctl.json xp1_hold null): off on the car, no clock / IMU / MCU read while off, no other layer used their state.
try:import vehicle_model   # vehicle-model-r1 (cleanup-20261117): car constants in one place (vehicle_model.json 'use'; empty = every module's own value)
except Exception:vehicle_model=None
M_PER_COUNT=vehicle_model.get('m_per_count',2.22e-4) if vehicle_model is not None else 2.22e-4   # encoder metres per count (was the literal 2.22e-4 at 3 sites)
# crest-marks-r1 (user 2026-11-04, offline candidate 2026-11-05): after the crest ramp (pitch >= 2.2 on the upper section) line up
# between the two crest marks (thick white starts of the downhill edges); left offset only. When applied it replaces
# downhill-approach (the two never add up); downhill-approach stays logged for comparison.
# crest-marks-r1 applied (user 2026-11-05): chosen over approach-r1, which fired early on laps that went down.
CREST_MARKS_APPLY=True
# downhill-mirror-identity-r1 (2026-11-05, offline replay downhill-mirror-replay-20261105): the slope-entry mirror took the real right
# edge of a right arrival as the left edge and put the target outside the road. The left/right lines are tracked on the mask from the
# crest (crest-marks armed) into the slope entry; a lone line that is the tracked RIGHT edge would not be mirrored, the LEFT edge would,
# unknown -> mirror only left of the axis. Record-only: the decision is logged in downhill_mirror_identity, the mirror is unchanged.
MIRROR_IDENTITY_APPLY,MID_WINDOW_S=False,2.
# downhill-mirror-identity-r1 applied (user 2026-11-05, raw-20261105e e358c5fd / 1410ae24): on the slope the lone line was mirrored on
# alternate frames, steering 1750 <-> ~1000; every mirrored frame had identity RIGHT (the real right edge). Now a lone line tracked
# as the RIGHT edge is not mirrored; LEFT or unknown (axis fallback) as before.
MIRROR_IDENTITY_APPLY=True
# upper-lateral-r1 (user 2026-11-05, same runs): see upper_lateral.py. Applied in f6c6c908.
# upper-lateral-r2 record-only (raw-20261105f 2cbfd49b / 954b8b45): on the crest the only near line is the thick right edge; the
# normal path is estimated far left (onto the floor-track loops below), so the image error stayed -0.5 .. -0.97 while the car
# yawed 10-14 deg left under the -150 us offset; heading hold asked right (954b 1426 -> 1661) but the offset kept the command
# inside the steering play and the car went down turned left (954b left the track). The offset is still computed and logged.
UPPER_LATERAL_APPLY=False
# upper-lateral-r3 (user 2026-11-05, 686df4c1): applied again, but only before the first crest mark of this upper-section pass
UPPER_LATERAL_BEFORE_MARKS_APPLY,UPPER_LATERAL_MAX_PITCH=True,1.5   # and not on the crest hump (2cbfd49b / 954b8b45 offsets at +0.8..+4 deg)
MANUAL_END_REVISION='manual-end-r2'
DOWN_RELEASE_REVISION,DOWN_STEEP,DOWN_RELEASE_S='downhill-release-r1',-8.,.8
BOTTOM_RIGHT_REVISION,BOTTOM_RIGHT_PITCH='bottom-right-r1',-8.   # user 2026-11-05: right turns free at the bottom (see _bottom_right)
DOWN_RIGHT_HOLD_REVISION,RIGHT_HOLD_ERR,RIGHT_GZ_MAX='downhill-hold-r1',.5,45.   # user 2026-11-05: keep right turns on a lost downhill path
JG_REVISION,JG_PREV,JG_JUMP,JG_SHORT_S,JG_LONG_S='error-jump-guard-r1',.8,1.,.3,1.   # user 2026-11-05: see the guard before diff-assist
# bottom-curve-r1 (user 2026-11-05, live ff90ff2c: out on the outside of the right curve at the level bottom). After the release
# (downhill False, still inside the 4 s down_tail) two rules worked against the right turn: gyro damping took the curve entry
# for a weave (gz -20 vs low-pass -6.8 deg/s -> -90 us left at error +0.27 / +0.15), and the lost-path DOWNHILL_HOLD kept the
# 0.5 s median within centre +150 (1540 while the last command was 1621; also 65076008 / c67e20c3 / 3b8b5596 at the bottom).
# Now on the level bottom: gyro damping only updates its filter, and the hold (and its yaw-rate limit) allows centre +300
# to the right (the downhill-hold-r4 value); the left limit stays -150. The slope itself is unchanged. Experiment values.
BOTTOM_CURVE_REVISION,BOTTOM_HOLD_RIGHT_US='bottom-curve-r1',300.
# turn-pwm-r1 (user 2026-11-04, live 2a62a163 / 14d6d55c / 03a5e46a / 9c02d7da): the runs whose dashes were lost mid-turn sent PWM 15
# on the heading-blind / hold frames (speed_adapt.hold_pwm -> min_pwm; the upper-section cap LANE_LOCK_PWM only covered valid frames),
# turned wider, crossed to the right of the dashed line (dash line 185-600 px left of the car at the last sighting) and failed after the
# slope; the runs that held PWM 13 through the turn ended left of / on the dashed line and went down. Upper section (route OPEN): every
# frame at most LANE_LOCK_PWM; from the dashed TURN until the heading error is within TURN_PWM_GATE at most TURN_PWM. Experiment values.
TURN_PWM_REVISION,TURN_PWM,TURN_PWM_GATE=speed_policy.TURN_PWM_REVISION,speed_policy.TURN_PWM,speed_policy.TURN_PWM_GATE   # speed-policy-r1: 'turn-pwm-r4', 18, 20 from speed_policy.json   # r4 (user 2026-11-10 17:0x): dashed turn PWM 15 -> 18   # r2 (user 2026-11-05): 11 -> 13, same radius (about 80 cm at 11 and 13), about 1.5 x the turning speed; r3 (user 2026-11-09 9515: TURN/FOLLOW 0.20-0.29 m/s at 13): 13 -> 15

ROOT=Path(__file__).resolve().parent

def curve2_late(proposal):
    """curve-ff-late-r1 (user 2026-11-11 8be5 26.3-26.7 s): at 159-180 of the 210 deg curve 2 the curve floor pushed 1986 again, the car
    turned past and swung +-10 deg twice. No curve floor in curve 2 (start idx1 / after_down idx2) once the gyro turn is >= CURVE2_FF_OFF_DEG."""
    try:
        cm=proposal.get('curve_map') or {}
        return (cm.get('seg'),cm.get('idx')) in (('start',1),('after_down',2)) and isinstance(cm.get('turned_deg'),(int,float)) and abs(float(cm['turned_deg']))>=CURVE2_FF_OFF_DEG
    except Exception:return False
CURVE2_FF_OFF_DEG=150.
def curve3_cap(proposal):
    """curve-ff-c3cap-r1 (user 2026-11-11 7b24 41-46 s): in curve 3 (start idx2 / after_down idx3) the curve floor pushed 1941-1986 at each small
    bend (PID 1743-1858), the car cut in and swung back. The floor is capped at CURVE3_FF_CAP_US there (a PID command above it is kept)."""
    try:
        cm=proposal.get('curve_map') or {}
        return (CURVE3_FF_CAP_US if single_centre is None else single_centre.shift(CURVE3_FF_CAP_US)) if (cm.get('seg'),cm.get('idx')) in (('start',2),('after_down',3)) else None   # single-centre-r1: off -> the same 1850
    except Exception:return None
CURVE3_FF_CAP_US=1850.
DASH_ALIGN_MIN_TURN_DEG=50.   # dash-align-min-r1
LANE_TURN_GUARD_DEG=15.   # lane-turn-guard-r1
class LivePID:
    def __init__(self,manual,camera,operation_lock,motor_busy):
        self.manual=manual;self.camera=camera;self.operation_lock=operation_lock;self.motor_busy=motor_busy
        self.lock=threading.RLock();self.worker=None;self.worker_stop=threading.Event()
        self.state='idle';self.error=None;self.latest=None;self.preview=None;self.run=None
        self.owner=None;self.browser_at=0.;self.browser_seq=-1;self.consecutive=0
        self.last_preview_client=time.monotonic()
        self.preview_retention_s=300;self.preview_unload_reason="NOT_LOADED"
        self.stream_diagnostics={};self.path_algorithm='legacy9'
        self.models=json.loads((ROOT/'pid_models.json').read_text(encoding='utf-8'))
        threading.Thread(target=self._watch,daemon=True).start()

    def status(self):
        with self.lock:
            self.last_preview_client=time.monotonic()
            latest=dict(self.latest) if self.latest else None
            if latest:latest['age_s']=time.monotonic()-latest['capture_at']
            return dict(feedback_observation_version=1,feedback_usage="record_only",controller_feedback_enabled=False,road_state_version=1,geometry_preview_version=1,prediction_horizon_s=.15,exit_release_version=0,connector_version=1,connector_center_offset=getattr(self,'connector_center_offset',0.),preview_turn_version=1,integrated_k_version=2,curve_outward_version=1,curve_outward_px_640=getattr(self,'curve_outward_px_640',0.),visual_center_version=1,preview_session_version=1,preview_retention_s=self.preview_retention_s,preview_unload_reason=self.preview_unload_reason,k_sampling_version="k-adaptive-r1",control_publish_version=2,preview_seq=getattr(self,'preview_seq',0),camera_pipeline_version=1,stream_diagnostics=dict(self.stream_diagnostics),k_live_revision='k-shared-lane-live-r1',path_selection_version=1,road_pid_version=3,path_algorithm=getattr(self,'path_alias',self.path_algorithm),
                        path_algorithms=[dict(id=k,**v) for k,v in PATHS.items()],
                        backlash_version=getattr(PID,'compensation_version',2),play_comp_version=1,lost_turn_version=1,upper_pre_turn_version=1,brake_zones_version=1,bottom_hold_pwm_version=1,upper_post_turn_version=1,solid_takeover_version=2,curve_ff_version=2,gain_schedule_version=4,start_centre_version=1,heading_term_version=2,yaw_inner_version=14,weave_mode_version=1,jump_filter_version=2,bottom_release_version=1,crossing_pass_version=1,upper_path_version=2,curve_smooth_version=1,straight_slew_version=1,weave_slow_version=2,cmd_comp_version=1,bottom_inner_version=1,curve_map_version=5,dash_start3_version=1,lane_default_skip_version=1,dash_turn_vision_version=1,lane_k_l_version=1,bottom_commit_switch_version=1,autonomous_pid_version=1,direction_version=1,max_duration_s=None,custom_duration_version=1,
                        remaining_s=pause_resume.remaining_s(self),
                        state=self.state,error=self.error,latest=latest,run=self.run,
                        model_id=getattr(self,'model_id',None),row_ratio=getattr(self,'row_ratio',None),
                        models=[{'id':k,'label':v['label']} for k,v in self.models.items()],
                        ready_frames=self.consecutive,profile=self.manual.profile(),imu_enabled=False,imu_boost_version=1,imu_boost=getattr(self,'boost_status',None),crest_fallback_version=1,dash_route_version=1,pid_hold_version=1,heading_hold_version=1,speed_adapt_version=1,centre_estimator_version=1,lane_segment_version=1,downhill_entry_version=1,corner_width_version=1,gyro_damp_version=4,diff_assist_version=1,centre_estimate=(self.centre_est.summary() if getattr(self,'centre_est',None) is not None else None),preview_parameter_version=1,pause_resume_version=1,pause=pause_resume.status_pause(self),timing=pause_resume.timing(self))

    def configure_preview(self,p):
        value=p.get('preview_retention_s')
        if type(value) is not int or value<0:raise ValueError('保留秒數須為非負整數；0表示不自動卸載')
        with self.lock:
            self.preview_retention_s=value
            return dict(ok=True,preview_retention_s=value,hardware_output=False)

    def preview_expired(self,now):
        with self.lock:
            expired=self.preview_retention_s>0 and now-self.last_preview_client>self.preview_retention_s and not (self.run and self.run['state']=='running')
            if expired:self.preview_unload_reason='IDLE_TIMEOUT'
            return expired

    def prepare(self,p):
        with self.lock:
            if self.manual.busy() or self.motor_busy():raise ValueError('Stop vehicle control before loading model')
            if self.run and self.run['state']=='running':raise ValueError('Stop PID first')
            if self.worker and self.worker.is_alive():raise ValueError('Preview already running; stop it before changing model')
            model=p.get('model_id','v4-default');row=p.get('row_ratio',.75)
            if model not in self.models:raise ValueError('Unknown registered model')
            algorithm=p.get('path_algorithm','legacy9');selected(algorithm)
            # dash-route-r7: crest-start test option = camera-pair-v2-junction with the route window open from the start
            alias=algorithm;algorithm={'camera-pair-v2-junction-crest':'camera-pair-v2-junction','camera-pair-v2-junction-uphill':'camera-pair-v2-junction','camera-pair-v2-junction-crest-full':'camera-pair-v2-junction','camera-pair-v2-junction-flag':'camera-pair-v2-junction'}.get(algorithm,algorithm)   # flag-start-r1
            if algorithm=='temporal-v2' and not (ROOT.parent/'laptop_workbench'/'temporal_lane.py').is_file():
                raise ValueError('整條邊界預覽依賴尚未部署')
            if type(row) not in (int,float) or not math.isfinite(row) or not .1<=row<=.95:raise ValueError('Invalid lookahead row')
            if algorithm=='calibrated-v7':
                from calibrated_path import validate_row
                validate_row(row)
            if algorithm in ROAD_ALGORITHMS:
                missing=[n for n in ROAD_FILES if not (ROOT.parent/'laptop_workbench'/n).is_file()]
                if missing:raise ValueError('道路v3依賴未部署：'+','.join(missing))
            if algorithm=='junction-k-lane-v3' and not (ROOT/'k_lane_runtime'/'junction_k_lane.py').is_file():
                raise ValueError('完整K依賴未部署')
            if algorithm=='junction-k-integrated-r2' and not (ROOT/'k_integrated_live.py').is_file():raise ValueError('完整K r2依賴未部署')
            if algorithm in ('standalone-line-v1', 'standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction') and not (ROOT/'standalone_line_path.py').is_file():raise ValueError('獨立線估計尚未部署')
            if algorithm in ('straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction') and not (ROOT/'live_connector_path.py').is_file():raise ValueError('即時連接線尚未部署')
            if algorithm=='camera-pair-v2-junction' and not all((ROOT/n).is_file() for n in ('junction_live.py','junction_corridor.py','junction_switch.py')):raise ValueError('路口通道切換尚未部署')
            offset=p.get('visual_center_offset_px_640',0.)
            if type(offset) not in (int,float) or not math.isfinite(offset) or not -319<=offset<=319:raise ValueError('Invalid visual center offset')
            self.connector_center_offset=float(offset)
            amount=validate_amount(p.get('curve_outward_px_640',0.))
            if amount and algorithm not in ('junction-k-lane-v3','junction-k-integrated-r2'):raise ValueError('彎道外移目前支援K路徑；其他算法請設0')
            self.curve_outward_px_640=amount
            self.preview_unload_reason=None
            self.stream_diagnostics={};self.path_algorithm=algorithm;self.path_alias=alias
            self.model_id=model;self.row_ratio=row;self.state='loading';self.error=None;self.latest=None;self.preview=None;self.consecutive=0
            self.preview_seq=0
            self.last_preview_client=time.monotonic();self.worker_stop.clear()
            self.worker=threading.Thread(target=self._loop,daemon=True);self.worker.start()
            return {'ok':True,'hardware_output':False}

    def _us_new(self):   # upper-single-r1/r2: built once per run in start(); upper_single.json (+ mask-dash homography) read there (edit -> next run)
        try:
            us=upper_single.UpperSingle() if upper_single is not None else None
            if us is not None:
                us.md=None
                if us.enabled and (us.c.get('mask_dash') or us.c.get('hold_dir_fit')) and mask_dash is not None:   # upper-single-r4 hold_dir_fit: angle helper only; find() still needs mask_dash (wants_mask)
                    try:us.md=mask_dash.MaskDash(us.c)   # reads web/camera_ground_calibration.json; failure = no mask source
                    except Exception:us.md=None
            return us
        except Exception:return None
    def _us_inputs(self,dr):
        """upper-single-r1: (route_open, armed, yaw, gz, pre-turn mean heading, heading-hold reference) of this frame. armed = the
        heading-hold reference is set (dashed TURN started), not done, route OPEN and the dash route is not in TURN. IMU errors -> None."""
        hd=getattr(self,'heading',None);route_open=dr.get('route')=='OPEN';turning=getattr(getattr(self,'dash',None),'mode',None)=='TURN'
        ref=None if hd is None else getattr(hd,'ref',None);ref0=None if hd is None else getattr(hd,'ref0',None)
        armed=bool(route_open and not turning and ref is not None and ref0 is not None and not getattr(hd,'done',False))
        yaw=gz=None;tn=time.monotonic()
        if hd is not None:
            try:yaw=hd._yaw(tn)
            except Exception:yaw=None
            try:gz=hd._gz(tn)
            except Exception:gz=None
        import heading_hold as hhm
        pre=None if ref0 is None else float(ref0)-float(hhm.TURN_DEG)
        return route_open,armed,yaw,gz,pre,ref
    def _us_last_sent(self):
        ls=self.hold.last_sent(time.monotonic()) if getattr(self,'hold',None) is not None else None
        return None if ls is None else (ls['steer_us'],self.hold.sent_t)
    def _ue_kw(self,us):   # upper-edges-r1 (upper-edges-20261115): upper_single edge inputs; {} when edge_source is off (call unchanged). No clock / IMU / MCU read: last lap_pos update, last MCU status (_in_v), this frame's vision result
        if us is None or getattr(us,'ue',None) is None:return {}
        try:
            sn=self._lp_since('DASHe');v_=getattr(self,'_in_v',None) or {};b_=v_.get('enc_b') or {}
            odo=(int(b_['forward'])-int(b_['reverse']))*M_PER_COUNT if 'forward' in b_ and 'reverse' in b_ else None
            return dict(edge_det=((self.latest or {}).get('path_diagnostic') or {}).get('upper_edges'),y_rel_m=None if sn is None else sn-float(us.ue.c['k_crest_m']),odo_m=odo)
        except Exception:return {}
    def _ue_req(self):   # upper-edges-r1: worker request flag while upper_single is active (the worker then adds path_diagnostic.upper_edges); {} when off
        try:
            us=getattr(self,'upper',None)
            return {'ue':1} if us is not None and getattr(us,'ue',None) is not None and us.state is not None else {}
        except Exception:return {}
    def _us_pre_armed(self,dr):   # upper-single-r3: the armed conditions without 'not turning' (hand-over trigger input; attributes only)
        try:
            hd=getattr(self,'heading',None)
            return bool(dr.get('route')=='OPEN' and hd is not None and getattr(hd,'ref',None) is not None and getattr(hd,'ref0',None) is not None and not getattr(hd,'done',False))
        except Exception:return False
    def _us_valid(self,proposal,dr):
        """upper-single-r1 valid-frame hook, right after heading_hold.step: the command of this frame or None (= today's behaviour).
        The normal-path error is computed for the log only (normal_fallback false). Any exception -> None + reason ERROR."""
        try:
            us=getattr(self,'upper',None)
            if us is None or not us.enabled:self._us_on=False;return None
            route_open,armed,yaw,gz,pre,ref=self._us_inputs(dr)
            armed=armed and (proposal.get('heading_hold') or {}).get('reason')!='TURN' and 'dash_turn' not in proposal
            rp=self.run['parameters'];sel=self.latest.get('selected_source');vc=float(rp.get('visual_center_offset_px_640',0.))
            dl=(dr['line_a'],dr['line_b']) if sel=='dash_route' and dr.get('line_a') is not None and dr.get('line_b') is not None else None
            try:
                pts=self.latest.get('path') if sel is None else (self.latest.get('path_diagnostic') or {}).get('base_path')
                en=(offscreen_path_error if self.path_algorithm in ('camera-pair-v2', 'camera-pair-v2-junction') else path_error)(pts,self.row_ratio,visual_center_offset_px_640=vc) if pts else None
            except Exception:en=None
            md=getattr(us,'md',None);fa=ml=ma=None;mi=None   # upper-single-r2: fit angle (HOLD direction) and the mask dashed line
            if md is not None:
                try:
                    if dl is not None and us.c.get('hold_dir'):fa=md.angle_deg(dl[0],dl[1])
                except Exception:fa=None
                try:
                    pt=dr.get('pitch_rel_deg')
                    if dl is None and armed and us.wants_mask() and self.latest.get('mask_png') and pt is not None and abs(float(pt))<=float(us.c['mask_max_pitch_deg']):
                        tm0=time.perf_counter();r_,mi=md.find(self.latest['mask_png'],us.expect_deg(yaw));mi=dict(mi,ms=round((time.perf_counter()-tm0)*1e3,2))
                        if r_ is not None:ml=(r_['a'],r_['b']);ma=r_['ang'];mi.update(n=r_['n'],b_cm=round(r_['b_cm'],1),ang=round(r_['ang'],1))
                except Exception as e_:ml=ma=None;mi=dict(reason='ERROR',error=type(e_).__name__)   # the mask source is simply unavailable
            us.pre_armed=self._us_pre_armed(dr);out,info=us.step(time.monotonic(),route_open,armed,True,yaw,gz,float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),dash_line=dl,e_normal=en,
                             row_ratio=self.row_ratio,vc_px=vc,last_sent=self._us_last_sent(),pre_deg=pre,ref_deg=ref,mask_line=ml,fit_ang=fa,mask_ang=ma,**self._ue_kw(us))   # upper-edges-r1: {} when off
            if info is not None:
                proposal['upper_single']=info
                if mi is not None:info['mask']=mi
            if out is not None:
                info['replaced_us']=round(float(proposal['steer_us']),1);proposal['steer_us']=out
                if getattr(self,'solid',None) is not None:self.solid.reset()   # no takeover and no release (heading.ref rebase) while upper-single steers
            self._us_on=out is not None
            return out
        except Exception as e:
            self._us_on=False;proposal['upper_single']=dict(revision=US_REVISION,reason='ERROR',error=type(e).__name__);return None
    def _us_hh_pwm(self):
        """upper-single-r1: heading-hold error for _upper_pwm / _post_turn on an UPPER_SINGLE_HOLD frame (HEADING_BLIND frames carry it)."""
        try:
            hd=self.heading;err=float(hd.ref)-float(hd._yaw(time.monotonic()))
            return dict(revision=HEADING_REVISION,active=True,reason='UPPER_SINGLE_HOLD',ref_deg=round(float(hd.ref),2),err_deg=round(err,2))
        except Exception:return dict(revision=HEADING_REVISION,active=True,reason='UPPER_SINGLE_HOLD')
    def _us_invalid(self,dr):
        """upper-single-r1 invalid-frame hook, before the lane / heading blind holds: (steer_us, info) or None (= today's behaviour).
        Without a returned command its info (EXPIRED / NOT_ARMED / ERROR) goes to self.latest['upper_single'] (logged as vision)."""
        try:
            us=getattr(self,'upper',None)
            if us is None or not us.enabled:self._us_on=False;return None
            route_open,armed,yaw,gz,pre,ref=self._us_inputs(dr);rp=self.run['parameters']
            us.pre_armed=self._us_pre_armed(dr);out,info=us.step(time.monotonic(),route_open,armed,False,yaw,gz,float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),
                             row_ratio=self.row_ratio,vc_px=float(rp.get('visual_center_offset_px_640',0.)),last_sent=self._us_last_sent(),pre_deg=pre,ref_deg=ref,**self._ue_kw(us))   # upper-edges-r1: {} when off
            self._us_on=out is not None
            if out is None:
                if info is not None and self.latest is not None:self.latest['upper_single']=info
                return None
            return out,info
        except Exception as e:
            self._us_on=False
            if self.latest is not None:self.latest['upper_single']=dict(revision=US_REVISION,reason='ERROR',error=type(e).__name__)
            return None
    def _approach(self,proposal,li,dr,ls):   # crest-marks-r1 / upper-lateral-r3 on lane-segment frames (b1-dead-layers-r1: the record-only downhill-approach was removed)
        out=ls
        if dr.get('route')!='OPEN':self.cm_seen=False   # upper-lateral-r3
        try:
            cm=getattr(self,'cmarks',None)
            if cm is not None:
                co,ci=cm.offset(self.latest,li,dr.get('route')=='OPEN',bool(getattr(getattr(self,'dash',None),'followed',False)),dr.get('pitch_rel_deg'))
                if ci is not None:
                    mk=ci.get('marks') or {}
                    if mk.get('nl') or mk.get('nr'):self.cm_seen=True   # upper-lateral-r3: a crest mark was found
                    rp=self.run['parameters'];w=max(float(rp['min_us']),min(float(rp['max_us']),ls+co))
                    proposal['crest_marks']=dict(ci,applied=CREST_MARKS_APPLY,lane_steer_us=round(ls,1),would_steer_us=round(w,1),steer_us=round(w if CREST_MARKS_APPLY else out,1))
                    if CREST_MARKS_APPLY:out=w
        except Exception as e:proposal['crest_marks']=dict(revision=CREST_MARKS_REVISION,reason='ERROR',error=type(e).__name__)
        try:   # upper-lateral-r1 (user 2026-11-05, e358c5fd / 1410ae24): camera error on heading frames before the slope, left only
            ul=getattr(self,'ulat',None)
            if ul is not None:
                jst=((self.latest.get('path_diagnostic') or {}).get('junction') or {}).get('state')
                ie=proposal.get('image_error') if self.latest.get('selected_source') is None and jst in (None,'FOLLOW') else None
                uo,ui=ul.offset(time.monotonic(),ie,li.get('source'),dr.get('route')=='OPEN',bool(getattr(getattr(self,'dash',None),'followed',False)))
                if ui is not None:
                    rp=self.run['parameters'];w=max(float(rp['min_us']),min(float(rp['max_us']),ls+uo));use=(UPPER_LATERAL_APPLY or (UPPER_LATERAL_BEFORE_MARKS_APPLY and not getattr(self,'cm_seen',False) and (dr.get('pitch_rel_deg') is None or float(dr.get('pitch_rel_deg'))<UPPER_LATERAL_MAX_PITCH))) and w<out   # upper-lateral-r3
                    proposal['upper_lateral']=dict(ui,applied=use,marks_seen=bool(getattr(self,'cm_seen',False)),lane_steer_us=round(ls,1),before_us=round(out,1),would_steer_us=round(w,1))
                    if use:out=w   # the stronger left of crest-marks and this one; they do not add up
        except Exception as e:proposal['upper_lateral']=dict(revision=upper_lateral.REVISION,reason='ERROR',error=type(e).__name__)
        return out
    def _mid_update(self,result,tn,pitch):   # downhill-mirror-identity-r1: line identity from the armed crest into the slope entry
        mi=getattr(self,'mid',None)
        if mi is None:return
        try:
            rt=(result.get('dash_route') or {}).get('route');cm=getattr(self,'cmarks',None);st=getattr(getattr(self,'entry',None),'slope_t0',None)
            on=(rt=='OPEN' and cm is not None and cm.armed) or (rt=='DONE' and pitch is not None and pitch<-2. and (st is None or tn-st<=MID_WINDOW_S))
            if not on or not result.get('mask_png'):return
            m=crest_marks_decode(result['mask_png'])
            if m is not None:mi.update(m,tn)
        except Exception as e:result['downhill_mirror_identity']=dict(revision=MID_REVISION,reason='UPDATE_ERROR',error=type(e).__name__)
    def _mirror_gate(self,result,mr,tn):   # downhill-mirror-identity-r1: returns mr (mirror) or None (no mirror, only when applied)
        mi=getattr(self,'mid',None)
        if mi is None:return mr
        try:
            le=((result.get('path_diagnostic') or {}).get('line_estimate') or {})
            R=(((le.get('offscreen_center') or {}).get('current_boundaries') or {}).get('R')) or []
            y=float(self.row_ratio)*359.*2;q=[p for p in R if p and len(p)>=2 and abs(p[1]-y)<=12]
            rx=min(q,key=lambda p:abs(p[1]-y))[0]/2. if q else None
            lone=mid_boundary_x(R);idd,dist=mi.decide(lone,tn);axis=rx is not None and rx<MID_AXIS;keep=(idd=='LEFT') if idd is not None else axis
            info=dict(revision=MID_REVISION,identity=idd,dist=dist,lone_x=None if lone is None else round(lone,1),R_x=None if rx is None else round(rx,1),axis_mirror=axis,identity_mirror=keep,
                      applied=MIRROR_IDENTITY_APPLY,tracks={k:None if v is None else round(v,1) for k,v in mi.x.items()})
            result['downhill_mirror_identity']=info
            if MIRROR_IDENTITY_APPLY and not keep:info['skipped_mirror']=True;return None
            return mr
        except Exception as e:result['downhill_mirror_identity']=dict(revision=MID_REVISION,reason='GATE_ERROR',error=type(e).__name__);return mr
    def _start_hold_s(self,params):
        """start-centre-r1: seconds the servo needs from its last commanded pulse to the centre at the saved rate, plus a margin (0 when unknown)."""
        try:
            sc=((self.manual.state() or {}).get('vehicle') or {}).get('steer_us_command');rate=max(50.,float(params.get('steer_rate') or 800.))
            if sc is None:return 0.
            return min(START_CENTRE_MAX_S,abs(float(sc)-float(params['center_us']))/rate+START_CENTRE_MARGIN_S)
        except Exception:return 0.
    def _bm_cfg(self):
        if not hasattr(self,'_bm_c'):
            c=dict(enabled=False,turn_deg=130.,min_turned_deg=15.,target_dps=30.,ff_us=280.,k_us_per_dps=6.,cap_us=420.,pwm=15.,left_floor_us=None,max_s=2.5)
            try:c.update(json.loads((Path(__file__).parent/'bottom_map.json').read_text()))
            except Exception:pass
            self._bm_c=c
        return self._bm_c

    def _bm_turned(self):
        y0=getattr(self,'_bm_yaw0',None)
        if y0 is None:return None
        return float(y0)-float(self.heading._yaw(time.monotonic()))   # IMU yaw: right negative -> right turn is positive here

    def _bottom_map(self,now):
        """bottom-map-r1: steering for the unfinished bottom right curve without a valid path, or None."""
        c=self._bm_cfg()
        if not c.get('enabled') or getattr(self,'_bm_yaw0',None) is None or getattr(self,'hold',None) is None or self.hold.last_t is None:return None
        if now-self.hold.last_t>float(c['max_s'])+.4:return None
        tr=self._bm_turned()
        if tr is None or tr>=float(c['turn_deg']) or tr<float(c.get('min_turned_deg',15.)):return None   # curve 4 starts 2-4 s after the landing (fa44 / ca95): only an already started right curve is continued
        gz=float(self.heading._gz(time.monotonic()));c0=float(self.run['parameters']['center_us'])
        u=float(c['ff_us'])+float(c['k_us_per_dps'])*(-float(c['target_dps'])-gz)*-1.   # gz below -target (turning right faster) -> less right
        u=max(0.,min(float(c['cap_us']),u))
        st=max(float(self.run['parameters']['min_us']),min(float(self.run['parameters']['max_us']),c0+u))
        return dict(revision='bottom-map-r1',reason='MAP_TURN',turned_deg=round(tr,1),gz=round(gz,1),steer_us=round(st,1),pwm=float(c['pwm']))

    def _wheel_cps(self):
        """speed-adapt-r11: left wheel counts/s over the last 0.6 s (None when unknown)."""
        try:
            if getattr(self,'wspeed',None) is None:return None
            tw_=time.monotonic();v_=(self.manual.state() or {}).get('vehicle');self._in_v=v_   # b1-log: MCU status seen on this frame (proposal['inputs'])
            return self.wspeed.update(tw_,v_)
        except Exception:return None
    def _down_straight(self,prop,result,valid):
        """down-straight-r1 (user 2026-11-10 ac95: entered the yellow downhill on a held 1186 left, then 1948 / held 1877 right -> off the side).
        On the steep part of the downhill (pitch <= steep_deg, web/down_straight.json, re-read when the file changes): a valid command is
        kept within centre -+valid_cap_us; a lost-path hold steers by the gyro instead of the held command: yaw locked at the start of the
        loss + yaw-rate damping (left positive). Experiment values; enabled false or a missing file = off."""
        try:
            pth=Path(__file__).parent/'down_straight.json';mt=pth.stat().st_mtime if pth.exists() else None
            if getattr(self,'_dsc_mt','x')!=mt:
                self._dsc_mt=mt
                try:self._dsc=json.loads(pth.read_text()) if mt is not None else {}
                except Exception:self._dsc={}
            c=self._dsc or {}
            if not c.get('enabled'):return
            pr=(result.get('dash_route') or {}).get('pitch_rel_deg')
            if pr is None or float(pr)>float(c.get('steep_deg',-10.)):gyro_hold.release(self,'_ds_ref');return
            rp=self.run['parameters'];c0=float(rp['center_us']);lo,hi=float(rp['min_us']),float(rp['max_us'])
            if valid:
                gyro_hold.release(self,'_ds_ref')
                lw=c.get('law')   # slope-law-r1 (downhill-20261114): on a valid steep frame one law replaces the slope chain (PID + yaw_inner + downhill_speed / dlimit / downhill_gz):
                if isinstance(lw,dict) and lw.get('enabled',True) and prop.get('image_error') is not None and getattr(self,'heading',None) is not None:   # steer = centre + off + kp*e + kg*gz within +-cap (gz left +); no 'law' / enabled false = off
                    gzl=self.heading._gz(time.monotonic())
                    if gzl is not None:
                        st0=float(prop['steer_us']);ul=float(lw.get('off_us',0.))+float(lw['kp_us'])*float(prop['image_error'])+float(lw['kg_us_per_dps'])*float(gzl);cpl=float(lw.get('cap_us',250.))
                        sl=max(lo,min(hi,c0+max(-cpl,min(cpl,ul))))
                        prop['down_straight']=dict(revision='slope-law-r1',mode='LAW',pid_steer_us=round(st0,1),steer_us=round(sl,1),e=round(float(prop['image_error']),3),gz=round(float(gzl),1),pitch=round(float(pr),1));prop['steer_us']=sl
                        return
                cap=c.get('valid_cap_us')
                if cap is None:return
                st=float(prop['steer_us']);sc=max(c0-float(cap),min(c0+float(cap),st))
                if sc!=st:prop['down_straight']=dict(revision='down-straight-r1',mode='CAP',pid_steer_us=round(st,1),steer_us=round(sc,1),pitch=round(float(pr),1));prop['steer_us']=sc
                return
            if getattr(self,'heading',None) is None:return
            g=self._gyro_hold('_ds_ref',c,c0,lo,hi,'downhill')   # crossing-pass-r9: shared with the crossing-window hold; gyro-hold-r1 profile downhill
            if g is None:return
            out,yerr,gz=g
            prop['down_straight']=dict(revision='down-straight-r1',mode='GYRO',held_us=round(float(prop['steer_us']),1),steer_us=round(out,1),yaw_err=round(yerr,2),gz=None if gz is None else round(float(gz),1),pitch=round(float(pr),1))
            prop['steer_us']=out
        except Exception as e:prop['down_straight']=dict(revision='down-straight-r1',reason='ERROR',error=type(e).__name__)
    def _gyro_hold(self,ref_attr,c,c0,lo,hi,prof,base=None):
        """down-straight-r1 GYRO mode / crossing-pass-r9: steering without a valid path from the gyro. gyro-hold-r1 (integration B3b): the
        law, reference capture and defaults are gyro_hold.hold (profile prof of gyro_hold.json for keys missing from c). Returns
        (steer_us, yaw-ref, gz) or None without a yaw; IMU read errors propagate to the caller's try. base: xp-hold-r10 (crossing hold
        only; None = around c0 as before; the down_straight call passes none)."""
        return gyro_hold.hold(self,ref_attr,self.heading,c,gyro_hold.profile(prof),c0,lo,hi,base)
    def _xp_seen(self,prop):
        """crossing-pass-r9: every valid frame resets the gap reference; a non-WARMUP valid frame records whether the crossing-window code
        was inside the window on this frame (self.cmap.dist / the window code only run on valid frames, so an invalid frame uses this state)."""
        gyro_hold.release(self,'_xh_ref')
        if prop.get('reason')!='WARMUP':xp=prop.get('crossing_pass');self._xp_in=time.monotonic() if isinstance(xp,dict) and 'dist_after_m' in xp else None
    def _xp_hold(self,prop):
        """crossing-pass-r9 (user 2026-11-11 dca0 31.15 s: path invalid inside the crossing window, PID_HOLD kept 1673 for 0.33 s, the car
        swung 10 deg right): an invalid frame while the last valid frame was inside the crossing window steers by the gyro hold (yaw locked at
        the first lost frame + yaw-rate damping, crossing_pass.json hold_gyro) instead of the held command, for up to max_s after the last
        valid frame; then the existing rules. hold_gyro missing / null = off. A downhill GYRO hold already applied has priority.
        xp-hold-r10 (integration B5): hold_gyro.rel_base 'last' / 'mean' -> the hold acts around a base from the sent commands
        (gyro_hold.rel_base, limit rel_lim_us) instead of center_us; null / missing = r9 unchanged."""
        try:
            xc=getattr(self,'_xp_c',None) or {};c=xc.get('hold_gyro')
            if not c or not xc.get('enabled'):return
            if (prop.get('down_straight') or {}).get('mode')=='GYRO':return
            xin=getattr(self,'_xp_in',None)
            if xin is None or getattr(self,'_xp_end',None) is None:gyro_hold.release(self,'_xh_ref');return
            ls=time.monotonic()-xin
            if gyro_hold.expired(ls,c,gyro_hold.profile('crossing')):prop['crossing_hold']=dict(revision='crossing-pass-r9',reason='EXPIRED',lost_s=round(ls,3));return
            if getattr(self,'heading',None) is None:return
            rp=self.run['parameters'];c0=float(rp['center_us']);rb=gyro_hold.rel_base(c,getattr(self,'hold',None),ls,c0)   # xp-hold-r10 (integration B5, port of b4/c1): None = off (r9 hold around center_us)
            g=self._gyro_hold('_xh_ref',c,c0,float(rp['min_us']),float(rp['max_us']),'crossing',rb)
            if g is None:return
            out,yerr,gz=g
            prop['crossing_hold']=dict(revision='crossing-pass-r9',held_us=round(float(prop['steer_us']),1),steer_us=round(out,1),yaw_err=round(yerr,2),gz=None if gz is None else round(float(gz),1),lost_s=round(ls,3),**({} if rb is None else dict(rel='xp-hold-r10',base_us=round(rb,1))))
            prop['steer_us']=out
        except Exception as e:prop['crossing_hold']=dict(revision='crossing-pass-r9',reason='ERROR',error=type(e).__name__)
    def _right_hold(self,steer_us):
        """downhill-hold-r1: the last valid error was clearly to the right and the command is right of centre."""
        try:return bool((getattr(self,'dh_last_err',None) or 0.)>=RIGHT_HOLD_ERR and float(steer_us)>float(self.run['parameters']['center_us']))
        except Exception:return False
    def _bottom_right(self,steer_us):
        """bottom-right-r1: True at the bottom of the slope (it was below DOWN_STEEP and the pitch is back above BOTTOM_RIGHT_PITCH,
        live_pid's down_level_t0) for a command right of centre."""
        try:return bool(getattr(self,'down_steep',False) and getattr(self,'down_level_t0',None) is not None
                        and float(steer_us)>float(self.run['parameters']['center_us']))
        except Exception:return False
    def _down_gz(self,proposal,hold_us=None):
        """downhill-speed-r2..r4: yaw-rate limit on the downhill (against rotations faster than GZ_MAX); none within
        ENTRY_DELAY_S of the slope start (r4). Changes proposal['steer_us'] in place. Holds pass hold_us: the result stays
        within centre -+hold_us, so the 0.5 s median of sent hold commands cannot ratchet the term up frame after frame."""
        if getattr(self,'heading',None) is None:return
        try:
            tn=time.monotonic();st=getattr(getattr(self,'entry',None),'slope_t0',None)
            if st is not None and tn-st<DOWN_GZ_DELAY:return
            gzd=self.heading._gz(tn);term=down_gz_term(gzd)
            rh=self._right_hold(proposal['steer_us'])   # downhill-hold-r1: right rotation allowed up to RIGHT_GZ_MAX
            if rh and gzd is not None and gzd<0:term=max(-DOWN_GZ_CAP,min(DOWN_GZ_CAP,DOWN_K_GZ*(gzd+RIGHT_GZ_MAX))) if gzd<-RIGHT_GZ_MAX else 0.
            if rh and hold_us is not None:hold_us=(hold_us if not isinstance(hold_us,tuple) else hold_us[0],float(self.run['parameters']['max_us'])-float(self.run['parameters']['center_us']))
            if term and term<0 and self._bottom_right(proposal['steer_us']):   # bottom-right-r1: no left push on a right turn at the bottom
                proposal['downhill_gz']=dict(revision=DSPEED_REVISION,gz=round(gzd,1),term_us=round(term,1),skipped=BOTTOM_RIGHT_REVISION);return
            if term:
                rp=self.run['parameters']
                proposal['downhill_gz']=dict(revision=DSPEED_REVISION,gz=round(gzd,1),term_us=round(term,1),pid_steer_us=round(float(proposal['steer_us']),1))
                lo,hi=float(rp['min_us']),float(rp['max_us'])
                if hold_us is not None:   # bottom-curve-r1: (left, right) limits allowed
                    c0=float(rp['center_us']);hl,hr=hold_us if isinstance(hold_us,tuple) else (hold_us,hold_us);lo,hi=max(lo,c0-hl),min(hi,c0+hr)
                proposal['steer_us']=max(lo,min(hi,float(proposal['steer_us'])+term))
        except Exception:pass

    def start(self,p):
        params=validate(p.get('parameters'))
        mode=p.get('execution_mode','browser_lease')
        if mode not in ('browser_lease','orin_timed','orin_continuous'):raise ValueError('Unknown PID execution mode')
        if mode=='orin_continuous' and (not getattr(self,'ops07',None) or p.get('_ops07_ticket') is not self.ops07.ticket):raise ValueError('Use 07 arm-hold API')
        owner=p.get('owner');seconds=p.get('seconds')
        if not isinstance(owner,str) or not re.fullmatch('[0-9a-f]{32}',owner):raise ValueError('Invalid owner')
        if p.get('area_clear') is not True:raise ValueError('Confirm operating area')
        if mode!='orin_continuous' and not (mode=='orin_timed' and seconds is None) and (type(seconds) not in (int,float) or not math.isfinite(seconds) or seconds<1):raise ValueError('Duration must be finite and at least 1 second (orin_timed: or null = no deadline)')
        if type(p.get('parameter_revision')) is not int or p['parameter_revision']<1:raise ValueError('Save parameter revision')
        if params['pwm']!=int(params['pwm']):raise ValueError('PWM must be an integer')
        if params['max_gap']>.5:raise ValueError('Live maximum frame age is 0.5 seconds')
        with self.operation_lock,self.lock:
            selected(self.path_algorithm,control=True)
            if p.get('path_algorithm','legacy9')!=getattr(self,'path_alias',self.path_algorithm):raise ValueError('路徑算法與預覽不符')
            if self.manual.busy() or self.motor_busy():raise ValueError('Another controller is active')
            if self.state!='ready' or self.consecutive<3 or not self.latest or time.monotonic()-self.latest['capture_at']>params['max_gap']:raise ValueError('Need 3 consecutive fresh valid model results')
            if mode=='orin_continuous' and self.latest.get('weights_sha256')!=self.ops07.package['recognition']['weights_sha256']:raise ValueError('07 模型權重與方案不符')
            if params.get('curve_outward_px_640',0.)!=getattr(self,'curve_outward_px_640',0.):raise ValueError('彎道外移量已變更；請重新載入預覽')
            if self.path_algorithm in ('straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction') and params.get('visual_center_offset_px_640',0.)!=self.connector_center_offset:raise ValueError('視覺中心已變更，請重新載入連接線預覽')
            if params['row_ratio']!=self.row_ratio:raise ValueError('Restart preview with saved lookahead row')
            if path_features(self.latest.get('path',[]),params)['control_error'] is None and params.get('direction_enabled',0):raise ValueError('近遠兩列都必須有有效路徑')
            profile=self.manual.profile()
            if self.path_algorithm=='calibrated-v7':
                from calibrated_path import load_calibration,validate_row
                _,_,calibration_sha=load_calibration()
                if p.get('camera_pose_confirmed') is not True:
                    raise ValueError('請確認相機姿態、16:9未裁切影像與v7量測時一致')
                if p.get('calibration_sha256')!=calibration_sha or self.latest.get('path_diagnostic',{}).get('calibration_sha256')!=calibration_sha:
                    raise ValueError('V7 calibration identity mismatch')
                validate_row(params['row_ratio'])
                if params.get('direction_enabled'):validate_row(params['direction_far_row'])
            if params['center_us']!=profile['center_us']:
                raise ValueError('PID center must equal saved shared steering center')   # b1-dead-layers-r1: auto-centre-r1 removed (auto_centre.json enabled false)
            if params['min_us']<profile['min_us'] or params['max_us']>profile['max_us']:raise ValueError('PID limits exceed saved profile')
            if self.path_algorithm in ('junction-k-lane-v3', 'junction-k-integrated-r2', 'standalone-line-v1', 'standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):
                self.controller=(OffscreenPID if self.path_algorithm in ('camera-pair-v2', 'camera-pair-v2-junction') else CameraPairPID if self.path_algorithm=='camera-pair-v1' else PreviewErrorPID if self.path_algorithm=='preview-error-v1' else SharedRoadPID if self.path_algorithm=='straight-bearing-v1' else GeometryPreviewPID if self.path_algorithm in ('straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction') else StraightPreviewPID if self.path_algorithm=='standalone-line-v3' else PreviewTurnPID if self.path_algorithm=='standalone-line-v2' else KAdaptivePID)(params)
            elif self.path_algorithm in ROAD_ALGORITHMS:
                from road_live import RoadController
                self.controller=RoadController(params)
            else:self.controller=PID(params)
            self.owner=owner;self.browser_at=time.monotonic();self.browser_seq=-1
            # imu-boost-r1: manual_drive rejects per-frame PWM above the started value, so the run starts
            # with the uphill ceiling; the PID still sends the saved PWM except on the slope.
            self.boost=ImuBoost();self.crest=CrestFallback(self.boost);crest_alias=getattr(self,'path_alias',None)=='camera-pair-v2-junction-crest';crest_full=getattr(self,'path_alias',None)=='camera-pair-v2-junction-crest-full';self.dash=DashRoute(self.boost,crest_start=crest_alias or crest_full,lane='right_quarter' if crest_alias else None);self.hold=PidHold();self.heading=HeadingHold(self.boost);self.heading.lane_cap=450. if crest_alias else None;self.speed=SpeedAdapt();self.down_until=None;self.lane=LaneSegment(dash_only=False);self.lane.lock=True;self.entry=DownhillEntry();self.turn_yaw0=None;self.route_lap=0;self.dspeed=SpeedMeter();self.wspeed=SpeedMeter();self.centre_est=CentreEstimator(params['center_us'],self.boost.c['flat_pitch_deg'],self.boost.c['latest_path']);self._collision=None;self._ce_vhist=[];self.lane_play=self.lane.set_play(self.centre_est.est,params['center_us']);self.bottom=BottomJump();self.cmarks=CrestMarks();self.cm_seen=False;self.jg=None;self.pcomp=play_comp.PlayComp(output_stage.play_comp_config());self.cff=curve_ff.CurveFF();self.dturn_cfg=None;self.yin=yaw_inner.YawInner();self.cmap=curve_map.CurveMap();self._xp_end=None;self._xp_last=None;self._xp_rel=False;self._xp_in=None;gyro_hold.release(self,'_xh_ref');self.sslew=straight_slew.StraightSlew();self._ss_gz=[];self.ccomp=cmd_comp.CmdComp();self.gs_cfg=gain_schedule.load_config();self.iramp=i_ramp.load();self.start_t=time.monotonic();self.start_centred=False;self.start_hold_s=self._start_hold_s(params);self.bzones=brake_zones.BrakeZones();self.solid=solid_takeover.SolidTakeover();self.ulat=upper_lateral.UpperLateral();self.mid=BoundaryIdentity();self.dipg=FlatDipGuard();ceiling=self.speed.ceiling(self.boost.ceiling(int(params['pwm'])))
            self._ps_init()   # play-single-r1 (user 2026-11-12 676e / 1b6c: play_comp and lane r12 play stacked and triggered each other after the dashed turn): lane r12 play off when play_comp is on
            output_stage.unify_start(self)   # output-unify-r1 (output-unify-20261117): web/output_unify.json read once per run; absent / enabled false -> self.ounify None, output stage unchanged
            self.upper=self._us_new();self._us_on=False   # upper-single-r1 (user 2026-11-11 5235: after the dashed turn the steering swung 956-1770 us between 6 modules): reads upper_single.json once per run
            self.bown=bottom_owner.BottomOwner.from_file() if bottom_owner is not None else None   # bottom-owner-r2: reads bottom_owner.json once per run (enabled false -> never called)
            self.bguide=bottom_guide.BottomGuide.from_file() if bottom_guide is not None else None   # bottom-guide-r1: reads bottom_guide.json once per run (enabled false -> never called)
            self.xpc=xp_corridor.XpCorridor(xp_corridor.load_config())   # xp-corridor-r2: reads xp_corridor.json once per run (enabled false = no effect, no reads)
            self._xpa_c=self._xpa_load();self._xpa_a=None;self._xpa_n=None;self._xpa_wn=None;self._xpa_zn=None;self._xpa_vw=None;self._xpa_log=None;self._xpa_t180=None;self._xpa_veto=None   # xp-anchor-r2: CE plausibility state; xp-anchor-r1: reads xp_anchor.json once per run (missing / enabled false = None = off)
            self._cc=self._cc_load()   # crossings-ctl-r1: crossings_ctl.json once per run (missing / enabled false = None = off)
            self.clatch=(lambda c_:crest_latch.CrestLatch(c_) if c_ else None)(crest_latch.load_config()) if crest_latch is not None else None   # crest-latch-r1: crest_latch.json once per run (absent / enabled not true -> None = old behaviour, no extra reads)
            self.lff=self._lff_new();self.lff4=self._lff4_new();self.arc2=self._arc2_new()   # lap-ff-c4-r1 (curve4-ff-20261116): lap_ff.json 'c4' absent / disabled -> None; lap-ff-r1: reads lap_ff.json once per run (absent / enabled false -> None, the layer does not run)
            self.ctrim=self._ctrim_new()   # curve-trim-r1: reads curve_trim.json once per run (absent / enabled false -> None, the layer does not run)
            self.land4=self._l4_new()   # land4-r1 (field-20261122/landing4): reads land4.json once per run (absent / enabled false / no stage -> None, nothing runs)
            self.sbias=self._sb_new()   # start-bias-r1: start_bias.json once per run (absent / enabled not true -> None, layer not run)
            self.alat=self._alat_new()   # arc-lat-r1: reads arc_lat.json once per run (absent / enabled false -> None, the layer does not run)
            self._sc=single_centre.start(self,params['center_us']) if single_centre is not None else None   # single-centre-r1: reads single_centre.json once per run; off resets the module state (nothing changed); on rewrites the per-run lap_ff / lap_ff4 / arc2 / bottom_guide configs: straight points -> center_us, absolute caps -> center_us + (cap - 1480)
            self.stuck=stuck_stop.StuckStop.from_file() if stuck_stop is not None else None   # stuck-stop-r1: reads stuck_stop.json once per run (enabled false / missing / invalid -> None = never called, no read, no log)
            request=dict(control_version=2,owner=owner,area_clear=True,record=True,right_pct=ceiling,left_pct=ceiling,center_us=int(params['center_us']),steer_rate_us_s=int(params['steer_rate']),accel_seconds=0,decel_seconds=0,seconds=seconds,execution_mode=mode)
            result=self.manual.start(request,_pid=True)
            self.run=dict(id=result['run_id'],state='running',started=time.time(),deadline=None if mode=='orin_continuous' or seconds is None else time.monotonic()+seconds,
                          seconds=seconds,execution_mode=mode,parameters=params,parameter_revision=p['parameter_revision'],model_id=self.model_id,
                          path_algorithm=getattr(self,'path_alias',self.path_algorithm),path_algorithm_version=PATHS[getattr(self,'path_alias',self.path_algorithm)]['version'],base_path_algorithm=self.path_algorithm,
                          weights_sha256=self.latest['weights_sha256'],imu_enabled=bool(self.boost.c.get('enabled')),imu_boost=dict(self.boost.c,revision=IMU_BOOST_REVISION,run_pwm_ceiling=ceiling),crest_fallback=CREST_REVISION,dash_route=DASH_REVISION,pid_hold=dict(revision=HOLD_REVISION,hold_s=HOLD_S),heading_hold=HEADING_REVISION,speed_adapt=dict(self.speed.c,revision=SPEED_REVISION,run_pwm_ceiling=ceiling),reason=None)
            if self.path_algorithm in ('standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):self.run['turn_preview_policy']=dict(self.controller.policy)
            if self.path_algorithm=='calibrated-v7':
                self.run.update(calibration_sha256=calibration_sha,camera_pose_confirmed=True)
            self.run['play_single']=getattr(self,'_ps_info',None)   # play-single-r1: run-start decision, logged in the start line
            ri_=rate_indep.load()   # rate-indep-r1: None (file absent / disabled) -> no key, frame counts as today
            if ri_ is not None:self.run['rate_indep']=ri_
            if getattr(self,'_sc',None):self.run['single_centre']=self._sc   # single-centre-r1: logged only when on
            if getattr(self,'_p2',None):self.run['pipeline2']=dict(self._p2)   # pipeline2-20261116: logged only when on
            if getattr(self,'_ov',None):self.run['worker_overlap']=dict(self._ov)   # worker-overlap-r1: logged only when on
            self.log=(self.manual.root/result['run_id']/'pid.jsonl').open('a',encoding='utf-8')
            self.log.write(json.dumps({'kind':'start',**self.run})+'\n');self.log.flush()
            pause_resume.on_start(self)   # pause-resume-r1 (pause-resume-20261120): no pause state, run timing starts
            return dict(ok=True,run_id=result['run_id'],execution_mode=mode)

    def heartbeat(self,p):
        with self.lock:
            if not self.run or self.run['state']!='running' or p.get('owner')!=self.owner or p.get('run_id')!=self.run['id']:raise ValueError('Not PID owner')
            if self.run.get('execution_mode','browser_lease')=='browser_lease' and time.monotonic()-self.browser_at>1:raise ValueError('PID_BROWSER_LOST')
            seq=p.get('seq')
            if type(seq) is not int or seq<=self.browser_seq:raise ValueError('Stale heartbeat')
            self.browser_seq=seq;self.browser_at=time.monotonic();return {'ok':True}

    # manual-end-r1 (live 8c10cf89, 2026-11-04): a physical e-stop advances the MCU generation; the manual session then fails
    # (STALE_GENERATION) and the next PID command was refused ('This page does not own manual driving'). That exception ended
    # the run as PID_MODEL_FAILURE, left the model state 'failed' and unloaded the preview. Now a refused command after the
    # manual session of this run has ended stops the run with PHYSICAL_ESTOP (vehicle reports reason PHYSICAL_ESTOP) or
    # MANUAL_SESSION_ENDED, records the manual reason, and keeps the preview running. Other pid_input errors are unchanged.
    def _manual_ended(self,wait=True):
        m=self.manual
        with m.lock:
            cur=m.current or {}
            if m.busy() or not self.run or cur.get('id')!=self.run['id']:return None
            info=dict(revision=MANUAL_END_REVISION,manual_status=cur.get('status'),manual_reason=cur.get('reason'))
        v={}
        for n in range(4 if wait else 1):   # the LOCKED status can arrive a little after the refused ack
            try:v=(m.state() or {}).get('vehicle') or {}
            except Exception:v={}
            if v.get('reason')=='PHYSICAL_ESTOP' or not wait or n==3:break
            time.sleep(.1)
        info.update(vehicle_state=v.get('state'),vehicle_reason=v.get('reason'),vehicle_generation=v.get('generation'),
                    stop_reason='PHYSICAL_ESTOP' if v.get('reason')=='PHYSICAL_ESTOP' else 'MANUAL_SESSION_ENDED')
        return info

    # speed-policy-r1 (integration B3d): the upper-section PWM helpers live in speed_policy.py; these wrappers keep the old names
    def _pre_turn(self,p):return speed_policy.pre_turn(self,p)   # upper-pre-turn-r1
    def _pre_turn_cap(self):return speed_policy.pre_turn_cap(self)   # upper-pre-turn-r1
    def _post_turn(self,p):return speed_policy.post_turn(self,p)   # upper-post-turn-r1
    def _jf_pt(self,p):return speed_policy.remember_post_turn(self,p)   # jump-filter-r2 (HIDDEN WRITE made explicit in speed_policy)
    def _post_turn_cap(self):return speed_policy.post_turn_cap(self)   # upper-post-turn-r1
    def _upper_pwm(self,p):speed_policy.upper_cap(self,p)   # turn-pwm-r1
    def _ps_init(self):
        """play-single-r1: once per run in start(). play_comp.json enabled -> the lane-segment r12 play is off for this run (only the
        output stage compensates). Disabled -> nothing changes (lane r12 as before). Any error -> no output stage this run (pcomp None)."""
        try:
            pcm=getattr(self,'pcomp',None);on=bool(pcm is not None and pcm.c.get('enabled'));lp=getattr(self,'lane_play',None)
            if on:self.lane.play=None;self.lane_play=None
            self._ps_info=dict(revision=PS_REVISION,output_stage=on,play_comp=None if pcm is None else dict(pcm.c),
                               lane_r12='OFF_PLAY_COMP' if on else ('ON' if lp is not None else 'NO_OFFSETS'),lane_r12_offsets=lp)
        except Exception as e:
            self.pcomp=None;self._ps_info=dict(revision=PS_REVISION,reason='ERROR',error=type(e).__name__,output_stage=False)
    def _ps_skip(self,p):
        """play-single-r1: start-centring frames (start-centre-r1 not done) and a WARMUP at the centre (PWM 0, nothing held) are sent
        uncompensated and do not step the PlayComp state."""
        return (p.get('start_centre') or {}).get('done') is False or (p.get('reason')=='WARMUP' and not p.get('warmup_held'))
    def _ps_out(self,p,pc):   # output-stage-r1 (integration B3a): moved to output_stage.compensate (play-single-r1 step, unchanged)
        return output_stage.compensate(self,p,pc,PS_REVISION)
    def _pid_send(self,p,pc=None):   # output-stage-r1 (integration B3a): ops07 hold -> cmd_comp.sent (pre-compensation) -> play comp -> pid_input, in output_stage.send (play-single-r1 path, unchanged)
        return output_stage.send(self,p,pc,ops07_support,PS_REVISION)

    def _stuck_check(self,p):
        """stuck-stop-r1: called only when stuck_stop.json is enabled (self.stuck not None), before each dispatch (valid and lost-line rows).
        Reads the MCU status once (manual.state()); the rule is in stuck_stop.py. mode 'stop': the run is ended through end() (reason
        stuck_stop.STOP_REASON) and True is returned (the caller does not send this frame); mode 'log': records only. Never raises."""
        try:
            ev,rec=self.stuck.step(time.monotonic(),(self.manual.state() or {}).get('vehicle'))
            if rec is not None:p['stuck_stop']=rec
            if ev is not None and self.stuck.mode=='stop':
                self.run['stuck_stop']=ev;self.log.write(json.dumps({'kind':'stuck_stop','wall':time.time(),**ev})+'\n')
                self.end(stuck_stop.STOP_REASON);return True
        except Exception as e:p['stuck_stop']=dict(revision=getattr(stuck_stop,'REVISION','stuck-stop-r1'),reason='ERROR',error=type(e).__name__)
        return False
    def end(self,reason='USER_STOP'):
        with self.lock:
            if self.run and self.run['state']=='running':
                self.manual.request_stop(reason)
                self.run.update(state='stopped',reason=reason,ended=time.time())
                pause_resume.on_end(self,reason)   # pause-resume-r1 (pause-resume-20261120): an open pause is closed in the log, timing frozen
                if ops07_support.continuous(self.run) and getattr(self,'ops07',None):self.ops07.ended(reason)
                if getattr(self,'centre_est',None) is not None:
                    try:self.centre_est.save();self.run['centre_estimate']=self.centre_est.summary()
                    except Exception:pass
                # Capture diagnostics after requesting stop. Never delay stop for image I/O.
                try:
                    now=time.monotonic()
                    diagnostic=dict(kind='stop_diagnostic',wall=time.time(),reason=reason,
                        vision=self.latest,
                        vision_age_s=now-self.latest['capture_at'] if self.latest else None,
                        max_gap=self.run['parameters']['max_gap'],
                        model_state=self.state,model_error=self.error,
                        hardware_output=False)
                    if self.preview:
                        name='pid-stop-preview.jpg'
                        (self.manual.root/self.run['id']/name).write_bytes(self.preview)
                        diagnostic['preview_file']=name
                        diagnostic['preview_note']='Last model preview; may be stale, not necessarily the triggering camera frame.'
                    self.log.write(json.dumps(diagnostic)+'\n')
                except Exception as exc:
                    self.log.write(json.dumps({'kind':'diagnostic_error','error':type(exc).__name__})+'\n')
                self.log.write(json.dumps({'kind':'stop','reason':reason,'at':time.time()})+'\n');self.log.close()
            return {'ok':True,'run':self.run}

    def pause(self,p):return pause_resume.pause(self,p)   # pause-resume-r1 (pause-resume-20261120): POST /api/pid/pause
    def resume(self,p):return pause_resume.resume(self,p)   # pause-resume-r1 (pause-resume-20261120): POST /api/pid/resume
    def mask_view(self):return pause_resume.mask_view(self)   # pause-resume-r1 (pause-resume-20261120): GET /api/pid/mask (no lock)
    def stop_preview(self):
        self.end('PREVIEW_STOPPED');self.preview_unload_reason='USER_STOP';self.worker_stop.set();return {'ok':True}

    def _watch(self):
        while True:
            time.sleep(.05)
            self.check_safety()
            if getattr(self,'ops07',None):
                try:self.ops07.tick()
                except Exception:self.end('OPS07_RUNTIME_ERROR')

    def check_safety(self):
        with self.lock:
            if not self.run or self.run['state']!='running':return
            now=time.monotonic();reason=None
            if self.run.get('execution_mode','browser_lease')=='browser_lease' and now-self.browser_at>1:reason='PID_BROWSER_LOST'
            elif pause_resume.deadline_expired(self,now):reason='PID_DURATION_COMPLETE'   # pause-resume-r1 (pause-resume-20261120): null deadline / open pause never expire
            elif not self.latest or now-self.latest['capture_at']>self.run['parameters']['max_gap']:reason='PID_VISION_STALE'
            elif not self.manual.busy():
                reason='VEHICLE_CONTROLLER_STOPPED'
                # manual-end-r2 (live 0f9529aa 2026-11-04: a physical e-stop was caught here first and logged as
                # VEHICLE_CONTROLLER_STOPPED): same classification as _pid_send, without waiting (this runs every 50 ms)
                try:
                    info=self._manual_ended(wait=False)
                    if info is not None:
                        self.run['manual_session']=info;self.log.write(json.dumps({'kind':'manual_session_end','wall':time.time(),**info})+'\n')
                        if info['stop_reason']=='PHYSICAL_ESTOP':reason='PHYSICAL_ESTOP'
                except Exception:pass
            if reason:self.end(reason)

    # ---- lost-policy-r1 (integration B3c): rows and modifiers of the lost-line table (lost_policy.py / lost_policy.json; table LOST_TABLE
    # at the end of the file). Bodies moved from the old inline branch of _loop; F.now / F.result / F.stale / F.downhill / F.down_tail are this frame's.
    def _lp_blind_pre(self,F):   # heading-hold-r2: blind holds only on a fresh frame with heading + hold objects
        return not F.stale and getattr(self,'heading',None) is not None and getattr(self,'hold',None) is not None
    def _lp_blind_prelude(self,F):
        F.l_rp=self.run['parameters'];F.l_dr=F.result.get('dash_route') or {}
    def _lp_us_hold(self,F):   # upper-single-r1 (user 2026-11-11 5235): gyro HOLD of the single owner first
        return self._us_invalid(F.l_dr)
    def _lp_lane_blind(self,F):   # downhill-entry-r1: keep the lane command, no pull-back to the two-lane heading
        return self.entry.blind_hold(time.monotonic()) if getattr(self,'entry',None) is not None else None
    def _lp_heading_blind(self,F):   # heading-hold-r2: after the dashed-row turn, steer by heading without a valid path
        rp=F.l_rp;dr=F.l_dr
        return self.heading.blind(time.monotonic(),dr.get('route')=='OPEN',getattr(self.dash,'mode',None)=='TURN',self.hold.last_t,rp['center_us'],rp['min_us'],rp['max_us'])
    def _lm_blind(self,F,blind):   # the reason comes from the info revision (as before), not from the row that proposed it
        last=self.hold.last or {};bp=dict(steer_us=blind[0],pwm=last.get('pwm',self.run['parameters']['pwm']),valid=False,reason='UPPER_SINGLE_HOLD' if blind[1].get('revision')==US_REVISION else 'LANE_BLIND_HOLD' if blind[1].get('revision')==ENTRY_REVISION else 'HEADING_BLIND',heading_hold=blind[1])
        if bp['reason']=='UPPER_SINGLE_HOLD':bp['upper_single']=bp.pop('heading_hold');bp['heading_hold']=self._us_hh_pwm()   # upper-single-r1: own log key; the PWM caps (_upper_pwm) still see the heading-hold error as on HEADING_BLIND frames
        return bp
    def _lp_pid_hold(self,F):   # pid-hold-r1: keep the last valid command for up to HOLD_S on invalid fresh frames (downhill-r3: also on the downhill)
        return None if F.stale or getattr(self,'hold',None) is None else self.hold.invalid(F.now)
    def _lm_pid_hold(self,F,held):
        return held
    def _lp_downhill(self,F):   # downhill-r1..r3: in the 4 s down_tail, after the 0.4 s hold, up to downhill_hold_max_s after the last valid frame
        now=F.now
        return True if F.down_tail and not F.stale and getattr(self,'hold',None) is not None and self.hold.last is not None and self.hold.last_t is not None and now-self.hold.last_t<=lost_policy.param('downhill_hold_max_s') else None
    def _lm_downhill(self,F,_):
        now=F.now;downhill=F.downhill
        dp=speed_policy.downhill_hold_pwm(self,F)   # speed-policy-r1 (bottom-hold-pwm-r1)
        rp=self.run['parameters'];hs=float(self.hold.last['steer_us'])   # downhill-r3: last valid command (r2 forced >= centre+300 and pushed the car out on the right)
        # downhill-hold-r4 (live b9fa9cff): median of the last 0.5 s of sent commands, within centre -150 .. +300
        try:
            hm=self.hold.median_recent(now,.5)
            if hm is not None:hs=hm
            rcap=DOWN_HOLD_RIGHT_US if downhill else BOTTOM_HOLD_RIGHT_US   # bottom-curve-r1: +300 on the level bottom
            if self._right_hold(hs):rcap=float(rp['max_us'])-float(rp['center_us'])   # downhill-hold-r1
            c0h=float(rp['center_us']);hs=max(c0h-lost_policy.param('downhill_hold_left_us'),min(c0h+rcap,hs))   # downhill-speed-r1: right hold limit 300 -> 150
        except Exception:pass
        return dict(steer_us=hs,pwm=dp,valid=False,reason='DOWNHILL_HOLD',lost_s=round(now-self.hold.last_t,3),last_steer_us=round(self.hold.last['steer_us'],1))
    def _lp_lost(self,F):   # heading-hold-r5: after the 0.4 s hold, keep the last valid yaw for up to LOST_MAX_S
        lost=None
        if not F.stale and getattr(self,'heading',None) is not None and getattr(self,'hold',None) is not None:
            try:
                rp=self.run['parameters'];lost=self.heading.lost(time.monotonic(),self.hold.last_t,rp['center_us'],rp['min_us'],rp['max_us'])
            except Exception:lost=None
            # lost-turn-r1 (c2514de9 33.5 s + 18 evening episodes): a last valid command >= LOST_TURN_US from centre is kept instead
            # crossing-pass-r7 (981f 31.9 s): not inside the crossing window -> the yaw hold steers
            try:
                if lost is not None and getattr(self,'_xp_end',None) is None and self.hold.last is not None and abs(float(self.hold.last['steer_us'])-float(rp['center_us']))>=LOST_TURN_US:
                    lost=(max(float(rp['min_us']),min(float(rp['max_us']),float(self.hold.last['steer_us']))),dict(lost[1],reason='LOST_TURN',revision=LOST_TURN_REVISION,heading_us=round(float(lost[0]),1),last_steer_us=round(float(self.hold.last['steer_us']),1)))
            except Exception:pass
        return lost
    def _lm_lost(self,F,lost):
        last=self.hold.last or {}
        return dict(steer_us=lost[0],pwm=last.get('pwm',self.run['parameters']['pwm']),valid=False,reason=('LOST_TURN' if (lost[1] or {}).get('reason')=='LOST_TURN' else 'LOST_HEADING'),heading_hold=lost[1])
    def _lx_hold_pwm(self,F,p):
        speed_policy.hold_pwm(self,F,p)   # speed-policy-r1: speed_adapt.hold_pwm (writes speed_adapt.cur)
    def _lx_turn_pwm(self,F,p):
        self._upper_pwm(p)   # turn-pwm-r1
    def _lx_down_straight(self,F,p):
        self._down_straight(p,F.result,False)   # down-straight-r1 (GYRO hold on the steep part)
    def _lx_crossing_hold(self,F,p):
        self._xp_hold(p)   # crossing-pass-r9 (user 2026-11-11 dca0 31.15 s): gyro hold inside the crossing window
    def _lx_bottom_map(self,F,p):   # bottom-map-r1: the level bottom without lines follows the map's right curve with the gyro instead of the held command
        if self._bo_skip_map(F,p):return   # bottom-owner-r2: its gyro rate hold replaces MAP_TURN inside the scope
        if not F.downhill:
            try:
                bmi=self._bottom_map(F.now)
                if bmi is not None:p['steer_us']=bmi['steer_us'];p['pwm']=min(float(p['pwm']),float(bmi['pwm']));p['bottom_map']=bmi
            except Exception as e:p['bottom_map']=dict(revision='bottom-map-r1',reason='ERROR',error=type(e).__name__)
    def _lx_bottom_curve(self,F,p):
        if not F.downhill:p['bottom_curve']=dict(revision=BOTTOM_CURVE_REVISION,right_cap_us=BOTTOM_HOLD_RIGHT_US)
    def _lx_downhill_gz_hold(self,F,p):   # downhill-speed-r4: the hold also gets the yaw-rate limit (8d889766 167.7-168.9 s)
        self._down_gz(p,DOWN_HOLD_US if F.downhill else (DOWN_HOLD_US,BOTTOM_HOLD_RIGHT_US))
    def _lx_downhill_clamp(self,F,p):   # downhill-speed-r2: the plain hold also stays within centre -+HOLD_US on the downhill
        if F.downhill:
            c0p=float(self.run['parameters']['center_us']);rhi=(float(self.run['parameters']['max_us'])-c0p) if self._right_hold(p['steer_us']) else DOWN_HOLD_US   # downhill-hold-r1
            p['steer_us']=max(c0p-DOWN_HOLD_US,min(c0p+rhi,float(p['steer_us'])))
    def _lx_as_pid_hold(self,F,p):   # the logged PID_HOLD proposal is a copy of the held command (the later modifiers act on it)
        return dict(p,valid=False,reason='PID_HOLD')
    def _lx_downhill_gz_if(self,F,p):   # downhill-speed-r4
        if F.downhill:self._down_gz(p,DOWN_HOLD_US)
    # ---- segment-ctx-r1 (integration B2, 2026-11-12): the valid-frame steering / PWM layers. Bodies moved from _loop unchanged in effect;
    # the values they share are on F (segment_ctx.Frame, one per _loop, persists across frames like the old locals); the eligibility
    # expressions read the per-frame segment context F.ctx (segment_ctx.Ctx). Order and eligibility: VALID_LAYERS at the end of the file.
    def _upper_turning(self,p,dr,err):return speed_policy.turning(p,dr,err)   # turn-pwm-r1 / upper-post-turn-r1 (speed-policy-r1)
    def _b2_tag(self,p,F):   # segment-ctx-r1 log: segment name + last steering writer of this frame; added after the dispatch (log only)
        try:p['seg']=F.ctx.segment(p,getattr(self,'_xp_end',None) is not None);p['owner']=F.owner
        except Exception:pass
    def _L_jump_filter(self,F):   # jump-filter-r2: a >= 0.7 sign flip of the error for <= 5 frames after the dashed turn is held, not followed
        proposal=F.p;now=F.now;captured=F.captured
        try:
            jf_s=(lambda s_:s_[1] if s_ is not None and s_[0] is captured else None)(F.jf_s0)
            e_j=proposal.get('error');jp=getattr(self,'_jf_prev',None);tn=time.monotonic()
            if proposal['reason']!='WARMUP' and e_j is not None:
                if (jf_s is not None and jp is not None and tn-jp[0]<=1.2 and abs(float(e_j)-jp[1])>=.7 and float(e_j)*jp[1]<0 and getattr(self,'_jf_n',0)<rate_indep.thr(5) and not getattr(self,'_us_on',False)   # upper-single-r1: no re-send while it steers
                        and getattr(self,'hold',None) is not None and self.hold.last_sent(now) is not None):
                    self.controller.__dict__.clear();self.controller.__dict__.update(jf_s);hs_=self.hold.last_sent(now)
                    proposal['jump_filter']=dict(revision='jump-filter-r2',held=True,error_seen=round(float(e_j),3),error_prev=round(jp[1],3),steer_seen=round(float(proposal.get('steer_us') or 0),1),n=getattr(self,'_jf_n',0)+1,steer_us=round(float(hs_['steer_us']),1))   # b1-log: steer_us = held value sent
                    proposal.update(steer_us=hs_['steer_us'],error=jp[1]);self._jf_n=getattr(self,'_jf_n',0)+rate_indep.w()
                else:self._jf_prev=(tn,float(e_j));self._jf_n=0
        except Exception as e:proposal['jump_filter']=dict(revision='jump-filter-r2',reason='ERROR',error=type(e).__name__)
    def _L_start_centre(self,F):   # start-centre-r1: PWM 0 at the centre while the servo travels from its previous command
        proposal=F.p
        try:
            c0s=float(self.run['parameters']['center_us']);els=time.monotonic()-getattr(self,'start_t',time.monotonic());hs=float(getattr(self,'start_hold_s',0.) or 0.)
            if els>=hs:self.start_centred=True;proposal['start_centre']=dict(revision=START_CENTRE_REVISION,done=True,hold_s=round(hs,2),elapsed_s=round(els,2))
            else:proposal.update(steer_us=c0s,pwm=0);proposal['start_centre']=dict(revision=START_CENTRE_REVISION,done=False,hold_s=round(hs,2),elapsed_s=round(els,2))
        except Exception as e:self.start_centred=True;proposal['start_centre']=dict(revision=START_CENTRE_REVISION,reason='ERROR',error=type(e).__name__)
    def _L_imu_boost(self,F):
        speed_policy.boost(self,F)   # speed-policy-r1 step 1: imu_boost
    def _L_dash_route(self,F):   # dash-route-r4: this frame's dash-route dict (the TURN end below may replace it with a FOLLOW copy)
        F.dr=self.latest.get('dash_route') or {}
    def _L_crest_marks_arm(self,F):
        try:self.cmarks.arm(F.dr.get('route')=='OPEN',F.dr.get('pitch_rel_deg'))   # crest-marks-r1: arming on every valid frame
        except Exception:pass
    def _L_dash_turn_end(self,F):   # dash-route-r20: the TURN ends after TURN_MAX_DEG of gyro yaw (then heading hold / dash alignment)
        proposal=F.p;dr=F.dr
        try:
            yw=self.heading._yaw(time.monotonic())
            if getattr(self,'turn_yaw0',None) is None:self.turn_yaw0=yw
            if yw-self.turn_yaw0>=TURN_MAX_DEG:
                self.dash.mode='FOLLOW';self.dash.followed=True;self.dash.turned=True;F.dr=dict(dr,mode='FOLLOW');proposal['dash_turn_end']=dict(turned_deg=round(yw-self.turn_yaw0,1))
        except Exception:pass
    def _L_dash_turn(self,F):   # dash-route-r4: firmer left turn while the dashed-row TURN is active
        proposal=F.p;dr=F.dr
        tu=turn_steer(self.run['parameters']['center_us'],self.run['parameters']['min_us']);tvi=None
        try:   # dash-turn-vision-r1 (user 2026-11-07, live ee70dc45): steer the TURN by the visible dashes, not a fixed amount
            dtc=getattr(self,'dturn_cfg',None)
            if dtc is None:
                try:dtc=self.dturn_cfg=json.loads((Path(__file__).with_name('dash_turn.json')).read_text(encoding='utf8'))
                except (OSError,ValueError):dtc=self.dturn_cfg={}
            if dtc.get('enabled') and dr.get('x_row') is not None:
                c0t=float(self.run['parameters']['center_us']);tv=c0t+float(dtc.get('k_turn',800.))*(float(dr['x_row'])-float(dtc.get('axis',338.)))/320.
                tv=max(float(self.run['parameters']['min_us']),min(c0t-float(dtc.get('turn_min_us',200.)),tv));tvi=dict(revision='dash-turn-vision-r1',x_row=dr['x_row'],fixed_us=round(tu,1),vision_us=round(tv,1));tu=tv
        except Exception as e:tvi=dict(revision='dash-turn-vision-r1',reason='ERROR',error=type(e).__name__)
        proposal['dash_turn']=dict(pid_steer_us=proposal['steer_us'],turn_us=tu,applied=tu<proposal['steer_us'])
        if tvi is not None:proposal['dash_turn']['vision']=tvi
        proposal['steer_us']=min(proposal['steer_us'],tu)
        if getattr(self,'heading',None) is not None:self.heading.on_turn_start(time.monotonic())
        if getattr(getattr(self,'heading',None),'lane_cap',None):proposal['steer_us']=max(proposal['steer_us'],self.run['parameters']['center_us']-300.)   # heading-hold-r9: softer TURN in the crest option
    def _L_heading_hold(self,F):   # heading-hold-r1: after the dashed-row turn keep the lane heading until the downhill
        proposal=F.p;dr=F.dr;ctx=F.ctx
        try:
            rp=self.run['parameters']
            jst0=ctx.jstate()
            nrm=ctx.sel is None and jst0 in (None,'FOLLOW')   # heading-hold-r4
            proposal['steer_us'],proposal['heading_hold']=self.heading.step(time.monotonic(),dr.get('route')=='OPEN',dr if ctx.sel=='dash_route' else {},proposal['steer_us'],rp['center_us'],rp['min_us'],rp['max_us'],normal=nrm,vis_err=proposal.get('image_error') if nrm else None)
        except Exception as e:proposal['heading_hold']=dict(revision=HEADING_REVISION,reason='HEADING_ERROR',error=type(e).__name__)
    def _L_upper_single(self,F):   # upper-single-r1 (user 2026-11-11 5235): heading_hold still runs (reference, dash gate, PWM), its steering is replaced
        F.usv=self._us_valid(F.p,F.dr)
    def _L_lane_segment(self,F):   # lane-segment-r1 (user 2026-11-03, live 43340794): after the dashed-row TURN the steering follows the dashed line's extension, or the side lines / gyro without dashes
        proposal=F.p;dr=F.dr
        try:
            hh=proposal.get('heading_hold') or {}
            if hh.get('active') and hh.get('reason')!='TURN' and dr.get('route')=='OPEN' and ((hh.get('err_deg') is not None and abs(hh['err_deg'])<=(LANE_LOCK_GATE if getattr(self.lane,'lock',False) else 40.)) or (getattr(self.lane,'lock',False) and dr.get('line_a') is not None)):   # lane-segment-r7/r9: heading-hold turn until within LOCK_GATE_DEG, or at once when the dashed line is fitted
                try:gzl=self.heading._gz(time.monotonic())
                except Exception:gzl=None
                rp=self.run['parameters']
                ls,li=self.lane.step(time.monotonic(),dr,hh.get('yaw_deg'),gzl,float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),self.row_ratio*359.,ref=hh.get('ref_deg'),heading_us=proposal['steer_us'],prev_us=(getattr(getattr(self,'hold',None),'sent_cmd',None) or {}).get('steer_us'))   # lane-segment-r9: heading-hold reference / steering; r12: last sent command (steering play)
                proposal['lane_segment']=li
                if ls is not None:
                    li['heading_steer_us']=round(proposal['steer_us'],1)
                    if str(li.get('source') or '').endswith('_default'):li['skipped']='DEFAULT_GAP';ls=proposal['steer_us']   # lane-default-skip-r1 (user 2026-11-07, live 6d870429): the side-line + default-gap dash estimate sat left of the lane; keep the heading / PID steering
                    ls=self._approach(proposal,li,dr,ls)   # downhill-approach-r1
                    try:   # lane-turn-guard-r1 (user 2026-11-11 7b24 58.1-59.0 s): while |heading error| >= LANE_TURN_GUARD_DEG the lane command may not steer further away than the heading hold
                        ehg=hh.get('err_deg');hsu=float(proposal['steer_us'])
                        if isinstance(ehg,(int,float)) and ((ehg>=LANE_TURN_GUARD_DEG and ls>hsu) or (ehg<=-LANE_TURN_GUARD_DEG and ls<hsu)):
                            li['turn_guard']=dict(revision='lane-turn-guard-r1',err_deg=round(float(ehg),2),lane_us=round(float(ls),1),steer_us=round(hsu,1));ls=hsu
                    except Exception:pass
                    proposal['steer_us']=ls
                    if getattr(self,'entry',None) is not None:self.entry.note_lane(time.monotonic(),ls)
        except Exception as e:proposal['lane_segment']=dict(revision=LANE_REVISION,reason='LANE_ERROR',error=type(e).__name__)
    def _L_solid_takeover(self,F):   # solid-takeover-r1 (user 2026-11-06 eb7d1033): after the dashes the camera takes over when its error stays beyond 0.4 on one side for 0.5 s
        proposal=F.p;dr=F.dr;ctx=F.ctx
        try:
            jst1=ctx.jstate()
            nrm1=ctx.sel is None and jst1 in (None,'FOLLOW')
            sts,sti=self.solid.step(time.monotonic(),dr.get('route')=='OPEN',proposal.get('heading_hold'),bool(dr.get('used')),proposal.get('image_error'),nrm1,dr.get('pitch_rel_deg'))
            if sti is not None:proposal['solid_takeover']=sti
            if sti is not None and sti.get('released') and getattr(self,'heading',None) is not None and (proposal.get('heading_hold') or {}).get('yaw_deg') is not None:   # solid-takeover-r2: HIDDEN WRITE (kept): rebases heading_hold's reference and I term
                sti['rebased_from_deg']=round(float(self.heading.ref),2);self.heading.ref=float(proposal['heading_hold']['yaw_deg']);self.heading.i_us=0.;sti['rebased_to_deg']=round(self.heading.ref,2)
            if sts is not None:
                sti['replaced_us']=round(float(proposal['steer_us']),1);rp=self.run['parameters'];proposal['steer_us']=max(float(rp['min_us']),min(float(rp['max_us']),sts));sti['steer_us']=round(float(proposal['steer_us']),1)   # b1-log: after value
        except Exception as e:proposal['solid_takeover']=dict(revision=solid_takeover.REVISION,reason='ERROR',error=type(e).__name__)
    def _L_i_ramp(self,F):   # i-ramp-r1 (ramp-bias-20261119): release the right-curve integral memory while climbing (controller.i only)
        try:
            if getattr(self.iramp,'wants_lap',False):   # i-exit-r1 (exit-bias-20261119): lap_pos state of its last update (as lap_ff reads it); no exit keys -> the call below, unchanged
                tp=getattr(self,'_lpos',None);an=getattr(tp,'anchor',None) if tp else None
                lpi=dict(lap=tp.lap,anchor=None if an is None else an[0],since_anchor_m=None if an is None else tp.D-an[1],lost=tp.lost) if tp else None
                inf=self.iramp.step(time.monotonic(),F.dpr,self.controller,lap=lpi)
            else:inf=self.iramp.step(time.monotonic(),F.dpr,self.controller)
            if inf is not None:F.p['i_ramp']=inf
        except Exception as e:F.p['i_ramp']=dict(revision=i_ramp.REVISION,reason='ERROR',error=type(e).__name__)
    def _L_gain_schedule(self,F):   # gain-schedule-r1..r4 (live 6838a1b9 / c6bc3b36): steering offset scaled by ref PWM / current PWM on normal-path frames
        proposal=F.p;ctx=F.ctx
        gsk=gain_schedule.k_for_pwm(getattr(self,'gs_cfg',None) or gain_schedule.DEFAULTS,self.speed.cur) if (getattr(self,'speed',None) is not None and self.speed.cur) else None   # gain-schedule-r4: json switch
        if gsk is not None:   # gain-schedule-r2: reference 15 (r4: ref_pwm / enabled from gain_schedule.json)
            try:
                jg=ctx.jstate()   # value unused (as before); kept because its evaluation can raise -> GS_ERROR
                sp=(ctx.sel in ('dash_route','crest_midpoint')   # junction-normal-r1: a junction take is a normal path here
                    or ctx.hh_active(proposal) or 'dash_turn' in proposal or F.downhill
                    or (proposal.get('imu_boost') or {}).get('mode')=='uphill')   # gain-schedule-r3 (live a0e410ed): on the uphill the 0.6 cut left the car at the right edge (PID 1081 -> sent 1187)
                if not sp:
                    k=gsk;c0=float(self.run['parameters']['center_us'])
                    proposal['gain_schedule']=dict(revision=gain_schedule.REVISION,pwm=round(self.speed.cur,2),k=round(k,3),pid_steer_us=round(proposal['steer_us'],1))
                    k,rei=right_entry.k_for(k,proposal['steer_us'],c0)   # right-entry-r1: right turns past +240 us fade the cut out
                    if rei is not None:proposal['gain_schedule']['right_entry']=rei
                    proposal['steer_us']=c0+(proposal['steer_us']-c0)*k
            except Exception as e:proposal['gain_schedule']=dict(revision='gain-schedule-r1',reason='GS_ERROR',error=type(e).__name__)
    # yaw-inner-r1 group (yaw_inner.py, user 2026-11-07, .515/.517): curve map, cmd comp, crossing pass, curve smooth, inner loop, crossing term cap,
    # weave flag. One try around the group (as before): an exception in a step without its own try ends the group with yaw_inner ERROR.
    def _L_yaw_prelude(self,F):   # yaw-inner-r1..r6: inner-loop eligibility (ely) and its inputs
        proposal=F.p;ctx=F.ctx
        F.dry=dry=self.latest.get('dash_route') or {};F.pdy=dry.get('pitch_rel_deg')
        F.dny=dny=bool(F.downhill and ctx.steep10())   # yaw-inner-r6: the steep part of the downhill
        F.bty=bty=bool(not F.downhill and ctx.level())   # bottom-inner-r1: level ground once the slope rules are released (a3a7 92.0-94.8 s)
        F.ely=(ctx.proposal_only(proposal) and ctx.pitch is not None and (ctx.route in ('WAIT_CLIMB','CLIMBED','OPEN') or (ctx.route=='DONE' and (dny or bty))) and ctx.normal_rest(proposal)
               and (not F.down_tail or dny or bty))   # r6: every pitch passes, yaw_inner sorts flat / climb / crest / downhill / transition
        try:F.gzy=self.heading._gz(time.monotonic()) if getattr(self,'heading',None) is not None else None
        except Exception:F.gzy=None
        F.cmf=None;F.c4x=None   # c4-exit-r1: reset per frame; curve-map-r1 (curve_map.py, user 2026-11-07): yaw-rate floor while a mapped curve is not finished
    def _L_curve_map(self,F):
        proposal=F.p;ctx=F.ctx
        try:
            fl=bool(ctx.flat_wc());blk=self._c2_block(fl,proposal) if getattr(self,'_cc',None) else False   # crossings-ctl-r1 c2-block-r1 (off = False)
            cmf,cmi=self.cmap.step(time.monotonic(),F.gzy,ctx.route,fl and not blk,near=proposal.get('image_error'),far=(proposal.get('turn_preview') or {}).get('far_projected_error'),speed=getattr(self,'_cm_speed',None))   # curve-map-r5: previous frame's wheel speed
            if cmi is not None:proposal['curve_map']=cmi
            if not F.ely:cmf=None
            c4x=self._c4x(cmi)   # c4-exit-r1 (c4-exit-20261120): curve_map.json c4_exit absent / off -> None, nothing changes
            if c4x is not None:
                F.c4x=c4x;proposal['curve_map']['c4_exit']=c4x
                if c4x.get('floor'):cmf=None   # no yaw-rate floor on the main straight after the curve-4 exit
            F.cmf=cmf
        except Exception as e:proposal['curve_map']=dict(revision=curve_map.REVISION,reason='ERROR',error=type(e).__name__);F.cmf=None
    def _c4x(self,cmi):   # c4-exit-r1 (c4-exit-20261120): inside [from_m, to_m] after this lap's lap_pos C4e crossing a curve_map FLOOR / RELEASED is not a curve for the controller
        try:
            c=self.cmap.c.get('c4_exit')
            if not isinstance(c,dict) or not c.get('enabled') or not isinstance(cmi,dict) or cmi.get('reason') not in ('FLOOR','RELEASED'):return None
            lp=getattr(self,'_lpos',None)
            if not lp or getattr(lp,'lost',True):return None
            an=str(c.get('anchor','C4e'));dc=None
            for e in reversed(getattr(lp,'events',None) or []):   # = lap_speed.LapSpeed.crossing: last crossing of `an`, only in the current lap
                if e.get('name')==an and 'd_cross' in e:
                    dc=float(e['d_cross']) if e.get('lap')==getattr(lp,'lap',None) else None;break
            if dc is None:return None
            rel=float(lp.D)-dc
            if not float(c.get('from_m',0.))<=rel<=float(c.get('to_m',9.)):return None
            return dict(revision='c4-exit-r1',rel_m=round(rel,2),floor=bool(c.get('floor',True)),ctx=bool(c.get('ctx',True)))
        except Exception:return None
    def _L_curve_ctx(self,F):   # this frame's curve state for the layers below (yaw-inner-r11 / r13, weave-slow-r1, curve-map-r2 / r5)
        proposal=F.p;ctx=F.ctx;gzy=F.gzy
        F.cmi_=cmi_=proposal.get('curve_map') or {}   # yaw-inner-r11: far hold only in a curve (map FLOOR/RELEASED or |gz| >= 15 dps)
        F.inc=bool(cmi_.get('reason') in ('FLOOR','RELEASED')) or (gzy is not None and math.isfinite(float(gzy)) and abs(float(gzy))>=15.)
        F.msg=msg=straight_slew.mean_gz(self._ss_gz,time.monotonic(),gzy,self.sslew.c) if getattr(self,'sslew',None) is not None else None   # weave-slow-r1: 2 s gz mean once per frame (a weave cancels, a curve does not)
        F.trn=trn=msg is not None and abs(msg)>=float(self.sslew.c['mean_gz_max']) if getattr(self,'sslew',None) is not None else True;F.ssg=False
        F.mpc=bool(cmi_.get('reason')=='FLOOR' or (cmi_.get('reason')=='RELEASED' and cmi_.get('released')!='ANGLE_DONE' and trn)) or bool(ctx.route=='DONE' and F.down_tail and not F.downhill and ctx.level())   # bottom-release-r1: level bottom -> pull cap   # yaw-inner-r13: mapped curve not finished -> pull cap; weave-slow-r1: only while really turning
        if (getattr(F,'c4x',None) or {}).get('ctx'):F.mpc=bool(ctx.route=='DONE' and F.down_tail and not F.downhill and ctx.level())   # c4-exit-r1 ctx: no mapped-curve pull cap after the curve-4 exit (the level-bottom part kept)
        F.wcy=wcy=self._wheel_cps();self._cm_speed=None if wcy is None else wcy*M_PER_COUNT;F.lky=lky=self.cmap.lock_dps(None if wcy is None else wcy*M_PER_COUNT) if F.cmf is not None else None   # curve-map-r2: yaw-rate lock replaces the floor (None when the lock is off)   # curve-map-r5: _cm_speed (m/s) for the next cmap.step
        if lky is not None:F.cmf=None;proposal['curve_map']['lock_target_dps']=round(lky,2)
    def _L_cmd_comp(self,F):   # cmd-comp-r1: subtract the in-flight steering on flat straights, before the inner loop
        proposal=F.p;ctx=F.ctx;rp=self.run['parameters'];cmi_=F.cmi_
        try:
            stc=bool(F.ely and ctx.flat_wc() and cmi_.get('reason') not in ('FLOOR','RELEASED') and not F.inc)
            sc_,cci=self.ccomp.step(time.monotonic(),proposal['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),float(rp.get('kp') or 0.),F.gzy,straight=stc)
            if cci is not None:proposal['cmd_comp']=cci;proposal['steer_us']=sc_
        except Exception as e:proposal['cmd_comp']=dict(revision=cmd_comp.REVISION,reason='ERROR',error=type(e).__name__)
    def _xpa_load(self):   # xp-anchor-r1 (map-20261114): xp_anchor.json; None = off (crossing_pass / xp_corridor / crossing_source keep the curve_map CURVE_END anchor)
        try:c=json.loads((Path(__file__).parent/'xp_anchor.json').read_text(encoding='utf8'))
        except Exception:return None
        return c if isinstance(c,dict) and c.get('enabled') else None
    def _xpa_update(self,cmi_):   # xp-anchor-r1: virtual curve-2 end (curve_map metres) = lap_pos C2e crossing + k_m, set once per C2e; logs a CURVE_END that the anchor replaces
        c=getattr(self,'_xpa_c',None)
        if not c:return
        try:
            self._xpa_track(cmi_,c)   # xp-anchor-r2 (absent keys = returns at once)
            lp=getattr(self,'_lpos',None)
            if lp and lp.c2e is not None and not lp.lost and lp.c2e['n']!=getattr(self,'_xpa_n',None):
                since=float(lp.D)-float(lp.c2e['d_cross'])
                if since<=float(c.get('max_decide_m',2.0)):
                    self._xpa_n=lp.c2e['n'];self._xpa_a=float(self._lpos_cmd)-since+float(c.get('k_m',2.8))
                    self._xpa_log=dict(revision='xp-anchor-r1',event='ANCHOR',anchor_m=round(self._xpa_a,2),c2e_since_m=round(since,2),lap=lp.c2e.get('lap'),s_m=round(lp.s(),2))
            a=getattr(self,'_xpa_a',None)
            if a is not None and isinstance(cmi_,dict) and cmi_.get('reason')=='CURVE_END' and (cmi_.get('seg'),cmi_.get('idx')) in (('start',1),('after_down',2)) and cmi_.get('dist_m') is not None and float(self.cmap.dist)-a<float(c.get('horizon_m',10.)):
                self._xpa_log=dict(revision='xp-anchor-r1',event='CURVE_END_SEEN',ce_minus_anchor_m=round(float(cmi_['dist_m'])-a,2),turned_deg=cmi_.get('turned_deg'))
        except Exception as ex:self._xpa_log=dict(revision='xp-anchor-r1',reason='ERROR',error=type(ex).__name__)
    def _xpa_track(self,cmi_,c):   # xp-anchor-r2 (verify-20261114, run 3cf4): curve_map-only plausibility of a curve-2 CURVE_END: metres from turned_deg >= ce_veto_level_deg to the CURVE_END
        T=c.get('ce_veto_after_level_m')
        if T is None or not isinstance(cmi_,dict):return
        key=(cmi_.get('seg'),cmi_.get('idx'));r=cmi_.get('reason');c2=key in (('start',1),('after_down',2))
        if r in ('FLOOR','RELEASED') and c2:
            if getattr(self,'_xpa_t180',None) is None and (cmi_.get('turned_deg') or 0)>=float(c.get('ce_veto_level_deg',180.)):self._xpa_t180=float(self.cmap.dist)
        elif r=='CURVE_END':
            t0=getattr(self,'_xpa_t180',None);self._xpa_t180=None
            if c2 and t0 is not None and cmi_.get('dist_m') is not None and float(cmi_['dist_m'])-t0>float(T):
                self._xpa_veto=float(cmi_['dist_m']);self._xpa_log=dict(revision='xp-anchor-r2',event='CE_VETO',after_level_m=round(float(cmi_['dist_m'])-t0,2),turned_deg=cmi_.get('turned_deg'),window=bool(c.get('ce_veto_window')),corridor=bool(c.get('ce_veto_corridor')))
        elif r=='FLOOR':self._xpa_t180=None
    def _xpa_vetoed(self,d,key):   # xp-anchor-r2: True when the CURVE_END at curve_map distance d was judged implausible and switch `key` is on
        c=getattr(self,'_xpa_c',None);v=getattr(self,'_xpa_veto',None)
        try:return bool(c and c.get(key) and v is not None and d is not None and float(d)==v)
        except Exception:return False
    def _xpa_cur(self,key):   # xp-anchor-r1: the position anchor of this pass when switch `key` is on, None = no anchor (CURVE_END as before)
        c=getattr(self,'_xpa_c',None);a=getattr(self,'_xpa_a',None)
        if not c or not c.get(key) or a is None:return None
        try:return a if float(self.cmap.dist)-a<float(c.get('horizon_m',10.)) else None
        except Exception:return None
    def _xpa_cor(self,proposal,we,F):   # xp-anchor-r1: corridor scope anchor. pos_corridor: the position anchor from the window start distance on (flat lap frame, not FLOOR); ce_max_err_m: a CURVE_END-anchored scope is not armed when lap_pos puts that CURVE_END farther than this from C2e + k_m
        c=getattr(self,'_xpa_c',None)
        if not c:return we
        try:
            xa=self._xpa_cur('pos_corridor')
            if xa is not None:
                xc=getattr(self,'_xp_c',{}) or {}
                return xa if (float(self.cmap.dist)-xa>=float(xc.get('start_m',.4)) and F.cmi_.get('reason')!='FLOOR' and F.ctx.flat_wc()) else None
            if we is not None and self._xpa_vetoed(we,'ce_veto_corridor'):return None   # xp-anchor-r2: CE-anchored scope of an implausible CURVE_END
            if we is not None and c.get('ce_max_err_m') is not None and self._xpa_cur('pos_window') is None:
                lp=getattr(self,'_lpos',None)
                if lp and not lp.lost and lp.c2e is not None and float(lp.D)-float(lp.c2e['d_cross'])<15.:
                    err=we-(float(self._lpos_cmd)-(float(lp.D)-float(lp.c2e['d_cross'])))-float(c.get('k_m',2.8))
                    if abs(err)>float(c['ce_max_err_m']):
                        if getattr(self,'_xpa_vw',None)!=we:self._xpa_vw=we;proposal['xp_anchor']=dict(revision='xp-anchor-r1',event='CORRIDOR_VETO',ce_minus_expected_m=round(err,2))
                        return None
        except Exception as ex:proposal['xp_anchor']=dict(revision='xp-anchor-r1',reason='ERROR',error=type(ex).__name__)
        return we
    def _xpa_ref(self,xk,tn,gz,p):   # xp-anchor-r1: while xp_corridor captures psi0, |gz| > ref_max_gz_dps restarts the capture; none by ref_give_up_after_m after the anchor -> corridor off for this pass. False = skip the corridor step this frame (command unchanged, as in REF)
        c=getattr(self,'_xpa_c',None)
        if not c or c.get('ref_max_gz_dps') is None or xk.psi0 is not None or gz is None or abs(float(gz))<=float(c['ref_max_gz_dps']):return True
        xk.ref=[];xk.t0=tn
        if xk.dist_after is not None and xk.dist_after>float(c.get('ref_give_up_after_m',1.9)):
            xk.active=False;xk.mode=None;p['xp_corridor']=dict(revision=xp_corridor.REVISION,mode='OFF',reason='REF_GAVE_UP',gz=round(float(gz),1),dist_after_m=round(xk.dist_after,2));return False
        p['xp_corridor']=dict(revision=xp_corridor.REVISION,mode='REF',reason='REF_RESTART_GZ',gz=round(float(gz),1),dist_after_m=None if xk.dist_after is None else round(xk.dist_after,2));return False
    def _cc_load(self):   # crossings-ctl-r1: crossings_ctl.json; None = every crossings-ctl rule off
        try:c=json.loads((Path(__file__).parent/'crossings_ctl.json').read_text(encoding='utf8'))
        except Exception:return None
        return c if isinstance(c,dict) and c.get('enabled') else None
    def _lp_since(self,anchor):   # crossings-ctl-r1: metres since the lap_pos anchor `anchor` (last lap_pos update), None if not anchored there / lost
        lp=getattr(self,'_lpos',None)
        if not lp or lp.lost or lp.anchor is None or lp.anchor[0]!=anchor:return None
        return float(lp.D)-float(lp.anchor[1])
    def _c2_block(self,flat,proposal):   # c2-block-r1 (crossings-20261114): no curve-2 start on the diagonal before c2_block_until_m after the curve-1 end
        try:
            u=self._cc.get('c2_block_until_m')
            if u is None or self.cmap.cur is not None or (self.cmap.seg,self.cmap.idx) not in (('start',1),('after_down',2)):return False
            sn=self._lp_since('C1e')
            if sn is None or not 0.<=sn<float(u):return False
            if flat:proposal['c2_block']=dict(revision='c2-block-r1',since_c1e_m=round(sn,2))
            return True
        except Exception as e:proposal['c2_block']=dict(revision='c2-block-r1',reason='ERROR',error=type(e).__name__);return False
    def _csz_load(self):   # crossing-source-zone-r1: crossing_source.json (enabled + zone keys); None = no zone field in requests
        try:c=json.loads((Path(__file__).parent/'crossing_source.json').read_text(encoding='utf8'))
        except Exception:return None
        return c if isinstance(c,dict) and c.get('enabled') else None
    def _csz_track(self,cmi_):   # crossing-source-zone-r1: anchor at the curve-2 CURVE_END (same test as crossing_pass), end at the next FLOOR
        c=getattr(self,'_csz_c',None)
        if not c or not isinstance(cmi_,dict):return
        xz=self._xpa_cur('pos_zone')   # xp-anchor-r1: the zone starts from the position anchor (ends by zone_end_m); the CURVE_END / FLOOR events are not used while it is current
        if xz is not None:
            if getattr(self,'_xpa_zn',None)!=xz:self._xpa_zn=xz;self._csz_end=xz
            return
        try:
            if cmi_.get('reason')=='CURVE_END' and (cmi_.get('seg'),cmi_.get('idx')) in (('start',1),('after_down',2)) and (cmi_.get('turned_deg') or 0)>=float(c.get('zone_min_turned_deg',100.)) and cmi_.get('dist_m') is not None:self._csz_end=float(cmi_['dist_m'])
            elif cmi_.get('reason')=='FLOOR':self._csz_end=None
        except Exception:self._csz_end=None
    def _csz_req(self):   # crossings-ctl-r1 lap-zone-r1: xp2 (curve_map / xp-anchor) first, then the lap_pos zones (crossing_source.json lap_zones; absent = {})
        r=self._csz_req_xp2()
        if r or not getattr(self,'_csz_c',None) or not self._csz_c.get('lap_zones'):return r
        return self._csz_lap()
    def _csz_lap(self):   # lap-zone-r1: zone from the lap position of the last lap_pos update (previous valid frame)
        try:
            c=self._csz_c;lp=getattr(self,'_lpos',None)
            if not lp or lp.lost or lp.anchor is None:return {}
            name=lp.anchor[0];since=float(lp.D)-float(lp.anchor[1]);zs=tuple(c.get('zones') or ())
            for z,spec in (c.get('lap_zones') or {}).items():
                if z not in zs:continue
                for rg in (spec if isinstance(spec,list) else [spec]):
                    if name==rg.get('anchor') and float(rg.get('from_m',0.))<=since<float(rg.get('to_m',0.)):return {'zone':z}
        except Exception:return {}
        return {}
    def _csz_req_xp2(self):   # crossing-source-zone-r1: extra request fields for the worker ({} outside the zone / switch off)
        c=getattr(self,'_csz_c',None);e=getattr(self,'_csz_end',None)
        if not c or e is None:return {}
        try:xd=float(self.cmap.dist)-e
        except Exception:return {}
        if xd>=float(c.get('zone_end_m',4.5)):self._csz_end=None;return {}
        return {'zone':'xp2'} if xd>=float(c.get('zone_start_m',.4)) else {}
    def _L_crossing_pass(self,F):   # crossing-pass-r1: odometry window after curve 2 -> reduced camera command, the inner loop holds the heading
        proposal=F.p;ctx=F.ctx;rp=self.run['parameters'];cmi_=F.cmi_
        self._xpa_update(cmi_);self._csz_track(cmi_)   # crossing-source-zone-r1; xp-anchor-r1: position anchor first (off = returns at once)
        try:
            if not hasattr(self,'_xp_c'):
                try:self._xp_c=json.loads((Path(__file__).parent/'crossing_pass.json').read_text())
                except Exception:self._xp_c={}
            xc=self._xp_c
            if xc.get('enabled'):
                xw=self._xpa_cur('pos_window')   # xp-anchor-r1: the position anchor replaces the curve-2 CURVE_END for this pass (None = CURVE_END as before)
                if xw is not None:
                    if getattr(self,'_xpa_wn',None)!=xw:self._xpa_wn=xw;self._xp_end=xw;self._xp_rel=False;self._xp_last=None;self._xp_jump=False
                elif cmi_.get('reason')=='CURVE_END' and (cmi_.get('seg'),cmi_.get('idx')) in (('start',1),('after_down',2)) and (cmi_.get('turned_deg') or 0)>=float(xc.get('min_turned_deg',100.)) and cmi_.get('dist_m') is not None and not self._xpa_vetoed(cmi_.get('dist_m'),'ce_veto_window'):self._xp_end=float(cmi_['dist_m']);self._xp_rel=False;self._xp_last=None;self._xp_jump=False
                xe=getattr(self,'_xp_end',None);xd=None if xe is None else float(self.cmap.dist)-xe
                if xd is not None and xd>float(xc.get('end_m',2.2)):self._xp_end=None
                elif xd is not None and xd>=float(xc.get('start_m',.4)) and cmi_.get('reason')!='FLOOR' and ctx.flat_wc():
                    # crossing-pass-r4p (2026-11-09, 8b5d / 7d42 / 84ba): |error| >= release_err hands the full camera back for the rest of this window; after the window the camera fades back over fade_s
                    # crossing-pass-r5 (2026-11-10 4e52 37.0 s): after >= jump_gap_s without a valid path the first error flips sign by >= jump_de -> no release for the rest of this window
                    pex=getattr(self,'_xp_pe',None);enx=float(proposal.get('error') or 0.)
                    if xc.get('jump_de') is not None and pex is not None and time.monotonic()-pex[0]>=float(xc.get('jump_gap_s',.5)) and enx*pex[1]<0 and abs(enx-pex[1])>=float(xc['jump_de']):self._xp_jump=True
                    if xc.get('jump_step') is not None and pex is not None and time.monotonic()-pex[0]<=float(xc.get('jump_step_s',.3)) and abs(enx-pex[1])>=float(xc['jump_step']):self._xp_jump=True   # crossing-pass-r6 (2026-11-10 0f1e 31.59 s)
                    if not getattr(self,'_xp_rel',False) and not getattr(self,'_xp_jump',False) and abs(enx)>=float(xc.get('release_err',9.)):self._xp_rel=True;self._xp_last=None
                    if getattr(self,'_xp_rel',False):proposal['crossing_pass']=dict(revision='crossing-pass-r4p',active=False,released=True,dist_after_m=round(xd,2),error=round(float(proposal.get('error') or 0.),3))
                    else:
                        c0x=float(rp['center_us']);sx=float(proposal['steer_us']);proposal['steer_us']=c0x+float(xc.get('cam_frac',.3))*(sx-c0x);self._xp_last=time.monotonic()
                        proposal['crossing_pass']=dict(revision='crossing-pass-r6',active=True,jump_hold=bool(getattr(self,'_xp_jump',False)),dist_after_m=round(xd,2),camera_us=round(sx,1),steer_us=round(proposal['steer_us'],1))
                if proposal.get('error') is not None and proposal.get('reason')=='PROPOSAL_ONLY':self._xp_pe=(time.monotonic(),float(proposal['error']))   # crossing-pass-r5: last normal valid error and its time
                if proposal.get('crossing_pass') is None and getattr(self,'_xp_last',None) is not None:   # crossing-pass-r4p: fade cam_frac -> 1 over fade_s after the window
                    fdt=time.monotonic()-self._xp_last;fsx=float(xc.get('fade_s',0.))
                    if fsx>0. and fdt<fsx and cmi_.get('reason')!='FLOOR' and abs(float(proposal.get('error') or 0.))<float(xc.get('release_err',9.)) and ctx.route=='WAIT_CLIMB':   # route only, no pitch test (unlike the window above)
                        cfx=float(xc.get('cam_frac',.3));fx=cfx+(1.-cfx)*fdt/fsx;c0x=float(rp['center_us']);sx=float(proposal['steer_us']);proposal['steer_us']=c0x+fx*(sx-c0x)
                        proposal['crossing_pass']=dict(revision='crossing-pass-r4p',fade=round(fx,2),camera_us=round(sx,1),steer_us=round(proposal['steer_us'],1))
                    else:self._xp_last=None
        except Exception as e:proposal['crossing_pass']=dict(revision='crossing-pass-r1',reason='ERROR',error=type(e).__name__)
        if getattr(self,'_xpa_log',None) is not None:proposal['xp_anchor']=self._xpa_log;self._xpa_log=None   # xp-anchor-r1 log (events only)
    def _L_xp_corridor(self,F):   # xp-corridor-r2: after curve 2, from the crossing window's first frame to end_m after the curve end: |yaw - psi0| > soft pushes back toward psi0, >= hard = gyro hold (xp_corridor.py)
        xk=getattr(self,'xpc',None)
        if xk is None or not xk.enabled:return
        proposal=F.p
        try:
            xp=proposal.get('crossing_pass');we=getattr(self,'_xp_end',None) if isinstance(xp,dict) and 'dist_after_m' in xp else None
            we=self._xpa_cor(proposal,we,F)   # xp-anchor-r1 (off = unchanged)
            if we is None and not xk.active:return   # outside the corridor: no clock / IMU read
            rp=self.run['parameters'];tn=time.monotonic()
            cd=getattr(getattr(self,'cmap',None),'dist',None)
            if not xk.scope(tn,we,None if cd is None else float(cd)):return
            if not self._xpa_ref(xk,tn,getattr(F,'gzy',None),proposal):return   # xp-anchor-r1: psi0 capture gz guard (off = True)
            try:yw=self.heading._yaw(tn) if getattr(self,'heading',None) is not None else None
            except Exception:yw=None
            out,inf=xk.step(tn,yw,getattr(F,'gzy',None),float(proposal['steer_us']),float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),True)
            if inf is not None:proposal['xp_corridor']=inf;proposal['steer_us']=out
        except Exception as e:proposal['xp_corridor']=dict(revision=xp_corridor.REVISION,reason='ERROR',error=type(e).__name__)
    def _lx_xp_corridor(self,F,p):   # xp-corridor-r2: lost frame inside the corridor while |dev| > soft -> the corridor's gyro hold instead of the held value
        xk=getattr(self,'xpc',None)
        if xk is None or not xk.enabled or not xk.active:return
        try:
            rp=self.run['parameters'];tn=time.monotonic()
            if not xk.scope(tn,None,None) or getattr(self,'heading',None) is None:return
            try:yw=self.heading._yaw(tn)
            except Exception:yw=None
            try:gz=self.heading._gz(tn)
            except Exception:gz=None
            if not self._xpa_ref(xk,tn,gz,p):return   # xp-anchor-r1: psi0 capture gz guard on lost frames (off = True)
            out,inf=xk.step(tn,yw,gz,float(p['steer_us']),float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),False)
            if inf is not None:p['xp_corridor']=inf;p['steer_us']=out
        except Exception as e:p['xp_corridor']=dict(revision=xp_corridor.REVISION,reason='ERROR',error=type(e).__name__)
    def _L_curve_smooth(self,F):   # curve-smooth-r1: in a mapped curve on the flat the camera command is low-passed before the inner loop
        proposal=F.p;ctx=F.ctx;cmi_=F.cmi_
        try:
            tcs=float(self.yin.c.get('curve_smooth_tau_s') or 0.);ncs=time.monotonic()
            inc_s=bool(tcs>0. and (cmi_.get('reason')=='FLOOR' or (cmi_.get('reason')=='RELEASED' and cmi_.get('released')!='ANGLE_DONE')) and ctx.flat_wc() and not (getattr(F,'c4x',None) or {}).get('ctx'))   # c4-exit-r1 ctx: no curve_smooth low-pass after the curve-4 exit
            pcs=getattr(self,'_cs',None)
            if inc_s and pcs is not None and 0.<ncs-pcs[0]<=.5:
                raw_cs=float(proposal['steer_us']);fcs=pcs[1]+(1.-math.exp(-(ncs-pcs[0])/tcs))*(raw_cs-pcs[1]);proposal['steer_us']=fcs;self._cs=(ncs,fcs)
                proposal['curve_smooth']=dict(revision='curve-smooth-r1',camera_us=round(raw_cs,1),steer_us=round(fcs,1),tau_s=tcs)
            else:self._cs=(ncs,float(proposal['steer_us']))
        except Exception as e:proposal['curve_smooth']=dict(revision='curve-smooth-r1',reason='ERROR',error=type(e).__name__)
    def _L_yaw_inner(self,F):   # yaw-inner-r1: inner loop on the gyro yaw rate (normal path frames only: ely)
        proposal=F.p;ctx=F.ctx;rp=self.run['parameters'];cmi_=F.cmi_;wcy=F.wcy;ely=F.ely
        c_y=self.yin.c;b_=getattr(self,'_ss_gz',None) or [];w_=[g_ for t_,g_ in b_ if t_>=time.monotonic()-float(c_y.get('weave_unhold_s') or 1.5)];flat_y=bool(ctx.flat_wc())
        nhw=bool(flat_y and c_y.get('weave_unhold_dps') is not None and len(w_)>=5 and max(w_)>=float(c_y['weave_unhold_dps']) and min(w_)<=-float(c_y['weave_unhold_dps']))
        nhm=bool(flat_y and c_y.get('map_done_unhold_margin_deg') is not None and cmi_.get('released')=='ANGLE_DONE' and (cmi_.get('turned_deg') or 0)>=(cmi_.get('expected_deg') or 1e9)-float(c_y['map_done_unhold_margin_deg']))   # weave-mode-r1
        sty,yii=self.yin.step(time.monotonic(),F.gzy,proposal['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),ely,speed=None if wcy is None else wcy*M_PER_COUNT,bend=curve_ff.path_bend_deg(self.latest.get('path'),8) if ely else None,pitch=F.pdy,far=(proposal.get('turn_preview') or {}).get('far_projected_error'),near=proposal.get('image_error'),floor=F.cmf,in_curve=F.inc,map_curve=F.mpc,lock=F.lky,lock_cfg=self.cmap.c if F.lky is not None else None,no_hold=nhw or nhm)   # yaw-inner-r3: path bend for the curve hold   # yaw-inner-r2: counts/s -> m/s (0.0222 m per 100 counts)
        F.yii=yii
        if yii is not None:proposal['yaw_inner']=yii;proposal['steer_us']=sty
    def _L_xp_term_cap(self,F):   # crossing-pass-r6: inside the crossing window the inner-loop term is limited to +-term_cap_us (0f1e: +114..120 on top of the camera)
        proposal=F.p;yii=F.yii
        try:
            xcy=getattr(self,'_xp_c',{}) or {}
            if yii is not None and xcy.get('term_cap_us') is not None and F.ctx.xp_window(proposal) and isinstance(yii.get('term_us'),(int,float)) and isinstance(yii.get('pid_steer_us'),(int,float)) and abs(float(yii['term_us']))>float(xcy['term_cap_us']):
                tcx=math.copysign(float(xcy['term_cap_us']),float(yii['term_us']));rpx=self.run['parameters']
                proposal['steer_us']=max(float(rpx['min_us']),min(float(rpx['max_us']),float(yii['pid_steer_us'])+tcx));proposal['crossing_pass']['term_capped_us']=round(tcx,1)
        except Exception:pass
    def _L_weave_flag(self,F):   # weave-slow-r1: RELEASED is a straight when the 2 s mean is small (b1: straight-slew step removed, ssg kept)
        try:F.ssg=bool(F.ely and F.ctx.flat_wc() and F.cmi_.get('reason')!='FLOOR' and not F.trn)
        except Exception:pass
    def _lap_pos(self,proposal):   # lap-pos-r1 (map-20261114, log only): camera-free lap position (lap_pos.py) from this frame's inputs; changes no command, reads no IMU / clock / MCU status
        try:
            tp=getattr(self,'_lpos',None)
            if tp is None or getattr(self,'_lpos_run',None)!=self.run.get('id'):
                import lap_pos
                self._lpos_run=self.run.get('id')
                try:cfg=lap_pos.load_config()
                except Exception:cfg={}
                tp=self._lpos=lap_pos.LapPos(cfg) if isinstance(cfg,dict) and cfg.get('enabled') else False
            if tp is False:return
            i=proposal.get('inputs') or {};b=proposal.get('imu_boost') or {}
            if i.get('t_send') is None:return
            proposal['lap_pos']=tp.update(i.get('t_send'),i.get('gz'),b.get('pitch_rel_deg'),i.get('enc_b'))
            self._lpos_cmd=float(self.cmap.dist)   # curve_map distance at this update (xp-anchor-r1 converts lap_pos odometer -> curve_map metres)
        except Exception as e:proposal['lap_pos']=dict(revision='lap-pos-r1',reason='ERROR',error=type(e).__name__)
    def _L_gz_up(self,F):   # b1-dead-layers-r1: the raw gz gyro-damp logged, for upper_path (same read, same place, same rounding)
        F.gz_up=None
        if getattr(self,'heading',None) is not None:
            try:
                gzd=self.heading._gz(time.monotonic())
                if gzd is not None and math.isfinite(gzd):F.gz_up=round(gzd,2)
            except Exception:F.gz_up=None
    def _L_curve_ff(self,F):   # curve-ff-r1 (curve_ff.py, user 2026-11-06 c68f9a58): floor towards max_us when the camera error is large, straights untouched
        proposal=F.p;ctx=F.ctx
        try:
            rp=self.run['parameters']
            elc=(ctx.proposal_only(proposal) and ctx.flat_wc() and ctx.normal_rest(proposal)
                 and not F.downhill and not F.down_tail and not ctx.xp_any(proposal) and not curve2_late(proposal))   # crossing-pass-r7 (981f 132.0 s): no curve floor inside the crossing window / fade; curve-ff-late-r1
            try:gzc=self.heading._gz(time.monotonic()) if getattr(self,'heading',None) is not None else None   # curve-ff-r2: yaw rate for the curve gate
            except Exception:gzc=None
            cvc=None   # curve-ff-bycurve-r1: lap_pos segment for per-curve floor values; only looked up when curve_ff.json has by_curve
            if self.cff.c.get('by_curve'):
                try:
                    tpc=getattr(self,'_lpos',None)
                    if tpc is not None and getattr(self,'_lpos_run',None)==self.run['id'] and not tpc.lost:cvc=tpc.segment()
                except Exception:cvc=None
            stc,cfi=self.cff.step(proposal['steer_us'],proposal.get('image_error'),float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),elc,now=time.monotonic(),gz=gzc,path=self.latest.get('path'),curve=cvc)   # curve-ff-r2: gate on path bend / yaw
            if cfi is not None and cfi.get('applied') and curve3_cap(proposal) is not None and float(stc)>curve3_cap(proposal):   # curve-ff-c3cap-r1
                cfi=dict(cfi,c3_cap_us=curve3_cap(proposal),floor_before_cap_us=round(float(stc),1));stc=max(float(proposal['steer_us']),curve3_cap(proposal))
            if cfi is not None:proposal['curve_ff']=cfi;proposal['steer_us']=stc
        except Exception as e:proposal['curve_ff']=dict(revision=curve_ff.REVISION,reason='ERROR',error=type(e).__name__)
    def _lff_new(self):   # lap-ff-r1: built once per run in start(); lap_ff.json absent / enabled false / broken = None (layer off)
        try:
            if lap_ff is None:return None
            c=lap_ff.load_config()
            return lap_ff.LapFF(c) if isinstance(c,dict) and c.get('enabled') else None
        except Exception:return None
    def _L_lap_ff(self,F):   # lap-ff-r1 (lap-ff-20261115): steering floor into the right curves 1/2/3 after curve_ff (the last steering floor before the speed group).
        proposal=F.p;ctx=F.ctx   # inputs: lap_pos state of its last update (previous valid frame), far / near error, wheel speed (_cm_speed), pitch; no clock / IMU read
        try:
            tp=getattr(self,'_lpos',None);lp=None;yw=None
            if tp:
                an=tp.anchor;yw=tp.yaw
                lp=dict(lap=tp.lap,anchor=None if an is None else an[0],anchor_d=None if an is None else an[1],since_anchor_m=None if an is None else tp.D-an[1],lost=tp.lost,odo_m=tp.D)
            rp=self.run['parameters']
            el=bool(ctx.proposal_only(proposal) and ctx.flat_wc() and ctx.normal_rest(proposal) and not F.downhill and not F.down_tail and not ctx.xp_any(proposal))   # curve_ff's eligibility without curve2_late
            if lp is not None and getattr(self.lff,'band_on',False):self._lff_imu(lp)   # c2-band-r1: IMU heading / yaw rate (band configured only)
            st,inf=self.lff.step(lp,(proposal.get('turn_preview') or {}).get('far_projected_error'),proposal.get('image_error'),True,getattr(self,'_cm_speed',None),ctx.pitch,yw,
                                 proposal['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),el)
            if inf is not None:
                proposal['lap_ff']=inf
                if st!=float(proposal['steer_us']):proposal['steer_us']=st
        except Exception as e:proposal['lap_ff']=dict(revision=getattr(lap_ff,'REVISION','lap-ff-r1'),reason='ERROR',error=type(e).__name__)
    def _lff_imu(self,lp):   # c2-band-r1: one IMU heading + yaw-rate read (same clock) into the lap_pos dict
        try:
            tn=time.monotonic();lp['yaw_imu']=float(self.heading._yaw(tn));g_=self.heading._gz(tn)
            if g_ is not None:lp['gz_imu']=float(g_)
        except Exception:pass
    def _lff_lost(self,F,p):   # c2-band-r1: lost frame inside the curve-2 band -> heading-indexed profile instead of the held / heading command
        try:
            tp=getattr(self,'_lpos',None)
            if not tp or getattr(self.lff,'act',None) is None:return p
            an=tp.anchor;lpd=dict(lap=tp.lap,anchor=None if an is None else an[0],anchor_d=None if an is None else an[1],since_anchor_m=None if an is None else tp.D-an[1],lost=tp.lost,odo_m=tp.D)
            self._lff_imu(lpd);rp=self.run['parameters']
            r=self.lff.lost(lpd,getattr(self,'_cm_speed',None),p['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),F.now)
            if r is not None:b=p['steer_us'];p['steer_us']=r[0];p['lap_ff_lost']=r[1];F.owner=segment_ctx.owner_after(F.owner,'lap_ff_lost',b,p)
        except Exception as e:p['lap_ff_lost']=dict(revision='c2-band-r1',reason='ERROR',error=type(e).__name__)
        return p
    def _b2_lost(self,F,p):   # blind-v2 (blind2-20261117): invalid frame inside an active v2 crossing window -> same heading / lateral law (no camera)
        try:
            B2=self.arc2['blind_c2']
            if B2.w is None or B2.psi is None or B2.w.get('done'):return p
            lp,yw=self._arc2_lp()
            if lp is None:return p
            gz=None
            try:gz=float(self.heading._gz(time.monotonic()))
            except Exception:gz=None
            rp=self.run['parameters'];r=B2.lost(lp,lp.get('yaw_imu'),gz,p['steer_us'],float(rp['min_us']),float(rp['max_us']),F.now)
            if r is not None:b=p['steer_us'];p['steer_us']=r[0];p['arc2_blind_c2']=r[1];F.owner=segment_ctx.owner_after(F.owner,'arc2_blind_c2',b,p)
        except Exception as e:p['arc2_blind_c2']=dict(revision='arc2-blind-v2',reason='ERROR',error=type(e).__name__)
        return p
    def _lff4_new(self):   # lap-ff-c4-r1: built once per run; lap_ff.json absent / disabled / no enabled 'c4' -> None (layer off)
        try:
            if lap_ff is None or not hasattr(lap_ff,'load_c4'):return None
            c4=lap_ff.load_c4(lap_ff.load_config())
            return lap_ff.LapFF4(c4) if c4 else None
        except Exception:return None
    def _L_lap_ff4(self,F):   # lap-ff-c4-r1: curve-4 steering floor from the slope landing, AFTER bottom_owner (floor <= cap_us < bottom_owner cap: never cut, never fights it)
        proposal=F.p;ctx=F.ctx
        try:
            tp=getattr(self,'_lpos',None);lp=None;yw=None
            if tp:
                an=tp.anchor;yw=tp.yaw
                lp=dict(lap=tp.lap,anchor=None if an is None else an[0],anchor_d=None if an is None else an[1],since_anchor_m=None if an is None else tp.D-an[1],lost=tp.lost,odo_m=tp.D,t=tp.t,yaw_at=tp._yaw_at)   # + update time / yaw history (lap-ff-c4-r1)
                try:lp['yaw_imu']=float(self.heading._yaw(time.monotonic()));lp['yaw_land_imu']=getattr(self,'_bm_yaw0',None)   # lap-ff-c4-r1: 50 Hz IMU heading now + landing heading (one extra IMU read, only while the layer is on)
                except Exception:pass
                if getattr(self.lff4,'band_on',False):self._lff4_gz(lp)   # c4-band-r1: yaw rate (band configured only)
            rp=self.run['parameters']
            el=bool(ctx.proposal_only(proposal) and (F.down_level is not None or not F.downhill) and not ctx.xp_any(proposal) and F.usv is None)   # valid normal frame, landed (or not on a slope), no crossing window
            st,inf=self.lff4.step(lp,proposal.get('image_error'),True,getattr(self,'_cm_speed',None),yw,proposal['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),el,now=F.now)
            if inf is not None:
                proposal['lap_ff4']=inf
                if st!=float(proposal['steer_us']):b=proposal['steer_us'];proposal['steer_us']=st;F.owner=segment_ctx.owner_after(F.owner,'lap_ff4',b,proposal)
        except Exception as e:proposal['lap_ff4']=dict(revision='lap-ff-c4-r1',reason='ERROR',error=type(e).__name__)
    def _lff4_gz(self,lp):   # c4-band-r1: one IMU yaw-rate read into the lap_pos dict
        try:
            g_=self.heading._gz(time.monotonic())
            if g_ is not None:lp['gz_imu']=float(g_)
        except Exception:pass
    def _lff4_lost(self,F,p):   # c4-band-r1: lost frame inside the curve-4 band -> heading-indexed profile + gyro term instead of the held / heading command
        try:
            tp=getattr(self,'_lpos',None)
            if not tp or getattr(self.lff4,'act',None) is None:return p
            an=tp.anchor;lpd=dict(lap=tp.lap,anchor=None if an is None else an[0],anchor_d=None if an is None else an[1],since_anchor_m=None if an is None else tp.D-an[1],lost=tp.lost,odo_m=tp.D)
            try:lpd['yaw_imu']=float(self.heading._yaw(time.monotonic()))
            except Exception:pass
            self._lff4_gz(lpd);rp=self.run['parameters']
            r=self.lff4.lost(lpd,getattr(self,'_cm_speed',None),p['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),F.now)
            if r is not None:b=p['steer_us'];p['steer_us']=r[0];p['lap_ff4_lost']=r[1];F.owner=segment_ctx.owner_after(F.owner,'lap_ff4_lost',b,p)
        except Exception as e:p['lap_ff4_lost']=dict(revision='c4-band-r1',reason='ERROR',error=type(e).__name__)
        return p
    def _arc2_new(self):   # arc2-r1 (arc2-20261116): arc2.json absent / enabled false -> None (no layer runs); built once per run
        try:
            import arc2
            raw=arc2.load_config()
            if raw is None:return None
            c3=lap_ff.load_block(raw.get('c3'),lap_ff.load_config()) if lap_ff is not None and hasattr(lap_ff,'load_block') else None
            o=dict(land=arc2.LandArc.from_config(raw),blind=arc2.Blind.from_config(raw),c3=lap_ff.LapFF4(c3,'c3') if c3 else None)
            o['pre']=arc2.SlopePre.from_config(raw) if hasattr(arc2,'SlopePre') else None   # slope-arc-r1 A: key 'pre' absent / disabled -> None (stage not run)
            o['blind_c2']=arc2.Blind.from_config(raw,'blind_c2') if isinstance(raw.get('blind_c2'),dict) else None   # c2-blind-r1: key absent / disabled -> None
            return o if any(o.values()) else None
        except Exception:return None
    def _a2_lost(self,F,lp):   # land-smooth-20261120 (arc2-land-lost-r1): approach floor on lost frames (lap_pos state of its last update, no IMU / clock read)
        try:
            tpl,_=self._arc2_lp(False);rp=self.run['parameters']
            r=self.arc2['land'].ext.lost(tpl,lp['steer_us'],float(rp['min_us']),float(rp['max_us']),F.now)
            if r is not None:b=lp['steer_us'];lp=dict(lp,steer_us=r[0],arc2_land_lost=r[1]);F.owner=segment_ctx.owner_after(F.owner,'arc2_land_lost',b,lp)
        except Exception as e:lp=dict(lp,arc2_land_lost=dict(revision='arc2-land-lost-r1',reason='ERROR',error=type(e).__name__))
        return lp
    def _arc2_lp(self,yaw_imu=True):   # lap_pos state of its last update + optional IMU heading (one IMU read)
        tp=getattr(self,'_lpos',None)
        if not tp:return None,None
        an=tp.anchor
        lp=dict(lap=tp.lap,anchor=None if an is None else an[0],anchor_d=None if an is None else an[1],since_anchor_m=None if an is None else tp.D-an[1],lost=tp.lost,odo_m=tp.D,t=tp.t,yaw_at=tp._yaw_at)
        if yaw_imu:
            try:lp['yaw_imu']=float(self.heading._yaw(time.monotonic()))
            except Exception:pass
        return lp,tp.yaw
    def _L_arc2_flat(self,F):   # arc2-r1 c3 floor (C) and V-crossing blind straight (D): after lap_ff, same eligibility as lap_ff
        proposal=F.p;ctx=F.ctx;A=self.arc2
        try:
            lp,yw=self._arc2_lp()
            rp=self.run['parameters']
            el=bool(ctx.proposal_only(proposal) and ctx.flat_wc() and ctx.normal_rest(proposal) and not F.downhill and not F.down_tail and not ctx.xp_any(proposal))
            if A['c3'] is not None and lp is not None:
                lp3=dict(lp,pitch=ctx.pitch)
                st,inf=A['c3'].step(lp3,proposal.get('image_error'),True,getattr(self,'_cm_speed',None),yw,proposal['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),el,now=F.now)
                if inf is not None:
                    proposal['arc2_c3']=inf
                    if st!=float(proposal['steer_us']):b=proposal['steer_us'];proposal['steer_us']=st;F.owner=segment_ctx.owner_after(F.owner,'arc2_c3',b,proposal)
            if A['blind'] is not None:
                gz=None
                try:gz=float(self.heading._gz(time.monotonic()))
                except Exception:gz=None
                st,inf=A['blind'].step(lp,proposal.get('image_error'),True,None if lp is None else lp.get('yaw_imu'),gz,proposal['steer_us'],float(rp['min_us']),float(rp['max_us']),el,now=F.now)
                if inf is not None:
                    proposal['arc2_blind']=inf
                    if st!=float(proposal['steer_us']):b=proposal['steer_us'];proposal['steer_us']=st;F.owner=segment_ctx.owner_after(F.owner,'arc2_blind',b,proposal)
            B2=A.get('blind_c2')
            if B2 is not None:   # c2-blind-r1: circle crossing; own eligibility (the crossing_pass window does not exclude it: it supersedes it on valid frames)
                if A['blind'] is None:
                    try:gz=float(self.heading._gz(time.monotonic()))
                    except Exception:gz=None
                el2=bool(ctx.proposal_only(proposal) and ctx.flat_wc() and ctx.normal_rest(proposal) and not F.downhill and not F.down_tail)
                st,inf=B2.step(lp,proposal.get('image_error'),True,None if lp is None else lp.get('yaw_imu'),gz,proposal['steer_us'],float(rp['min_us']),float(rp['max_us']),el2,now=F.now,vis=((self.latest or {}).get('path_diagnostic') or {}).get('line_estimate'))   # blind-v2: vis (line_estimate) used only with law v2
                if inf is not None:
                    proposal['arc2_blind_c2']=inf
                    if st!=float(proposal['steer_us']):b=proposal['steer_us'];proposal['steer_us']=st;F.owner=segment_ctx.owner_after(F.owner,'arc2_blind_c2',b,proposal)
        except Exception as e:proposal['arc2_flat']=dict(revision='arc2-r1',reason='ERROR',error=type(e).__name__)
    def _L_arc2_land(self,F):   # arc2-r1 landing profile owner (A): after lap_ff4, before bottom_guide; valid normal frames after the landing
        proposal=F.p;ctx=F.ctx;A=self.arc2
        try:
            lp,yw=self._arc2_lp()
            rp=self.run['parameters']
            el=bool(ctx.proposal_only(proposal) and (F.down_level is not None or not F.downhill) and not ctx.xp_any(proposal) and F.usv is None)
            st,inf=A['land'].step(lp,proposal.get('image_error'),getattr(self,'_cm_speed',None),None if lp is None else lp.get('yaw_imu'),getattr(self,'_bm_yaw0',None),
                                  proposal['steer_us'],float(rp['min_us']),float(rp['max_us']),el,now=F.now)
            if inf is not None:
                proposal['arc2_land']=inf
                if st!=float(proposal['steer_us']):b=proposal['steer_us'];proposal['steer_us']=st;F.owner=segment_ctx.owner_after(F.owner,'arc2_land',b,proposal)
        except Exception as e:proposal['arc2_land']=dict(revision='arc2-r1',reason='ERROR',error=type(e).__name__)
    def _L_arc2_pre(self,F):   # slope-arc-r1 A (slope-arc-20261116): slope pre-rotation bias before yaw_inner; lap_pos state of its last update, no IMU / clock read
        proposal=F.p;A=self.arc2
        try:
            tp=getattr(self,'_lpos',None)
            if not tp:return
            rp=self.run['parameters'];an=tp.anchor
            el=bool(F.ctx.proposal_only(proposal) and F.down_tail)
            st,inf=A['pre'].step(None if an is None else an[0],None if an is None else an[1],tp.D,tp.lost,tp.yaw,tp.hist,proposal['steer_us'],float(rp['min_us']),float(rp['max_us']),el,F.down_level is not None)
            if inf is not None:
                proposal['arc2_pre']=inf
                if st!=float(proposal['steer_us']):b=proposal['steer_us'];proposal['steer_us']=st;F.owner=segment_ctx.owner_after(F.owner,'arc2_pre',b,proposal)
        except Exception as e:proposal['arc2_pre']=dict(revision='arc2-pre-r1',reason='ERROR',error=type(e).__name__)
    # speed-adapt-r1 group: faster while the normal path is stable, base PWM otherwise. One try around the group (as before).
    # ---- land4-r1 (field-20261122/landing4, offline candidate): slope + landing steered on the absolute heading of the upper straight (land4.py / land4.json).
    # self.land4 None (json absent / disabled) -> none of these is called: no IMU / MCU / clock read is added.
    def _sb_new(self):   # start-bias-r1 (field-20261122/next_restart)
        try:
            import start_bias
            return start_bias.load_config()
        except Exception:return None
    def _L_start_bias(self,F):   # last valid-frame layer: a short steering offset after the start hold (start_bias.py)
        p=F.p
        try:
            import start_bias
            if not getattr(self,'start_centred',True) or p.get('reason')!='PROPOSAL_ONLY':return
            el=time.monotonic()-getattr(self,'start_t',time.monotonic())-float(getattr(self,'start_hold_s',0.) or 0.);rp=self.run['parameters']
            st,inf,done=start_bias.apply(self.sbias,el,p['steer_us'],float(rp['min_us']),float(rp['max_us']))
            if inf is not None:p['start_bias']=inf;p['steer_us']=st
            if done:self.sbias=None
        except Exception as e:p['start_bias']=dict(revision='start-bias-r1',reason='ERROR',error=type(e).__name__);self.sbias=None
    def _l4_new(self):
        try:
            import land4
            return land4.Land4.from_file()
        except Exception:return None
    def _l4_step(self,F,p,valid):   # one frame (valid / WARMUP / lost): reads the IMU and the MCU status only on the upper straight, the slope and the landing
        L=self.land4;ctx=F.ctx;rp=self.run['parameters']
        up=bool(valid and ctx.hh_active(p));yaw=gz=enc=ms=None
        if L.wants_imu(up,ctx.down_tail):
            if getattr(self,'heading',None) is not None:
                tn=time.monotonic()
                try:yaw=float(self.heading._yaw(tn))
                except Exception:yaw=None
                try:gz=float(self.heading._gz(tn))
                except Exception:gz=None
            try:
                v_=(self.manual.state() or {}).get('vehicle') or {};b_=v_.get('enc_b') or {}
                if 'forward' in b_ and 'reverse' in b_:enc=int(b_['forward'])-int(b_['reverse']);ms=v_.get('ms')
            except Exception:enc=ms=None
        ls=None
        try:ls=(self.hold.last_sent(F.now) or {}).get('steer_us') if getattr(self,'hold',None) is not None else None
        except Exception:ls=None
        return L.step(F.now,bool(valid and p.get('reason')=='PROPOSAL_ONLY'),p.get('image_error'),yaw,gz,enc,ms,getattr(self,'_cm_speed',None),up,bool(ctx.down_tail),ctx.down_level,
                      p['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),ls)
    def _L_land4(self,F):   # valid / WARMUP frames: last steering writer before the upper-single override (the chain value is its input)
        p=F.p
        try:
            st,inf=self._l4_step(F,p,True)
            if inf is not None:
                p['land4']=inf
                if st!=float(p['steer_us']):p['steer_us']=st
        except Exception as e:p['land4']=dict(revision='land4-r1',reason='ERROR',error=type(e).__name__)
    def _l4_lost(self,F,lp):   # lost frames: same law without the camera term (after every other lost-frame hook)
        try:
            st,inf=self._l4_step(F,lp,False)
            if inf is not None:
                b=lp['steer_us'];lp=dict(lp,land4=inf)
                if st!=float(b):lp['steer_us']=st;F.owner=segment_ctx.owner_after(F.owner,'land4_lost',b,lp)
        except Exception as e:lp=dict(lp,land4=dict(revision='land4-r1',reason='ERROR',error=type(e).__name__))
        return lp
    def _ctrim_new(self):   # curve-trim-r1: built once per run in start(); curve_trim.json absent / enabled false / broken = None (layer off)
        try:return curve_trim.build() if curve_trim is not None else None
        except Exception:return None
    def _L_curve_trim(self,F):   # curve-trim-r1 (curves-20261120): outward trim in the curve-1 / curve-3 lap_pos windows; after lap_ff / arc2_flat, before arc_lat / the speed group
        proposal=F.p;ctx=F.ctx
        try:
            tp=getattr(self,'_lpos',None);lp=None
            if tp:
                an=tp.anchor;lp=dict(lap=tp.lap,anchor=None if an is None else an[0],anchor_d=None if an is None else an[1],since_anchor_m=None if an is None else tp.D-an[1],lost=tp.lost)
            rp=self.run['parameters']
            bl=any((proposal.get(k_) or {}).get('state') in ('BLEND','HOLD','EXT','OWN','HAND') for k_ in ('arc2_blind','arc2_blind_c2'))   # crossing windows own the command
            el=bool(ctx.proposal_only(proposal) and ctx.flat_wc() and ctx.normal_rest(proposal) and not F.downhill and not F.down_tail and not ctx.xp_any(proposal) and not bl)
            st,inf=self.ctrim.step(lp,proposal.get('image_error'),proposal['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),el)
            if inf is not None:
                proposal['curve_trim']=inf
                if st!=float(proposal['steer_us']):b=proposal['steer_us'];proposal['steer_us']=st;F.owner=segment_ctx.owner_after(F.owner,'curve_trim',b,proposal)
        except Exception as e:proposal['curve_trim']=dict(revision='curve-trim-r1',reason='ERROR',error=type(e).__name__)
    def _alat_new(self):   # arc-lat-r1: built once per run in start(); arc_lat.json absent / enabled false / broken = None (layer off)
        try:return arc_lat.build() if arc_lat is not None else None
        except Exception:return None
    def _L_arc_lat(self,F):   # arc-lat-r1 (arc-lat-20261119): curve zones steer on the car's measured lateral position (path_diagnostic.arc_fit pts + odometric map); after lap_ff / arc2_flat, before the speed group
        proposal=F.p;ctx=F.ctx
        try:
            tp=getattr(self,'_lpos',None);lp=dict(s_m=tp.s(),lost=tp.lost) if tp else None
            try:d_=json.loads(Path(self.boost.c['latest_path']).read_text(encoding='utf8'));imu=(_pclk.imu_t(float(d_['t'])),float(d_['yaw_deg']),float(d_['gyro_dps'][2]))   # one IMU latest-file read (t, heading, yaw rate)
            except Exception:imu=None
            rp=self.run['parameters'];lt=self.latest or {}
            bl=any((proposal.get(k_) or {}).get('state') in ('BLEND','HOLD','EXT','OWN','HAND') for k_ in ('arc2_blind','arc2_blind_c2'))   # crossing windows own the command
            el=dict(flat=bool(ctx.proposal_only(proposal) and ctx.flat_wc() and ctx.normal_rest(proposal) and not F.downhill and not F.down_tail and not ctx.xp_any(proposal) and not bl),
                    land=bool(ctx.proposal_only(proposal) and (F.down_level is not None or not F.downhill) and not ctx.xp_any(proposal) and F.usv is None and not bl))
            st,inf=self.alat.step(time.monotonic(),imu,getattr(self,'_cm_speed',None),lt.get('capture_at'),(lt.get('path_diagnostic') or {}).get('arc_fit'),lp,el,ctx.pitch,
                                  proposal['steer_us'],float(rp['center_us']),float(rp['min_us']),float(rp['max_us']),band_prof=((proposal.get('lap_ff') or {}).get('band') or {}).get('prof_us'))
            if inf is not None:
                proposal['arc_lat']=inf
                if st!=float(proposal['steer_us']):b=proposal['steer_us'];proposal['steer_us']=st;F.owner=segment_ctx.owner_after(F.owner,'arc_lat',b,proposal)
        except Exception as e:proposal['arc_lat']=dict(revision='arc-lat-r1',reason='ERROR',error=type(e).__name__)
    def _L_speed_prelude(self,F):
        speed_policy.adapt_inputs(self,F)   # speed-policy-r1 step 2: speed_adapt inputs
    def _L_upper_path(self,F):   # upper-path-r1: recorded upper-section heading profile by odometry (upper_path.json, off by default)
        proposal=F.p;ctx=F.ctx
        try:
            if getattr(self,'upath',None) is None:self.upath=upper_path.UpperPath()
            rpu=self.run['parameters'];su,uinfo=self.upath.step(time.monotonic(),ctx.route,(self.manual.state() or {}).get('vehicle'),F.gz_up,proposal['steer_us'],float(rpu['center_us']),float(rpu['min_us']),float(rpu['max_us']),err=proposal.get('error'))
            if uinfo is not None:proposal['upper_path']=uinfo
            # upper-path-skip-r1 (user 2026-11-10 ac95 61.6-63 s): once the heading hold is active after the turn, its command is kept
            if su is not None and ctx.hh_active(proposal):
                if uinfo is not None:uinfo['skipped']='HEADING_HOLD';uinfo['would_steer_us']=round(float(su),1)
                su=None
            if su is not None:proposal['steer_us']=su
        except Exception as e:proposal['upper_path']=dict(revision='upper-path-r1',reason='ERROR',error=type(e).__name__)
    def _L_speed_adapt(self,F):
        speed_policy.adapt(self,F)   # speed-policy-r1 step 3: speed_adapt.step
    def _L_speed_caps(self,F):
        speed_policy.caps(self,F)   # speed-policy-r1 step 4: upper-section cap -> brake zones -> turn-pwm cap
    def _L_downhill_dcap(self,F):   # downhill-dcap-r1 (live b9fa9cff 104.2-105.8 s): on the downhill the D term is limited to +-150 (steering rebuilt from P+I+D)
        proposal=F.p
        rp=self.run['parameters'];c0=float(rp['center_us']);dc=max(-150.,min(150.,float(proposal['d'])));u=float(proposal['p'])+float(proposal.get('i') or 0)+dc
        g=float(rp.get('right_gain',1.)) if u>=0 else float(rp.get('left_gain',1.))
        proposal['downhill_dcap']=dict(revision='downhill-dcap-r1',d=round(float(proposal['d']),1),d_cap=dc,pid_steer_us=round(proposal['steer_us'],1))
        proposal['steer_us']=max(float(rp['min_us']),min(float(rp['max_us']),c0+g*u));proposal['downhill_dcap']['steer_us']=round(proposal['steer_us'],1)   # b1-log: after value
    def _L_downhill_speed(self,F):   # downhill-speed-r1 (live 9fd69ba7): steering offset scaled by REF_SPEED / encoder speed
        proposal=F.p
        try:
            spd=self.dspeed.update(time.monotonic(),(self.manual.state() or {}).get('vehicle'));kd=self.dspeed.k(spd,right=float(proposal['steer_us'])>float(self.run['parameters']['center_us']))   # downhill-right-r1
            if spd is None:   # speed-guard-r1 fix C (2026-11-13): no speed yet on a descent's first frame -> no scaling, as before (round(None) raised TypeError)
                if kd<1.:proposal['downhill_speed']=dict(revision=DSPEED_REVISION,reason='NO_SPEED',k=round(kd,3))   # logged only where the TypeError was logged
                return
            if kd<1. and F.down_level is not None and F.dpr is not None and F.dpr>=BOTTOM_RIGHT_PITCH and float(proposal['steer_us'])>float(self.run['parameters']['center_us']):proposal['downhill_speed']=dict(revision=DSPEED_REVISION,speed=round(spd),k=round(kd,3),skipped='LEVEL',by=BOTTOM_REVISION)   # downhill-bottom-r1: no scaling of right commands on the level bottom
            elif kd<1.:
                c0=float(self.run['parameters']['center_us']);proposal['downhill_speed']=dict(revision=DSPEED_REVISION,speed=round(spd),k=round(kd,3),pid_steer_us=round(proposal['steer_us'],1))
                proposal['steer_us']=c0+(proposal['steer_us']-c0)*kd
        except Exception as e:proposal['downhill_speed']=dict(revision=DSPEED_REVISION,reason='ERROR',error=type(e).__name__)
    def _L_downhill_assist(self,F):   # downhill-r3: right-curve assist only for a clear right error
        proposal=F.p
        ra=float(self.run['parameters']['center_us'])+DOWN_ASSIST_US   # downhill-speed-r1: +300 -> +150
        if proposal['steer_us']<ra:proposal['downhill_assist']=dict(pid_steer_us=round(proposal['steer_us'],1),min_us=ra);proposal['steer_us']=min(ra,float(self.run['parameters']['max_us']))
    def _L_downhill_dlimit(self,F):   # downhill-dlimit-r1 (live 3fd37b76 109.1-109.4 s): D may slow a right correction but not turn it into a left one
        proposal=F.p
        c0=float(self.run['parameters']['center_us']);pi=float(proposal['p'])+float(proposal['i'])
        if pi>0 and proposal['steer_us']<c0:
            proposal['downhill_dlimit']=dict(revision='downhill-dlimit-r1',pid_steer_us=round(proposal['steer_us'],1),p_plus_i=round(pi,1));proposal['steer_us']=c0
    def _L_downhill_gz(self,F):
        self._down_gz(F.p)   # downhill-speed-r2..r4: yaw-rate limit on the downhill
    def _L_down_straight(self,F):
        self._down_straight(F.p,F.result,True)   # down-straight-r1
    def _L_xp_seen(self,F):
        self._xp_seen(F.p)   # crossing-pass-r9: valid frame -> reset the gap hold, remember the in-window state
    def _L_downhill_bottom(self,F):   # downhill-bottom-r1 (downhill_bottom.py): on the level bottom a one-frame left jump keeps the median of the last 0.5 s of sent commands
        proposal=F.p;now=F.now
        try:
            try:yb=self.heading._yaw(time.monotonic()) if getattr(self,'heading',None) is not None else None
            except Exception:yb=None
            bj=self.bottom.step(now,F.down_tail,F.dpr,proposal.get('image_error'),yb,proposal.get('desired_us'))   # r2: desired for the right-edge floor
            if bj is not None and self._bo_on(F):proposal['downhill_bottom']=dict(bj,applied=False,why='BOTTOM_OWNER');bj=None   # bottom-owner-r2: no JUMP / HOLD while the bottom owner is active
            hm=self.hold.median_recent(now,.5) if bj is not None and getattr(self,'hold',None) is not None else None
            if bj is not None and hm is not None:
                rp=self.run['parameters'];c0=float(rp['center_us'])
                hv=float(hm);fl=bj.get('floor_us')   # downhill-bottom-r2: jump from the right edge holds at least the PID desired before it
                if fl is not None and float(fl)>hv:hv=float(fl)
                proposal['downhill_bottom']=dict(bj,pid_steer_us=round(proposal['steer_us'],1),hold_us=round(hv,1),median_us=round(float(hm),1))
                proposal['steer_us']=max(c0-150.,min(float(rp['max_us']),hv))
                if F.downhill:self._down_gz(proposal)
            elif bj is not None:proposal['downhill_bottom']=dict(bj,applied=False,why='NO_SENT_HISTORY')
        except Exception as e:proposal['downhill_bottom']=dict(revision=BOTTOM_REVISION,reason='ERROR',error=type(e).__name__)
    def _L_bottom_left_floor(self,F):   # bottom-left-r1 (bottom_left.py): at the level bottom the command is not further left than centre -LEFT_FLOOR_US
        proposal=F.p
        try:
            if bottom_left.in_window(F.down_tail,getattr(self,'down_steep',False),F.dpr):
                fs,fl=bottom_left.floor(proposal['steer_us'],self.run['parameters']['center_us'])
                if fl:proposal['bottom_left_floor']=dict(revision=bottom_left.REVISION,pid_steer_us=round(float(proposal['steer_us']),1),floor_us=fs);proposal['steer_us']=fs
        except Exception as e:proposal['bottom_left_floor']=dict(revision=bottom_left.REVISION,reason='ERROR',error=type(e).__name__)
    def _L_jump_guard(self,F):   # error-jump-guard-r1 (user 2026-11-05, 18b7fb7f): a one-frame jump of the error from the right edge across the road
        proposal=F.p;now=F.now
        try:
            je=proposal.get('image_error');jt=time.monotonic();jg=getattr(self,'jg',None)
            if jg is None:jg=self.jg=dict(prev=None,prev_t=None,since=None,sat_t=None,n=0)
            if je is not None and proposal.get('reason')=='PROPOSAL_ONLY':
                pe=jg['prev'];je=float(je)
                if pe is not None and pe>=JG_PREV and je<=pe-JG_JUMP and jg['prev_t'] is not None and jt-jg['prev_t']<=.6:
                    if jg['since'] is None:jg['since']=jt;jg['n']=0
                    jg['n']+=1
                    win=JG_LONG_S if (jg['sat_t'] is not None and jg['prev_t']-jg['sat_t']>=1.) else JG_SHORT_S
                    ls=(self.hold.last_sent(now) or {}).get('steer_us') if getattr(self,'hold',None) is not None else None
                    if jt-jg['since']<=win and ls is not None and float(ls)>float(self.run['parameters']['center_us']):   # right commands only
                        proposal['jump_guard']=dict(revision=JG_REVISION,prev_err=round(pe,3),err=round(je,3),held_us=round(float(ls),1),pid_steer_us=round(float(proposal['steer_us']),1),frames=jg['n'],window_s=win)
                        proposal['steer_us']=float(ls)
                    else:
                        proposal['jump_guard']=dict(revision=JG_REVISION,accepted=True,prev_err=round(pe,3),err=round(je,3),frames=jg['n'])
                        jg['prev']=je;jg['prev_t']=jt;jg['since']=None;jg['sat_t']=None
                else:
                    jg['since']=None;jg['prev']=je;jg['prev_t']=jt
                    jg['sat_t']=(jg['sat_t'] if jg['sat_t'] is not None else jt) if je>=.95 else None
        except Exception as e:proposal['jump_guard']=dict(revision=JG_REVISION,reason='ERROR',error=type(e).__name__)
    def _L_dh_last_err(self,F):
        if not (F.p.get('jump_guard') or {}).get('held_us'):self.dh_last_err=F.p.get('image_error')   # downhill-hold-r1: last valid error (not a guarded jump)
    def _L_wheel(self,F):
        F.wheel={}   # b1-dead-layers-r1: diff-assist removed (diff_assist.json enabled false); both wheels get proposal pwm
    def _L_bottom_map_floor(self,F):   # bottom-map-r1: while the bottom right curve is unfinished, no left command beyond left_floor_us (5262 81.06 s)
        proposal=F.p
        try:
            if F.down_tail and not F.downhill and getattr(self,'_bm_yaw0',None) is not None:
                bmc=self._bm_cfg();bmt=self._bm_turned()
                if bmc.get('enabled') and bmc.get('left_floor_us') is not None and bmt is not None and float(bmc.get('min_turned_deg',15.))<=bmt<float(bmc['turn_deg']):
                    flr=float(self.run['parameters']['center_us'])-float(bmc['left_floor_us'])
                    if float(proposal['steer_us'])<flr:proposal['bottom_map']=dict(revision='bottom-map-r1',reason='LEFT_FLOOR',turned_deg=round(bmt,1),from_us=round(float(proposal['steer_us']),1));proposal['steer_us']=flr
        except Exception as e:proposal['bottom_map']=dict(revision='bottom-map-r1',reason='ERROR',error=type(e).__name__)
    def _L_upper_single_final(self,F):   # upper-single-r1: no later writer (jump guard, play comp, ...) changes its command
        proposal=F.p;usv=F.usv
        try:
            if float(proposal['steer_us'])!=float(usv):ov_=round(float(proposal['steer_us']),1);proposal['steer_us']=usv;(proposal.get('upper_single') or {})['overrode_us']=ov_
        except Exception:pass

    # ---- bottom-owner-r2 (integration v2, 2026-11-13): one owner from the landing (down_level_t0) to curve 4's FLOOR (bottom_owner.py)
    def _bo_on(self,F,valid=True):   # scope of this moment; valid frames pass this frame's curve_map (FLOOR of seg after_down idx 0 ends it)
        bo=getattr(self,'bown',None)
        if bo is None or not bo.on:return False
        try:
            cm=(F.p.get('curve_map') or {}) if valid else {}
            return bo.scope(F.now,getattr(self,'down_level_t0',None),cm.get('reason')=='FLOOR' and cm.get('seg')=='after_down' and cm.get('idx')==0)
        except Exception:return False
    def _bo_gz(self,F):   # r2: one gz read per lost frame (cached on F by F.now); None = IMU read failed
        c=getattr(F,'_bo_gzc',None)
        if c is not None and c[0]==F.now:return c[1]
        try:g=float(self.heading._gz(time.monotonic()))
        except Exception:g=None
        F._bo_gzc=(F.now,g);return g
    def _L_bottom_owner(self,F):   # valid frames: right cap (cap_us for limit_s, then ramp_s to the servo limit, re-armed after a lost stretch) + left guard (optional)
        proposal=F.p
        try:
            if self._bo_on(F):
                rp=self.run['parameters'];o,info=self.bown.limit(F.now,proposal['steer_us'],rp['center_us'],rp['min_us'],rp['max_us'],proposal.get('reason')=='WARMUP')   # r2: WARMUP never re-arms the cap timer
                proposal['bottom_owner']=info
                if info['cap'] is not None:proposal['steer_us']=o
        except Exception as e:proposal['bottom_owner']=dict(revision=bottom_owner.REVISION,reason='ERROR',error=type(e).__name__)
    def _bo_skip_map(self,F,p):   # lost frames: MAP_TURN is not applied while the rate hold will act (scope active, lost streak <= lost_max_s, gz readable)
        try:
            bo=getattr(self,'bown',None)
            if bo is None or not bo.on or F.downhill or not self._bo_on(F,False) or not bo.lost_due(F.now):return False
            if self._bo_gz(F) is None:return False   # r2: IMU read failed -> MAP_TURN as before (the rate hold will PASS on this frame)
            p['bottom_map']=dict(revision='bottom-map-r1',reason='SKIPPED',by=bottom_owner.REVISION);return True
        except Exception:return False
    def _bo_lost(self,F,lp):   # lost frames: gyro rate hold instead of the held command / MAP_TURN; None from the module -> lp unchanged
        try:
            if not self._bo_on(F,False):return lp
            rp=self.run['parameters'];gz=self._bo_gz(F);tn=time.monotonic()
            try:y0=getattr(self,'_bm_yaw0',None);tl=None if y0 is None else float(self.heading._yaw(tn))-float(y0)
            except Exception:tl=None
            r=self.bown.lost(F.now,gz,tl,rp['center_us'],rp['min_us'],rp['max_us'])
            if r is None:lp['bottom_owner']=dict(revision=bottom_owner.REVISION,mode='PASS',why='LOST_MAX' if gz is not None else 'NO_GZ');return lp
            b=lp.get('steer_us');lp=dict(lp,steer_us=r[0],bottom_owner=dict(r[1],replaced=lp.get('reason'),held_us=None if b is None else round(float(b),1)))
            F.owner=segment_ctx.owner_after(F.owner,'bottom_owner',b,lp)
        except Exception as e:lp['bottom_owner']=dict(revision=bottom_owner.REVISION,reason='ERROR',error=type(e).__name__)
        return lp

    # ---- bottom-guide-r1 (bottom-20261114): inside the bottom-owner scope; (a) distance-based heading-profile hold on lost frames, (b) curl guard
    def _bg_steep(self):   # first steep frame of a slope: reference yaw for the profile (None if the IMU read fails)
        try:y=float(self.heading._yaw(time.monotonic()))
        except Exception:y=None
        self.bguide.steep(y)
    def _bg_enc(self):   # rear encoder counts (forward - reverse) from the MCU status, None when unknown
        try:
            b_=((self.manual.state() or {}).get('vehicle') or {}).get('enc_b') or {}
            return int(b_['forward'])-int(b_['reverse']) if 'forward' in b_ and 'reverse' in b_ else None
        except Exception:return None
    def _bg_latch(self):   # scope frame: latch odometer / landing yaw on a new landing; -> distance since the landing (m) or None
        bo=getattr(self,'bown',None)
        if bo is None or not bo.on or not bo.active:return None
        enc=self._bg_enc()
        self.bguide.landing(getattr(self,'down_level_t0',None),enc,getattr(self,'_bm_yaw0',None))
        return self.bguide.dist(enc)
    def _bg_curl(self,result):   # (b) worker result inside the scope (as of the previous frame): reject a left-curling single left line
        try:
            bo=getattr(self,'bown',None)
            if not (self.run and self.run['state']=='running') or not isinstance(result,dict):return result
            if bo is None or not bo.on or not bo.active or getattr(self,'down_level_t0',None) is None:self.bguide.curl_n=0;return result
            le=(result.get('path_diagnostic') or {}).get('line_estimate') or {}
            L=((le.get('offscreen_center') or {}).get('current_boundaries') or {}).get('L') or []
            rej,info=self.bguide.curl_check(le.get('sources') or {},L,result.get('valid') is True)
            if info is None:return result
            return dict(result,valid=False,bottom_curl=info) if rej else dict(result,bottom_curl=info)
        except Exception as e:return dict(result,bottom_curl=dict(revision=bottom_guide.REVISION,reason='ERROR',error=type(e).__name__)) if isinstance(result,dict) else result
    def _bg_lost(self,F,lp):   # (a) lost frame: replace the r2 RATE_HOLD command by the profile hold (same gating: scope, lost_max_s, gz readable)
        try:
            s=self._bg_latch()
            if s is None and not (getattr(self,'bown',None) is not None and self.bown.active):return lp
            if not self.bguide.c.get('hold') or (lp.get('bottom_owner') or {}).get('mode')!='RATE_HOLD':return lp
            rp=self.run['parameters'];gz=self._bo_gz(F)
            try:yaw=float(self.heading._yaw(time.monotonic()))
            except Exception:yaw=None
            v=getattr(self,'_cm_speed',None)
            r=self.bguide.hold(s,yaw,gz,v,rp['center_us'],rp['min_us'],rp['max_us'])
            if r is None:lp['bottom_guide']=dict(revision=bottom_guide.REVISION,mode='PASS',s_m=None if s is None else round(s,3),why='S_MAX' if s is not None and yaw is not None else 'NO_INPUT');return lp
            b=lp.get('steer_us');lp=dict(lp,steer_us=r[0],bottom_guide=dict(r[1],replaced_us=None if b is None else round(float(b),1)))
            if self.bguide.c.get('hold_pwm') is not None:lp['pwm']=self.bguide.hold_pwm(lp.get('pwm'))
            F.owner=segment_ctx.owner_after(F.owner,'bottom_guide',b,lp)
        except Exception as e:lp['bottom_guide']=dict(revision=bottom_guide.REVISION,reason='ERROR',error=type(e).__name__)
        return lp
    def _bg_lag(self,s):   # bottom-guide-r2: heading lag behind the distance profile (deg) or None (one IMU yaw read)
        try:y=float(self.heading._yaw(time.monotonic()))
        except Exception:y=None
        return self.bguide.lag(s,y)
    def _bg_cap(self,F,s):   # bottom-guide-r2 (c): valid frame capped by bottom_owner -> min(PID, cap + k*(lag-deg0)) while the car lags the profile
        try:
            bo=F.p.get('bottom_owner') or {}
            if bo.get('mode')!='LIMIT' or not str(bo.get('cap') or '').startswith('RIGHT') or s is None:return
            r=self.bguide.cap_open(self._bg_lag(s),bo['from_us'],F.p['steer_us'],bo['right_cap_us'],s)
            if r is None:return
            b=F.p['steer_us'];F.p['steer_us']=r[0];F.p['bottom_guide_cap']=r[1];F.owner=segment_ctx.owner_after(F.owner,'bottom_guide',b,F.p)
        except Exception as e:F.p['bottom_guide_cap']=dict(revision=bottom_guide.R2_REVISION,reason='ERROR',error=type(e).__name__)
    def _bg_corr(self,result):   # bottom-guide-r2 (d): corridor kept by bottom-left-r1 although the baseline is clearly more right and the car lags -> baseline
        try:
            bo=getattr(self,'bown',None);b=result.get('bottom_left') or {}
            if bo is None or not bo.on or not bo.active or getattr(self,'down_level_t0',None) is None or b.get('action')!='KEEP':return result
            s=self._bg_latch();lag=self._bg_lag(s)
            if not self.bguide.corr_check(b.get('corridor_e'),b.get('baseline_e'),lag,s):return result
            bp=(((result.get('path_diagnostic') or {}).get('junction') or {}).get('baseline_path') or [])
            if len(bp)<2:return result
            return dict(result,path=bp,bottom_corr=dict(revision=bottom_guide.R2_REVISION,action='BASELINE',corridor_e=b.get('corridor_e'),baseline_e=b.get('baseline_e'),lag_deg=round(lag,1),s_m=round(s,3)))
        except Exception as e:return dict(result,bottom_corr=dict(revision=bottom_guide.R2_REVISION,reason='ERROR',error=type(e).__name__))
    def _bg_laglost(self,result):   # bottom-guide-r2 (e): valid frame, heading far behind the profile, camera asks for little right -> lost frame (profile hold)
        try:
            bo=getattr(self,'bown',None)
            if not (self.run and self.run['state']=='running') or not isinstance(result,dict) or result.get('valid') is not True:return result
            if bo is None or not bo.on or not bo.active or getattr(self,'down_level_t0',None) is None:return result
            s=self._bg_latch();lag=self._bg_lag(s)
            ofl=self.run['parameters'].get('visual_center_offset_px_640',0.)
            e=(offscreen_path_error if self.path_algorithm in ('camera-pair-v2', 'camera-pair-v2-junction') else path_error)(result.get('path',[]),self.row_ratio,visual_center_offset_px_640=ofl)
            if not self.bguide.lag_lost_check(e,lag,s):return result
            return dict(result,valid=False,bottom_laglost=dict(revision=bottom_guide.R2_REVISION,e=round(e,3),lag_deg=round(lag,1),s_m=round(s,3)))
        except Exception as e:return dict(result,bottom_laglost=dict(revision=bottom_guide.R2_REVISION,reason='ERROR',error=type(e).__name__))
    def _L_bottom_guide(self,F):   # valid scope frames: landing latch + distance log (no command change)
        try:
            s=self._bg_latch()
            if s is not None or (getattr(self,'bown',None) is not None and self.bown.active):F.p['bottom_guide']=dict(revision=bottom_guide.REVISION,mode='TRACK',s_m=None if s is None else round(s,3),curl_run=self.bguide.curl_n)
            if self.bguide.c.get('cap_lag') is not None:self._bg_cap(F,s)   # bottom-guide-r2 (c): open the bottom-owner cap while the heading lags the profile
        except Exception as e:F.p['bottom_guide']=dict(revision=bottom_guide.REVISION,reason='ERROR',error=type(e).__name__)
    # ---- pipeline2-20261116 (pipeline-ahead-r1 / dash-offthread-r1): web/pipeline2.json, read at each model / preview start
    # (_loop start). Absent / enabled not true -> None: the loop below is today's loop (no extra reads, no thread, same requests).
    # ---- worker-overlap-r1 (overlap-20261117): web/worker_overlap.json key "main", read at each model / preview start (like
    # pipeline2.json), used only when the worker's ready line also reports 'overlap'. Then two requests may be in flight: the next
    # NEW frame (same checks as pipeline-ahead) is sent while this frame's result is awaited, as soon as the worker reports that
    # its mask stage of this frame is done ({"kind":"mask_ready"}). Results are still consumed one at a time, in order, with the
    # same seq / model / weights checks; camera / max_gap / check_safety untouched. Absent / off / old worker -> None: today's loop.
    def _ov_load(self,ready):
        try:
            if not isinstance(ready,dict) or not ready.get('overlap'):return None
            c=json.loads((Path(__file__).parent/'worker_overlap.json').read_text(encoding='utf-8'))
            if not isinstance(c,dict) or c.get('enabled') is not True or c.get('main') is not True:return None
            ps=c.get('poll_s',.002)
            return dict(revision='worker-overlap-r1',worker=str(ready.get('overlap')),poll_s=float(ps) if type(ps) in (int,float) and .0005<=ps<=.02 else .002)
        except Exception:return None
    def _p2_load(self):
        try:
            c=json.loads((Path(__file__).parent/'pipeline2.json').read_text(encoding='utf-8'))
            if not isinstance(c,dict) or c.get('enabled') is not True:return None
            out=dict(revision='pipeline2-r1',ahead=c.get('ahead') is True,event_wait=c.get('event_wait') is True,dash_offthread=c.get('dash_offthread') is True)
            ws=c.get('wait_s',.06);out['wait_s']=float(ws) if type(ws) in (int,float) and .005<=ws<=.2 else .06
            return out
        except Exception:return None
    def _p2_wait(self,previous_frame):   # pipeline-ahead-r1: block until the camera publishes a control frame other than previous_frame (bounded); no-op without the hook
        wn=getattr(self.camera,'wait_next',None)
        if wn is None:return
        try:wn(previous_frame,self._p2['wait_s'])
        except Exception:pass
    def _p2_peek(self,previous_frame,previous_capture):
        """pipeline-ahead-r1: the newest control frame if it is NEW and passes every check of the top of the loop; otherwise None (the
        top of the loop then picks it up after this frame, with today's checks / errors / PID_CAMERA_LOST handling)."""
        try:
            ap,ac,af=getattr(self.camera,'peek',self.camera)()
            if self.worker_stop.is_set() or not ap or type(ac) not in (int,float) or not math.isfinite(ac):return None
            if _pclk.offset():ac=_pclk.to_virtual(ac)   # pause-clock-r1: capture time on the control timeline
            now=time.monotonic()
            if ac>now or now-ac>.5 or type(af) is not int:return None
            if previous_frame is not None and af<=previous_frame:return None
            if previous_capture is not None and ac<=previous_capture:return None
            return ap,ac,af
        except Exception:return None
    def _p2_dash_submit(self,seq,payload):   # dash-offthread-r1: route OPEN -> decode / white mask / blocks / RANSAC / side lines of this JPEG on a helper thread
        try:
            p2=getattr(self,'_p2',None);d=getattr(self,'dash',None)
            if not p2 or not p2.get('dash_offthread') or d is None or not (self.run and self.run['state']=='running') or getattr(d,'state',None)!='OPEN':return
            import dash_route as _dr
            if not hasattr(_dr,'precompute'):return
            ex=getattr(self,'_p2_ex',None)
            if ex is None:
                from concurrent.futures import ThreadPoolExecutor
                ex=self._p2_ex=ThreadPoolExecutor(1,thread_name_prefix='dash-pre')
            jobs={k:v for k,v in (getattr(self,'_p2_dj',None) or {}).items() if k>=seq-2};jobs[seq]=ex.submit(_dr.precompute,payload);self._p2_dj=jobs
        except Exception:pass
    def _p2_dash_take(self,seq):   # dash-offthread-r1: the precompute job of THIS frame (same JPEG object), None -> today's synchronous try_path
        jobs=getattr(self,'_p2_dj',None)
        return jobs.pop(seq,None) if jobs else None
    def _loop(self):
        proc=None;name='orin-pid-'+uuid.uuid4().hex[:12]
        try:
            spec=self.models[self.model_id];weights=Path(spec['weights'])
            command=['docker','run','--rm','-i','--name',name,'--network','none','--runtime','nvidia',
                     '--user',f'{os.getuid()}:{os.getgid()}','--cap-drop','ALL','--security-opt','no-new-privileges:true',
                     '--memory','4g','--shm-size','512m','-e','OPENBLAS_NUM_THREADS=1','-e','OMP_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-e','HOME=/tmp','-e','USER=jetson','-e','LOGNAME=jetson','-e','TORCHINDUCTOR_CACHE_DIR=/tmp/torch_cache','-e','YOLO_CONFIG_DIR=/tmp/yolo',
                     '-v',f'{ROOT}:{ROOT}:ro','-v',f'{weights.parent}:{weights.parent}:ro','-v','/home/jetson/orin_interface_v1/logs/imu:/home/jetson/orin_interface_v1/logs/imu:ro','orin-notebook:v1',   # ground-identity-r1 (2026-11-10): the path worker reads gz/pitch from the IMU logger (read-only)
                     'python3','-u',str(ROOT/'live_model_worker.py'),str(ROOT/'pid_models.json'),self.model_id,self.path_algorithm]
            log=(self.manual.root/'pid-model.log').open('ab')
            if self.path_algorithm in ('temporal-v2','calibrated-v7')+ROAD_ALGORITHMS:
                shared=ROOT.parent/'laptop_workbench'
                filenames=('temporal_lane.py','curve_replay.py','ground_calibration.py') if self.path_algorithm=='calibrated-v7' else ('temporal_lane.py','curve_replay.py')
                if self.path_algorithm in ROAD_ALGORITHMS:filenames=ROAD_FILES
                for filename in filenames:
                    command[2:2]=['-v',f'{shared/filename}:{shared/filename}:ro']
            proc=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log);log.close()
            replies=queue.Queue(maxsize=2)
            def reader():
                for line in proc.stdout:
                    try:
                        item=json.loads(line)
                        if item.get('kind')=='preview':
                            if item.get('protocol')==1 and item.get('model_id')==self.model_id and item.get('path_algorithm')==self.path_algorithm:
                                data=base64.b64decode(item['preview'])
                                with self.lock:
                                    if item['seq']>getattr(self,'preview_seq',0):
                                        self.preview=data;self.preview_seq=item['seq']
                            continue
                        replies.put(item,timeout=2)
                    except (ValueError,queue.Full):continue
            threading.Thread(target=reader,daemon=True).start()
            def response(timeout):
                deadline=time.monotonic()+timeout
                while time.monotonic()<deadline:
                    if self.worker_stop.is_set():raise ValueError('MODEL_PREVIEW_CANCELLED')
                    if proc.poll() is not None:raise ValueError('GPU worker exited; inspect pid-model.log')
                    try:return replies.get(timeout=.2)
                    except queue.Empty:pass
                raise ValueError('GPU worker timeout')
            ready=response(90)
            if not ready.get('ready') or ready.get('protocol')!=1:raise ValueError('Model protocol not ready')
            if ready.get('model_id')!=self.model_id or not isinstance(ready.get('weights_sha256'),str) or not ready['weights_sha256']:
                raise ValueError('Model identity not ready')
            digest=ready['weights_sha256']
            seq=0;previous_frame=None;previous_capture=None
            road_preview=None
            if self.path_algorithm in ('junction-k-lane-v3', 'junction-k-integrated-r2', 'standalone-line-v1', 'standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):
                from visual_pid import DEFAULTS
                road_preview=(OffscreenPID if self.path_algorithm in ('camera-pair-v2', 'camera-pair-v2-junction') else CameraPairPID if self.path_algorithm=='camera-pair-v1' else PreviewErrorPID if self.path_algorithm=='preview-error-v1' else SharedRoadPID if self.path_algorithm=='straight-bearing-v1' else GeometryPreviewPID if self.path_algorithm in ('straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction') else StraightPreviewPID if self.path_algorithm=='standalone-line-v3' else PreviewTurnPID if self.path_algorithm=='standalone-line-v2' else KAdaptivePID)(dict(DEFAULTS,row_ratio=self.row_ratio,curve_outward_px_640=self.curve_outward_px_640))
            elif self.path_algorithm in ROAD_ALGORITHMS:
                from road_live import RoadController, mark_sample
                from visual_pid import DEFAULTS
                road_preview=RoadController(dict(DEFAULTS,row_ratio=self.row_ratio))
            F=segment_ctx.Frame()   # segment-ctx-r1: values shared by the valid-frame layers (were locals of this loop; kept across frames like them)
            self._csz_c=self._csz_load();self._csz_end=None   # crossing-source-zone-r1: read at each run / preview start
            p2=self._p2_load();self._p2=p2;pend=None   # pipeline2-20261116: pipeline2.json at each model / preview start; None = today's loop
            ov=self._ov_load(ready);self._ov=ov;ovr=set()   # worker-overlap-r1: None = today's loop (ovr = seqs whose mask stage is done)
            while not self.worker_stop.is_set():
                if self.preview_expired(time.monotonic()):break
                if getattr(self,'_pz',None) is not None or _pclk.offset():previous_capture,pend=pause_resume.loop_top(self,previous_capture,pend)   # pause-resume-r1 (pause-resume-20261120): apply an accepted resume between frames
                if pend is not None:seq,payload,captured,frame=pend;pend=None   # pipeline-ahead-r1: this frame's request is already in the worker
                else:
                    if p2 is not None and p2['event_wait']:self._p2_wait(previous_frame)   # pipeline-ahead-r1: worker idle -> wait for a NEW frame (camera condition, bounded)
                    payload,captured,frame=self.camera()
                    if self.worker_stop.is_set():break
                    if _pclk.offset() and type(captured) in (int,float) and math.isfinite(captured):captured=_pclk.to_virtual(captured)   # pause-clock-r1
                    if type(captured) not in (int,float) or not math.isfinite(captured) or captured>time.monotonic():
                        raise ValueError('Invalid camera capture timestamp')
                    if not payload or time.monotonic()-captured>.5:
                        with self.lock:self.state='camera_unavailable';self.consecutive=0
                        self.end('PID_CAMERA_LOST');time.sleep(.1);continue
                    if frame==previous_frame:time.sleep(.01);continue
                    if type(frame) is not int or (previous_frame is not None and frame<previous_frame):
                        raise ValueError('Camera frame sequence regressed')
                    if previous_capture is not None and captured<=previous_capture:
                        raise ValueError('Camera capture timestamp did not advance')
                    previous_frame=frame;previous_capture=captured;seq+=1
                    proc.stdin.write((json.dumps(dict(seq=seq,jpeg=base64.b64encode(payload).decode(),row_ratio=self.row_ratio,path_algorithm=self.path_algorithm,captured=_pclk.to_real(captured) if _pclk.offset() else captured,visual_center_offset_px_640=getattr(self,'connector_center_offset',0.),route_epoch=self.run['id'] if self.run and self.run['state']=='running' else 'preview',**self._csz_req(),**self._ue_req(),**({'ov':1} if ov else {})))+'\n').encode());proc.stdin.flush()
                    if p2 is not None:self._p2_dash_submit(seq,payload)   # dash-offthread-r1
                if ov is None:result=response(3)
                else:   # worker-overlap-r1: wait for THIS frame's result (same 3 s / stop / worker-exit rules as response); meanwhile, once the
                    # worker's mask stage of this frame is done, send the next NEW frame (pipeline-ahead checks) so its mask overlaps this path stage
                    deadline_=time.monotonic()+3;result=None
                    while result is None:
                        if self.worker_stop.is_set():raise ValueError('MODEL_PREVIEW_CANCELLED')
                        if proc.poll() is not None:raise ValueError('GPU worker exited; inspect pid-model.log')
                        if time.monotonic()>=deadline_:raise ValueError('GPU worker timeout')
                        try:it_=replies.get(timeout=ov['poll_s'] if pend is None and seq in ovr else .2)
                        except queue.Empty:it_=None
                        if it_ is not None:
                            if it_.get('kind')=='mask_ready':
                                if it_.get('protocol')==1 and it_.get('model_id')==self.model_id and type(it_.get('seq')) is int:ovr.add(it_['seq'])
                            else:result=it_
                        if result is None and pend is None and seq in ovr:
                            nx_=self._p2_peek(previous_frame,previous_capture)
                            if nx_ is not None:
                                a_payload,a_captured,a_frame=nx_;previous_frame=a_frame;previous_capture=a_captured
                                proc.stdin.write((json.dumps(dict(seq=seq+1,jpeg=base64.b64encode(a_payload).decode(),row_ratio=self.row_ratio,path_algorithm=self.path_algorithm,captured=_pclk.to_real(a_captured) if _pclk.offset() else a_captured,visual_center_offset_px_640=getattr(self,'connector_center_offset',0.),route_epoch=self.run['id'] if self.run and self.run['state']=='running' else 'preview',**self._csz_req(),**self._ue_req(),ov=1))+'\n').encode());proc.stdin.flush()
                                self._p2_dash_submit(seq+1,a_payload);pend=(seq+1,a_payload,a_captured,a_frame)
                    ovr={s_ for s_ in ovr if s_>seq}
                if result.get('protocol')!=1 or result.get('seq')!=seq or result.get('model_id')!=self.model_id:raise ValueError('Model sequence/protocol mismatch')
                if result.get('path_algorithm')!=self.path_algorithm:raise ValueError('Path algorithm protocol mismatch')
                if result.get('weights_sha256')!=digest or (self.run and self.run['state']=='running' and self.run['weights_sha256']!=digest):
                    raise ValueError('Model weights identity changed')
                if self.path_algorithm=='calibrated-v7':
                    from calibrated_path import load_calibration
                    expected=self.run.get('calibration_sha256') if self.run and self.run['state']=='running' else load_calibration()[2]
                    if result.get('path_diagnostic',{}).get('calibration_sha256')!=expected:
                        raise ValueError('V7 calibration identity changed')
                if p2 is not None and p2['ahead'] and pend is None:   # worker-overlap-r1: pend may already hold the next frame (always None when overlap is off). pipeline-ahead-r1: the next NEW frame goes to the worker before this one is processed (request flags = state now)
                    nx_=self._p2_peek(previous_frame,previous_capture)
                    if nx_ is not None:
                        a_payload,a_captured,a_frame=nx_;previous_frame=a_frame;previous_capture=a_captured
                        proc.stdin.write((json.dumps(dict(seq=seq+1,jpeg=base64.b64encode(a_payload).decode(),row_ratio=self.row_ratio,path_algorithm=self.path_algorithm,captured=_pclk.to_real(a_captured) if _pclk.offset() else a_captured,visual_center_offset_px_640=getattr(self,'connector_center_offset',0.),route_epoch=self.run['id'] if self.run and self.run['state']=='running' else 'preview',**self._csz_req(),**self._ue_req(),**({'ov':1} if ov else {})))+'\n').encode());proc.stdin.flush()
                        self._p2_dash_submit(seq+1,a_payload);pend=(seq+1,a_payload,a_captured,a_frame)
                encoded_preview=result.pop('preview',None)
                preview=base64.b64decode(encoded_preview) if encoded_preview else None
                with self.lock:
                    if ops07_support.holding_frame(self,result,captured,frame,seq,preview):continue
                    if getattr(self,'_pz',None) is not None and pause_resume.hold_frame(self,result,captured,frame,seq,preview):continue   # pause-resume-r1 (pause-resume-20261120): paused -> centre / PWM 0, nothing stepped
                    rate_indep.frame(captured,((self.latest or {}).get('dash_route') or {}).get('route')=='OPEN')   # rate-indep-r1: capture interval of this frame, upper section = previous frame in the dash OPEN window (no-op when off)
                    proposal=None;sample_row=self.row_ratio
                    # crest-fallback-r1: only during a run, only when the selected path is invalid
                    # and the IMU says the car is pitched up (see crest_fallback.py).
                    # dash-route-r1: after the crest (IMU route window) the dashed line overrides the
                    # path: left turn at the dashed row, then follow the dashed line (dash_route.py).
                    if self.run and self.run['state']=='running' and getattr(self,'dash',None) is not None:
                        try:
                            dash_open,dash_info=self.dash.update_route(time.monotonic())
                            if getattr(self.dash,'lap',0)!=getattr(self,'route_lap',0):   # dash-route-r21: new lap -> fresh heading / lane / downhill-entry state
                                self.route_lap=self.dash.lap;lc=getattr(self.heading,'lane_cap',None) if getattr(self,'heading',None) is not None else None
                                self.heading=HeadingHold(self.boost);self.heading.lane_cap=lc;self.lane=LaneSegment(dash_only=False);self.lane.lock=True
                                self.entry=DownhillEntry();self.turn_yaw0=None;self.down_until=None;dash_info=dict(dash_info,lap=self.route_lap)   # bottom-commit-r2: the bottom window is kept across the lap step (it ends by itself)
                            if dash_open:
                                if getattr(self,'heading',None) is not None:self.dash.expect_rel=self.heading.expected_rel(time.monotonic())   # dash-route-r11
                                try:ne=(offscreen_path_error if self.path_algorithm in ('camera-pair-v2', 'camera-pair-v2-junction') else path_error)(result.get('path',[]),self.row_ratio,visual_center_offset_px_640=self.run['parameters'].get('visual_center_offset_px_640',0.)) if result.get('valid') is True else None   # dash-route-r23: normal-path error for the delayed turn start
                                except Exception:ne=None
                                try:   # dash-align-min-r1: the camera "aligned" end of the TURN needs >= DASH_ALIGN_MIN_TURN_DEG of gyro yaw since the TURN start
                                    ty0=getattr(self,'turn_yaw0',None);self.dash.align_ok=True if ty0 is None else (self.heading._yaw(time.monotonic())-ty0)>=DASH_ALIGN_MIN_TURN_DEG
                                except Exception:self.dash.align_ok=True
                                self.dash.lp=self._arc2_lp(False)[0] if getattr(self.dash,'tt',None) and self.dash.tt.get('pos_backup') else None   # dash-pos-backup-r1 (dash-late-20261116): last lap_pos state for the position backup TURN; attribute only, None when upper_turn.json has no pos_backup
                                self.dash.v_mps=getattr(self,'_cm_speed',None)   # upper-turn-r1 (upper-20261115): last frame's wheel speed (m/s) for the speed-aware turn delay (dash_route upper_turn.json); attribute only
                                pj_=self._p2_dash_take(seq)   # dash-offthread-r1: None when off
                                dash_path,more=self.dash.try_path(time.monotonic(),payload,self.row_ratio,normal_err=ne) if pj_ is None else self.dash.try_path(time.monotonic(),payload,self.row_ratio,normal_err=ne,pre=pj_);dash_info=dict(dash_info,**more)
                                if dash_path:result.update(path=dash_path,valid=True,selected_source='dash_route')
                        except Exception as e:dash_info=dict(revision=DASH_REVISION,used=False,reason='DASH_ERROR',error=type(e).__name__)
                        result['dash_route']=dash_info
                    if self.run and self.run['state']=='running' and getattr(self,'crest',None) is not None:
                        jsrc=((result.get('path_diagnostic') or {}).get('junction') or {}).get('source')
                        crest_mode=None if result.get('selected_source')=='dash_route' else 'invalid' if not result.get('valid') else 'corridor' if jsrc=='corridor' else None
                        cl_=getattr(self,'clatch',None)   # crest-latch-r1: None = off
                        if cl_ is not None:
                            try:crest_mode=cl_.mode(time.monotonic(),result.get('dash_route'),result.get('selected_source'),crest_mode)
                            except Exception:pass
                        if crest_mode is None:self.crest.normal_valid()
                        else:
                            try:crest_path,crest_info=self.crest.try_path(time.monotonic(),result,self.row_ratio,crest_mode)
                            except Exception as e:crest_path,crest_info=None,dict(revision=CREST_REVISION,used=False,reason='CREST_ERROR',error=type(e).__name__)
                            result['crest_fallback']=crest_info
                            if crest_path:result.update(path=crest_path,valid=True,selected_source='crest_midpoint')
                            if cl_ is not None:
                                try:
                                    if cl_.after(time.monotonic(),result.get('dash_route'),crest_mode,crest_info)=='INVALID':result.update(path=[],valid=False)   # latched, no midpoint: lost-line rules, not the single-line estimate / corridor
                                except Exception:pass
                        if cl_ is not None and cl_.last is not None:result['crest_latch']=cl_.last
                    # downhill-nocorridor-r1 (live 65c2b828 105.9-107.6 s, b9fa9cff 104.0-104.2 s): on the downhill the junction switch
                    # took its corridor path (error -0.5 .. -0.96, left) while the baseline pointed right (+0.75 .. +1.0); the car went
                    # off on the left. After the dashed section (route DONE) and pitched down, the baseline path is used; without a
                    # baseline the frame is invalid (downhill hold) instead of following the corridor.
                    if self.run and self.run['state']=='running':
                        try:
                            drn=result.get('dash_route') or {};jn=((result.get('path_diagnostic') or {}).get('junction') or {})
                            pn=drn.get('pitch_rel_deg')
                            if drn.get('route')=='DONE' and pn is not None and pn< -2. and result.get('selected_source') is None and str(jn.get('source') or '').startswith('corridor'):
                                bp=jn.get('baseline_path') or []
                                result['downhill_nocorridor']=dict(revision='downhill-nocorridor-r1',junction_source=jn.get('source'),baseline_points=len(bp))
                                if len(bp)>=2:result.update(path=bp)
                                else:result.update(path=[],valid=False)
                        except Exception as e:result['downhill_nocorridor']=dict(revision='downhill-nocorridor-r1',reason='ERROR',error=type(e).__name__)
                    # bottom-left-r1 (bottom_left.py, live 9ce62fb8 32.36 s): at the level bottom a corridor take to the left of the baseline is
                    # not followed (baseline instead, or invalid -> lost-path hold); window from the previous frames' down_tail / down_steep
                    if self.run and self.run['state']=='running':
                        try:
                            dbl=(result.get('dash_route') or {}).get('pitch_rel_deg');tbl=time.monotonic()
                            if bottom_left.in_window(getattr(self,'down_until',None) is not None and tbl<=self.down_until,getattr(self,'down_steep',False),dbl):
                                ofb=self.run['parameters'].get('visual_center_offset_px_640',0.)
                                efb=offscreen_path_error if self.path_algorithm in ('camera-pair-v2', 'camera-pair-v2-junction') else path_error
                                act,binfo=bottom_left.gate(result,lambda pts:efb(pts,self.row_ratio,visual_center_offset_px_640=ofb))
                                if binfo is not None:result['bottom_left']=binfo
                                if act=='BASELINE':result.update(path=(((result.get('path_diagnostic') or {}).get('junction') or {}).get('baseline_path') or []))
                                elif act=='INVALID':result.update(path=[],valid=False)
                        except Exception as e:result['bottom_left']=dict(revision=bottom_left.REVISION,reason='ERROR',error=type(e).__name__)
                        if getattr(self,'bguide',None) is not None and self.bguide.on and self.bguide.c.get('corr_right') is not None:result=self._bg_corr(result)   # bottom-guide-r2 (d)
                    # downhill-entry-r1 (live 20aa810f): right after the two-lane section a lone 'right+width' path is the outer
                    # (left) edge of the downhill right curve -> mirrored about that edge (downhill_entry.py)
                    if self.run and self.run['state']=='running' and getattr(self,'entry',None) is not None:
                        try:
                            dpe=(result.get('dash_route') or {}).get('pitch_rel_deg');tn=time.monotonic()
                            self.entry.note_pitch(tn,dpe)   # downhill-entry-r2: window timed from the real slope start
                            self._mid_update(result,tn,dpe)   # downhill-mirror-identity-r1
                            if result.get('valid') and result.get('selected_source') is None:
                                mr=self.entry.mirror(tn,result,dpe,(result.get('dash_route') or {}).get('route')=='DONE')
                                if mr is not None:mr=self._mirror_gate(result,mr,tn)   # downhill-mirror-identity-r1 (record-only unless MIRROR_IDENTITY_APPLY)
                                if mr is not None:result.update(path=mr[0],downhill_entry=mr[1])
                        except Exception as e:result['downhill_entry']=dict(revision=ENTRY_REVISION,reason='ENTRY_ERROR',error=type(e).__name__)
                        # corner-width-r1 (live 3fd37b76 ramp corner): a single flat outer line -> centre further from the line (corner_width.py);
                        # not on mirrored downhill-entry frames and not when the junction switch selected its corridor path
                        try:
                            jsc=((result.get('path_diagnostic') or {}).get('junction') or {}).get('source')
                            if result.get('valid') and result.get('selected_source') is None and not result.get('downhill_entry') and (jsc!='corridor' or result.get('downhill_nocorridor') or (result.get('bottom_left') or {}).get('action')=='BASELINE' or (result.get('bottom_corr') or {}).get('action')=='BASELINE'):
                                cw=corner_widen(result)
                                if cw is not None:result.update(path=cw[0],corner_width=cw[1])
                        except Exception as e:result['corner_width']=dict(revision=CORNER_REVISION,reason='CORNER_ERROR',error=type(e).__name__)
                        # downhill-entry-r3: on the downhill a single-line path is moved towards its line (narrower lane at the control rows)
                        try:
                            dpn=(result.get('dash_route') or {}).get('pitch_rel_deg')   # only while on the slope: at the flat bottom the width is the flat one again
                            if result.get('valid') and result.get('selected_source') is None and dpn is not None and dpn< -5.:
                                nw=downhill_narrow(result)
                                if nw is not None:result.update(path=nw[0],downhill_width=nw[1])
                        except Exception as e:result['downhill_width']=dict(revision=ENTRY_REVISION,reason='WIDTH_ERROR',error=type(e).__name__)
                        # downhill-pair-r1 (user 2026-11-04, e55523e7 27.05 s): on the slope the road is the gap between the two rightmost
                        # white lines; a target left of that gap (flat-track line paired with the right edge) is moved to its centre
                        try:
                            dpp=result.get('dash_route') or {}
                            pp,pinfo=downhill_pair.apply(result,result.get('sample_row_ratio') or self.row_ratio,dpp.get('pitch_rel_deg'),dpp.get('route')=='DONE')
                            if pinfo is not None:result['downhill_pair']=pinfo
                            if pp is not None:result.update(path=pp)
                        except Exception as e:result['downhill_pair']=dict(revision=downhill_pair.REVISION,reason='PAIR_ERROR',error=type(e).__name__)
                    if getattr(self,'bguide',None) is not None and self.bguide.on and self.bguide.c.get('curl'):result=self._bg_curl(result)   # bottom-guide-r1 (b): left-curling single left line inside the bottom scope -> lost frame
                    if getattr(self,'bguide',None) is not None and self.bguide.on and self.bguide.c.get('lag_lost') is not None:result=self._bg_laglost(result)   # bottom-guide-r2 (e)
                    jf_s0=None   # jump-filter-r2: snapshot of the running PID controller (upper section after the dashed turn), tagged with this frame
                    try:
                        lpt=getattr(self,'_last_post_turn',None)
                        if self.run and self.run['state']=='running' and (result.get('dash_route') or {}).get('route')=='OPEN' and lpt is not None and lpt[1] and time.monotonic()-lpt[0]<=.5:
                            import copy as _jfc;jf_s0=(captured,_jfc.deepcopy(self.controller.__dict__))
                    except Exception:jf_s0=None
                    # pid-hold-r2: an invalid frame of a running PID does not step (and reset) the controller
                    if road_preview is not None and not (self.run and self.run['state']=='running' and not result.get('valid')):
                        active=self.run and self.run['state']=='running'
                        controller=self.controller if active else road_preview
                        points=result.get('path',[]) if result.get('valid') else []
                        if self.path_algorithm in ('junction-k-lane-v3', 'junction-k-integrated-r2', 'standalone-line-v1', 'standalone-line-v2', 'standalone-line-v3', 'straight-connector-v1', 'straight-bearing-v1', 'preview-error-v1', 'camera-pair-v1', 'camera-pair-v2', 'camera-pair-v2-junction'):
                            envelope=result.get('path_diagnostic',{}).get('k',{}).get('mask_gate',{}).get('lane_envelope',[])
                            proposal=controller.step_path(captured,points,envelope)
                            outward=proposal.get('outward')
                            if outward is not None:
                                result['road_center_path']=points
                                result['path']=outward['path']
                                result['outward']={k:v for k,v in outward.items() if k!='path'}
                                if preview is not None:preview=overlay(preview,outward['path'])
                            result['curve_outward_px_640']=self.curve_outward_px_640
                            result['curve_outward_version']=1
                        else:proposal=controller.step_path(captured,points)
                        if 'turn_preview' in proposal:result['turn_preview']=proposal['turn_preview']
                        sample_row=proposal['sample_row_ratio']
                        result.update(sample_row_ratio=sample_row,requested_row_ratio=self.row_ratio, sample_reason=proposal['sample_reason'],sampling_version=proposal.get('sampling_version'),derivative_rebased=proposal.get('derivative_rebased'),sample_clamped=proposal.get('sample_clamped'),sample_bounds=proposal.get('sample_bounds'))
                        if preview is not None:
                            from road_live import mark_sample
                            preview=mark_sample(preview,sample_row)
                    offset=self.run['parameters'].get('visual_center_offset_px_640',0.) if self.run and self.run['state']=='running' else 0.
                    error=(offscreen_path_error if self.path_algorithm in ('camera-pair-v2', 'camera-pair-v2-junction') else path_error)(result.get('path',[]),sample_row,visual_center_offset_px_640=offset)
                    result.update(visual_center_offset_px_640=offset,visual_center_version=1)
                    now=time.monotonic();valid=result.get('valid') is True and error is not None and now-captured<=.5
                    previous=self.latest
                    self.stream_diagnostics.update(
                        frames=self.stream_diagnostics.get('frames',0)+1,
                        max_control_age_s=max(self.stream_diagnostics.get('max_control_age_s',0),now-captured),
                        max_result_gap_s=max(self.stream_diagnostics.get('max_result_gap_s',0),now-previous['consumed_at'] if previous else 0),
                        max_latest_age_before_update_s=max(self.stream_diagnostics.get('max_latest_age_before_update_s',0),now-previous['capture_at'] if previous else 0))
                    if preview is not None:self.preview=preview
                    self.latest=dict(result,capture_at=captured,frame_seq=frame,error=error,valid=valid,consumed_at=now,control_age_s=now-captured)
                    if _pclk.offset():self.latest['pause_offset_s']=round(_pclk.offset(),6)   # pause-clock-r1: capture_at / consumed_at = Orin monotonic - this
                    self.consecutive=self.consecutive+1 if valid else 0;self.state='ready' if valid else 'no_reliable_path'
                    if self.run and self.run['state']=='running':
                        # downhill-r3 (user 2026-11-03, live 53b2171a / c933191a): r2's right-only clamp and forced centre+300 hold
                        # pushed the car out on the right while the path pointed left; now: assist only for errors > 0.4, a lost
                        # path keeps the last valid command, left corrections allowed.
                        # downhill-r1 (user 2026-11-02, live 41502243): the downhill is a big right curve. While pitched
                        # down (and DOWN_HOLD_S after) the path error to the right gets at least DOWN_MIN_RIGHT_US, and a
                        # lost path keeps the last valid command (still turning) instead of the straight-on lost heading.
                        dpr=None
                        try:
                            dpr=(result.get('dash_route') or {}).get('pitch_rel_deg')
                            if getattr(self,'dipg',None) is not None:   # flat-dip-guard-r1: short pitch dips on the flat lap are not a downhill
                                dpr=self.dipg.step(now,dpr,(result.get('dash_route') or {}).get('route'))
                                if self.dipg.info is not None and self.latest is not None:self.latest['flat_dip_guard']=self.dipg.info
                            if dpr is not None and dpr< -5.:self.down_until=now+4.
                        except Exception:pass
                        # downhill-release-r1 (user 2026-11-04, live 75c85728 / cfafa5c5: the flat curve after the slope went badly). The 4 s
                        # latch after the last pitch < -5 kept the downhill-only steering rules on at the flat bottom: speed scaling cut a
                        # requested 1750 to 1612-1640 (k .61-.63) for 0.8 s while the error was +1.0 for 3.5 s (75c85728 30.0-33.6 s), and the
                        # D limit held 1390 while the PID asked 1276-1390 on the left bend at the bottom (cfafa5c5 29.85-30.55 s, then off the
                        # track). Now those rules end once the pitch has stayed above DOWN_STEEP for DOWN_RELEASE_S after this slope reached it;
                        # 20 slope passes had at most 0.64 s above -8 mid-slope. The lost-path DOWNHILL_HOLD keeps the old 4 s window
                        # (down_tail), and so does the D cap: 9eedee0a 30.45 s at the flat bottom a left -> right line switch gave D -677 and
                        # the cap kept 1264 (uncapped ~1012). Slopes that never went below DOWN_STEEP keep the old latch. Experiment values.
                        if dpr is not None and dpr<DOWN_STEEP and not getattr(self,'down_steep',False) and getattr(self,'bguide',None) is not None and self.bguide.on:self._bg_steep()   # bottom-guide-r1: reference yaw of this slope
                        if dpr is not None and dpr<DOWN_STEEP and not getattr(self,'down_steep',False):self._ds_t0=now   # slope-pre-right-r1: start of the steep slope
                        if dpr is not None and dpr<DOWN_STEEP:self.down_steep=True;self.down_level_t0=None
                        elif dpr is not None and getattr(self,'down_steep',False) and getattr(self,'down_level_t0',None) is None:self.down_level_t0=now
                        down_tail=getattr(self,'down_until',None) is not None and now<=self.down_until
                        if not down_tail:self.down_steep=False;self.down_level_t0=None
                        down_level=getattr(self,'down_level_t0',None)
                        # bottom-map-r1 (user 2026-11-10 5262: after the slope the track is the known right curve 4; lost lines must not mean straight on):
                        # yaw at the landing (first level moment after the steep slope) -> how far the car has turned right since
                        if down_level is not None and getattr(self,'_bm_t0',None)!=down_level:
                            self._bm_t0=down_level
                            try:self._bm_yaw0=self.heading._yaw(time.monotonic()) if getattr(self,'heading',None) is not None else None
                            except Exception:self._bm_yaw0=None
                        if not down_tail:self._bm_yaw0=None;self._bm_t0=None;self._ds_t0=None
                        downhill=down_tail and not (down_level is not None and now-down_level>=DOWN_RELEASE_S)
                        if down_tail and not downhill and self.latest is not None:self.latest['downhill_release']=dict(revision=DOWN_RELEASE_REVISION,level_s=round(now-down_level,3),tail_s=round(self.down_until-now,3))
                        F.ctx=segment_ctx.Ctx(self.latest,dpr,down_tail,downhill,down_level,getattr(self,'down_steep',False));F.owner=None   # segment-ctx-r1: segment context of this frame
                        if not valid or now-captured>self.run['parameters']['max_gap']:
                            # lost-policy-r1 (integration B3c): the lost-line decision is the table LOST_TABLE (lost_policy.py / lost_policy.json): blind holds
                            # (upper-single / lane / heading) -> PID_HOLD (0.4 s) -> DOWNHILL_HOLD -> LOST_HEADING / LOST_TURN -> stop; then the row's modifiers
                            stale=now-captured>self.run['parameters']['max_gap']
                            F.now=now;F.result=result;F.stale=stale;F.downhill=downhill;F.down_tail=down_tail
                            lrow,lv=lost_policy.decide(LOST_TABLE,self,F)
                            if lrow is None:self.end('PID_INVALID_OR_STALE_PATH');continue
                            lp=lost_policy.finish(lrow,lv,self,F,segment_ctx.owner_after)
                            if getattr(self,'lff',None) is not None and getattr(self.lff,'band_on',False):lp=self._lff_lost(F,lp)   # c2-band-r1 (no band key -> not called)
                            if getattr(self,'lff4',None) is not None and getattr(self.lff4,'band_on',False):lp=self._lff4_lost(F,lp)   # c4-band-r1 (no c4.band key -> not called)
                            if getattr(self,'arc2',None) is not None and getattr(self.arc2.get('blind_c2'),'lost_on',False):lp=self._b2_lost(F,lp)   # blind-v2 (blind2-20261117): law v2 only
                            if getattr(self,'bown',None) is not None and self.bown.on:lp=self._bo_lost(F,lp)   # bottom-owner-r2: gyro rate hold inside the bottom scope
                            if getattr(self,'bguide',None) is not None and self.bguide.on:lp=self._bg_lost(F,lp)   # bottom-guide-r1 (a): distance-based heading-profile hold where the r2 rate hold acts
                            if getattr(self,'arc2',None) is not None and getattr(self.arc2.get('land'),'lost_on',False):lp=self._a2_lost(F,lp)   # land-smooth-20261120: lost frame in the arc2 approach window (no land.approach.lost key -> not called)
                            if getattr(self,'land4',None) is not None:lp=self._l4_lost(F,lp)   # land4-r1: slope / landing heading law on lost frames (land4.json absent -> not called)
                            if getattr(self,'stuck',None) is not None and self._stuck_check(lp):continue   # stuck-stop-r1 (lost-line rows)
                            if not self._pid_send(dict(owner=self.owner,run_id=self.run['id'],seq=seq,target_us=lp['steer_us'],pwm=lp['pwm'],observation_context=vision_context(self.run['id'],seq,self.latest,lp)),lp):continue   # manual-end-r1; play-single-r1: hold-type frame, compensated at the output stage
                            self.hold.sent(now,lp['steer_us'],lp['pwm']);self._b2_tag(lp,F)   # segment-ctx-r1 log
                            self.log.write(json.dumps({'kind':'sample','wall':time.time(),'vision':self.latest,'proposal':lp,lrow.flag:True,'command_dispatched':True,'physical_response_verified':False,'pid_input_seq':seq,'feedback_observation_version':1})+'\n');self.log.flush()
                            continue
                        if proposal is None:proposal=self.controller.step_path(captured,result.get('path',[]))
                        # pid-hold-r3: after hold / heading-blind frames the controller's own time gap may reject the
                        # first valid frame (it resets itself); restart it on this frame instead of ending the run
                        if proposal.get('reason')=='INVALID_OR_STALE_TIME' and getattr(self,'hold',None) is not None and self.hold.last_sent(now) is not None:
                            proposal=self.controller.step_path(captured,result.get('path',[]))
                        if proposal['reason']=='WARMUP':
                            rc=(self.hold.recent(now) or self.hold.last_sent(now)) if getattr(self,'hold',None) is not None else None
                            if rc is not None:proposal.update(steer_us=rc['steer_us'],pwm=rc['pwm'],warmup_held=True)   # pid-hold-r2
                            else:proposal.update(steer_us=self.run['parameters']['center_us'],pwm=0)
                        elif not proposal['valid']:self.end(proposal['reason']);continue
                        F.p=proposal;F.result=result;F.now=now;F.captured=captured;F.jf_s0=jf_s0;F.dpr=dpr;F.down_tail=down_tail;F.downhill=downhill;F.down_level=down_level;F.usv=None   # segment-ctx-r1: this frame's values for the layer table
                        F.owner='pid' if proposal['reason']!='WARMUP' else ('warmup_hold' if proposal.get('warmup_held') else 'warmup_centre')
                        segment_ctx.run(VALID_LAYERS,self,F)   # segment-ctx-r1: jump filter ... upper-single override, in today's order (VALID_LAYERS at the end of this file)
                        wheel=F.wheel
                        if proposal['reason']!='WARMUP':   # b1-log: replay inputs of this frame (clock before dispatch, the upper_path gz, MCU status seen by _wheel_cps)
                            try:
                                v_=getattr(self,'_in_v',None) or {};b_=v_.get('enc_b') or {}
                                proposal['inputs']=dict(t_send=round(time.monotonic(),4),gz=getattr(F,'gz_up',None),veh_ms=v_.get('ms'),enc_b=(int(b_['forward'])-int(b_['reverse'])) if 'forward' in b_ and 'reverse' in b_ else None,steer_cmd=v_.get('steer_us_command'))
                            except Exception as e:proposal['inputs']=dict(reason='ERROR',error=type(e).__name__)
                        self._lap_pos(proposal)   # lap-pos-r1 (log only): proposal.lap_pos
                        pause_resume.lap_hook(self,proposal)   # pause-resume-r1 (pause-resume-20261120): run timing (laps), no command change
                        if getattr(self,'stuck',None) is not None and self._stuck_check(proposal):continue   # stuck-stop-r1 (valid rows, after the inputs log)
                        if not self._pid_send(dict(owner=self.owner,run_id=self.run['id'],seq=seq,target_us=proposal['steer_us'],pwm=(wheel['pwm_right']+wheel['pwm_left'])/2. if wheel else proposal['pwm'],observation_context=vision_context(self.run['id'],seq,self.latest,proposal),**wheel),None if self._ps_skip(proposal) else proposal):continue   # manual-end-r1
                        if getattr(self,'hold',None) is not None:
                            self.hold.sent(now,proposal['steer_us'],proposal['pwm'])
                            # centre-estimator-r1: record-only straight-point estimate (does not change the command)
                            if getattr(self,'centre_est',None) is not None:
                                try:
                                    vq=getattr(self,'_cm_speed',None);hq=getattr(self,'_ce_vhist',[]);hq=[x for x in hq if now-x[0]<=1.]   # centre-estimator-r2 / auto-centre-r2: collision = wheel speed from >0.45 m/s to <0.08 within 1 s at PWM >= 13
                                    if vq is not None:hq.append((now,float(vq)))
                                    self._ce_vhist=hq
                                    if getattr(self,'_collision',None) is None and vq is not None and float(vq)<.08 and float(proposal.get('pwm') or 0)>=13. and any(v>.45 for _,v in hq):
                                        self._collision=dict(t_run_s=round(time.time()-float(self.run.get('started') or time.time()),1),pwm=proposal.get('pwm'));self.run['collision']=self._collision
                                    drc=self.latest.get('dash_route') or {}
                                    ce_ok=bool(drc.get('route')=='WAIT_CLIMB' and (proposal.get('curve_map') or {}).get('reason') not in ('FLOOR','RELEASED') and getattr(self,'_collision',None) is None)
                                    cs=self.centre_est.step(now,proposal['steer_us'],proposal['pwm'],allow=ce_ok)
                                    if cs:self.log.write(json.dumps({'kind':'centre_sample','wall':time.time(),**cs})+'\n')
                                except Exception:pass
                            if proposal['reason']!='WARMUP':self.hold.valid(now,proposal['steer_us'],proposal['pwm'])
                        if ops07_support.continuous(self.run):self.ops07.observe(proposal)
                        self._b2_tag(proposal,F)   # segment-ctx-r1 log: proposal.seg / proposal.owner
                        self.log.write(json.dumps({'kind':'sample','wall':time.time(),'vision':self.latest,'proposal':proposal,'command_dispatched':True,'physical_response_verified':False,'pid_input_seq':seq,'feedback_observation_version':1})+'\n');self.log.flush()
        except Exception as e:
            with self.lock:self.error=str(e) or type(e).__name__;self.state='failed';self.consecutive=0
            self.end('PID_MODEL_FAILURE')
        finally:
            if proc:
                try:proc.stdin.close()
                except OSError:pass
                try:subprocess.run(['docker','stop','-t','1',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=10)
                except (OSError,subprocess.TimeoutExpired):pass
                try:proc.terminate()
                except OSError:pass
            with self.lock:
                if self.state!='failed':self.state='idle'


# ---- segment-ctx-r1 (integration B2): the valid-frame steering / PWM pipeline as data. Same order as the inline code it replaces; each
# eligibility is that block's old `if` (blocks whose test sat inside their try keep it there when it can raise). F.p = proposal,
# F.ctx = segment context, F.dr = dash-route dict, F.usv = upper-single command, F.down_tail / F.downhill / F.down_level / F.dpr = windows.
_SL=segment_ctx.Layer;_SG=segment_ctx.Group
def _nw(s,F):return F.p.get('reason')!='WARMUP'
def _yaw_err(s,F,e):F.p['yaw_inner']=dict(revision=yaw_inner.REVISION,reason='ERROR',error=type(e).__name__)
def _speed_err(s,F,e):F.p['speed_adapt']=dict(revision=SPEED_REVISION,reason='SPEED_ERROR',error=type(e).__name__)
VALID_LAYERS=(
    _SL('jump_filter',None,LivePID._L_jump_filter),
    _SL('start_centre',lambda s,F:not getattr(s,'start_centred',True),LivePID._L_start_centre),
    _SG('steer',lambda s,F:F.p['reason']!='WARMUP',(
        _SL('imu_boost',None,LivePID._L_imu_boost),
        _SL('dash_route',None,LivePID._L_dash_route),
        _SL('crest_marks_arm',lambda s,F:getattr(s,'cmarks',None) is not None,LivePID._L_crest_marks_arm),
        _SL('dash_turn_end',lambda s,F:F.ctx.sel=='dash_route' and F.dr.get('used') and F.dr.get('mode')=='TURN' and getattr(s,'heading',None) is not None,LivePID._L_dash_turn_end),
        _SL('dash_turn',lambda s,F:F.ctx.sel=='dash_route' and F.dr.get('used') and F.dr.get('mode')=='TURN',LivePID._L_dash_turn),   # F.dr after dash_turn_end (FOLLOW copy when the TURN just ended)
        _SL('heading_hold',lambda s,F:getattr(s,'heading',None) is not None,LivePID._L_heading_hold),
        _SL('upper_single',None,LivePID._L_upper_single),
        _SL('lane_segment',lambda s,F:getattr(s,'lane',None) is not None and F.usv is None,LivePID._L_lane_segment),
        _SL('solid_takeover',lambda s,F:getattr(s,'solid',None) is not None and F.usv is None,LivePID._L_solid_takeover),
        _SL('gain_schedule',None,LivePID._L_gain_schedule),
        _SG('yaw_group',lambda s,F:getattr(s,'yin',None) is not None,(
            _SL('yaw_prelude',None,LivePID._L_yaw_prelude),
            _SL('curve_map',lambda s,F:getattr(s,'cmap',None) is not None,LivePID._L_curve_map),
            _SL('curve_ctx',None,LivePID._L_curve_ctx),
            _SL('cmd_comp',lambda s,F:getattr(s,'ccomp',None) is not None,LivePID._L_cmd_comp),
            _SL('crossing_pass',None,LivePID._L_crossing_pass),
            _SL('curve_smooth',None,LivePID._L_curve_smooth),
            _SL('arc2_pre',lambda s,F:getattr(s,'arc2',None) is not None and s.arc2.get('pre') is not None,LivePID._L_arc2_pre),   # slope-arc-r1 A: before yaw_inner (absent -> not run)
            _SL('yaw_inner',None,LivePID._L_yaw_inner),
            _SL('xp_term_cap',None,LivePID._L_xp_term_cap),
            _SL('weave_flag',lambda s,F:getattr(s,'sslew',None) is not None,LivePID._L_weave_flag)),_yaw_err),
        _SL('xp_corridor',None,LivePID._L_xp_corridor),   # xp-corridor-r2
        _SL('gz_up',None,LivePID._L_gz_up),
        _SL('curve_ff',lambda s,F:getattr(s,'cff',None) is not None,LivePID._L_curve_ff),
        _SL('lap_ff',lambda s,F:getattr(s,'lff',None) is not None,LivePID._L_lap_ff),   # lap-ff-r1: absent / disabled -> not run
        _SL('arc2_flat',lambda s,F:getattr(s,'arc2',None) is not None and (s.arc2['c3'] is not None or s.arc2['blind'] is not None or s.arc2.get('blind_c2') is not None),LivePID._L_arc2_flat),   # arc2-r1 C / D; c2-blind-r1
        _SL('curve_trim',lambda s,F:getattr(s,'ctrim',None) is not None,LivePID._L_curve_trim),   # curve-trim-r1: absent / disabled -> not run
        _SL('arc_lat',lambda s,F:getattr(s,'alat',None) is not None,LivePID._L_arc_lat),   # arc-lat-r1: absent / disabled -> not run
        _SG('speed',lambda s,F:getattr(s,'speed',None) is not None,(
            _SL('speed_prelude',None,LivePID._L_speed_prelude),
            _SL('upper_path',None,LivePID._L_upper_path),
            _SL('speed_adapt',None,LivePID._L_speed_adapt)),_speed_err))),
    _SL('speed_caps',None,LivePID._L_speed_caps),   # speed-policy-r1: lane-lock cap, brake zones, turn-pwm cap (their eligibility is inside, unchanged)
    _SL('downhill_dcap',lambda s,F:F.down_tail and F.p.get('reason')!='WARMUP' and F.p.get('d') is not None and F.p.get('p') is not None and abs(float(F.p['d']))>150.,LivePID._L_downhill_dcap),   # downhill-release-r1: kept for the 4 s tail
    _SL('downhill_speed',lambda s,F:F.downhill and F.p.get('reason')!='WARMUP' and getattr(s,'dspeed',None) is not None,LivePID._L_downhill_speed),
    _SL('downhill_assist',lambda s,F:F.downhill and F.p.get('reason')!='WARMUP' and (F.p.get('image_error') or 0)>.4,LivePID._L_downhill_assist),
    _SL('downhill_dlimit',lambda s,F:F.downhill and F.p.get('reason')!='WARMUP' and F.p.get('p') is not None and F.p.get('i') is not None,LivePID._L_downhill_dlimit),
    _SL('downhill_gz',lambda s,F:F.downhill and F.p.get('reason')!='WARMUP',LivePID._L_downhill_gz),
    _SL('down_straight',_nw,LivePID._L_down_straight),
    _SL('xp_seen',None,LivePID._L_xp_seen),
    _SL('downhill_bottom',lambda s,F:getattr(s,'bottom',None) is not None and F.p.get('reason')!='WARMUP',LivePID._L_downhill_bottom),
    _SL('bottom_left_floor',None,LivePID._L_bottom_left_floor),
    _SL('jump_guard',None,LivePID._L_jump_guard),
    _SL('dh_last_err',None,LivePID._L_dh_last_err),
    _SL('wheel',None,LivePID._L_wheel),
    _SL('bottom_map_floor',None,LivePID._L_bottom_map_floor),
    _SL('bottom_owner',lambda s,F:getattr(s,'bown',None) is not None and s.bown.on,LivePID._L_bottom_owner),   # bottom-owner-r2: last writer before the upper-single override
    _SL('lap_ff4',lambda s,F:getattr(s,'lff4',None) is not None,LivePID._L_lap_ff4),
    _SL('arc2_land',lambda s,F:getattr(s,'arc2',None) is not None and s.arc2['land'] is not None,LivePID._L_arc2_land),   # arc2-r1 A: after lap_ff4, before bottom_guide   # lap-ff-c4-r1: after bottom_owner, before bottom_guide (absent -> not run)
    _SL('bottom_guide',lambda s,F:getattr(s,'bguide',None) is not None and s.bguide.on,LivePID._L_bottom_guide),   # bottom-guide-r1: landing latch / distance log (no command change)
    _SL('land4',lambda s,F:getattr(s,'land4',None) is not None,LivePID._L_land4),   # land4-r1: absent / disabled -> not run
    _SL('upper_single_final',lambda s,F:F.usv is not None,LivePID._L_upper_single_final),
    _SL('i_ramp',lambda s,F:getattr(s,'iramp',None) is not None and F.p.get('reason')!='WARMUP',LivePID._L_i_ramp),
    _SL('start_bias',lambda s,F:getattr(s,'sbias',None) is not None,LivePID._L_start_bias))   # i-ramp-r1: integral state only (no steer write); absent json -> skipped

# ---- lost-policy-r1 (integration B3c): the lost-line table, built once at import from lost_policy.json (order of groups / rows, modifiers)
LOST_TABLE=lost_policy.build(
    dict(blind=(LivePID._lp_blind_pre,LivePID._lp_blind_prelude,True),pid_hold=(None,None,False),downhill=(None,None,False),lost=(None,None,False)),
    dict(UPPER_SINGLE_HOLD=(LivePID._lp_us_hold,LivePID._lm_blind,'heading_blind',None),LANE_BLIND_HOLD=(LivePID._lp_lane_blind,LivePID._lm_blind,'heading_blind',None),
         HEADING_BLIND=(LivePID._lp_heading_blind,LivePID._lm_blind,'heading_blind',None),PID_HOLD=(LivePID._lp_pid_hold,LivePID._lm_pid_hold,'hold','pid_hold'),
         DOWNHILL_HOLD=(LivePID._lp_downhill,LivePID._lm_downhill,'downhill_hold','downhill_hold'),LOST_HEADING=(LivePID._lp_lost,LivePID._lm_lost,'lost_heading',None)),
    dict(hold_pwm=('hold_pwm',LivePID._lx_hold_pwm),turn_pwm=('turn_pwm',LivePID._lx_turn_pwm),down_straight=('down_straight',LivePID._lx_down_straight),
         crossing_hold=('crossing_hold',LivePID._lx_crossing_hold),bottom_map=('bottom_map',LivePID._lx_bottom_map),bottom_curve=('bottom_curve',LivePID._lx_bottom_curve),
         downhill_gz_hold=('downhill_gz',LivePID._lx_downhill_gz_hold),downhill_clamp=('downhill_hold_clamp',LivePID._lx_downhill_clamp),
         as_pid_hold=('as_pid_hold',LivePID._lx_as_pid_hold),downhill_gz_if_downhill=('downhill_gz',LivePID._lx_downhill_gz_if)))
for _g in LOST_TABLE:   # xp-corridor-r2: lost-frame modifier after crossing_hold (rows UPPER_SINGLE_HOLD / LANE_BLIND_HOLD / HEADING_BLIND / PID_HOLD / LOST_HEADING)
    for _r in _g.rows:
        if any(_m[0]=='crossing_hold' for _m in _r.mods):_r.mods=_r.mods+(('xp_corridor','xp_corridor',LivePID._lx_xp_corridor),)
