"""Current-boundary association and bounded missing-mask prediction.
No flow, full K, actuator state or future frames. Experimental image units.
"""
import cv2
import numpy as np
from live_connector_path import LiveConnectorLine
from straight_preview_path import StraightLineEstimate
from standalone_line_path import LineEstimate
from visual_pid import path_error

REVISION='road-state-r1'

class IdentityEstimate(StraightLineEstimate):
    def __init__(self,continue_steep=True):
        super().__init__(continue_steep)
        self.identity_reference={};self.identity_used=set();self.identity_repairs=[]
        self.prediction_reset=False

    def reset_route(self):
        super().reset_route();self.prediction_reset=True

    def _pick(self,trs,side):
        reference=self.identity_reference.get(side)
        # Route ownership is still handled by v3. Do not change its decisions
        # under the guise of boundary repair.
        if reference and not self.straight_started:
            matches=[]
            for tr in trs:
                if id(tr) in self.identity_used:continue
                common=sorted(set(reference)&set(tr['pts']))
                if len(common)<4 or common[-1]-common[0]<24:continue
                distance=float(np.median([abs(reference[y]-tr['pts'][y]) for y in common]))
                if distance<=30:matches.append((distance,tr))
            if matches:
                distance,tr=min(matches,key=lambda z:z[0])
                other='R' if side=='L' else 'L';opposite=self.identity_reference.get(other)
                if opposite:
                    common=sorted(set(opposite)&set(tr['pts']))
                    other_distance=float(np.median([abs(opposite[y]-tr['pts'][y]) for y in common])) if len(common)>=4 else float('inf')
                    if other_distance<distance:
                        fallback=LineEstimate._pick(self,[x for x in trs if id(x) not in self.identity_used],side)
                        if fallback:self.identity_used.add(id(fallback))
                        return fallback
                self.identity_used.add(id(tr))
                if tr['side']!=side:self.identity_repairs.append(dict(from_side=tr['side'],to_side=side,median_dx=distance))
                tr['side']=side
                return tr
        tr=LineEstimate._pick(self,[x for x in trs if id(x) not in self.identity_used],side)
        if tr:self.identity_used.add(id(tr))
        return tr

    def step(self,mask,t):
        self.identity_reference=dict(self.last) if self.t is not None and 0<t-self.t<=.4 else {}
        self.identity_used=set();self.identity_repairs=[]
        result=super().step(mask,t)
        result['boundary_identity_repairs']=self.identity_repairs
        return result

class RoadStateLine(LiveConnectorLine):
    def set_row(self,row):
        super().set_row(row)
        self.estimate=IdentityEstimate()
        self.reset_prediction()

    def reset_prediction(self):
        self.anchor=None;self.anchor_t=None;self.velocity=None
        self.accepted_count=0;self.last_input_t=None;self.was_predicted=False

    def step(self,image,mask,captured):
        if self.estimate.prediction_reset:
            self.reset_prediction();self.estimate.prediction_reset=False
        if self.last_input_t is not None and not 0<captured-self.last_input_t<=.4:self.reset_prediction()
        self.last_input_t=captured
        current=super().step(image,mask,captured)
        m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
        pixels=int(m.sum());diag=current['path_diagnostic'];est=diag['line_estimate'];route=est.get('straight_selection',{})
        detail=dict(revision=REVISION,source='observed_boundary_estimate',predicted=False,
                    observed_mask_pixels=pixels,prediction_horizon_s=.15,source_capture_s=captured,
                    anchor_age_s=0.,confidence_is_probability=False)
        if self.anchor_t is not None:detail['anchor_age_s']=captured-self.anchor_t
        observed=np.asarray(current['path'],float)/2 if current['path'] else np.empty((0,2))
        error=path_error(current['path'],self.row,visual_center_offset_px_640=self.center_px-319.75)
        reliable=(current['valid'] and error is not None and abs(error)<.6
                  and est.get('sources',{}).get('double',0)>=12
                  and not route.get('active') and route.get('reason')!='NO_CURRENT_STRAIGHT_SUPPORT')
        if pixels<=20:
            age=captured-self.anchor_t if self.anchor_t is not None else float('inf')
            if self.anchor is not None and self.accepted_count>=3 and 0<age<=.15:
                prediction=self.anchor.copy()
                if self.velocity is not None:prediction[:,0]+=np.clip(self.velocity*age,-30.,30.)
                # Drop only offscreen endpoints; never clip an offscreen center
                # into a valid target. The requested control row must remain.
                valid=(prediction[:,0]>=0)&(prediction[:,0]<640)
                indices=np.flatnonzero(valid)
                if len(indices) and np.all(valid[indices[0]:indices[-1]+1]):prediction=prediction[indices[0]:indices[-1]+1]
                predicted_error=path_error((prediction*2).tolist(),self.row,visual_center_offset_px_640=self.center_px-319.75)
                if len(prediction)>=3 and np.all((prediction[:,0]>=0)&(prediction[:,0]<640)) and predicted_error is not None and abs(predicted_error)<.6:
                    current.update(path=(prediction*2).tolist(),valid=True)
                    detail.update(source='predicted_missing_mask',predicted=True,source_capture_s=self.anchor_t,anchor_age_s=age,prediction_method='row_linear_motion' if self.velocity is not None else 'held_geometry')
                    self.was_predicted=True
            if not detail['predicted']:detail.update(source='invalid',reason='NO_RELIABLE_RECENT_ANCHOR')
        else:
            if self.was_predicted and len(observed)>=2 and self.anchor is not None:
                common=self.anchor[(self.anchor[:,1]>=observed[0,1])&(self.anchor[:,1]<=observed[-1,1])]
                expected=common[:,0].copy()
                if self.velocity is not None:
                    expected+=np.clip(np.interp(common[:,1],self.anchor[:,1],self.velocity)*(captured-self.anchor_t),-30,30)
                residual=float(np.median(abs(np.interp(common[:,1],observed[:,1],observed[:,0])-expected))) if len(common)>=4 else None
                detail.update(reacquisition_residual_px=residual,reacquisition='consistent' if residual is not None and residual<=30 else 'changed_current_observation')
                # Never blend a changed current observation back into the old
                # prediction. Replace it and rebuild confidence from new data.
                if residual is None or residual>30:self.accepted_count=0;self.velocity=None
                self.was_predicted=False
            if reliable:
                velocity=None
                if self.anchor_t is None or captured-self.anchor_t>.15:self.accepted_count=0
                if self.anchor is not None and 0<captured-self.anchor_t<=.15:
                    common=(observed[:,1]>=self.anchor[0,1])&(observed[:,1]<=self.anchor[-1,1])
                    if common.sum()>=4:
                        speed=(observed[common,0]-np.interp(observed[common,1],self.anchor[:,1],self.anchor[:,0]))/(captured-self.anchor_t)
                        if abs(float(np.median(speed)))*(captured-self.anchor_t)>30:self.accepted_count=0
                        # A row-dependent velocity predicts both lateral shift
                        # and image-space direction, rather than freezing yaw.
                        coefficients=np.polyfit(observed[common,1],speed,1)
                        velocity=np.clip(np.polyval(coefficients,observed[:,1]),-200,200)
                self.anchor=observed.copy();self.anchor_t=captured;self.velocity=velocity;self.accepted_count+=1
            else:
                self.anchor=None;self.anchor_t=None;self.velocity=None;self.accepted_count=0
            if not current['valid']:
                detail.update(source='invalid',reason='VISIBLE_BOUNDARY_NO_SUPPORTED_CENTER')
        diag.update(road_state=detail,estimator_revision=REVISION)
        return current
