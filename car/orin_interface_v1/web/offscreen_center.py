"""Finite off-image inferred centers, current-boundary provenance, fixed control row.
Only this candidate accepts x outside the image. No extra steering authority.
"""
import math
import numpy as np
from camera_pair import CameraPairEstimate,CameraPairLine,CameraPairPID
from standalone_line_path import fit_line

def continuous_paths(points):
    chains=[]
    for point in points:
        if len(point)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in point):return []
        x,y=point
        if not 0<=y<720:return []
        if chains and y<=chains[-1][-1][1]:return []
        if not chains or y-chains[-1][-1][1]>24 or abs(x-chains[-1][-1][0])>100:chains.append([])
        chains[-1].append([x,y])
    return [c for c in chains if len(c)>=2]

def path_error(points,row_ratio,width=1280,height=720,visual_center_offset_px_640=0.):
    row=row_ratio*(height-1)
    for chain in continuous_paths(points):
        for (x0,y0),(x1,y1) in zip(chain,chain[1:]):
            if y0<=row<=y1:
                x=x0+(x1-x0)*(row-y0)/(y1-y0)
                e=(x-(width-1)/2-visual_center_offset_px_640*width/640)/((width-1)/2)
                return max(-1.,min(1.,e))
    return None

class OffscreenEstimate(CameraPairEstimate):
    allow_offscreen=True
    def _pair(self,trs):
        l,r=super()._pair(trs)
        # Multiple admissible pairs need the existing temporal identity test.
        # A unique current pair remains independent of stale prior width.
        if self.pair_diagnostic['accepted']>1:
            matched=True
            for side,tr in [('L',l),('R',r)]:
                ref=self.identity_reference.get(side) or {}
                ys=sorted(set(ref)&set(tr['pts'])) if tr else []
                if len(ys)<4 or ys[-1]-ys[0]<24 or np.median([abs(ref[y]-tr['pts'][y]) for y in ys])>30:matched=False
            if not matched:
                self.pair_diagnostic.update(reason='AMBIGUOUS_CURRENT_PAIRS',selected=None)
                return None,None
        return l,r

    def step(self,mask,t):
        result=super().step(mask,t)
        ambiguous=self.pair_diagnostic['reason']=='AMBIGUOUS_CURRENT_PAIRS'
        if ambiguous:result['path']=[]
        supports={side:[[float(x)*2,int(y)*2] for y,x in sorted(pts.items())] for side,pts in self.last.items() if pts}
        result['offscreen_center']=dict(revision='offscreen-center-r2',capture_s=t,
            coordinate_frame='1280x720_unclipped_x',current_boundaries=supports,
            observed_boundary_source='current_v4_mask_inner_edge',center_source='inferred_not_observed',
            geometry='camera-inner-20261030-r1',width_ratio=result['width_ratio'],
            fitted_boundary_sides=result['line_fit'],route=result['straight_selection'],
            outside_points=sum(not 0<=x<1280 for x,y in result['path']),
            identity_ambiguous=ambiguous,render_only_clipping=True)
        return result

class OffscreenLine(CameraPairLine):
    algorithm='camera-pair-v2'
    def set_row(self,row):
        super().set_row(row);self.estimate=OffscreenEstimate()
    def step(self,image,mask,captured):
        result=super().step(image,mask,captured)
        diag=result['path_diagnostic'];proof=diag['line_estimate']['offscreen_center']
        # Missing image evidence is not repaired with an old road or command.
        if not proof['current_boundaries'] or proof['identity_ambiguous']:
            result.update(path=[],valid=False)
            diag['road_state'].update(source='invalid',predicted=False,reason='AMBIGUOUS_BOUNDARY_IDENTITY' if proof['identity_ambiguous'] else 'NO_CURRENT_BOUNDARY')
        diag.update(path_revision=self.algorithm,controller_revision='offscreen-center-r2')
        return result

class OffscreenPID(CameraPairPID):
    path_error=staticmethod(path_error)
    continuous_paths=staticmethod(continuous_paths)
    fixed_row=True
    def __init__(self,parameters):
        super().__init__(parameters)
        # 2026-10-31: the two-point weight / far row come from the saved
        # parameters (defaults .35 / .45 = the previous fixed values).
        self.policy.update(revision='offscreen-center-r2',fixed_row=True,unclipped_center=True,
            far_weight=float(parameters.get('preview_far_weight',.35)),far_row=float(parameters.get('preview_far_row',.45)),preview_parameter_version=1)
    def step_path(self,t,points,envelope=None):
        result=super().step_path(t,points,envelope)
        result['sampling_version']='offscreen-center-r2'
        result['sample_reason']='FIXED_REQUESTED' if result['sample_bounds'] else 'NO_SUPPORTED_FIXED_ROW'
        return result
