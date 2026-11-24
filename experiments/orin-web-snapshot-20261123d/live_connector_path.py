"""Causal experimental connector; v3 selects the route, no offline anchors.
Only modifies v3 CURRENT_STRAIGHT_APPROACH frames. No future frame/image motion.
"""
import numpy as np
import cv2
from straight_preview_path import StraightPreviewLine
REVISION='straight-connector-live-r1'
ALGORITHM='straight-connector-v1'
def connect(path,mask,center_px):
    diag=dict(revision=REVISION,applied=False,reason='SHORT_BASE_PATH',future_frames=False,offline_anchor=False,center_px=float(center_px),metric_calibration=False)
    p=np.asarray(path,dtype=float)/2
    if len(p)<4 or not np.isfinite(p).all():return path,diag
    p=p[np.argsort(p[:,1])];far=float(p[0,1]);near=float(p[-1,1])
    if far>=240 or near-far<40:return path,diag
    support=p[p[:,1]<=far+40]
    if len(support)<3:return path,diag
    slope=float(np.polyfit(support[:,1],support[:,0],1)[0]);end=p[0];origin=np.array([center_px,340.])
    controls=np.array([origin,[center_px,275.],end+np.array([slope*35,35]),end])
    u=np.linspace(0,1,161)[:,None]
    curve=(1-u)**3*controls[0]+3*(1-u)**2*u*controls[1]+3*(1-u)*u*u*controls[2]+u**3*controls[3]
    if not np.isfinite(curve).all() or np.any(curve[:,0]<0) or np.any(curve[:,0]>=640) or np.any(np.diff(curve[:,1])>=0):
        diag['reason']='OUT_OF_FRAME_OR_NONMONOTONIC';return path,diag
    # Dense raster check against current observed white mask; if a proposed
    # connector intersects it, retain the unchanged v3 path, not a new stop rule.
    raster=np.zeros((360,640),np.uint8);cv2.polylines(raster,[np.round(curve).astype(np.int32)],False,1,1)
    hits=int(np.count_nonzero((raster>0)&mask));diag.update(boundary_hits=hits,goal=end.tolist(),goal_slope=slope)
    if hits:diag['reason']='CROSSING_OBSERVED_LINE_KEEP_V3';return path,diag
    ys=np.arange(int(np.ceil(far)),341,4);ascending=curve[::-1]
    output=[[float(np.interp(y,ascending[:,1],ascending[:,0])*2),float(y*2)] for y in ys]
    diag.update(applied=True,reason='CURRENT_V3_EXIT_CONNECTED');return output,diag
class LiveConnectorLine(StraightPreviewLine):
    algorithm=ALGORITHM
    center_px=320.
    def step(self,image,mask,captured):
        result=super().step(image,mask,captured)
        base=result['path'];detail=result['path_diagnostic'];selection=detail['line_estimate'].get('straight_selection',{})
        note=dict(revision=REVISION,applied=False,reason='V3_NOT_IN_STRAIGHT_APPROACH',future_frames=False,offline_anchor=False,center_px=float(self.center_px))
        if result['valid'] and selection.get('active'):
            m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
            result['path'],note=connect(base,m,self.center_px)
        detail.update(path_revision=ALGORITHM,connector=note,base_path=base,connector_revision=REVISION,junction_policy='Unchanged v3 route selection; causal curve only during v3 straight approach')
        return result
