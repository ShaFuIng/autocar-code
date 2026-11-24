"""First-case offline candidate: current boundary-supported straight corridor.
Does not confirm an exit, change entry estimation or extend E memory lifetime.
"""
import cv2
import numpy as np
from junction_memory import JunctionMemory, PARAMS as E_PARAMS
from junction_straight import white_runs
PARAMS=dict(E_PARAMS,corridor_min_rows=10,corridor_min_span_px=72,corridor_clearance_px=18,corridor_y_range=[190,334])

def corridor_support(mask,entry):
    rows=[];blocked=[];path=[]
    for y in range(190,335,4):
        x=float(np.polyval(entry,y));path.append([x,float(y)])
        if not 18<=x<622:blocked.append(y);continue
        runs=white_runs(mask,y)
        if mask[y,max(0,int(x)-18):min(640,int(x)+19)].any():blocked.append(y)
        left=[b for a,b in runs if b<x];right=[a for a,b in runs if a>x]
        if left and right and x-max(left)>18 and min(right)-x>18:rows.append(y)
    span=max(rows)-min(rows) if rows else 0
    return dict(supported=len(rows)>=10 and span>=72 and not blocked,paired_rows=len(rows),span_px=span,blocked_rows=blocked),path

class JunctionCorridor(JunctionMemory):
    def step(self,image,mask,t,baseline):
        r=super().step(image,mask,t,baseline);j=r['junction']
        m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
        support,path=corridor_support(m,self.entry) if self.entry is not None else (dict(supported=False,reason='NO_ENTRY'),[])
        j['corridor_support']=support;j['parameters']=PARAMS
        if not r['valid'] and j['active'] and j['memory_aligned'] and support['supported']:
            r.update(path=[[x*2,y*2] for x,y in path],valid=True,quality=.35,mode='junction_corridor_inferred',reason='CURRENT_CORRIDOR_INFERENCE',road_segments=[dict(source='junction_inferred',points=[[x*2,y*2] for x,y in path],confidence=.35,age_s=0)],history_segments=[],history_path=[],history_points=0,coverage=dict(visible_rows=0,double_rows=0,components=1,visible_span_px=0,control_span_px=288))
            j.update(reason='CURRENT_CORRIDOR_INFERENCE',inferred_path=path,exit_confirmed=False)
        return r
