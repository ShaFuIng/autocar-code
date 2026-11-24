"""Offline mask-gated K candidate. Original junction_k and vehicle entry unchanged."""
import copy
from collections import deque
import cv2,numpy as np
from road_segments_v31 import RoadPath
from junction_k import VisibleTracker,evidence,center

def mask_evidence(mask,a):
 m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
 zones=[];branch_points=[]
 for name,low,high in [('far',150,210),('middle',210,270),('near',270,334)]:
  rows=[]
  for y in range(low,high,4):
   line=m[max(0,y-1):y+2].mean(0)>.3
   d=np.diff(np.r_[False,line,False].astype('int8'));starts=np.flatnonzero(d==1);ends=np.flatnonzero(d==-1)
   runs=[(int(l),int(r)) for l,r in zip(starts,ends) if 2<=r-l<=180]
   if len(runs)>=3:
    rows.append(y);branch_points.extend([[(l+r)/2,float(y)] for l,r in runs])
  zones.append(dict(name=name,branch_rows=len(rows),rows=rows))
 doubles=[n for n in a.get('path_samples',[]) if n['source']=='double' and 230<=n['y']<=326]
 supported=len(doubles)>=18 and np.ptp([n['y'] for n in doubles])>=64
 return dict(zones=zones,branch_points=branch_points,far_branch=zones[0]['branch_rows']>=2,
  opening=zones[1]['branch_rows']>=4 or zones[2]['branch_rows']>=3,
  near_opening=zones[2]['branch_rows']>=3,double_rows=len(doubles),a_supported=bool(supported))

class MaskGate:
 def __init__(self):
  self.state='A';self.far=deque(maxlen=8);self.opening=deque(maxlen=20);self.good=deque(maxlen=5)
  self.clear=0;self.stale=0;self.alpha=0.;self.cooldown=0;self.loss=0;self.crossed_gap=False;self.reason='FOLLOW_A'
 def step(self,d,reference,compatible,continuous):
  self.far.append(d['far_branch']);self.opening.append(d['opening']);self.good.append(compatible)
  self.loss=self.loss+1 if not d['a_supported'] else 0
  self.reason='FOLLOW_A';self.cooldown=max(0,self.cooldown-1)
  far_confirmed=sum(self.far)>=5
  structural=far_confirmed and sum(self.opening)>=3
  reached=d.get('near_opening',False) or self.loss>=3
  if self.state in ('A','SUSPECT'):
   self.state='SUSPECT' if sum(self.far)>=2 or d['opening'] else 'A'
   self.reason='SUSPECT_KEEP_A' if self.state=='SUSPECT' else 'FOLLOW_A'
   if structural and reached and reference and not self.cooldown:
    self.state='CROSS';self.alpha=0.;self.good.clear();self.clear=0;self.crossed_gap=self.loss>=3;self.reason='MASK_BRANCH_SEQUENCE_AND_K_REFERENCE'
   elif structural and not reference:self.reason='MASK_CONFIRMED_BUT_NO_K_REFERENCE_KEEP_A'
  elif self.state=='CROSS':
   self.crossed_gap=self.crossed_gap or self.loss>=3
   # A missing straight reference never deletes an available A path.
   if not reference:
    self.state='SUSPECT';self.reason='REFERENCE_LOST_RETURN_A';self.cooldown=5
   else:
    self.clear=self.clear+1 if not d['opening'] and not d['far_branch'] and d['a_supported'] and continuous else 0
    if self.clear>=6:
     self.state='RETURN_A';self.alpha=.2;self.reason='CLEAR_CURRENT_ROAD_CANCEL_JUNCTION'
    elif self.crossed_gap and len(self.good)==5 and sum(self.good)>=4 and not d['opening']:
     self.state='RETURN_A';self.alpha=.2;self.reason='EXIT_DOUBLE_CONFIRMED'
    else:self.reason='CROSS_KEEP_REFERENCE'
  elif self.state=='RETURN_A':
   if not reference:self.state='A';self.alpha=1.;self.reason='RETURN_A_NO_REFERENCE';self.cooldown=10
   elif not d['a_supported']:
    self.state='CROSS';self.alpha=0.;self.reason='EXIT_DOUBLE_LOST'
   else:
    self.alpha=min(1,self.alpha+.2);self.reason='BLEND_BACK_TO_A'
    if self.alpha>=.999:self.state='A';self.cooldown=10;self.far.clear();self.opening.clear();self.reason='HANDOFF_COMPLETE'
  return dict(state=self.state,reason=self.reason,far_votes=sum(self.far),opening_votes=sum(self.opening),reference_ready=reference,alpha=self.alpha,reached_opening=reached,crossed_gap=self.crossed_gap)

class KMaskPath:
 def __init__(self,row=.6):
  self.row=row;self.base=RoadPath('road-memory-v3.1',None,row);self.visible=VisibleTracker();self.gate=MaskGate();self.last_time=None;self.last_a=[];self.last_path=[]
 def step(self,image,mask,t):
  if self.last_time is not None and (t<=self.last_time or t-self.last_time>1):self.__init__(self.row)
  self.last_time=t
  a=self.base.step(image,mask,t);v=self.visible.step(image)
  return self.combine(a,v,mask)
 def combine(self,a,v,mask):
  d=mask_evidence(mask,a);ap=(np.array(a['path'])/2).tolist();ev=evidence(a,v['fused'])
  changes=[abs(center(ap,y)-center(self.last_a,y)) for y in [230,260,300,326] if center(ap,y) is not None and center(self.last_a,y) is not None]
  continuous=bool(changes and max(changes)<24);self.last_a=ap
  fits=v['fused'];reference=fits is not None
  g=self.gate.step(d,reference,ev['eligible'],continuous)
  path=ap;source='A'
  if reference and g['state'] in ('CROSS','RETURN_A'):
   f=np.mean(fits,axis=0);vp=[[float(np.polyval(f,y)),float(y)] for y in range(160,335,2)]
   if g['state']=='CROSS':path=vp;source='K'
   else:path=[[float((1-g['alpha'])*np.polyval(f,y)+g['alpha']*x),y] for x,y in ap];source='blend'
  steps={str(y):None if center(path,y) is None or center(self.last_path,y) is None else center(path,y)-center(self.last_path,y) for y in [210,260,330]};self.last_path=path
  r=copy.deepcopy(a);r.update(path=[[x*2,y*2] for x,y in path],valid=bool(path),mode='KM_'+g['state'],reason=g['reason'],hardware_output=False,
   k=dict(state=g['state'],source=source,revision='k-mask-v1',handoff_reason=g['reason'],mask_gate=dict(d,**g),evidence=ev,visible_decision=v['decision'],raw_steps=steps))
  if source!='A':r.update(quality=.35,road_segments=[dict(source='junction_inferred',points=r['path'],confidence=.35,age_s=0)],history_segments=[],history_path=[],path_samples=[dict(x=x,y=y,source='junction_inferred',confidence=.35,age_s=0) for x,y in path])
  return r
