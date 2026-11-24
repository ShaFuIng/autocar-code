"""Offline shared-lane gate. No vehicle-control imports or time-based exceptions."""
import copy
from collections import deque
import cv2,numpy as np
from junction_k_mask import KMaskPath,mask_evidence
from road_segments_v32 import RoadPath
from junction_k import center,evidence

def lane_evidence(mask,a,v):
 raw=mask_evidence(mask,a);d=copy.deepcopy(raw);ap=(np.array(a['path'])/2).tolist()
 edges=[np.asarray(e,float) for e in a.get('observed_edges',[])];m=cv2.resize(mask.astype('uint8'),(640,360),interpolation=cv2.INTER_NEAREST)>0
 # White components seeded only by selected near-entrance boundaries.
 _,labels=cv2.connectedComponents(m.astype('uint8'),8);owned=set()
 for e in edges:
  for x,y in e:
   if y<270:continue
   ix,iy=int(round(x)),int(round(y));patch=labels[max(0,iy-2):min(360,iy+3),max(0,ix-6):min(640,ix+7)]
   owned.update(int(z) for z in np.unique(patch) if z)
 d['owned_components']=len(owned)
 d['raw_zone_counts']=[z['branch_rows'] for z in raw['zones']];d['branch_points']=[];d['lane_envelope']=[]
 for z in d['zones']:
  z['rows']=[]
  bounds={'far':(150,210),'middle':(210,270),'near':(270,334)}[z['name']]
  for y in range(*bounds,4):
   cx=center(ap,y)
   if cx is None:continue
   xs=[float(np.interp(y,e[:,1],e[:,0])) for e in edges if len(e)>1 and e[0,1]<=y<=e[-1,1]]
   left=[x for x in xs if x<cx];right=[x for x in xs if x>cx]
   if left and right:lo,hi=max(left),min(right)
   elif xs:
    half=abs(min(xs,key=lambda x:abs(x-cx))-cx);lo,hi=cx-half,cx+half
   else:continue
   if hi-lo<16:continue
   margin=min(40.,.2*(hi-lo));d['lane_envelope'].append([float(lo),float(hi),float(y)])
   line=m[max(0,y-1):y+2].mean(0)>.3;ds=np.diff(np.r_[False,line,False].astype('int8'))
   runs=[(int(l),int(r)) for l,r in zip(np.flatnonzero(ds==1),np.flatnonzero(ds==-1)) if 2<=r-l<=180 and int(labels[y,int((l+r-1)/2)]) in owned]
   if len(runs)>=3:
    z['rows'].append(y);d['branch_points'].extend([[(l+r)/2,float(y)] for l,r in runs])
  z['branch_rows']=len(z['rows'])
 d['far_branch']=d['zones'][0]['branch_rows']>=2
 d['opening']=d['zones'][1]['branch_rows']>=3 or d['zones'][2]['branch_rows']>=2
 d['near_opening']=sum(y>=246 for z in d['zones'][1:] for y in z['rows'])>=2
 return reference_evidence(d,a,v)

def reference_evidence(d,a,v):
 ap=(np.array(a['path'])/2).tolist()
 fits=v['fused'];errors={};angle=None
 if fits is not None:
  f=np.mean(fits,axis=0)
  for y in [190,210,230,260,300,326]:
   x=center(ap,y)
   if x is not None:errors[str(y)]=float(np.polyval(f,y)-x)
  if '230' in errors and '326' in errors:angle=float(abs(np.degrees(np.arctan((errors['230']-errors['326'])/96))))
 near=[abs(x) for y,x in errors.items() if int(y)>=230];far=[abs(x) for y,x in errors.items() if int(y)<=210]
 d.update(entrance_errors=errors,entrance_angle_error=angle,reference_compatible=bool(len(near)>=3 and max(near)<30 and angle is not None and angle<12 and (not far or max(far)<30)))
 return d

class LaneGate:
 def __init__(self):
  self.state='A';self.openings=deque(maxlen=12);self.near=deque(maxlen=8);self.refs=deque(maxlen=5);self.good=deque(maxlen=5);self.alpha=0.;self.clear=0;self.cooldown=0;self.loss=0;self.passed=False
 def step(self,d,reference,compatible,continuous):
  self.openings.append(d['opening']);self.near.append(d['near_opening']);self.refs.append(d['reference_compatible']);self.good.append(compatible)
  self.cooldown=max(0,self.cooldown-1);reason='FOLLOW_A';self.loss=self.loss+1 if not d['a_supported'] else 0
  confirmed=sum(self.openings)>=5 and sum(self.near)>=3
  ready=reference and d['reference_compatible'] and sum(self.refs)>=3
  if self.state in ('A','SUSPECT'):
   self.state='SUSPECT' if d['opening'] or sum(self.openings)>=2 else 'A';reason='LANE_SUSPECT_KEEP_A' if self.state=='SUSPECT' else 'FOLLOW_A'
   if confirmed and ready and not self.cooldown:self.state='CROSS';self.good.clear();self.alpha=0.;self.clear=0;self.passed=False;reason='SAME_LANE_OPENING_AND_REFERENCE'
   elif confirmed and not ready:reason='K_ENTRANCE_MISMATCH_KEEP_A'
  elif self.state=='CROSS':
   self.passed=self.passed or self.loss>=3
   if not reference:self.state='SUSPECT';self.cooldown=10;reason='REFERENCE_LOST_RETURN_A'
   else:
    self.clear=self.clear+1 if d['a_supported'] and continuous and not d['opening'] else 0
    if (self.clear>=6 and not d['reference_compatible']) or (self.passed and len(self.good)==5 and sum(self.good)>=4 and not d['opening']):self.state='RETURN_A';self.alpha=.2;reason='LANE_EXIT_CONFIRMED'
    else:reason='CROSS_KEEP_REFERENCE'
  elif self.state=='RETURN_A':
   if not reference:self.state='A';self.alpha=1.;self.cooldown=10;reason='RETURN_A_NO_REFERENCE'
   elif not d['a_supported']:self.state='CROSS';self.alpha=0.;reason='EXIT_DOUBLE_LOST'
   else:
    self.alpha=min(1.,self.alpha+.2);reason='BLEND_BACK_TO_A'
    if self.alpha>=.999:self.state='A';self.cooldown=10;self.openings.clear();self.near.clear();reason='HANDOFF_COMPLETE'
  return dict(state=self.state,reason=reason,far_votes=sum(self.refs),opening_votes=sum(self.openings),near_votes=sum(self.near),reference_ready=ready,alpha=self.alpha,reached_opening=confirmed,crossed_gap=False)

class KLanePath(KMaskPath):
 def __init__(self,row=.6):
  super().__init__(row);self.base=RoadPath('road-entrance-v3.2',None,row);self.gate=LaneGate()
 def combine(self,a,v,mask,d=None):
  d=lane_evidence(mask,a,v) if d is None else reference_evidence(d,a,v);ap=(np.array(a['path'])/2).tolist();ev=evidence(a,v['fused'])
  changes=[abs(center(ap,y)-center(self.last_a,y)) for y in [230,260,300,326] if center(ap,y) is not None and center(self.last_a,y) is not None]
  continuous=bool(changes and max(changes)<24);self.last_a=ap;fits=v['fused'];reference=fits is not None
  g=self.gate.step(d,reference,ev['eligible'],continuous);path=ap;source='A'
  if reference and g['state'] in ('CROSS','RETURN_A'):
   f=np.mean(fits,axis=0)
   if g['state']=='CROSS':path=[[float(np.polyval(f,y)),float(y)] for y in range(160,335,2)];source='K'
   else:path=[[float((1-g['alpha'])*np.polyval(f,y)+g['alpha']*x),y] for x,y in ap];source='blend'
  steps={str(y):None if center(path,y) is None or center(self.last_path,y) is None else center(path,y)-center(self.last_path,y) for y in [210,260,330]};self.last_path=path
  r=copy.deepcopy(a);r.update(path=[[x*2,y*2] for x,y in path],valid=bool(path),mode='KL_'+g['state'],reason=g['reason'],hardware_output=False,k=dict(state=g['state'],source=source,revision='k-lane-v3',handoff_reason=g['reason'],mask_gate=dict(d,**g),evidence=ev,visible_decision=v['decision'],raw_steps=steps))
  if source!='A':r.update(quality=.35,road_segments=[dict(source='junction_inferred',points=r['path'],confidence=.35,age_s=0)],history_segments=[],history_path=[],path_samples=[dict(x=x,y=y,source='junction_inferred',confidence=.35,age_s=0) for x,y in path])
  return r
