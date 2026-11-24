"""Offline candidate only; current vehicle entry untouched."""
from collections import deque
import copy
import numpy as np
from junction_approach_candidate import ApproachGate
from junction_k_mask import mask_evidence

class TemporalBranchGate(ApproachGate):
 def __init__(self):
  super().__init__();self.branch_track=deque();self.support_at=None;self.branch_latch=None
 def can_publish_early(self):
  return False # Offline candidate requires current-frame branch/entrance check.
 def step(self,d,reference,compatible,continuous):
  t=d['capture_s']
  if self.last_t is not None and (t<=self.last_t or t-self.last_t>1):self.__init__()
  if d['a_supported'] and continuous:self.support_at=t
  recent=self.support_at is not None and t-self.support_at<=1.5
  while self.branch_track and t-self.branch_track[0][0]>1.5:self.branch_track.popleft()
  points=d.get('temporal_branch_points',[])
  if self.state in ('A','SUSPECT') and recent and continuous and reference and d['reference_compatible'] and points:
   # Associate the same branch by position, never by white pixel count alone.
   p=min(points,key=lambda p:abs(p[0]-self.branch_track[-1][1])+abs(p[1]-self.branch_track[-1][2])) if self.branch_track else max(points,key=lambda p:p[1])
   if not self.branch_track or (abs(p[0]-self.branch_track[-1][1])<=35 and -8<=p[1]-self.branch_track[-1][2]<=28):self.branch_track.append((t,*p))
  progress=self.branch_track[-1][2]-self.branch_track[0][2] if len(self.branch_track)>1 else 0
  if len(self.branch_track)>=4 and progress>=8:self.branch_latch=self.branch_track[-1][0]+1
  retained=self.branch_latch is not None and t<=self.branch_latch
  old=self.state;r=super().step(d,reference,compatible,continuous)
  if old in ('A','SUSPECT') and self.state in ('A','SUSPECT') and retained:
   r.update(state='SUSPECT',reason='TEMPORAL_BRANCH_WAIT_ENTRANCE');self.state='SUSPECT'
   if reference and d['reference_compatible'] and sum(self.refs)>=3 and continuous and not self.cooldown:
    self.state='CROSS';self.good.clear();self.alpha=0.;self.clear=0;self.passed=False
    r.update(state='CROSS',reason='TEMPORAL_BRANCH_AND_ENTRANCE',alpha=0.)
  r.update(branch_votes=len(self.branch_track),branch_progress_px=progress,branch_retained=retained,recent_double_support=recent)
  return r

def augment(d,mask,a):
 d=copy.deepcopy(d);raw=mask_evidence(mask,a);points=[]
 for y in sorted(set(p[1] for p in raw['branch_points'] if 210<=p[1]<=270)):
  env=next((e for e in d['lane_envelope'] if e[2]==y),None)
  if env is None:continue
  lo,hi,_=env;xs=sorted(p[0] for p in raw['branch_points'] if p[1]==y)
  if hi-lo<32:continue
  anchors=[min(xs,key=lambda x:abs(x-lo)),min(xs,key=lambda x:abs(x-hi))]
  if abs(anchors[0]-lo)>32 or abs(anchors[1]-hi)>32 or anchors[0]==anchors[1]:continue
  extra=[x for x in xs if x not in anchors and (0<lo-x<min(160,.55*(hi-lo)) or 0<x-hi<min(160,.55*(hi-lo)))]
  if extra:points.append([min(extra,key=lambda x:min(abs(x-lo),abs(x-hi))),y])
 d['temporal_branch_points']=points
 return d

def guard_single(a,gate):
 # Do not extrapolate a short isolated line through an unobserved opening.
 r=copy.deepcopy(a)
 if gate.state not in ('SUSPECT','CROSS') or a.get('motion',{}).get('valid'):return r
 segments=a.get('road_segments',[])
 if any(s['source']=='double' for s in segments):return r
 supported=[p for s in segments if s['source']=='single' for p in s['points']]
 if supported:
  lo=min(p[1] for p in supported);hi=max(p[1] for p in supported)
  r['path']=[p for p in a['path'] if lo<=p[1]<=hi];r['valid']=bool(r['path']);r['single_guard']='OBSERVED_SEGMENT_ONLY_NO_HISTORY'
 return r
