"""Reviewed temporal entrance and geometry guard; no hardware interfaces."""
from junction_k_lane import KLanePath,lane_evidence
from junction_branch_candidate import TemporalBranchGate,augment,guard_single
from junction_geometry_guard import geometry_guard

class ReviewedKLane(KLanePath):
 def __init__(self,row=.6):
  super().__init__(row);self.gate=TemporalBranchGate()
 def combine(self,a,v,mask,d=None):
  d=lane_evidence(mask,a,v) if d is None else d
  d=augment(d,mask,a)
  ga=geometry_guard(a)
  r=super().combine(guard_single(a,self.gate),v,mask,d)
  if ga.get('geometry_guard') and r['k']['source'] in ('A','blend'):
   cutoff=ga['geometry_guard']['cutoff_y']*2
   r['path']=[p for p in r['path'] if p[1]>=cutoff];r['valid']=len(r['path'])>=2
   r['geometry_guard']=ga['geometry_guard']
   # Diagnostic segments must not draw the rejected section back in.
   for key in ['road_segments','history_segments']:
    for segment in r.get(key,[]):segment['points']=[p for p in segment['points'] if p[1]>=cutoff]
   r['history_path']=[p for p in r.get('history_path',[]) if p[1]>=cutoff]
   r['path_samples']=[p for p in r.get('path_samples',[]) if p['y']>=cutoff/2]
  return r
