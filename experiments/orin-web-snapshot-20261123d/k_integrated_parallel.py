"""Same-frame A/K processes. No vehicle or PID interfaces."""
import sys,time,multiprocessing,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent/'k_lane_runtime'))
from junction_k_lane import KLanePath,lane_evidence,reference_evidence,center
from junction_segment_candidate import selected_guard
from junction_continuity import CommittedSegmentCandidate as SegmentCandidate
from junction_branch_candidate import guard_single,augment
from junction_branch_candidate import TemporalBranchGate

def early_lane_payload(d):
 # The lane envelope is already established from this frame's mask/A edges.
 # It does not depend on pending K evidence. Snapshot it before final combine
 # mutates reference evidence; do not advertise an unverified K entrance.
 return dict(lane_envelope=copy.deepcopy(d.get('lane_envelope',[])),
             capture_s=d['capture_s'],evidence_phase='A_CURRENT_FRAME_K_PENDING')

def early_decision(tracker,a,d):
 """Prove the current control selection is independent of pending K evidence.

 Probe copies of the real gate, including this frame's approach/branch votes.
 Never advance live state or substitute an older K result.
 """
 if tracker.gate.state not in ('A','SUSPECT') or len(a.get('path',[]))<2:return None
 if selected_guard(a).get('geometry_guard') or guard_single(a,tracker.gate)['path']!=a['path']:return None
 ap=[[x/2,y/2] for x,y in a['path']]
 changes=[abs(center(ap,y)-center(tracker.last_a,y)) for y in [230,260,300,326]
          if center(ap,y) is not None and center(tracker.last_a,y) is not None]
 continuous=bool(changes and max(changes)<24)
 outcomes=[]
 # In A/SUSPECT, `compatible` only appends to the exit-quality deque; it
 # cannot trigger entry. Probe absent and strongest possible K entrance.
 # A present but incompatible reference has the same entry outcome as absent.
 for reference in (False,True):
  trial=dict(d);trial['reference_compatible']=reference
  g=copy.deepcopy(tracker.gate).step(trial,reference,True,continuous)
  if g['state'] not in ('A','SUSPECT'):return None
  outcomes.append(g)
 if len({g['state'] for g in outcomes})!=1:return None
 return outcomes[0]

def prepare_k(pipe):
 import cv2
 cv2.setNumThreads(2)
 tracker=KLanePath().visible
 pipe.send('ready')
 try:
  while True:
   job=pipe.recv()
   if job is None:break
   seq,image,reset=job
   if reset:tracker=KLanePath().visible
   try:pipe.send((seq,True,tracker.step(image)))
   except Exception as e:pipe.send((seq,False,repr(e)))
 finally:pipe.close()

class IntegratedParallelKLane:
 def __init__(self,row=.6):
  self.row=row;self.tracker=SegmentCandidate(row);self.last_capture=None;self.seq=0;self.reset_requested=False
  ctx=multiprocessing.get_context('spawn');self.pipe,child=ctx.Pipe()
  self.process=ctx.Process(target=prepare_k,args=(child,),daemon=True)
  self.process.start();child.close()
  if not self.pipe.poll(20) or self.pipe.recv()!='ready':
   self.process.terminate();self.process.join();raise RuntimeError('K preparation startup failed')
 def set_row(self,row):
  self.row=row;self.reset_requested=True
 def step(self,image,mask,captured,on_a=None):
  reset=self.reset_requested or (self.last_capture is not None and (captured<=self.last_capture or captured-self.last_capture>1))
  self.reset_requested=False
  if reset:
   self.tracker=SegmentCandidate(self.row)
  self.last_capture=captured;self.seq+=1;seq=self.seq;start=time.perf_counter()
  gate=self.tracker.gate
  early=False
  self.pipe.send((seq,image,reset))
  a_start=time.perf_counter()
  a=self.tracker.base.step(image,mask,captured)
  a_ms=(time.perf_counter()-a_start)*1000
  decision_start=time.perf_counter()
  d=None;decision=None
  if gate.state in ('A','SUSPECT') and len(a.get('path',[]))>=2:
   d=lane_evidence(mask,a,dict(fused=None));d['capture_s']=captured
   d=augment(d,mask,a)
   decision=early_decision(self.tracker,a,d)
  early=decision is not None
  decision_ms=(time.perf_counter()-decision_start)*1000
  early_result=None;ready_ms=None
  if early:
   # Only early publication consumes the preliminary evidence. Final combine
   # still computes evidence against the same-frame K reference below.
   state=decision['state']
   early_result=copy.deepcopy(a)
   early_result.update(mode='KL_'+state,reason='LANE_SUSPECT_KEEP_A' if state=='SUSPECT' else 'FOLLOW_A',
                       k=dict(state=state,source='A',revision='k-integrated-r2',same_frame_seq=seq,phase='A_PUBLISHED_K_PENDING',
                              mask_gate=early_lane_payload(d)))
   ready_ms=(time.perf_counter()-start)*1000
   early_result['parallel']=dict(early_a=True,same_frame_seq=seq,ready_ms=ready_ms,
       a_ms=a_ms,decision_ms=decision_ms,k_wait_ms=None,combine_ms=None,complete_ms=None,
       publication_reason='CURRENT_FRAME_GATE_INDEPENDENT_OF_K')
   if on_a is not None:on_a(copy.deepcopy(early_result))
  wait_start=time.perf_counter()
  if not self.pipe.poll(10):raise RuntimeError('K same-frame result timeout')
  result_seq,ok,v=self.pipe.recv()
  if result_seq!=seq:raise RuntimeError('K frame mismatch')
  if not ok:raise RuntimeError(v)
  wait_ms=(time.perf_counter()-wait_start)*1000
  combine_start=time.perf_counter()
  # Exit-quality check needs the same-frame K reference before combine.
  augmented=d is not None
  if d is None:d=lane_evidence(mask,a,v);d['capture_s']=captured
  else:d=reference_evidence(d,a,v)
  r=self.tracker.combine(a,v,mask,d,augmented=augmented)
  if early_result is not None:
   if r['path']!=early_result['path'] or r['mode']!=early_result['mode']:raise RuntimeError('Early A/control finalization mismatch')
  r['k']['revision']='k-integrated-r2'
  r['parallel']=dict(early_a=early,same_frame_seq=seq,ready_ms=ready_ms if early else (time.perf_counter()-start)*1000,complete_ms=(time.perf_counter()-start)*1000,
      a_ms=a_ms,decision_ms=decision_ms,k_wait_ms=wait_ms,combine_ms=(time.perf_counter()-combine_start)*1000,
      publication_reason='CURRENT_FRAME_GATE_INDEPENDENT_OF_K' if early else 'WAIT_SAME_FRAME_K')
  return r
 def close(self):
  if self.process.is_alive():
   self.pipe.send(None);self.process.join(3)
   if self.process.is_alive():self.process.terminate();self.process.join()
  self.pipe.close()
