"""Same-frame A/K processes. No vehicle or PID interfaces."""
import sys,time,multiprocessing,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent/'k_lane_runtime'))
from junction_k_lane import KLanePath,lane_evidence
from junction_reviewed_path import ReviewedKLane
from junction_branch_candidate import TemporalBranchGate

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

class ParallelKLane:
 def __init__(self,row=.6):
  self.row=row;self.tracker=ReviewedKLane(row);self.last_capture=None;self.seq=0;self.reset_requested=False
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
   self.tracker=ReviewedKLane(self.row)
  self.last_capture=captured;self.seq+=1;seq=self.seq;start=time.perf_counter()
  gate=self.tracker.gate
  early=gate.can_publish_early()
  self.pipe.send((seq,image,reset))
  a=self.tracker.base.step(image,mask,captured)
  # This geometry is independent of K and is reused when K completes.
  d=lane_evidence(mask,a,dict(fused=None));d['capture_s']=captured
  early_result=None;ready_ms=None
  if early:
   state='SUSPECT' if d['opening'] or sum(list(gate.openings)[-11:])+int(d['opening'])>=2 else 'A'
   early_result=copy.deepcopy(a)
   early_result.update(mode='KL_'+state,reason='LANE_SUSPECT_KEEP_A' if state=='SUSPECT' else 'FOLLOW_A',
                       k=dict(state=state,source='A',revision='k-branch-geometry-r1',same_frame_seq=seq,phase='A_PUBLISHED_K_PENDING'))
   ready_ms=(time.perf_counter()-start)*1000
   if on_a is not None:on_a(early_result)
  if not self.pipe.poll(10):raise RuntimeError('K same-frame result timeout')
  result_seq,ok,v=self.pipe.recv()
  if result_seq!=seq:raise RuntimeError('K frame mismatch')
  if not ok:raise RuntimeError(v)
  r=self.tracker.combine(a,v,mask,d)
  if early_result is not None:
   if r['path']!=early_result['path'] or r['mode']!=early_result['mode']:raise RuntimeError('Early A/control finalization mismatch')
  r['k']['revision']='k-branch-geometry-r1'
  r['parallel']=dict(early_a=early,same_frame_seq=seq,ready_ms=ready_ms if early else (time.perf_counter()-start)*1000,complete_ms=(time.perf_counter()-start)*1000)
  return r
 def close(self):
  if self.process.is_alive():
   self.pipe.send(None);self.process.join(3)
   if self.process.is_alive():self.process.terminate();self.process.join()
  self.pipe.close()
