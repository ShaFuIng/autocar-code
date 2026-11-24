"""Pure CPU adapter parity probe; no camera, UART, PID start or network."""
import sys,json,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path[:0]=[str(HERE/'payload/web'),'/home/jetson/orin_interface_v1/web','/home/jetson/orin_interface_v1/web/k_lane_runtime']
import cv2,numpy as np
from k_integrated_live import IntegratedK
from junction_segment_candidate import SegmentCandidate
from junction_k_lane import lane_evidence
from warm_color_candidate import WarmColor
from history_gap_candidate import HistoryGap

def main():
 cv2.setNumThreads(2);data=np.load(HERE/'fixtures.npz');live=IntegratedK();ref=SegmentCandidate();color=WarmColor();gap=HistoryGap();last=None;rows=[]
 try:
  for i,(image,mask,t) in enumerate(zip(data['images'],data['masks'],data['times'])):
   if last is not None and (t<=last or t-last>1):ref=SegmentCandidate();color=WarmColor();gap=HistoryGap()
   last=t;processed=live.preprocess(image,float(t));start=time.perf_counter();r=live.step(processed,mask,float(t));elapsed=(time.perf_counter()-start)*1000
   corrected,_=color.step(image,float(t));a=ref.base.step(corrected,mask,float(t));v=ref.visible.step(corrected);d=lane_evidence(mask,a,v);d['capture_s']=float(t);b=gap.step(image,mask,float(t),ref.combine(a,v,mask,d))
   assert r['path']==b['path'] and r['path_diagnostic']['mode']==b['mode'],i
   rows.append(dict(index=i,valid=r['valid'],mode=b['mode'],ms=elapsed))
 finally:live.close()
 print(json.dumps(dict(passed=True,frames=len(rows),opencv=cv2.__version__,numpy=np.__version__,p95_path_ms=float(np.percentile([r['ms'] for r in rows],95)),max_path_ms=max(r['ms'] for r in rows),rows=rows)))
if __name__=='__main__':main()
