import sys,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'laptop_workbench/data/experiments/k-approach'
sys.path.insert(0,str(ROOT/'laptop_workbench/runtimes/opencv-4.11'))
from analyze_junction_4396573e import load_tunnel
sys.path[:0]=[str(OUT/'candidate-runtime'),str(ROOT/'web/k_lane_runtime')]
from k_parallel_path import ParallelKLane
def main():
 expected=json.loads((OUT/'latest.json').read_text());tr=ParallelKLane(.6);counts={'frames':0,'early_a':0}
 try:
  for (seq,t,im,mask,_,_),row in zip(load_tunnel()[0],expected):
   early=[];r=tr.step(im,mask,t,on_a=early.append);e=row['candidate']
   assert r['path']==e['path'] and r['mode']==e['mode'],seq
   assert len(early)==int(r['parallel']['early_a'])
   if early:assert early[0]['path']==r['path'] and early[0]['mode']==r['mode']
   counts['frames']+=1;counts['early_a']+=len(early)
 finally:tr.close()
 counts.update(exact_path_and_mode_parity=True,hardware_output=False)
 (OUT/'parallel-validation.json').write_text(json.dumps(counts,indent=2))
 print(counts)
if __name__=='__main__':main()
