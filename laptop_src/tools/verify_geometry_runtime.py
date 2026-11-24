import sys,json,gzip,base64,time,copy
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'laptop_workbench/runtimes/opencv-4.11'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime')]
import cv2,numpy as np
from junction_reviewed_path import ReviewedKLane
from junction_k_lane import lane_evidence
OUT=ROOT/'laptop_workbench/data/experiments/k-geometry-deployment';results={}
run=ROOT.parent/'firmware_builds/i2c_retry_v3/run-0d77c3ca/0d77c3ca05284af9b8e5ad4d0ef81d5c'
live={x['vision']['seq']:x for x in map(json.loads,(run/'pid.jsonl').read_text().splitlines()) if x.get('vision',{}).get('mask_png')}
for name in ['latest','fullclip']:
 tr=ReviewedKLane();expected=json.loads((OUT.parent/'k-geometry-guard'/f'{name}.json').read_text());mismatch=[];times=[]
 for i,line in enumerate(gzip.open(OUT.parent/'k-branch-temporal'/f'{name}-geometry.jsonl.gz','rt',encoding='utf-8')):
  c=json.loads(line);seq=c['seq'];a=c['a'];v=c['v'];t=c['t']
  raw=np.frombuffer(base64.b64decode(live[seq]['vision']['mask_png']),np.uint8) if name=='latest' else np.fromfile(str(ROOT/f'laptop_workbench/data/analysis/f7e170a0869d4fdd8632575997f39a4a/C/{seq:06d}.png'),np.uint8)
  mask=cv2.imdecode(raw,0)>0;start=time.perf_counter();d=lane_evidence(mask,a,dict(fused=None));d['capture_s']=t;r=tr.combine(a,v,mask,d);times.append((time.perf_counter()-start)*1000)
  e=expected[i]['candidate']
  if any(r[k]!=e[k] for k in ['path','mode','valid','reason']):mismatch.append(seq)
 results[name]=dict(frames=len(expected),mismatches=mismatch,combine_mean_ms=float(np.mean(times)),combine_max_ms=max(times));print(name,results[name],flush=True)
(OUT/'runtime-parity.json').write_text(json.dumps(results,indent=2));assert all(not r['mismatches'] for r in results.values())
