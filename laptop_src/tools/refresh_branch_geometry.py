import sys,json,gzip,base64
from pathlib import Path
ROOT=Path('Orin_Interface_v1').resolve();sys.path[:0]=[str(ROOT/'laptop_workbench/runtimes/opencv-4.11'),str(ROOT/'tools'),str(ROOT/'web'),str(ROOT/'web/k_lane_runtime')]
import cv2,numpy as np
from junction_branch_candidate import augment
p=ROOT/'laptop_workbench/data/experiments/k-branch-temporal';run=ROOT.parent/'firmware_builds/i2c_retry_v3/run-0d77c3ca/0d77c3ca05284af9b8e5ad4d0ef81d5c'
masks={r['vision']['seq']:r['vision']['mask_png'] for r in map(json.loads,(run/'pid.jsonl').read_text().splitlines()) if r.get('vision',{}).get('mask_png')}
for name in ['latest','fullclip']:
 f=p/(name+'-geometry.jsonl.gz');out=p/(name+'-updated.gz')
 with gzip.open(f,'rt',encoding='utf8') as src,gzip.open(out,'wt',encoding='utf8') as dst:
  for line in src:
   c=json.loads(line);seq=c['seq'];raw=np.frombuffer(base64.b64decode(masks[seq]),np.uint8) if name=='latest' else np.fromfile(str(ROOT/f'laptop_workbench/data/analysis/f7e170a0869d4fdd8632575997f39a4a/C/{seq:06d}.png'),np.uint8)
   mask=cv2.imdecode(raw,0)>0;c['d']=augment(c['d'],mask,c['a']);dst.write(json.dumps(c)+'\n')
 out.replace(f)
