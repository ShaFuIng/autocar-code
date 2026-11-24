import sys,json,base64
from pathlib import Path
import cv2,numpy as np
root=Path(__file__).resolve().parents[1];out=root/'laptop_workbench/data/experiments/k-adaptive';run=root.parent/'firmware_builds/first_fault_v1/run-db7089a3'
sys.path.insert(0,str(out/'before'));sys.path.insert(0,str(out/'payload/web'))
from visual_pid import PID
from k_adaptive_pid import KAdaptivePID
sys.path.append(str(root/'web'));sys.path.insert(0,str(root/'web/k_lane_runtime'));from junction_k_lane import KLanePath
cv2.setNumThreads(2)
d=json.loads((run/'evidence.json').read_text(encoding='utf-8'));logs=[json.loads(l) for k,v in d['files'].items() if k.endswith('/pid.jsonl') for l in v.splitlines()];meta=[json.loads(l) for k,v in d['files'].items() if 'frames_' in k for l in v.splitlines()]
params=logs[0]['parameters'];tracker=KLanePath(.6);new=KAdaptivePID(params);old=PID(params);cap=cv2.VideoCapture(str(run/'video_002.avi'))
items=[]
for x in logs:
 v=x.get('vision',{})
 if v.get('seq',0)<330:continue
 f=min(meta,key=lambda z:abs(z['capture_monotonic']-v['capture_at']))
 items.append((v['seq'],v['capture_at'],f['frame'],v['mask_png']))
items += [(f['frame'],next(x['capture_monotonic'] for x in meta if x['frame']==f['frame']),f['frame'],f['mask']) for f in json.loads((out/'tail-masks.json').read_text())]
results=[];tiles=[]
for seq,t,index,encoded in items:
 cap.set(cv2.CAP_PROP_POS_FRAMES,index);ok,image=cap.read();assert ok;image=cv2.resize(image,(640,360));mask=cv2.imdecode(np.frombuffer(base64.b64decode(encoded),np.uint8),0)>0
 r=tracker.step(image,mask,t);a=old.step_path(t,r['path'] if r['valid'] else []);b=new.step_path(t,r['path'] if r['valid'] else [])
 if seq>=370 or index>=283:
  results.append(dict(seq=seq,video_frame=index,captured=t,mode=r['mode'],path=r['path'],baseline=a,candidate=b))
  vis=image.copy();vis[mask]=(vis[mask]*.6+np.array([100,20,100])).clip(0,255).astype('uint8')
  for p,q in zip(r['path'],r['path'][1:]):cv2.line(vis,tuple((np.array(p)/2).astype(int)),tuple((np.array(q)/2).astype(int)),(255,100,0),2)
  cv2.line(vis,(0,216),(639,216),(0,255,255),1);y=int(b['sample_row_ratio']*359.5);cv2.line(vis,(0,y),(639,y),(0,255,0),1)
  cv2.putText(vis,f'video {index} row {b["sample_row_ratio"]:.3f} old={a["valid"]} new={b["valid"]}',(8,20),0,.48,(0,255,0),1);tiles.append(vis)
(out/'tail-replay.json').write_text(json.dumps(dict(note='Recomputed same v4 masks for last four AVI frames; prior images nearest recorded frames, not bit-identical live replay',rows=results),indent=2));cv2.imencode('.jpg',np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles)-1,2)]))[1].tofile(str(out/'tail-review.jpg'))
print([(r['video_frame'],r['mode'],r['baseline']['valid'],r['candidate']['valid'],round(r['candidate']['sample_row_ratio'],4)) for r in results])


