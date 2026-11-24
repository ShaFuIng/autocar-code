import sys,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'laptop_workbench/runtimes/opencv-4.11'))
from analyze_junction_4396573e import *
from junction_k_lane import KLanePath
cv2.setNumThreads(2);tracker=KLanePath(.6);rows=[];tiles=[]
for seq,t,im,mask,live,meta in load_tunnel()[0]:
 r=tracker.step(im,mask,t);k=r['k'];gate=k['mask_gate']
 rows.append(dict(seq=seq,captured=t,mode=r['mode'],path=r['path'],gate=gate,decision=k['visible_decision'],motion=r['motion'],coverage=r['coverage'],rejections=r['rejections']))
 if seq in [60,75,90,105,120,138,139]:
  v=im.copy();v[mask]=(v[mask]*.65+np.array([255,0,255])*.35).astype('uint8')
  if live['path']:cv2.polylines(v,[np.int32(np.array(live['path'])/2)],False,(255,255,0),2)
  cv2.putText(v,f'seq{seq} live={live["path_diagnostic"]["mode"]}',(10,22),0,.6,(0,255,255),2)
  tiles.append(v)
(OUT/'replay.json').write_text(json.dumps(rows,indent=2),encoding='utf8')
cv2.imencode('.jpg',np.vstack(tiles))[1].tofile(str(OUT/'review.jpg'))
for r in rows:
 g=r['gate']
 if r['seq']%5==0 or r['seq']>=135:print(r['seq'],r['mode'],g['raw_zone_counts'],[z['branch_rows'] for z in g['zones']],g['opening_votes'],g['near_votes'],g['reference_compatible'],g['reference_ready'],g['reason'],r['decision'],flush=True)
